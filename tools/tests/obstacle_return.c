/* Exercise the real return state machine using controlled side echoes. */
#include <assert.h>
#include <stdio.h>
#include "modules/obstacle/scan_plan.c"
#include "modules/obstacle/obstacle.c"

static UINT flags;
static uint32_t side_seq,front_seq;
static int32_t side_mm=-1;
static int32_t front_mm=1000;
static uint32_t fault_bits;

INT tm_printf(UB const *format,...) {(void)format;return 0;}
UINT obs_state_flags_read(void) {return flags;}
void obs_state_flags_set(UINT bits) {flags|=bits;}
void obs_state_flags_clear(UINT bits) {flags&=~bits;}
void obs_state_set_phase(obs_state_phase_t phase) {(void)phase;}
void obs_state_set_route(obs_state_route_t route) {(void)route;}
void obs_state_set_width_cm(int32_t width) {(void)width;}
void obs_state_count_bypass(void) {}
void obs_state_faults_update(uint32_t set,uint32_t clear)
{fault_bits=(fault_bits|set)&~clear;}
static scan_plan_bypass_t loaded_plan;
void obs_state_plan_load(scan_plan_bypass_t *plan) {*plan=loaded_plan;}
static int32_t watch_deg;
void obs_state_watch_angle_set(int32_t angle) {watch_deg=angle;}
uint32_t obs_state_side_seq_get(void) {return side_seq;}
int32_t obs_state_side_range_get(void) {return side_mm;}
uint32_t obs_state_front_seq_get(void) {return front_seq;}
int32_t obs_state_front_range_get(void) {return front_mm;}
int32_t servo_left_limit_deg(void) {return 70;}
int32_t servo_right_limit_deg(void) {return 70;}

static obstacle_drive_t step(int32_t range,bool black,uint32_t now)
{
    obstacle_drive_t drive={0};
    side_mm=range;++side_seq;
    run_phase(&g_seq,&drive,black,now);
    return drive;
}

/* One period with each line sensor given, as obstacle_bypass_step does. */
static obstacle_drive_t step_ir(bool left,bool right,uint32_t now)
{
    obstacle_drive_t drive={0};
    side_mm=4000;++side_seq;
    g_seq.b_ir_left=left;g_seq.b_ir_right=right;
    run_phase(&g_seq,&drive,left||right,now);
    g_seq.b_ir_left=0U;g_seq.b_ir_right=0U;
    return drive;
}

static void passing(int32_t side)
{
    flags=0U;front_mm=1000;fault_bits=0U;
    g_seq=(obstacle_seq_t){0};
    g_seq.b_active=true;g_seq.phase=OBS_PHASE_PASS_SIDE;
    g_seq.b_planned=true;
    g_seq.side=side;g_seq.look_deg=-side*OBS_SIDE_LOOK_DEG;
    g_seq.pose.y_mm=(float)side*300.0F;g_seq.lateral_mm=300;
    g_seq.face_x_mm=300.0F;g_seq.rear_x_mm=300.0F; /* As handle_plan. */
    g_seq.search_side=side;
}

static obstacle_drive_t finish_side(int32_t side)
{
    passing(side);
    g_seq.pose.x_mm=500.0F;
    (void)step(240,false,100);(void)step(240,false,150);
    assert(g_seq.b_seen);
    g_seq.pose.x_mm=510.0F;
    for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS;++i)
        (void)step(-1,false,200U+i*50U);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    /* Rear bound: where the first empty ray crossed the object's side. */
    assert(g_seq.rear_x_mm >= g_seq.clear_rear_mm);
    assert(g_seq.end_x_mm >= g_seq.rear_x_mm);
    /* The body clearance for the return angle, at least the whole rear
     * body plus margin past the rear (19 Nov), and no more. */
    {
        float const body=(float)(OBS_ROBOT_REAR_MM+OBS_SAFETY_MARGIN_MM);
        float const arc=return_clearance_mm(&g_seq);
        float const want=g_seq.rear_x_mm+(arc>body?arc:body);
        assert(g_seq.end_x_mm>=want-0.5F && g_seq.end_x_mm<=want+0.5F);
    }
    g_seq.pose.x_mm=g_seq.end_x_mm-1.0F;
    (void)step(-1,false,600);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
    (void)step(-1,false,650);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    return step(-1,false,700);
}

int main(void)
{
    /* Every intermediate return angle must retain the body-side margin,
     * including the smallest permitted gap and larger gaps. */
    for(int gap=OBS_HALF_PATH_MM;gap<=500;gap++)
    {
        obstacle_seq_t seq={0};seq.beside_mm=(float)gap;
        float const past=return_clearance_mm(&seq);
        for(int deg=0;deg<=OBS_RETURN_DEG;deg++)
        {
            float const rad=(float)deg*OBS_RAD_PER_DEG;
            float const clearance=past*OBS_SINF(rad) +
                ((float)gap-OBS_TRACK_WIDTH_MM/2)*OBS_COSF(rad);
            assert(clearance+0.001F>=OBS_BODY_PAST_WHEEL_MM+
                                      OBS_SAFETY_MARGIN_MM);
        }
    }
    printf("PASS swept return clearance for all gaps/angles; sequencer %zu bytes\n",
           sizeof(obstacle_seq_t));
    obstacle_seq_t geometry={0};
    pose_update(&geometry,-1056,1056);
    assert(geometry.pose.x_mm>219.90F && geometry.pose.x_mm<219.93F);
    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-OBS_SENSOR_TO_AXLE_MM-1.0F;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE && !g_seq.b_seen);
    g_seq.pose.x_mm+=2.0F;
    (void)step(-1,false,150);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    assert(g_seq.end_x_mm>=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM);
    /* 19 Nov: the whole body (rear plus margin) clears the assumed rear
     * before turning in; turning earlier swung back in beside the box. */
    assert(g_seq.end_x_mm+0.5F>=g_seq.rear_x_mm+OBS_ROBOT_REAR_MM+
           OBS_SAFETY_MARGIN_MM);
    g_seq.pose.x_mm=g_seq.end_x_mm-1.0F;
    (void)step(-1,false,200);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
    (void)step(-1,false,250);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    puts("PASS never-seen object returns after the assumed depth and body");

    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    (void)step(240,false,150);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
    (void)step(240,false,200);
    assert(g_seq.b_seen && g_seq.phase==OBS_PHASE_PASS_SIDE);
    puts("PASS late echoes from an unseen object switch to normal tracking");

    obstacle_drive_t drive=finish_side(1);
    assert(drive.left_rpm==OBS_DRIVE_RPM && drive.right_rpm==0);
    puts("PASS confirmed side absence plus whole-body travel permits right return");
    g_seq.pose.heading_rad=-OBS_RETURN_DEG*OBS_RAD_PER_DEG;
    (void)step(4000,false,1100);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE);
    g_seq.pose.y_mm=80.0F;
    (void)step(4000,true,1200);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE);
    drive=step(4000,true,1250);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE && g_seq.b_black_seen &&
           !drive.b_line_follow);
    g_seq.pose.y_mm=5.0F;
    (void)step(4000,false,1300);
    assert(g_seq.phase==OBS_PHASE_ALIGN && g_seq.turn_dir==1);
    (void)step_ir(true,false,1350); /* leading (left) sensor crosses */
    g_seq.pose.heading_rad=0.0F;
    (void)step(4000,false,1400);
    assert(g_seq.phase==OBS_PHASE_VERIFY);
    for(uint32_t i=0;i<OBS_VERIFY_CYCLES;i++)
        drive=step(4000,true,1500U+i*50U);
    assert(drive.b_done && !g_seq.b_active);
    puts("PASS full return, IR reacquisition, alignment and handback");

    /* Issue: after the return the robot sometimes ended just beside the
     * line and line following drove on along the white. Straightening up
     * swept both sensors over it: the line is on the trailing (right)
     * side, so rejoin towards it rather than hand back. */
    {
        float const y_before=80.0F;
        drive=finish_side(1);
        g_seq.pose.heading_rad=-OBS_RETURN_DEG*OBS_RAD_PER_DEG;
        (void)step(4000,false,1100);
        g_seq.pose.y_mm=y_before;
        (void)step(4000,true,1200);(void)step(4000,true,1250);
        g_seq.pose.y_mm=5.0F;
        (void)step(4000,false,1300);
        g_seq.pose.y_mm=40.0F; /* beyond OBS_ALIGN_ON_LINE_MM */
        assert(g_seq.phase==OBS_PHASE_ALIGN && g_seq.b_line_past);
        (void)step_ir(true,false,1350);
        (void)step_ir(false,true,1400); /* trailing sensor too: swept over */
        g_seq.pose.heading_rad=0.0F;
        drive=step(4000,false,1450);
        assert(g_seq.phase==OBS_PHASE_TURN_BACK && g_seq.b_seeking);
        assert(g_seq.search_side==1 && g_seq.seek_legs==1U);
        assert(g_seq.turn_target_ddeg==-OBS_SEEK_DEG*DDEG_PER_DEG);
        assert(!drive.b_done);
        puts("PASS line swept past both sensors: rejoin to the right");

        /* Neither sensor reached it again: the line is on the leading
         * (left) side. */
        drive=finish_side(1);
        g_seq.pose.heading_rad=-OBS_RETURN_DEG*OBS_RAD_PER_DEG;
        (void)step(4000,false,1100);
        g_seq.pose.y_mm=y_before;
        (void)step(4000,true,1200);(void)step(4000,true,1250);
        g_seq.pose.y_mm=5.0F;
        (void)step(4000,false,1300);
        g_seq.pose.y_mm=40.0F; /* beyond OBS_ALIGN_ON_LINE_MM */
        g_seq.pose.heading_rad=0.0F;
        drive=step(4000,false,1400);
#if (0 != OBS_RETURN_TO_BLACK)
        /* 23 Nov: first keep turning up to OBS_ALIGN_MORE_DEG for it (a
         * stop at an angle to the line puts it further round). */
        assert(g_seq.phase==OBS_PHASE_ALIGN && g_seq.b_align_more);
        assert(g_seq.turn_target_ddeg==OBS_ALIGN_MORE_DEG*DDEG_PER_DEG);
        assert(drive.right_rpm>0 && drive.left_rpm==0);
        g_seq.pose.heading_rad=OBS_ALIGN_MORE_DEG*OBS_RAD_PER_DEG;
        drive=step(4000,false,1450);
#endif
        assert(g_seq.phase==OBS_PHASE_TURN_BACK && g_seq.search_side==-1);
        assert(g_seq.turn_target_ddeg==OBS_SEEK_DEG*DDEG_PER_DEG);
        puts("PASS turn fell short of the line: rejoin to the left");

        /* Right bypass mirrors: right sensor leads, left trails. */
        drive=finish_side(-1);
        g_seq.pose.heading_rad=OBS_RETURN_DEG*OBS_RAD_PER_DEG;
        (void)step(4000,false,1100);
        g_seq.pose.y_mm=-y_before;
        (void)step(4000,true,1200);
        drive=step(4000,true,1250);
        assert(g_seq.phase==OBS_PHASE_SEARCH_LINE && g_seq.b_black_seen &&
               !drive.b_line_follow);
        g_seq.pose.y_mm=-5.0F;
        (void)step(4000,false,1300);
        assert(g_seq.phase==OBS_PHASE_ALIGN && g_seq.turn_dir==-1);
        (void)step_ir(false,true,1350);
        g_seq.pose.heading_rad=0.0F;
        (void)step(4000,false,1400);
        assert(g_seq.phase==OBS_PHASE_VERIFY);
        (void)finish_side(-1);
        g_seq.pose.heading_rad=OBS_RETURN_DEG*OBS_RAD_PER_DEG;
        (void)step(4000,false,1100);
        g_seq.pose.y_mm=-y_before;
        (void)step(4000,true,1200);(void)step(4000,true,1250);
        g_seq.pose.y_mm=-5.0F;
        (void)step(4000,false,1300);
        g_seq.pose.y_mm=-40.0F; /* beyond OBS_ALIGN_ON_LINE_MM */
        (void)step_ir(true,true,1350);
        g_seq.pose.heading_rad=0.0F;
        (void)step(4000,false,1400);
        assert(g_seq.phase==OBS_PHASE_TURN_BACK && g_seq.search_side==-1);
        puts("PASS right bypass: leading sensor only hands back, both rejoin");

        /* Logged 19 Nov 13:05: odometry within 11-17 mm of the line, both
         * sensors crossed, and the seek legs that followed made the rejoin
         * worse. Within OBS_ALIGN_ON_LINE_MM hand back, telling line
         * following which side the line is on. */
        drive=finish_side(1);
        g_seq.pose.heading_rad=-OBS_RETURN_DEG*OBS_RAD_PER_DEG;
        (void)step(4000,false,1100);
        g_seq.pose.y_mm=y_before;
        (void)step(4000,true,1200);(void)step(4000,true,1250);
        g_seq.pose.y_mm=5.0F;
        (void)step(4000,false,1300);
        (void)step_ir(true,false,1350);
        (void)step_ir(false,true,1400);
        g_seq.pose.heading_rad=0.0F;
        drive=step(4000,false,1450);
        assert(drive.b_done && !g_seq.b_active && drive.line_hint==-1);
        puts("PASS beside the line by odometry: hand back, lean towards it");
    }

    /* The rejoin check is for the main return's large turn only: a seek
     * leg's small straightening turn never starts another rejoin. */
    {
        drive=finish_side(1);
        enter_phase(&g_seq,OBS_PHASE_SEARCH_LINE,0);
        g_seq.b_seeking=true;g_seq.seek_legs=1U;g_seq.search_side=1;
        g_seq.search_ddeg=-OBS_SEEK_DEG*DDEG_PER_DEG;
        g_seq.pose.heading_rad=-OBS_SEEK_DEG*OBS_RAD_PER_DEG;
        g_seq.pose.y_mm=-40.0F;
        (void)step(4000,true,1050);(void)step(4000,true,1100);
        g_seq.pose.y_mm=5.0F;
        (void)step(4000,false,1150);
        assert(g_seq.phase==OBS_PHASE_ALIGN && !g_seq.b_align_check);
        (void)step_ir(true,true,1200); /* both crossed: not checked here */
        g_seq.pose.heading_rad=0.0F;
        (void)step(4000,false,1250);
        assert(g_seq.phase==OBS_PHASE_VERIFY && g_seq.seek_legs==1U);
        puts("PASS seek leg straightening is not second-guessed");
    }

    passing(1);
    (void)step(240,false,100);(void)step(240,false,150);
    assert(g_seq.b_seen);
    for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS-1U;++i)
        (void)step(-1,false,200U+i*50U);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
    (void)step(-1,false,500);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    (void)step(240,false,550);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE && g_seq.clear_hits==0U);
    puts("PASS brief dropout cannot end tracking and reappearance cancels clearance");

    drive=finish_side(-1);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    assert(drive.left_rpm==0 && drive.right_rpm==OBS_DRIVE_RPM);
    puts("PASS right bypass mirrors the return turn to the left");

    passing(1);front_mm=100;
    ++front_seq;(void)step(-1,false,100);
    ++front_seq;drive=step(-1,false,150);
    assert(g_seq.phase==OBS_PHASE_STOPPING && drive.b_stop);
    puts("PASS confirmed front obstruction stops the passing leg");

    /* Seen all the way, staying just short of the hard travel limit. */
    passing(1);
    for(unsigned i=0;
        400.0F+(float)i*40.0F<g_seq.face_x_mm+OBS_PASS_MAX_MM;++i)
    {
        g_seq.pose.x_mm=400.0F+(float)i*40.0F;
        drive=step(240,false,100U+i*50U);
        assert(g_seq.phase==OBS_PHASE_PASS_SIDE && !drive.b_fault);
    }
    assert(g_seq.rear_x_mm>g_seq.face_x_mm+OBS_PASS_DEFAULT_MM);
    puts("PASS long obstacle stays under side watch beyond the former depth estimate");

    (void)finish_side(1);
    enter_phase(&g_seq,OBS_PHASE_PASS_END,700);
    g_seq.side_seq=side_seq; /* No new sample is available in this check. */
    drive=(obstacle_drive_t){0};
    run_phase(&g_seq,&drive,false,750);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    drive=(obstacle_drive_t){0};
    run_phase(&g_seq,&drive,false,1701);
    assert(g_seq.phase==OBS_PHASE_FAULT && drive.b_stop && drive.b_fault);
    assert(fault_bits==OBS_FAULT_TIMEOUT);
    puts("PASS cached clear samples cannot authorize a turn and a stalled worker stops");

    passing(1);g_seq.pose.x_mm=500.0F;
    (void)step(240,false,100);(void)step(240,false,150);
    g_seq.pose.x_mm=550.0F;
    for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS;i++)
        (void)step(-1,false,200U+i*50U);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    float const prior_end=g_seq.end_x_mm;
    g_seq.pose.heading_rad=-10.0F*OBS_RAD_PER_DEG;
    (void)step(-1,false,600);
    assert(g_seq.end_x_mm<prior_end-20.0F);
    assert(g_seq.end_x_mm>=g_seq.hit_rear_mm);
    puts("PASS a fresh more-sideways clearance ray tightens the return bound");

    (void)finish_side(1);
    enter_phase(&g_seq,OBS_PHASE_PASS_END,700);
    g_seq.pose.y_mm=80.0F;g_seq.pose.x_mm=g_seq.end_x_mm-10.0F;
    (void)step(-1,true,750);(void)step(-1,true,800);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    puts("PASS an early black mark cannot bypass the body-clearance check");

    (void)finish_side(1);
    g_seq.phase=OBS_PHASE_SCANNING;g_seq.resume_phase=OBS_PHASE_PASS_END;
    drive=(obstacle_drive_t){0};
    handle_plan(&g_seq,&drive,5000);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE && g_seq.clear_hits==0U);
    assert(g_seq.side_sample_msec==5000U);
    puts("PASS a front-guard rescan requires fresh side confirmation before return");

    passing(1);g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_MAX_MM;
    g_seq.b_seen=true;g_seq.side_hits=OBS_SIDE_SEEN_HITS;
    drive=step(240,false,100); /* Seen and still there. */
    assert(g_seq.phase==OBS_PHASE_FAULT && drive.b_stop && drive.b_fault);
    assert(fault_bits==OBS_FAULT_NO_ROUTE);
    puts("PASS an obstacle that never ends stops at the configured travel limit");

    /* 18 Nov video/log: odometry said 231 mm out against a 200 mm offset
     * target, so the old pass steered back in by up to 5 degrees, towards
     * the box; the side ultrasonic then measured the box face only 62 mm
     * away (inside the half width) and the corner rode onto it. */
    for(int32_t side=-1;side<=1;side+=2)
    {
        /* Further out than planned, nothing seen yet: hold, never turn in. */
        passing(side);
        g_seq.lateral_mm=200;g_seq.pose.y_mm=(float)side*231.0F;
        g_seq.pose.x_mm=200.0F;
        obstacle_drive_t d=step(-1,false,100);
        assert(side*(d.right_rpm-d.left_rpm)>=0);
        assert(offset_heading(&g_seq)==0);
        /* The side echo measures the gap short: turn out at once, hard,
         * whatever odometry says. */
        d=step(62,false,150);
        assert(g_seq.push_ddeg>0U);
        assert(side*offset_heading(&g_seq)>=100); /* at least 10 degrees */
        assert(side*(d.right_rpm-d.left_rpm)>0);   /* turning out */
        assert(g_seq.lateral_mm>=231+40);          /* target moved out */
        /* A later echo with room again: no more push, still no turn in. */
        d=step(400,false,200);
        assert(g_seq.push_ddeg==0U);
        assert(side*offset_heading(&g_seq)>=0);
        /* Repeated short echoes do not stack the offset target. */
        passing(side);
        g_seq.lateral_mm=200;g_seq.pose.y_mm=(float)side*231.0F;
        g_seq.pose.x_mm=200.0F;
        for(unsigned i=0;i<5;i++)(void)step(62,false,100U+i*50U);
        assert(g_seq.lateral_mm<=231+(int32_t)OBS_HALF_PATH_MM);
    }
    puts("PASS passing never steers into the object; a short side gap turns "
         "out at once");

    /* An unconfirmed end assumes the box is as deep as it scanned wide,
     * not just OBS_PASS_DEFAULT_MM (40 mm, shallower than the 15 cm box). */
    passing(1);
    g_seq.depth_guess_mm=150;
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-OBS_SENSOR_TO_AXLE_MM+
        5.0F;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE);   /* 40 mm alone would end */
    g_seq.pose.x_mm=g_seq.face_x_mm+150.0F-OBS_SENSOR_TO_AXLE_MM+1.0F;
    (void)step(-1,false,150);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    assert(g_seq.rear_x_mm>=g_seq.face_x_mm+150.0F);
    assert(g_seq.end_x_mm>g_seq.rear_x_mm);
    puts("PASS default end assumes the scanned width as depth");

    /* From a real plan: a 150 mm box 220 mm ahead, echoing -20..20 deg. */
    {
        scan_plan_point_t pts[OBS_SCAN_POINTS]={{0}};
        int32_t n=0;
        for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG &&
            n<OBS_SCAN_POINTS;a=scan_plan_next_angle(a))
        {
            pts[n].angle_deg=a;pts[n].range_mm=(a>=-20&&a<=20)?225U:4000U;
            ++n;
        }
        scan_plan_build_side(pts,n,0,0,&loaded_plan);
        assert(scan_plan_result(&loaded_plan)==SCAN_PLAN_ROUTE);
        g_seq=(obstacle_seq_t){0};g_seq.b_active=true;
        obstacle_drive_t d={0};
        handle_plan(&g_seq,&d,100U);
        int32_t const reported=scan_plan_width_cm(&loaded_plan)*10;
        printf("  plan for a 150 mm box: assumed depth %d mm (reported "
               "width %d mm)\n",(int)g_seq.depth_guess_mm,
               (int)scan_plan_width_cm(&loaded_plan)*10);
        /* The beam-corrected width, not the padded planning width. */
        assert(g_seq.depth_guess_mm<=OBS_OBJECT_MAX_MM);
        assert(g_seq.depth_guess_mm==reported ||
               g_seq.depth_guess_mm==OBS_OBJECT_MAX_MM);
        assert(g_seq.depth_guess_mm<scan_plan_fallback_cm(&loaded_plan)*10-
               (OBS_ROBOT_FRONT_MM+OBS_ROBOT_REAR_MM));
        /* A wide scan (echoes -40..40 deg) is still assumed no deeper than
         * the largest obstacle on the course. */
        for(int32_t i=0;i<n;i++)
            pts[i].range_mm=(pts[i].angle_deg>=-40&&pts[i].angle_deg<=40)?
                225U:4000U;
        scan_plan_build_side(pts,n,0,0,&loaded_plan);
        assert(scan_plan_result(&loaded_plan)==SCAN_PLAN_ROUTE);
        assert(scan_plan_width_cm(&loaded_plan)*10>OBS_OBJECT_MAX_MM);
        g_seq=(obstacle_seq_t){0};g_seq.b_active=true;
        handle_plan(&g_seq,&d,200U);
        assert(g_seq.depth_guess_mm==OBS_OBJECT_MAX_MM);
        loaded_plan=(scan_plan_bypass_t){0};
    }
    puts("PASS the plan sets the assumed depth from the scanned width, "
         "capped at OBS_OBJECT_MAX_MM");

    /* Odometry alone brings it back: no black needed to straighten up. */
    passing(1);
    enter_phase(&g_seq,OBS_PHASE_SEARCH_LINE,0);
    g_seq.search_ddeg=-OBS_RETURN_DEG*DDEG_PER_DEG;
    g_seq.pose.heading_rad=-OBS_RETURN_DEG*OBS_RAD_PER_DEG;
    g_seq.pose.y_mm=150.0F;
    (void)step(4000,false,100);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE);
    g_seq.pose.y_mm=5.0F;
    (void)step(4000,false,150);
#if (0 != OBS_RETURN_TO_BLACK)
    /* 23 Nov: the IR sensors decide. No black at the odometry line: keep
     * heading in, up to OBS_RETURN_SEARCH_MM beyond it, then seek. */
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE && !g_seq.b_black_seen);
    g_seq.pose.y_mm=-(float)OBS_RETURN_SEARCH_MM+1.0F;
    (void)step(4000,false,200);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE);
    g_seq.pose.y_mm=-(float)OBS_RETURN_SEARCH_MM-1.0F;
    drive=step(4000,false,250);
    puts("PASS return keeps heading in until black, bounded, then seeks");
#else
    assert(g_seq.phase==OBS_PHASE_ALIGN && !g_seq.b_black_seen);
    puts("PASS return straightens up on the odometry line without seeing black");

    /* IR never confirms the line: zig-zag instead of a LINE_LOST fault. */
    g_seq.pose.heading_rad=0.0F;g_seq.pose.y_mm=0.0F;
    (void)step(4000,false,200);
    assert(g_seq.phase==OBS_PHASE_VERIFY);
    for(uint32_t i=0;i<OBS_VERIFY_CYCLES;i++)
        drive=step(4000,false,250U+i*50U);
#endif
    assert(!drive.b_fault && !drive.b_done);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK && g_seq.b_seeking);
    assert(g_seq.seek_legs==1U);
    assert(g_seq.turn_target_ddeg==-OBS_SEEK_DEG*DDEG_PER_DEG);
    puts("PASS unconfirmed line starts a seek leg instead of faulting");

    /* The seek leg crosses the line: straighten up and hand back. */
    g_seq.pose.heading_rad=-OBS_SEEK_DEG*OBS_RAD_PER_DEG;
    (void)step(4000,false,1000);
    assert(g_seq.phase==OBS_PHASE_SEARCH_LINE);
    g_seq.pose.y_mm=-40.0F;
    (void)step(4000,true,1050);(void)step(4000,true,1100);
    assert(g_seq.b_black_seen && g_seq.phase==OBS_PHASE_SEARCH_LINE);
    g_seq.pose.y_mm=5.0F;
    (void)step(4000,false,1150);
    assert(g_seq.phase==OBS_PHASE_ALIGN);
    g_seq.pose.heading_rad=0.0F;
    (void)step_ir(true,false,1200);
    assert(g_seq.phase==OBS_PHASE_VERIFY);
    for(uint32_t i=0;i<OBS_VERIFY_CYCLES;i++)
        drive=step(4000,false,1250U+i*50U);
    assert(drive.b_done && !g_seq.b_active);
    puts("PASS seek leg finds the line, straightens and hands back");

    /* Missing the band zig-zags back; after the last leg it stops. */
    passing(1);
    enter_phase(&g_seq,OBS_PHASE_SEARCH_LINE,0);
    g_seq.b_seeking=true;g_seq.seek_legs=1U;g_seq.search_side=1;
    g_seq.pose.y_mm=-(float)OBS_SEEK_BAND_MM-1.0F;
    (void)step(4000,false,100);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK && g_seq.search_side==-1);
    assert(g_seq.turn_target_ddeg==OBS_SEEK_DEG*DDEG_PER_DEG);
    enter_phase(&g_seq,OBS_PHASE_SEARCH_LINE,200);
    g_seq.seek_legs=OBS_SEEK_LEGS;
    g_seq.pose.y_mm=(float)OBS_SEEK_BAND_MM+1.0F;
    drive=step(4000,false,300);
    /* Obstacle on a curve: no fault, hand back so line following spins
     * towards where the line should be (search_side 1 = towards -y). */
    assert(!drive.b_fault && drive.b_done && drive.line_hint==-1);
    puts("PASS seek zig-zags across the band, then hands back to spin for it");

    /* The servo follows the object round the turn out. */
    passing(1);
    g_seq.pose=(obstacle_pose_t){0.0F,0.0F,0.0F};
    g_seq.face_x_mm=260.0F;g_seq.aim_deg=OBS_AIM_NONE;
    enter_turn(&g_seq,OBS_PHASE_TURN_OUT,900,0);
    assert(0U!=(flags&OBS_EVT_SIDE_WATCH));
    (void)step(-1,false,50);
    assert(watch_deg==0);
    g_seq.pose.heading_rad=45.0F*OBS_RAD_PER_DEG;
    (void)step(-1,false,100);
    assert(watch_deg<-40 && watch_deg>=-70);
    puts("PASS servo stays pointed at the object while turning out");

    /* Something on the way out: rescan instead of a NO_ROUTE fault. */
    enter_phase(&g_seq,OBS_PHASE_DIAG_OUT,200);
    front_mm=100;fault_bits=0U;
    ++front_seq;(void)step(-1,false,250);
    ++front_seq;drive=step(-1,false,300);
    assert(g_seq.phase==OBS_PHASE_STOPPING && !drive.b_fault);
    assert(fault_bits==0U);
    puts("PASS blocked way out rescans rather than faulting");

    /* A wide initial pass credits only part of the outward pivot. */
    passing(1);
    g_seq.pose=(obstacle_pose_t){50.0F,0.0F,90.0F*OBS_RAD_PER_DEG};
    g_seq.lateral_mm=300;g_seq.out_ddeg=900;front_mm=1000;
    enter_phase(&g_seq,OBS_PHASE_DIAG_OUT,0);
    g_seq.pose.y_mm=300.0F-30.0F;
    (void)step(-1,false,50);
    int32_t const gain_left = (OBS_WIDE_PASS_PIVOT_GAIN_PCT > OBS_PIVOT_GAIN_PCT)
                              ? OBS_WIDE_PASS_PIVOT_GAIN_PCT
                              : OBS_PIVOT_GAIN_PCT;
    assert(g_seq.phase==(gain_left>=60?OBS_PHASE_TURN_PARALLEL
                                      :OBS_PHASE_DIAG_OUT));
    g_seq.pose.y_mm=300.0F;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_TURN_PARALLEL);
    if(gain_left>OBS_PIVOT_GAIN_PCT)
    {
        float const shortfall=(float)OBS_TRACK_WIDTH_MM*
                              (float)(gain_left+OBS_PIVOT_GAIN_PCT)/400.0F;

        for(int32_t k=0;k<3;k++)
        {
            int32_t const side=(k==1)?-1:1;
            passing(side);
            g_seq.pose=(obstacle_pose_t){50.0F,(float)side*(300.0F-shortfall),
                                       (float)side*90.0F*OBS_RAD_PER_DEG};
            g_seq.out_ddeg=900;
            g_seq.replans=(k==2)?1U:0U;
            enter_phase(&g_seq,OBS_PHASE_DIAG_OUT,0);
            (void)step(-1,false,50);
            assert(g_seq.phase==(k<2?OBS_PHASE_TURN_PARALLEL
                                     :OBS_PHASE_DIAG_OUT));
        }
    }
    puts("PASS only a wide initial pass credits extra outward pivot");

    /* Overshoot root cause 1: one unconfirmed echo used to disable the
     * unseen fallback, so the leg ran on to the OBS_PASS_MAX_MM fault.
     */
    {
        float const default_axle_x=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-
                                   OBS_SENSOR_TO_AXLE_MM;
        passing(1);g_seq.pose.x_mm=default_axle_x-50.0F;
        (void)step(240,false,100);
        assert(g_seq.side_hits==1U && !g_seq.b_seen);
        for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS;++i)
            (void)step(-1,false,150U+i*50U);
        assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
        g_seq.pose.x_mm=default_axle_x-1.0F;
        (void)step(-1,false,450);
        assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
        g_seq.pose.x_mm=default_axle_x+1.0F;
        (void)step(-1,false,500);
        assert(g_seq.phase==OBS_PHASE_PASS_END && g_seq.b_default_end);
        assert(g_seq.end_x_mm>=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM);
        assert(g_seq.end_x_mm<g_seq.face_x_mm+OBS_PASS_MAX_MM);
        g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
        drive=step(-1,false,550);
        assert(g_seq.phase==OBS_PHASE_TURN_BACK && !drive.b_fault);
        assert(g_seq.turn_target_ddeg==-OBS_RETURN_DEG*DDEG_PER_DEG);
        assert(drive.left_rpm==OBS_DRIVE_RPM && drive.right_rpm==0);
        puts("PASS single unconfirmed echo: default distance ends the leg");
    }

    /* Overshoot root cause 2: a seen object whose end is never confirmed
     * because self echoes keep breaking the absence run. It used to bounce
     * between PASS_END and PASS_SIDE until the travel-limit fault.
     */
    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-
                    OBS_SENSOR_TO_AXLE_MM-50.0F;
    (void)step(240,false,100);(void)step(240,false,150);
    assert(g_seq.b_seen);
    for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS;++i)
    {
        (void)step(-1,false,200U+i*100U);
        (void)step(30,false,250U+i*100U);
    }
    assert(g_seq.clear_hits<OBS_SIDE_CLEAR_HITS);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-
                    OBS_SENSOR_TO_AXLE_MM+1.0F;
    (void)step(-1,false,800);
    assert(g_seq.phase==OBS_PHASE_PASS_END && g_seq.b_default_end);
    (void)step(30,false,850);
    assert(g_seq.phase==OBS_PHASE_PASS_END);
    g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
    (void)step(-1,false,900);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    puts("PASS seen object with noisy end: default distance ends the leg");

    /* Sensor never detects the pass on a right bypass: same default, and
     * the return turn mirrors to the left.
     */
    passing(-1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-
                    OBS_SENSOR_TO_AXLE_MM;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_PASS_END && g_seq.b_default_end);
    g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
    drive=step(-1,false,150);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    assert(g_seq.turn_target_ddeg==OBS_RETURN_DEG*DDEG_PER_DEG);
    assert(drive.left_rpm==0 && drive.right_rpm==OBS_DRIVE_RPM);
    puts("PASS sensor never detects the pass: default applies, right mirrors");

    /* A confirmed end still finishes the leg before the default distance. */
    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM-
                    OBS_SENSOR_TO_AXLE_MM-100.0F;
    (void)step(240,false,100);(void)step(240,false,150);
    for(unsigned i=0;i<OBS_SIDE_CLEAR_HITS;++i)
        (void)step(-1,false,200U+i*50U);
    /* Ended by the side watch while the sensor was short of the default. */
    assert(g_seq.phase==OBS_PHASE_PASS_END && !g_seq.b_default_end);
    assert(g_seq.pose.x_mm+OBS_SENSOR_TO_AXLE_MM<
           g_seq.face_x_mm+OBS_PASS_DEFAULT_MM);
    g_seq.pose.x_mm=g_seq.end_x_mm+1.0F;
    (void)step(-1,false,500);
    assert(g_seq.phase==OBS_PHASE_TURN_BACK);
    puts("PASS confirmed end stops the leg before the default distance");

    /* An object still being seen at the default point keeps the leg going;
     * only the hard limit stops it.
     */
    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM;
    (void)step(240,false,100);(void)step(240,false,150);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE && !g_seq.b_default_end);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_MAX_MM;
    drive=step(240,false,200);
    assert(g_seq.phase==OBS_PHASE_FAULT && drive.b_fault);
    puts("PASS object still seen at the default point runs to the hard limit");

    /* A real echo after a default end means the object goes on. */
    passing(1);
    g_seq.pose.x_mm=g_seq.face_x_mm+OBS_PASS_DEFAULT_MM;
    (void)step(-1,false,100);
    assert(g_seq.phase==OBS_PHASE_PASS_END && g_seq.b_default_end);
    (void)step(240,false,150);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE && !g_seq.b_default_end);
    (void)step(-1,false,200);
    assert(g_seq.phase==OBS_PHASE_PASS_SIDE);
    puts("PASS echo after a default end resumes passing without flapping");

    /* Turn-out peek: at OBS_PEEK_DEG of the turn the sensor is ~100 mm out
     * on the chosen side, looking square at the face. Face still there:
     * the box sticks out on this side, so turn out the other way. */
    for(int32_t face_there=0;face_there<=1;face_there++)
    {
        scan_plan_point_t pts[OBS_SCAN_POINTS]={{0}};
        int32_t n=0;
        for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG &&
            n<OBS_SCAN_POINTS;a=scan_plan_next_angle(a))
        {
            pts[n].angle_deg=a;pts[n].range_mm=(a>=-20&&a<=20)?225U:1500U;
            ++n;
        }
        scan_plan_build_side(pts,n,0,0,&loaded_plan);
        assert(scan_plan_result(&loaded_plan)==SCAN_PLAN_ROUTE);
        assert(scan_plan_other_side_ok(&loaded_plan));
        flags=0U;fault_bits=0U;
        g_seq=(obstacle_seq_t){0};g_seq.b_active=true;
        obstacle_drive_t d={0};
        handle_plan(&g_seq,&d,1000U);
        assert(g_seq.phase==OBS_PHASE_TURN_OUT);
        assert(OBS_PEEK==0 || g_seq.peek_state==PEEK_FIRST);
        int32_t const first=g_seq.side;
        uint32_t t=1050U;
        /* Before the peek angle the turn runs as before. */
        g_seq.pose.heading_rad=(float)first*20.0F*OBS_RAD_PER_DEG;
        d=step(-1,false,t);
        assert(!d.b_stop);
        /* At the peek angle: stop, settle, aim square at the face. */
        g_seq.pose.heading_rad=
            (float)first*(float)(OBS_PEEK_DEG+5)*OBS_RAD_PER_DEG;
        d=step(-1,false,t+=50U);
        assert(OBS_PEEK==0 || d.b_stop);
        t+=OBS_STOP_SETTLE_MSEC;
        d=step(-1,false,t+=50U);
        assert(OBS_PEEK==0 || (d.b_stop && watch_deg==-first*(OBS_PEEK_DEG+5)));
        for(uint32_t k=0;k<OBS_PEEK_PINGS+OBS_PEEK_SKIP;k++)
            d=step(face_there?262:1500,false,t+=60U);
        if(OBS_PEEK!=0 && face_there)
        {
            /* Other way, with that side's own drive-out. */
            assert(g_seq.side==-first && g_seq.peek_state==PEEK_SECOND);
            assert(g_seq.phase==OBS_PHASE_TURN_OUT);
            assert(g_seq.turn_target_ddeg==-first*g_seq.out_ddeg);
            assert(g_seq.lateral_mm==
                   scan_plan_other_lateral_cm(&loaded_plan)*10);
            d=step(-1,false,t+=50U);
            assert(!d.b_stop); /* Swinging back. */
            /* Second side: whatever it sees, it goes (no ping-pong). */
            g_seq.pose.heading_rad=
                (float)g_seq.side*(float)(OBS_PEEK_DEG+5)*OBS_RAD_PER_DEG;
            d=step(-1,false,t+=50U);
            t+=OBS_STOP_SETTLE_MSEC;
            d=step(-1,false,t+=50U);
            for(uint32_t k=0;k<OBS_PEEK_PINGS+OBS_PEEK_SKIP;k++)
                d=step(262,false,t+=60U);
            assert(g_seq.side==-first && g_seq.peek_state==PEEK_DONE);
        }
        else
        {
            assert(g_seq.side==first);
            assert(OBS_PEEK==0 || g_seq.peek_state==PEEK_DONE);
        }
        d=step(-1,false,t+=50U);
        assert(!d.b_stop && g_seq.phase==OBS_PHASE_TURN_OUT &&
               0U==fault_bits);
        loaded_plan=(scan_plan_bypass_t){0};
    }
    assert(sizeof(obstacle_seq_t)==112U);
    puts("PASS turn-out peek: box ends -> carry on; box still there -> other "
         "side, once; sequencer still 112 bytes");

    return 0;
}
