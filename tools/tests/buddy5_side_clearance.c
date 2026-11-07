/* Side tracking: acquire the object, debounce absence and reacquire it. */
#include <assert.h>
#include <stdio.h>
#include "modules/obstacle/obstacle.c"

static uint32_t mock_seq;
static int32_t mock_mm;

INT tm_printf(UB const * format, ...)
{
    (void)format;
    return 0;
}

uint32_t obs_state_side_seq_get(void)
{
    return mock_seq;
}

int32_t obs_state_side_range_get(void)
{
    return mock_mm;
}

static void sample(obstacle_seq_t * p_seq, int32_t mm)
{
    mock_mm = mm;
    mock_seq++;
    (void)side_ping_update(p_seq);
}

int main(void)
{
    obstacle_seq_t seq = {0};
    seq.side = 1;
    seq.look_deg = -OBS_SIDE_LOOK_DEG;
    seq.lateral_mm = 300;
    seq.pose.y_mm = 300.0F;
    seq.pose.heading_rad = 0.0F;

    sample(&seq, 240);
    assert(!seq.b_seen);
    sample(&seq, -1);
    assert(!seq.b_seen && seq.side_hits == 1U);
    puts("PASS reviewed side-hit accumulation survives an intervening miss");

    sample(&seq, 240);
    assert(seq.b_seen && seq.clear_hits == 0U);
    assert(seq.lateral_mm >= 300);
    puts("PASS two qualifying side hits establish the object");

    sample(&seq, -1);
    sample(&seq, 4000);
    sample(&seq, 4000);
    assert(seq.clear_hits == 3U && seq.clear_hits < OBS_SIDE_CLEAR_HITS);
    for (unsigned i=3; i<OBS_SIDE_CLEAR_HITS; ++i) sample(&seq, -1);
    assert(seq.clear_hits == OBS_SIDE_CLEAR_HITS);
    puts("PASS sustained absence is required after first acquiring the object");
    (void)side_ping_update(&seq);
    assert(seq.clear_hits == OBS_SIDE_CLEAR_HITS);
    puts("PASS one side sample is consumed only once");
    sample(&seq, 240);
    assert(seq.clear_hits == 0U);
    puts("PASS object reappearance cancels the clear counter");

    seq.pose.x_mm=100.0F;
    sample(&seq,-1);
    assert(seq.clear_start_x_mm==100.0F);
    sample(&seq,0);
    assert(seq.clear_hits==0U);
    seq.pose.x_mm=120.0F;
    sample(&seq,-1);
    assert(seq.clear_start_x_mm==120.0F && seq.clear_hits==1U);
    puts("PASS invalid/self echo resets the continuous absence distance");

    obstacle_seq_t wide={0};
    wide.side=1;wide.look_deg=-OBS_SIDE_LOOK_DEG;
    wide.pose.y_mm=800.0F;wide.lateral_mm=800;
    sample(&wide,700);sample(&wide,700);
    assert(wide.b_seen && wide.clear_hits==0U);
    puts("PASS valid obstacle echoes beyond 500 mm remain tracked");

    obstacle_seq_t right = {0};
    right.side = -1;right.look_deg = OBS_SIDE_LOOK_DEG;
    right.pose.y_mm = -300.0F;right.lateral_mm = 300;
    sample(&right, 240);sample(&right, 240);
    assert(right.b_seen && right.rear_x_mm > 0.0F);
    puts("PASS mirrored right bypass tracks the object on its left");
    return 0;
}
