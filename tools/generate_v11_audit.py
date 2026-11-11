#!/usr/bin/env python3
"""Produce a traceable, conservative V11 Change Requirement 1 field inventory.

This is a review ledger, not a claim that automated source scanning proves
completeness or task/ISR safety. Unproven fields remain explicitly unresolved.
"""
import csv
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
INPUT=ROOT/'docs/V10/application_field_audit.csv'
OUTPUT=ROOT/'docs/V11/application_field_audit.csv'

# Field-specific measured source ranges and representations. Keys reflect the
# V10 inventory; some are continuous counters deliberately kept full-width.
PACKED={
 ('bc_code_pattern_t','element_pattern'):('0..511','9','uint16_t:9','worker, immutable table','9 bar/space pattern bits'),
 ('bc_fast_adc_stats_t','latest'):('0..4095','12','uint32_t:12','barcode worker','12-bit ADC full scale'),
 ('bc_fast_adc_stats_t','minimum'):('0..4095','12','uint32_t:12','barcode worker','12-bit ADC full scale'),
 ('bc_fast_adc_stats_t','maximum'):('0..4095','12','uint32_t:12','barcode worker','12-bit ADC full scale'),
 ('bc_port_stats_t','raw'):('0..1','1','unsigned int:1','semaphore-protected worker','raw GPIO'),
 ('bc_port_stats_t','state'):('0..5','3','unsigned int:3','semaphore-protected worker','scan FSM'),
 ('bc_port_stats_t','runs'):('0..29','5','unsigned int:5','semaphore-protected worker','Code 39 runs'),
 ('bc_port_stats_t','last_error'):('0..8','4','unsigned int:4','semaphore-protected worker','error enum'),
 ('bc_port_stats_t','last_runs'):('0..29','5','unsigned int:5','semaphore-protected worker','Code 39 runs'),
 ('bc_wave_status_t','state'):('0..7','3','unsigned char:3','barcode worker','scanner status'),
 ('bc_wave_status_t','white_mean'):('0..4095','12','unsigned int:12','barcode worker','ADC full scale'),
 ('bc_wave_status_t','white_sigma'):('0..4095','12','unsigned int:12','barcode worker','ADC spread'),
 ('bc_wave_status_t','last_valid'):('0..1','1','unsigned char:1','barcode worker','boolean'),
 ('bc_wave_status_t','last_letter'):('0..25 or 31 invalid','5','unsigned char:5','barcode worker','Code 39 letter'),
 ('bc_wave_status_t','last_reason'):('0..7','3','unsigned char:3','barcode worker','scanner rejection reason'),
 ('scan_plan_point_t','angle_deg'):('-40..40','7','signed int:7','obstacle task (local)','servo sweep, normalized by scanner configuration'),
 ('scan_plan_point_t','range_mm'):('0..4000 (4000 no echo)','12','unsigned int:12','obstacle task (local)','ultrasonic_ping rejects values >=4000'),
 ('B1_Event','kind'):('0..31','5','explicit 5 bits in 9-byte ring record','B1 store mutex','source 3 bits and kind 5 bits share octet'),
 ('B1_Event','ms'):('0..4294967295 (wrap)','32','explicit 4 bytes in 9-byte ring record','B1 store mutex','32-bit session clock'),
 ('B1_Event','value'):('0..65535','16','explicit 2 bytes in 9-byte ring record','B1 store mutex','event-specific numeric payload'),
 ('B1_SnapshotPayload','obstacle_cm'):('0..1023 (1023 unavailable)','10','unsigned int:10','B1 store mutex','clamped before writing packed field'),
 ('B1_SnapshotPayload','speed_right_mm_s'):('-2047..2047','12','signed int:12','B1 store mutex','clamped before writing packed field'),
 ('B1_Store','motions'):('32 × (time32,seq16,deltaL16,deltaR16)','320 bytes','B1_OdomRecord[32] (10 bytes each)','B1 store mutex','drops duplicate full motion state; round-trip tested'),
 ('B1_Store','events'):('24 × (time32,id16,value16,source3,kind5)','216 bytes','B1_Event[24] (9 bytes each)','B1 store mutex','explicit byte-safe serialization on Cortex-M0+'),
 ('B1_Line','event_kind'):('0..31','5','unsigned char:5','B1 store mutex','event enum'),
 ('B1_Terrain','event_kind'):('0..31','5','unsigned char:5','B1 store mutex','event enum'),
 ('B1_Mission','event_kind'):('0..31','5','unsigned char:5','B1 store mutex','event enum'),
 ('B1_Obstacle','event_kind'):('0..31','5','unsigned char:5','B1 store mutex','event enum'),
 ('obstacle_drive_t','b_stop'):('0..1','1','unsigned char:1','controller task','boolean'),
 ('obstacle_drive_t','b_line_follow'):('0..1','1','unsigned char:1','controller task','boolean'),
 ('obstacle_drive_t','b_done'):('0..1','1','unsigned char:1','controller task','boolean'),
 ('obstacle_drive_t','b_fault'):('0..1','1','unsigned char:1','controller task','boolean'),
 ('motion_status_t','b_valid'):('0..1','1','unsigned int:1','motion mutex','boolean'),
 ('steering_t','b_off_centre'):('0..1','1','unsigned int:1','controller task','boolean'),
 ('G7_TelemetryPeers','last_ir_bits'):('0..7','3','unsigned char:3','telemetry owner task','three IR flags'),
}

TIMESTAMPS={'now_msec','timestamp_msec','time_usec','edge_usec','left_edge_usec','right_edge_usec',
'hold_since','decode_idle_since','state_since','black_since','edge_since','last_barcode_ms',
'start_msec','result_msec','imu_last_ms','last_sensor_report_ms','last_barcode_report_ms',
'session_ms','last_heartbeat_ms','last_status_ms','checkpoint_ms','max_gap_usec','time_ms','ms','timestamp_usec',
'obstacle_last_status','last_process_ms'}
COUNTS={'count','left_count','right_count','travel_counts','turn_counts','black_from_counts',
'barcode_counts','last_barcode_counts','centre_from','turn_from','left_rpm_dummy',
'right_count','last_count','left_edge_usec','right_edge_usec','samples','errors',
'filtered_spikes','capture_invalid','gap_invalid','direction_invalid','skew_invalid',
'step_invalid','stale_edge','lock_lost','edges','scan_units','completed','rejected',
'last_event','last_scan_units','left_pwm_dummy','last_widths','last_barcode_seq',
'last_cmd_transaction','sequence','seq','event_id','previous_right_count',
'left_pwm','right_pwm','last_edge_usec','cumulative_mm','barcode_last_seq','last_process_ms',
'status_challenges','max_wheel_step_delta','max_sample_travel_units',
'max_sample_interval_msec','white_rejects','left_rpm','right_rpm','last_widths',
'widths','first_white_units','last_edge_units','start_units','centred_count',
'hold_count','bend_age','touch_count','obstacle_event_seq','imu_seq','hump_count','last_barcode_seq','event_value',
'right_delta_mm','right_speed_mm_s','left','right','both'}

with INPUT.open(newline='') as f:
    prior=list(csv.DictReader(f))

output=[]
for r in prior:
    source,cls,field=r['Source'],r['Struct'],r['Field']
    previous=r['Implementation']; current=previous
    result='NOT_FINALLY_VERIFIED'; reason='Exact range/ownership or safe minimum representation not yet proven.'
    rng=r['Range']; bits=r['Required_bits']; ownership=r['Ownership']
    if r['Status']=='REVIEWED_MINBIT':
        result='V10_ALREADY_PACKED';reason='Prior V10 field-level review; preservation checks carried forward.'
    if (cls,field) in PACKED:
        rng,bits,current,ownership,reason=PACKED[(cls,field)]
        result='V11_IMPLEMENTED_TESTED'
    elif field in TIMESTAMPS and ('uint32_t' in previous or 'uint32_t' in current):
        rng='0..2^32-1 modulo clock';bits='32';reason='Time is measured from a wraparound 32-bit clock; no shorter guaranteed session bound.'
        result='FULL_WIDTH_RETAINED'
    elif field in COUNTS and ('uint32_t' in previous or 'int32_t' in previous):
        rng=('signed -2^31..2^31-1' if 'int32_t' in previous else '0..2^32-1');bits='32'
        reason='Cumulative or diagnostic counter may span a mission/repeated sessions; no documented tighter limit.'
        result='FULL_WIDTH_RETAINED'
    elif field in ('cfg','scan','motion','line','terrain','mission','obstacle') and 'uint' not in previous:
        rng='composite of separately inventoried fields';bits='sum of members'
        reason='Composite container; its individual members must be reviewed separately.'
        result='COMPOSITE_REFER_TO_MEMBERS'
    elif cls=='T_CSEM':
        rng='RTOS API-defined';bits='API-defined';reason='RTOS kernel-owned ABI, not an application-defined storage field.'
        result='EXCLUDED_KERNEL_ABI'
    elif cls=='bc_port_result_t' and field=='info':
        rng='letter5,valid1,complete1,reverse1,quality2,error4,reserved2';bits='14 used in 16'
        reason='Public protocol contract retains 16-bit envelope; reserved bits must remain zero.'
        result='WIRE_COMPATIBILITY_REVIEW'
    elif cls=='bc_port_config_t' and field in ('left_forward_sign','right_forward_sign','black_gpio_level'):
        rng='±1 encoded as 1 bit; black level 0..1';bits='1 per field'
        reason='Public validated configuration API retained; internal runtime copy uses compact single-bit direction/level flags.'
        result='V11_INTERNAL_COPY_PACKED'
        current=previous+' (public); runtime copy: 1-bit flag'
    elif cls=='bc_port_config_t' and field=='scan':
        rng='validated arbitrary 32-bit calibration thresholds';bits='range-dependent'
        reason='Single scanner-owned validated copy; duplicate full port copy removed.'
        result='V11_DUPLICATE_REMOVED'
        current='bc_scan_t.cfg (single copy)'
    elif cls in ('B1_Store','B1_Frame') and field in ('unlock','lock_context','coordinator','coordinator_context'):
        rng='target pointer width';bits='pointer width';reason='Function/context pointer; ABI representation fixed by target architecture.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='bc_code_pattern_t' and field=='element_pattern':
        pass
    elif field in ('seq','event_id','last_barcode_seq','obstacle_event_seq','imu_seq',
                   'sequence','last_cmd_transaction') and ('uint16_t' in previous):
        rng='0..65535 modulo';bits='16';reason='Monotonic 16-bit transaction/event sequence; preserve full wrap semantics.'
        result='FULL_WIDTH_RETAINED'
    elif field in ('right_delta_mm','right_speed_mm_s','event_value') and (
            'int16_t' in previous or 'uint16_t' in previous):
        rng='full native signed/unsigned 16-bit producer range';bits='16'
        reason='Preserve full producer range; snapshot wire saturation is applied separately.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='ir_shared_diag_t' and field in ('samples','left','right','both'):
        rng='0..65535 modulo';bits='16'
        reason='Accumulating diagnostic counter; current API permits 16-bit wrap.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='terrain_snapshot' and field=='hump_count':
        rng='0..255';bits='8'
        reason='8-bit run count retained; narrower bounds need verified course maximum.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='scan_plan_bypass_t' and field in ('word_a','word_b'):
        rng='32-bit explicitly shifted/masked packed plan';bits='layout-defined'
        reason='Application fields are individually masked in scan_plan.c; the uint32 word is a packing carrier.'
        result='EXPLICIT_PACKED_PLAN'
    elif cls=='bc_scan_config_t' and ('uint32_t' in previous):
        rng='validated nonnegative calibration input, nominally full uint32'
        bits='32 without a documented tighter configuration limit'
        reason='Public calibration accepts full-width values subject to relative constraints; shrinking changes accepted inputs.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='nav_config_t' and ('uint32_t' in previous):
        rng='caller-provided timeout/encoder threshold, full uint32'
        bits='32 without a documented tighter configuration limit'
        reason='External configuration type has no verified upper bound; retain to avoid changed tuning range.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='bc_port_config_t' and ('uint16_t' in previous):
        rng='0..65535, with nonzero validation where required';bits='16'
        reason='Existing full 16-bit public calibrated limit; only one validated compact internal copy retained.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='terrain_jerk_vector' and ('int32_t' in previous):
        rng='signed intermediate difference; conservative int32 full scale';bits='32'
        reason='Do not truncate raw differential intermediate until physical sensor limits and filtering are proven.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='bc_scan_t' and field=='widths':
        rng='per-run uncalibrated travel delta; potentially uint32';bits='32'
        reason='Scanning API accepts full uint32 travel units; no verified tighter width bound.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='B1_Frame' and field=='payload_len':
        rng='0..48';bits='6'
        reason='Payload-length field is one whole byte in v1 frame. Requires versioned wire migration for strict packing.'
        result='WIRE_COMPATIBILITY_REVIEW'
    elif cls=='B1_Command' and field=='mapping':
        rng='four distinct letters 0..25';bits='20'
        reason='Existing MQTT mapping already packed to 20 bits; command envelope remains byte-aligned version 1.'
        result='WIRE_COMPATIBILITY_REVIEW'
    elif cls=='B1_Command' and field=='transaction':
        rng='0..65535 modulo';bits='16'
        reason='Full 16-bit transaction space prevents early ACK/duplicate wrap.'
        result='FULL_WIDTH_RETAINED'
    elif cls=='terrain_calibration' and field=='b_valid':
        rng='0..1';bits='1';reason='Flag known boolean; retain existing struct ABI until isolated protected storage is verified.'
        result='PRECISION_REVIEW_PENDING'
    elif cls in ('terrain_calibration','motion_status_t','wheel_speed_t','steering_t','G7_TelemetryPeers') and ('float ' in previous):
        rng='continuous real-valued calculation';bits='precision-dependent';reason='32-bit arithmetic float retained to preserve calibrated control precision. Fixed-point proof required before shrink.'
        result='PRECISION_REVIEW_PENDING'
    output.append({
        'Source':source,'Struct':cls,'Field':field,'V10_declaration':previous,
        'V11_declaration_or_encoding':current,'Known_or_required_range':rng,
        'Minimum_bits':bits,'Owner_or_synchronization':ownership,
        'Verification':result,'Rationale_or_remaining_work':reason
    })

output.extend([
 {'Source':'app_program/modules/barcode_scanning/bc_port.c','Struct':'bc_port_runtime_config_t',
  'Field':name,'V10_declaration':'duplicate full bc_port_config_t',
  'V11_declaration_or_encoding':representation,'Known_or_required_range':rng,
  'Minimum_bits':bits,'Owner_or_synchronization':'barcode-port task and semaphore',
  'Verification':'V11_IMPLEMENTED_TESTED',
  'Rationale_or_remaining_work':'Scanner cfg owns the single calibrated 6-threshold set; barcode port only stores 3 additional limits.'}
 for name,representation,rng,bits in [
  ('left_reverse','unsigned char:1','0..1','1'),
  ('right_reverse','unsigned char:1','0..1','1'),
  ('black_gpio_level','unsigned char:1','0..1','1'),
  ('max_wheel_step_delta','uint16_t','0..65535','16'),
  ('max_sample_travel_units','uint16_t','1..65535','16'),
  ('max_sample_interval_msec','uint16_t','1..65535','16')
 ]])
OUTPUT.parent.mkdir(parents=True,exist_ok=True)
with OUTPUT.open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(output[0]));writer.writeheader();writer.writerows(output)
from collections import Counter
print('Entries:',len(output)); print(Counter(r['Verification'] for r in output))
print('Output:',OUTPUT)
