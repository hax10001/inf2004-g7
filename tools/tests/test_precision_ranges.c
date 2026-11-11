/** Lossless representation boundaries reviewed against firmware producers. */
#include <stdio.h>
#include "modules/motion/steering.h"
#include "modules/barcode_scanning/bc_match.h"
#include "modules/barcode_scanning/bc_layout.h"
#include "modules/terrain/terrain.h"

int main(void)
{
    steering_t steer = {0};
    bc_match_result_t match = {0};
    bc_layout_result_t layout = {0};
    terrain_calibration calibration = {0};
    steer.last_side = -1;
    steer.bend_side = 1;
    match.support = 6;
    layout.bars = 96;
    layout.module_x10 = 1000;
    calibration.b_valid = 1;
    if ((steer.last_side != -1) || (steer.bend_side != 1) ||
        (match.support != 6U) || (layout.bars != 96U) ||
        (layout.module_x10 != 1000U) || !calibration.b_valid)
    {
        puts("FAIL: precision boundaries");
        return 1;
    }
    steer.last_side = 0;
    steer.bend_side = -1;
    match.support = 0;
    layout.bars = 0;
    layout.module_x10 = 300;
    calibration.b_valid = 0;
    if ((steer.last_side != 0) || (steer.bend_side != -1) ||
        (match.support != 0U) || (layout.bars != 0U) ||
        (layout.module_x10 != 300U) || calibration.b_valid)
    {
        puts("FAIL: precision lower boundaries");
        return 2;
    }
    printf("PASS: precision field bounds, steering bytes=%zu, "
           "matcher bytes=%zu, layout bytes=%zu\n",
           sizeof(steering_t), sizeof(bc_match_result_t),
           sizeof(bc_layout_result_t));
    return 0;
}
