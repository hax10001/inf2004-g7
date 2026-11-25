/** @file scan_plan.h
 *
 * @brief Turns a servo range profile into a bypass plan; no I/O is done.
 */
#ifndef SCAN_PLAN_H
#define SCAN_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define SCAN_PLAN_CLEAR 0U
#define SCAN_PLAN_ROUTE 1U
#define SCAN_PLAN_NO_ROUTE 2U

/** @brief One scan sample; positive angles look left of FRONT. */
typedef struct
{
    /* Actual sweep is -70..70 degrees (8 signed bits), and ultrasonic
     * readings are 0..3999 mm or 4000 mm for no echo (12 bits). The
     * scanner task builds this local record; no ISR or task shares it.
     * Values outside these ranges must be normalized before assignment. */
    int32_t    angle_deg:8;
    uint32_t     range_mm:12;
    /** Corner band only: the nearest echo heard on a single ping, which no
     *  second ping confirmed (range_mm leaves it out), 0 if none. 0..3999,
     *  12 bits. The planner trusts it only when the next corner bearing
     *  heard the same range. Zero it when building points by hand. */
    uint32_t     lone_mm:12;
} scan_plan_point_t;

/** @brief A bypass plan packed into two words, with distances in cm. */
typedef struct
{
    uint32_t word_a;
    uint32_t word_b;
} scan_plan_bypass_t;

/* Why a side's sideways need is what it is (scan_plan_side_info_t). */
#define SCAN_PLAN_NEED_MEASURED 0U /**< Edge offset plus clearance. */
#define SCAN_PLAN_NEED_BODY 1U     /**< Raised to half body plus margin. */
#define SCAN_PLAN_NEED_UNSEEN 2U   /**< Edge beyond sweep: UNKNOWN floor. */
#define SCAN_PLAN_NEED_MIN 3U      /**< Far-off edge: MIN_LATERAL floor. */

/** @brief One side of the scan, for the scanner task's debug log. */
typedef struct
{
    /* Bearings are -70..70 degrees (8 signed bits). The edge offset is
     * clamped to -2048..2047 mm (12 signed bits); ranges are clamped to
     * 0..4095 mm (12 bits). Only the scanner task uses this local record. */
    int32_t      edge_deg:8;    /**< Outermost object sample on this side. */
    int32_t      edge_mm:12;    /**< Edge offset outward from the line. */
    uint32_t     b_seen:1;      /**< The edge is inside the sweep. */
    uint32_t     reason:2;      /**< SCAN_PLAN_NEED_*. */
    uint32_t     b_blocked:1;   /**< Another object sits in the corridor. */
    int32_t      blocker_deg:8; /**< Nearest blocking sample, if blocked. */
    uint32_t     blocker_mm:12;
    uint32_t     open_mm:12;    /**< Nearest echo beyond the edge. */
    uint32_t     angle_index:2; /**< Turn-out angle index for this side. */
    uint32_t     compare_mm:12; /**< Need before the MIN_LATERAL floor. */
    uint32_t     b_corner:1;    /**< A corner echo was found this side. */
    uint32_t     corner_mm:12;  /**< Its sideways offset from the line. */
} scan_plan_side_info_t;

/** @brief How the bypass side was chosen, for the scanner task's log. */
typedef struct
{
    /* Sideways move each side needs, 0..4095 mm (12 bits); larger values
     * are clamped to 4095 before assignment. Flags are single bits. Only
     * the scanner task writes and reads this local record. */
    uint32_t     left_need_mm:12;
    uint32_t     right_need_mm:12;
    uint32_t     b_left_ok:1;  /**< Left corridor is clear. */
    uint32_t     b_right_ok:1; /**< Right corridor is clear. */
    uint32_t     b_tie:1;      /**< About centred: room past it decided. */
    uint32_t     b_object:1;   /**< An object was found ahead. */
    /* Object bearings -70..70 degrees; depth and width 0..4095 mm. */
    int32_t      low_deg:8;    /**< Rightmost object sample. */
    int32_t      high_deg:8;   /**< Leftmost object sample. */
    uint32_t     near_mm:12;   /**< Nearest face, forward of the sensor. */
    uint32_t     width_mm:12;  /**< Edge to edge across the line. */
    uint32_t     b_centre:1;   /**< centre_mm is from corner echoes. */
    int32_t      centre_mm:12; /**< Object centre off the line, + = left. */
    int32_t      tilt_x2:7;    /**< Face normal, half degrees, + = left. */
    uint32_t     front_width_mm:12; /**< Projected width, 0 if unresolved. */
    uint32_t     front_left_mm:12;  /**< Width on the left of sonar zero. */
    uint32_t     b_front_estimate:1; /**< At least one edge is inferred. */
    scan_plan_side_info_t left;
    scan_plan_side_info_t right;
} scan_plan_sides_t;

/** @brief Bypass heading change for an angle index.
 *  @param angle_index Index from scan_plan_angle_index().
 *  @return The angle in degrees.
 */
int32_t scan_plan_angle_deg (uint32_t angle_index);

/** @brief The bearing scanned after this one, going right to left: the
 *  fine step inside +-OBS_SCAN_FINE_DEG, the corner step out to
 *  +-OBS_CORNER_MAX_DEG, the normal step outside that.
 *  @param angle_deg A bearing of the scan sequence.
 */
int32_t scan_plan_next_angle (int32_t angle_deg);

/** @brief Find the object ahead, its edges, the clearer side and the path.
 *  @param[in] p_points Samples ordered from right to left.
 *  @param count Number of samples.
 *  @param[out] p_plan Receives the packed plan.
 */
void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan);

/** @brief Preserve confirmed front obstacle if a later scan misses it.
 *  @param[in,out] p_points Scan bearings ordered right to left.
 *  @param count Sample count.
 *  @param confirmed_front_mm The previous confirmed forward range.
 *  @param[out] p_plan Resulting packed bypass plan.
 */
void scan_plan_build_confirmed (scan_plan_point_t * p_points, int32_t count,
                                int32_t confirmed_front_mm,
                                scan_plan_bypass_t * p_plan);

/** @brief As scan_plan_build_confirmed(), but mid-bypass the side is fixed.
 *  @param required_side 1 keeps left, -1 keeps right; a blocked fixed side
 *  returns NO_ROUTE. 0 chooses between the measured clear corridors.
 */
void scan_plan_build_side (scan_plan_point_t * p_points, int32_t count,
                           int32_t confirmed_front_mm, int32_t required_side,
                           scan_plan_bypass_t * p_plan);

/** @brief As scan_plan_build_side(), also reporting both sides' needs.
 *  @param[out] p_sides Receives the side comparison; may be NULL.
 */
void scan_plan_build_side_report (scan_plan_point_t * p_points, int32_t count,
                                  int32_t confirmed_front_mm,
                                  int32_t required_side,
                                  scan_plan_bypass_t * p_plan,
                                  scan_plan_sides_t * p_sides);

/** No heading-off-the-line measurement: split about sonar zero. */
#define SCAN_PLAN_LINE_OFFSET_UNKNOWN INT16_MIN
/** @brief Controller task: the robot's heading off the line when it stopped
 *  for this obstacle, tenths of a degree, + = turned left, or
 *  SCAN_PLAN_LINE_OFFSET_UNKNOWN. The side comparison and the reported
 *  left/right split are measured about the line. Valid -250..250 (10 bits
 *  signed) in one aligned 16-bit object: a single store/load, written
 *  before the scan starts and read by the scanner task, so no lock. */
void scan_plan_set_line_offset_ddeg (int16_t offset_ddeg);
/** @brief SCAN_PLAN_CLEAR, SCAN_PLAN_ROUTE or SCAN_PLAN_NO_ROUTE. */
uint32_t scan_plan_result (scan_plan_bypass_t const * p_plan);
/** @brief True when the bypass goes round the left side. */
bool scan_plan_side_is_left (scan_plan_bypass_t const * p_plan);
/** @brief Index of the chosen bypass angle. */
uint32_t scan_plan_angle_index (scan_plan_bypass_t const * p_plan);
/** @brief Approximate object width in cm. */
int32_t scan_plan_width_cm (scan_plan_bypass_t const * p_plan);
/** @brief Distance to the nearest object face in cm. */
int32_t scan_plan_near_cm (scan_plan_bypass_t const * p_plan);
/** @brief Lateral path offset in cm. */
int32_t scan_plan_lateral_cm (scan_plan_bypass_t const * p_plan);
/** @brief Length of the diagonal run in cm. */
int32_t scan_plan_diag_cm (scan_plan_bypass_t const * p_plan);
/** @brief Distance to pass the object by dead reckoning, in cm. */
int32_t scan_plan_fallback_cm (scan_plan_bypass_t const * p_plan);
/** @brief The side not chosen has a clear corridor too (free choice). */
bool scan_plan_other_side_ok (scan_plan_bypass_t const * p_plan);
/** @brief Drive-out the side not chosen would need, cm. */
int32_t scan_plan_other_lateral_cm (scan_plan_bypass_t const * p_plan);

#endif /* SCAN_PLAN_H */

/*** end of file ***/

