/** @file line_follow.h
 * @brief Pure line-following decisions from raw sensor levels.
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

typedef enum
{
    LINE_FORWARD,
    LINE_CORRECT_LEFT,
    LINE_CORRECT_RIGHT,
    LINE_UNCERTAIN
} line_follow_action_t;

/** @brief Map sensor levels to the tested movement action. */
line_follow_action_t line_follow_decide (uint32_t left, uint32_t right);

/** @brief Return the existing diagnostic label for an action. */
char const * line_follow_text (line_follow_action_t action);

#endif /* LINE_FOLLOW_H */

/*** end of file ***/

