/** @file line_follow.c
 * @brief Preserve the tested IR truth table and diagnostic labels.
 */
#include "line_follow.h"

line_follow_action_t line_follow_decide (uint32_t left, uint32_t right)
{
    /* Both sensors detect white. */
    if ((0U == left) && (0U == right))
    {
        return LINE_FORWARD;
    }

    /* Left sensor detects black. */
    if ((1U == left) && (0U == right))
    {
        return LINE_CORRECT_LEFT;
    }

    /* Right sensor detects black. */
    if ((0U == left) && (1U == right))
    {
        return LINE_CORRECT_RIGHT;
    }

    /* Both sensors detect black. */
    return LINE_UNCERTAIN;
}

char const * line_follow_text (line_follow_action_t action)
{
    switch (action)
    {

        case LINE_FORWARD:
            return "FORWARD";
        case LINE_CORRECT_LEFT:
            return "CORRECT LEFT";
        case LINE_CORRECT_RIGHT:
            return "CORRECT RIGHT";
        case LINE_UNCERTAIN:
            return "BOTH BLACK - CHECK JUNCTION";
        default:
            return "UNKNOWN ACTION";
    }
}

/*** end of file ***/

