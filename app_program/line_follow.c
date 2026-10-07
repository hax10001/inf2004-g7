#include "line_follow.h"

LINE_ACTION line_follow_decide(UINT left, UINT right)
{
    /* Both sensors detect white */
    if (left == 0U && right == 0U) {
        return LINE_FORWARD;
    }

    /* Left sensor detects black */
    if (left == 1U && right == 0U) {
        return LINE_CORRECT_LEFT;
    }

    /* Right sensor detects black */
    if (left == 0U && right == 1U) {
        return LINE_CORRECT_RIGHT;
    }

    /* Both sensors detect black */
    return LINE_UNCERTAIN;
}
// This is to return a printf string for debugging purposes. It is not used in the main code.
const char *line_follow_text(LINE_ACTION action)
{
    switch (action) {

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