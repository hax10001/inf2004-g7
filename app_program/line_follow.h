#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <tk/tkernel.h>

typedef enum {
    LINE_FORWARD,
    LINE_CORRECT_LEFT,
    LINE_CORRECT_RIGHT,
    LINE_UNCERTAIN
} LINE_ACTION;

LINE_ACTION line_follow_decide(UINT left, UINT right);

const char *line_follow_text(LINE_ACTION action);

#endif // LINE_FOLLOW_H