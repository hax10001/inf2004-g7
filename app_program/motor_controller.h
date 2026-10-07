#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "line_follow.h"

void motor_controller_init(void);
void motor_controller_stop(void);
void motor_controller_apply(LINE_ACTION action);

#endif // MOTOR_CONTROLLER_H