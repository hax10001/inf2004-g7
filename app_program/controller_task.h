#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include <tk/tkernel.h>
#include "ir_shared.h"
#include "line_follow.h"
#include "motor_controller.h"

EXPORT void controller_task(INT stacd, void *exinf);

#endif // CONTROLLER_TASK_H