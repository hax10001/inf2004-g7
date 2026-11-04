/** @file buddy1_integration.h
 * @brief Buddy 1 integration points for the motion controller.
 */
#ifndef BUDDY1_INTEGRATION_H
#define BUDDY1_INTEGRATION_H
#include "buddy1.h"
/* Called only from normal application tasks, not ISRs.
 * The source structs are B1-facing adapter values,
     not other buddies' internal ABI. */
B1_Store *b1_runtime_store(void);
#endif
/*** end of file ***/

