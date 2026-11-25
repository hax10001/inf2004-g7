/** @file console.h
 * @brief Console-state access through the existing monitor implementation.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <tk/tkernel.h>

/* Console states: USB enumerated by the host, terminal open (DTR). */
#define CONSOLE_STATE_ENUMERATED 2U
#define CONSOLE_STATE_OPEN       3U

/** @brief Return USB enumeration state, or OPEN for a UART console. */
UW console_state (void);

#endif /* CONSOLE_H */

/*** end of file ***/

