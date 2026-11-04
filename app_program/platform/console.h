/** @file console.h
 * @brief Console-state access through the existing monitor implementation.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <tk/tkernel.h>

/** @brief Return USB enumeration state, or the original UART fallback. */
UW console_state (void);
/** @brief Return USB diagnostic progress, or the UART ready level. */
INT console_diag_level (void);

#endif /* CONSOLE_H */

/*** end of file ***/

