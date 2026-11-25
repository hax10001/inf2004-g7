/** @file console.c
 * @brief Console state from the USB CDC monitor, with UART fallbacks.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "console.h"

#if TM_CONSOLE_USB_CDC
/* These names belong to the supplied monitor ABI, not application code. */
extern UW  tm_usb_state (void);
#endif

UW console_state (void)
{
#if TM_CONSOLE_USB_CDC
    return tm_usb_state();
#else
    return CONSOLE_STATE_OPEN;
#endif
}

/*** end of file ***/

