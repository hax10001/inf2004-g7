/** @file console.c
 * @brief Retain UART console fallbacks without introducing SDK calls.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "console.h"

#if TM_CONSOLE_USB_CDC
/* These names belong to the supplied monitor ABI, not application code. */
extern UW  tm_usb_state (void);
extern INT tm_usb_diag_level (void);
#endif

UW console_state (void)
{
#if TM_CONSOLE_USB_CDC
    return tm_usb_state();
#else
    return 3U;
#endif
}

INT console_diag_level (void)
{
#if TM_CONSOLE_USB_CDC
    return tm_usb_diag_level();
#else
    return 8;
#endif
}

/*** end of file ***/

