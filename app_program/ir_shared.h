#ifndef IR_SHARED_H
#define IR_SHARED_H

#include <tk/tkernel.h>

typedef struct {
    UINT left;
    UINT right;
    UINT barcode;

    UW timestamp_ms;
    BOOL valid;
} IR_SNAPSHOT;

/* Call once before starting the tasks. */
ER ir_shared_init(void);

/* IR task publishes its latest readings. */
ER ir_shared_publish(UINT left, UINT right, UINT barcode);

/* Controller retrieves the latest readings. */
ER ir_get_latest(IR_SNAPSHOT *out);

#endif // IR_SHARED_H