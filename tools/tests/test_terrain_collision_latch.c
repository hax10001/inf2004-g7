/* Verify the real mutex-owned IMU event mailbox, including rearming.
 * No real I2C, motion commands, PWM or unsafe cross-core bit writes.
 */
#include <assert.h>
#include <stdio.h>
#include "modules/terrain/terrain.h"

static unsigned int locks=0U;
ID tk_cre_mtx(T_CMTX const *config)
{
    assert(config != NULL);
    return (ID)42;
}
ER tk_del_mtx(ID id) {(void)id;return E_OK;}
ER tk_loc_mtx(ID id, TMO timeout)
{
    (void)timeout;
    assert(id==42);
    ++locks;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    assert(id==42);
    assert(locks>0U);
    --locks;
    return E_OK;
}

int main(void)
{
    terrain_snapshot s={0};
    terrain_snapshot out={0};
    assert(terrain_init()==E_OK);
    s.b_valid=1U;
    s.b_collision=1U;
    assert(terrain_publish_snapshot(&s)==E_OK);
    s.b_collision=0U;
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==1U); /* Event remains pending until STOP. */
    assert(terrain_acknowledge_collision()==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==0U);
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==0U); /* Old event never repeats. */
    s.b_collision=1U;
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==1U); /* New independent collision still works. */
    uint32_t observed=out.collision_sequence;
    assert(terrain_publish_snapshot(&s)==E_OK); /* New event before ACK. */
    assert(terrain_ack_collision(observed)==E_OBJ);
    assert(terrain_get_snapshot(&out)==E_OK && out.b_collision);
    assert(out.collision_sequence!=observed);
    assert(terrain_ack_collision(out.collision_sequence)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(locks==0U);
    puts("PASS terrain collision: event latch, STOP acknowledgement, fresh rearm");
    return 0;
}
