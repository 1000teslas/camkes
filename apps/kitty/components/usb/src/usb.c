/*
 * Copyright 2016, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

#include <stdbool.h>
#include <camkes.h>
#include <camkes/dma.h>
#include <usb/usb.h>
#include <assert.h>
#include <camkes/io.h>
#include <platsupport/clock.h>
#include <utils/util.h>
#include <utils/zf_log_if.h>

usb_t uhost;

bool once = false;
void irq_handle(void) {

    usb_handle_irq(&uhost);
    if (usb_get_device(&uhost, 2) != NULL) {
        if (!once) {
            printf("a 2nd device\n");
            serial_start();
            once = true;
        }
    }

    irq_acknowledge();
}

/* There is a race in tracking this, but it's the best we can do to reinitiaize
 * a mutex back to its initial state when we reuse them */
typedef enum {
    MUTEX_FREE,
    MUTEX_LOCKED,
    MUTEX_UNLOCKED,
} mutex_state_t;

typedef struct {
    mutex_state_t state;
    int (*lock)();
    int (*unlock)();
} camkes_mutex_t;

static camkes_mutex_t mutexes[] = {
    {MUTEX_FREE, m1_lock, m1_unlock},
    {MUTEX_FREE, m2_lock, m2_unlock},
};

static void *mutex_new(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(mutexes); i++) {
        if (mutexes[i].state == MUTEX_FREE) {
            mutexes[i].state = MUTEX_UNLOCKED;
            return &mutexes[i];
        }
    }
    ZF_LOGF("Failed to find free mutex to allocate");
}

static int mutex_lock(void *mutex)
{
    camkes_mutex_t *m = (camkes_mutex_t*)mutex;
    int err = m->lock();
    if (!err) {
        m->state = MUTEX_LOCKED;
    }
    return err;
}

static int mutex_unlock(void *mutex)
{
    camkes_mutex_t *m = (camkes_mutex_t*)mutex;
    /* we have to change the state *before* we unlock as once we unlock we no longer have
     * ownership of the state. This means we cannot tolerate failures in unlock */
    m->state = MUTEX_UNLOCKED;
    int err = m->unlock();
    if (err) {
        ZF_LOGF("Cannot tolerate failures in unlock");
    }
    return err;
}

static int mutex_destroy(void *mutex)
{
    camkes_mutex_t *m = (camkes_mutex_t*)mutex;
    ZF_LOGF_IF(m->state == MUTEX_FREE, "Tried to destroy unallocated mutex");
    if (m->state == MUTEX_LOCKED) {
        int err = m->unlock();
        ZF_LOGF_IF(err, "Failed to unlock mutex");
    }
    m->state = MUTEX_FREE;
    return 0;
}

int run() {
    ps_io_ops_t io_ops = {0};
    ps_mutex_ops_t mutex_ops = {0};

    mutex_ops.mutex_new = mutex_new;
    mutex_ops.mutex_lock = mutex_lock;
    mutex_ops.mutex_unlock = mutex_unlock;
    mutex_ops.mutex_destroy = mutex_destroy;

    int err;
    err = camkes_io_ops(&io_ops);
    assert(err == 0);
    err = camkes_dma_manager(&io_ops.dma_manager);
    assert(err == 0);
    err = clock_sys_init(&io_ops, &io_ops.clock_sys);
    assert(err == 0);
    err = usb_init(USB_HOST_DEFAULT, &io_ops, &mutex_ops, &uhost);
    assert(!err);

    return 0;

}
