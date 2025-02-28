/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

#ifndef _NETWORK_EPIT_H_
#define _NETWORK_EPIT_H_

#include <platsupport/mach/epit.h>

epit_t *timer_drv;
volatile int cur_time;

#endif /* _NETWORK_EPIT_H_ */
