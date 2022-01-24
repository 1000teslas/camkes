/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

#include <stdio.h>

#include <platsupport/mach/epit.h>

#include <epit.h>
#include <camkes.h>

/* timeout every 100 milliseconds */
#define PRECISION (NS_IN_MS * 100)
static epit_t epit;

void irq_handle(void)
{
	/* Hardware routine. */
	epit_handle_irq(timer_drv);

	cur_time++;

	/* Signal other components. */
	timer_update_emit();

    irq_acknowledge();
}

void pre_init()
{
	epit_config_t config;

	/*
	 * Provide hardware info to platsupport.
	 * TODO: IRQ is hard-coded to EPIT1, replace is with ADL variable.
	 */
	config.vaddr = (void*)mmio_base;
	config.irq = EPIT1_INTERRUPT;
	config.prescaler = 0;

    int error = epit_init(&epit, config);
    if (error) {
		printf("EPIT device does not exist.\n");
    }

	timer_drv = &epit;

	cur_time = 0;

	/* Run in periodic mode and start the timer. */
    epit_set_timeout(timer_drv, PRECISION, true);
}

