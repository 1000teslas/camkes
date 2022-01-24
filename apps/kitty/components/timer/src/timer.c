/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

/*
 * Timer interface implementation.
 */

#include <stdio.h>

#include <epit.h>
#include <camkes.h>

/*
 * Get current time in ms.
 */
int tm_get_time()
{
	return cur_time * 100;
}

