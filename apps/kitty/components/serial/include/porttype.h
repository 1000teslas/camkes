/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/* UART data. */

#ifndef PORTTYPE_H
#define PORTTYPE_H

typedef struct UARTData {
	char data[0x4000];
} UARTData_t;

#endif 
