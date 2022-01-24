/* @LICENCE("NICTA", "2013")@ */
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

#include <displaysizes.h>

typedef struct HDMIData {
	char data[HDMILENGTH];
} HDMIData_t;

typedef struct IPUData {
	char data[IPULENGTH];
} IPUData_t;

typedef struct BUFData {
	char data[BUFLENGTH];
} BUFData_t;

#endif 
