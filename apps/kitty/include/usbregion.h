/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef _USB_REGION_H_
#define _USB_REGION_H_

#include <stdint.h>
#include "usbsizes.h"

typedef struct {
    char buf[USBLENGTH];
} UsbRegion;

typedef struct {
    char buf[USBLENGTH2];
} UsbRegion2;

typedef struct {
    char buf[CCMLENGTH];
} CCMRegion;

typedef struct {
    char buf[ANALOGLENGTH];
} AnalogRegion;

#endif /* _USB_REGION_H_ */
