/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef _RFID_STRING_H_
#define _RFID_STRING_H_

#include <stdint.h>

#define MAX_RFID_LENGTH 4

typedef struct {
     uint32_t size;
     char buf[MAX_RFID_LENGTH];

} rfid_string_t;


#endif /* _RFID_STRING_H_ */
