/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/**
 * Frequently used helper functions.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define DEVICE_START        (0xB0000000)

#define DBG_ERR   0
#define DBG_CRIT  1
#define DBG_WARN  2
#define DBG_INFO  3
#define DBG_LOG   4

#define DEBUGLEVEL DBG_LOG

#define D(level, args...) \
	do { \
		if((level) <= DEBUGLEVEL) { \
			printf("%s():%d: ", __func__, __LINE__); \
			printf(args); \
			printf("\n"); \
		} \
	} while (0);

/**
 * Collection of debugging functions
 */
static void print_hex(int level, int argc, ...) __attribute__((unused));
static void print_hex(int level, int argc, ...)
{
	va_list args;
	char *str;

	if (level > DEBUGLEVEL) {
		return;
	}
	
	va_start(args, argc);
	while (argc--) {
		str = va_arg(args, char*);
		for (int i = 0; str[i] != '\0'; i++) {
			printf("%X ", str[i]);
		}
	}
	
	printf("\n");
	va_end(args);
}

#define print_buffer(level, buffer, size) \
	do { \
		if ((level) <= DEBUGLEVEL) { \
			print_data(buffer, size, __func__, __LINE__); \
		} \
	} while (0);

static void print_data(const void *buffer, size_t size, ...) __attribute__((unused));
static void print_data(const void *buffer, size_t size, ...)
{
	char *buf = (char*)buffer;
	va_list ap;

	va_start(ap, size);
	printf("\e[1;31m------ Buffer Start(%s: %d)----\n",
		va_arg(ap, char*), va_arg(ap, int));
	va_end(ap);

	for (int i = 0; i < size; i++) {
		printf("%02x ", buf[i]);
		if (i != 0 && i % 16 ==0) {
			printf("\n");
		}
	}
	printf("\n");

	printf("------ Buffer End ----\n\e[0m");
}

/**
 * Read from 32-bit register.
 */
inline uint32_t readl(const void *address)
{
	//printf("readl at 0x%08x\n", address);
	return *((volatile uint32_t *)(address));
}

/**
 * Write to 32-bit register.
 */
inline void writel(uint32_t value, const void *address)
{
	//printf("writel at 0x%08x\n", address);
	*((volatile uint32_t *)(address)) = value;
}

inline void spinlock_lock(int *lock)
{
	while (!__sync_bool_compare_and_swap(lock, 0, 1));
}

inline void spinlock_unlock(int *lock)
{
	while (!__sync_bool_compare_and_swap(lock, 1, 0));
}

inline void barrier(void)
{
	__sync_synchronize();
}


// from sos
/*

inline uint32_t __raw_readl(uint32_t *addr)
{
	D(DBG_WARN, "Reading out of HDMI and IPU Bounds\n");
	return readl(addr);
}

inline uint8_t __raw_readb(uint32_t *addr)
{
	D(DBG_WARN, "Reading out of HDMI and IPU Bounds\n");
   uint32_t val = readl(addr);
   val &= 0xFF;
	return (uint8_t)val;
}
inline void __raw_writel(uint32_t value, uint32_t *addr)
{
	D(DBG_WARN, "Writing into of HDMI and IPU Bounds\n");
	writel(value,addr);
}
*/


