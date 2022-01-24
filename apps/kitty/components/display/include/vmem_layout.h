////////////////////////////////////////////////////////////////////////////////

#ifndef SOS_MEM_LAYOUT_H
#define SOS_MEM_LAYOUT_H

#include "debug.h"

////////////////////////////////////////////////////////////////////////////////

// WARNING: don't use memory in DMA mapping region
#define DMA_SIZE_BITS       (24) // 16MB
#define DMA_VSTART          (0x10000000)
#define DMA_VEND            (DMA_VSTART + (1ull << DMA_SIZE_BITS))

// sos maps frames into a physical memory window after dma mapping
#define SOS_STACK_PAGES     (16 * 256) // 8MB
#define SOS_STACK_SIZE      (SOS_STACK_PAGES * 4096)
#define SOS_STACK_TOP       (0x30000000) 
#define SOS_STACK_BASE      (SOS_STACK_TOP - SOS_STACK_SIZE)
#define FRAMETABLE_START    (0x3f000000)
#define PHYS_WINDOW_START   (0x40000000)

// ensure things aren't overlapping
//STATIC_ASSERT(DMA_VEND <= SOS_STACK_BASE, "DMA and STACK overlap!");

// devices mapped from here by map_device (modify if using this region)
#define DEVICE_START        (0xB0000000)
#define ROOT_VSTART         (0xC0000000)

// address space layout for a sos process
#define PROCESS_STACK_TOP   (0x90000000)
#define PROCESS_IPC_BUFFER  (0xA0000000)
#define PROCESS_VMEM_START  (0xC0000000)
#define PROCESS_SCRATCH     (0xD0000000)

////////////////////////////////////////////////////////////////////////////////

#endif // SOS_MEM_LAYOUT_H

////////////////////////////////////////////////////////////////////////////////
