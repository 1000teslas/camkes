////////////////////////////////////////////////////////////////////////////////
//
// HDMI display driver for Sabre Lite
//
// NOTE: requires U-Boot to set up clocks, this means the display mode is
// fixed to U-Boot default (1024x768, RGB565).
//
////////////////////////////////////////////////////////////////////////////////



#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//#include "disp.h"
//#include "dma.h"
#include "ipu_common.h"
#include <util.h>
#include "mxc_hdmi.h"
//#include "mapping.h"
#include <string.h>
#include <bits/limits.h>
#include <bits/errno.h>

#include <camkes/dma.h>

#include <camkes.h>

#include "font.h"
#include "image.h"
#include "bg.h"
#include "kitty_swipe.h"
#include "kitty1.h"
#include "kitty2.h"
#include "kitty3.h"
#include "kitty4.h"
#include "msg1.h"
#include "porttype.h"
//#include <fb.h>
//#include "dma.c"

////////////////////////////////////////////////////////////////////////////////

#define IDMAC_BUFALIGN	(PAGE_SIZE) // Alignment of IDMAC buffers in memory
#define IDMAC_RESIP		(MEM_TO_IC_PP_RES_CH11) // Memory -> resizer
#define IDMAC_RESOP		(IC_PP_RES_TO_MEM_CH22) // Resizer -> memory
#define IDMAC_MEMTOBG	(MEM_TO_DP_BG_CH23)		// Memory -> DP BG
#define IPU_CHANNEL		(0)
#define CPMEM_LENGTH	(16) // bytes
#define SRC_LEN			(0x4000)
#define DISP_BUFSIZE	(DISP_WIDTH * DISP_HEIGHT * DISP_DEPTH / 8)
#define MEM_BEGIN		(0x10000000)
#define MEM_END			(0x50000000)
#define DMA_BUFFERS		(2)
#define DMA_DISP		(0)
#define DMA_RES			(1)

////////////////////////////////////////////////////////////////////////////////

#define DISP_WIDTH	(1024)
#define DISP_HEIGHT	(768)
#define DISP_DEPTH	(16)

#define DISP_PIXFMT_RGB565 (0x1)
#define DISP_PIXFMT_RGB24  (0x2)

#define FB_WIDTH     (1024)
#define FB_HEIGHT    (768)
#define FB_SIZE      (FB_WIDTH * FB_HEIGHT)
#define FB_PIXELS    (FB_SIZE)
#define FB_BYTES     (FB_SIZE * sizeof(uint16_t))
#define FB_ROW_BYTES (FB_WIDTH * sizeof(uint16_t))


//////////////////////////////////////////////////////////////////////////////
#define HEADER_PIXEL(data,pixel) {\
	pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
	pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
	pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
	data += 4; \
}

#define TEXT_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33)));\
}

uint16_t *disp_bufs[DMA_BUFFERS];
uintptr_t dma_phys[DMA_BUFFERS];

uintptr_t phys = 0;
void *virt = NULL;
size_t buf_size = 0;

typedef struct image_rep {
   int x1;
   int y1;
   int width;
   int height;
   int background;
   char* data;
   char* cur;
} image;

static Image background;
static Image kitty1;
static Image kitty2;
static Image kitty3;
static Image kitty4;
static Image swipe;
static Image msg1;

#define HDMI_BUF_ALIGN 0x100000

static IPUData_t *hdmi_buf_0;
static BUFData_t *hdmi_buf_1;
static IPUData_t *hdmi_buf_2;
static BUFData_t *hdmi_buf_3;

///////////////////////////////////////////////////////////////////////////////

inline uint32_t __raw_readl(uint32_t *addr)
{


	if ((uintptr_t)addr >= HDMI_ARB_BASE_ADDR && (uintptr_t)addr <= HDMI_ARB_END_ADDR) {
		return readl((void*)((uint32_t)addr - HDMI_ARB_BASE_ADDR + (uint32_t)hdmi_base));
	} else if ((uintptr_t)addr >= IPU1_ARB_BASE_ADDR && (uintptr_t)addr <= IPU1_ARB_END_ADDR) {
		switch(((uint32_t)addr-IPU1_ARB_BASE_ADDR)/0x100000) {
			case 0: return readl((unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_0)); break;
			case 1: return readl((unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_1 - 0x100000)); break;
			case 2: return readl((unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_2 - 0x200000)); break;
			case 3: return readl((unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_3 - 0x300000)); break;
			default: D(DBG_WARN, "Reading out of HDMI and IPU Bounds\n");
						return readl(addr);
		}


	} else {
		D(DBG_WARN, "Reading out of HDMI and IPU Bounds\n");
		return readl(addr);
	}
}

inline uint8_t __raw_readb(uint32_t *addr)
{
   uint32_t val = __raw_readl(addr);
   val &= 0xFF;
	return (uint8_t)val;
}
inline void __raw_writel(uint32_t value, uint32_t *addr)
{
	if ((uintptr_t)addr >= HDMI_ARB_BASE_ADDR && (uintptr_t)addr <= HDMI_ARB_END_ADDR) {
		writel(value,addr - HDMI_ARB_BASE_ADDR + (uint32_t)hdmi_base);
	} else if ((uintptr_t)addr >= IPU1_ARB_BASE_ADDR && (uintptr_t)addr <= IPU1_ARB_END_ADDR) {

		switch(((uint32_t)addr-IPU1_ARB_BASE_ADDR)/0x100000) {
			case 0: writel(value,(unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_0)); break;
			case 1: writel(value,(unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_1 - 0x100000)); break;
			case 2: writel(value,(unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_2 - 0x200000)); break;
			case 3: writel(value,(unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_3 - 0x300000)); break;
			default: D(DBG_WARN, "writing into of HDMI and IPU Bounds\n");
						writel(value,addr);
		}


	} else {
		D(DBG_WARN, "writing into of HDMI and IPU Bounds\n");
		writel(value,addr);
	}
}

uint32_t *get_virtualAddr(uint32_t *addr) {
	if ((uintptr_t)addr >= HDMI_ARB_BASE_ADDR && (uintptr_t)addr <= HDMI_ARB_END_ADDR) {
		return (uint32_t *)((uint32_t)addr - HDMI_ARB_BASE_ADDR + (uint32_t)hdmi_base);
	} else if ((uintptr_t)addr >= IPU1_ARB_BASE_ADDR && (uintptr_t)addr <= IPU1_ARB_END_ADDR) {
		switch(((uint32_t)addr-IPU1_ARB_BASE_ADDR)/0x100000) {
			case 0: return ((uint32_t *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_0)); break;
			case 1: return ((uint32_t *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_1 - 0x100000)); break;
			case 2: return ((uint32_t *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_2 - 0x200000)); break;
			case 3: return ((uint32_t *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_3 - 0x300000)); break;
			default: D(DBG_WARN, "Converting out of HDMI and IPU Bounds\n");
						return addr;
		}


	} else {
		D(DBG_WARN, "Converting out of HDMI and IPU Bounds\n");
		return addr;
	}
}


////////////////////////////////////////////////////////////////////////////////

void *dma_malloc(void *cookie, size_t size, int align, int cached, ps_mem_flags_t flags)
{
	//return camkes_dma_alloc_page();
    return NULL;
}

uintptr_t dma_pin(void *cookie, void *addr, size_t size)
{
	return (uintptr_t)camkes_dma_get_paddr(addr);
}

void dma_cache_op(void *cookie, void *addr, size_t size, dma_cache_op_t op)
{
	return;
}

///////////////////////////////////////////////////////////////////////////////


//
// init hdmi display
//

static int hdmi_init(void) {
    // handled by U-Boot on startup
	return 0;
}

////////////////////////////////////////////////////////////////////////////////

// dma functions



//
//
//

static void ipu_cpmem_set_packing(uint32_t param,
    int w1, int w2, int w3, int w4,
    int o1, int o2, int o3, int o4)
{
	ipu_cpmem_set_field(param, INTERLEAVED_WID0, w1);
	ipu_cpmem_set_field(param, INTERLEAVED_WID1, w2);
	ipu_cpmem_set_field(param, INTERLEAVED_WID2, w3);
	ipu_cpmem_set_field(param, INTERLEAVED_WID3, w4);
	ipu_cpmem_set_field(param, INTERLEAVED_OFF0, o1);
	ipu_cpmem_set_field(param, INTERLEAVED_OFF1, o2);
	ipu_cpmem_set_field(param, INTERLEAVED_OFF2, o3);
	ipu_cpmem_set_field(param, INTERLEAVED_OFF3, o4);
}

////////////////////////////////////////////////////////////////////////////////

//
//
//

static void ipu_cpmem_set(int idmac_ch,
                          int pixfmt,
	 	                  int width,
                          int height,
		                  uintptr_t addr0,
                          uintptr_t addr1)
{
	ipu_cpmem_t params;
	memset(&params, 0, sizeof(params));
	int stride = 2;
////printf("%s: %d\n", __func__,__LINE__);
	uintptr_t cpmem_base = sizeof(params) * idmac_ch +
		/*DEVICE_START +*/ REGS_IPU_BASE(IPU_CHANNEL) + IPU_MEMORY_OFFSET;

	uint32_t param_base = (uint32_t)&params;

	ipu_cpmem_set_field(param_base, CPMEM_FW, width - 1);
	ipu_cpmem_set_field(param_base, CPMEM_FH, height - 1);

	ipu_cpmem_set_field(param_base, CPMEM_EBA0, addr0 / 8);
	ipu_cpmem_set_field(param_base, CPMEM_EBA1, addr1 / 8);
////printf("%s: %d\n", __func__,__LINE__);
	switch (pixfmt) {
		case DISP_PIXFMT_RGB24:
			stride = 3;
			ipu_cpmem_set_field(param_base, INTERLEAVED_BPP, BPP24);
			ipu_cpmem_set_field(param_base, CPMEM_PFS, PFS_RGB);
			ipu_cpmem_set_field(param_base, CPMEM_NPB, 19);
			ipu_cpmem_set_packing(param_base, 8, 8, 8, 8, 16, 8, 0, 24);
			break;
		case DISP_PIXFMT_RGB565:
		default:
			stride = 2;
			ipu_cpmem_set_field(param_base, INTERLEAVED_BPP, BPP16);
			ipu_cpmem_set_field(param_base, CPMEM_PFS, PFS_RGB);
			ipu_cpmem_set_field(param_base, CPMEM_NPB, 15);
			ipu_cpmem_set_packing(param_base, 5, 6, 5, 8, 0, 5, 11, 16);
	}
////printf("%s: %d\n", __func__,__LINE__);
	ipu_cpmem_set_field(param_base, NON_INTERLEAVED_SLY, stride * width- 1);
	//printf("cpmem_base = %08x\n", cpmem_base);
	//if (cpmem_base == 0x027005c0) memcpy((void *)ipu_membg, &params, sizeof(params));
	//else if (cpmem_base == 0x027002c0) memcpy((void *)ipu_resip, &params, sizeof(params));
	memcpy(get_virtualAddr((uint32_t*)cpmem_base), &params, sizeof(params));
}

////////////////////////////////////////////////////////////////////////////////

//
// Returns 1 if a display is detected, 0 otherwise
//

static inline int hdmi_detect(void)
{
	//printf("%p\n",(HDMI_ARB_BASE_ADDR));
	return (__raw_readb((uint32_t*)(HDMI_ARB_BASE_ADDR + HDMI_PHY_STAT0))
            & HDMI_DVI_STAT) ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////

static void allocate_hdmi_dma_bufs(void) {
    hdmi_buf_0 = camkes_dma_alloc(sizeof(*hdmi_buf_0), HDMI_BUF_ALIGN, 0);
    assert(hdmi_buf_0);
    hdmi_buf_1 = camkes_dma_alloc(sizeof(*hdmi_buf_1), HDMI_BUF_ALIGN, 0);
    assert(hdmi_buf_1);
    hdmi_buf_2 = camkes_dma_alloc(sizeof(*hdmi_buf_2), HDMI_BUF_ALIGN, 0);
    assert(hdmi_buf_2);
    hdmi_buf_3 = camkes_dma_alloc(sizeof(*hdmi_buf_3), HDMI_BUF_ALIGN, 0);
    assert(hdmi_buf_3);
}

static int allocate_hdmi_bufs(uint32_t num, uint32_t sizes,
		uintptr_t *phys, uint16_t **virt)
{
   int i;
   // allocate mem to virt
   for (i = 0; i < num; i++) {
      if (i == DMA_DISP) {
      	virt[i] = (uint16_t *)hdmi_buf_0;
         phys[i] = (uintptr_t)camkes_dma_get_paddr(hdmi_buf_0);
      }
      else if (i == DMA_RES) {
         virt[i] = (uint16_t *)hdmi_buf_2;
         phys[i] = (uintptr_t)camkes_dma_get_paddr(hdmi_buf_2);;
      }

		if (virt[i] == NULL) {
			////printf("%s: %d\n", __func__,__LINE__);
			return -ENOMEM;
		}
		////printf("%s: %d\n", __func__,__LINE__);
		assert(phys[i] >= MEM_BEGIN && phys[i] < MEM_END);
		D(DBG_LOG,
				"[disp] allocated display buffer at phys 0x%08x, virt 0x%08x\n",
				(uint32_t)phys[i], (uint32_t)virt[i]);
   }
   return 0;

}



//
// Allocate some DMA buffers
//

// static int allocate_dma_bufs(uint32_t num, uint32_t sizes,
// 		uintptr_t *phys, uint16_t **virt)
// {
// 	for (int i = 0; i < num; i++) {
// 		virt[i] = dma_malloc(NULL, DISP_BUFSIZE, IDMAC_BUFALIGN, 0, 0);
// 		if (virt[i] == NULL) {
// 			return -ENOMEM;
// 		}
// ////printf("%s: %d\n", __func__,__LINE__);
// 		phys[i] = (uintptr_t)dma_pin(NULL, virt[i], 0);
// 		assert(phys[i] >= MEM_BEGIN && phys[i] < MEM_END);
// ////printf("%s: %d\n", __func__,__LINE__);
// 		D(DBG_LOG,
// 				"[disp] allocated display buffer at phys 0x%08x, virt 0x%08x\n",
// 				(uint32_t)phys[i], (uint32_t)virt[i]);
// 	}
//
// 	return 0;
// }

void write_to_buffer(uint16_t * buf, uint16_t color)
{
   if ((uint32_t)buf >= ((uint32_t)disp_bufs[DMA_RES] + IPULENGTH)){
      uint32_t local = (uint32_t)buf - ((uint32_t)disp_bufs[DMA_RES] + IPULENGTH) + (uint32_t)hdmi_buf_3;
      buf = (uint16_t *)local;
   }
   *buf = color;
}

///////////////////////////////////////////////////////////////////////////////

void display_clearText (int xPos, int yPos, int lines) {
	uint16_t * buf = disp_bufs[DMA_RES];
   unsigned char pixel[3];
   background->cur = background->data;
   for (int y = 0; y < FB_HEIGHT; y++) {
       for (int x = 0; x < FB_WIDTH; x++) {
       		uint16_t color;
       		if (y < 200) {
           		HEADER_PIXEL(background->cur, pixel);
            	color = (pixel[0] >> 3) << 11
                     | (pixel[1] >> 2) << 5
                     | (pixel[2] >> 3);
            } else {
            	color = 0xFFFF;
            }

            if (x >= xPos && x <= (FB_WIDTH) &&
                y >= yPos && y <= (yPos + lines*16)) {
                write_to_buffer(buf, color);
            }
            buf++;
       }
   }
}

void printLetter (char text, int xPos, int yPos,
                  unsigned char red, unsigned char green, unsigned char blue) {

    unsigned char pixel[3];

    uint16_t *buf = disp_bufs[DMA_RES] + xPos + FB_WIDTH * yPos;

    char *img = (char *)((uint32_t)logo_data + 4*((text/16)*16*256 + (text%16)*16));

    for (int y = 0; y < 16; y++) {
		for (int x = 0; x < 16; x++) {
            HEADER_PIXEL(img,pixel);
            // If not white, print to screen
            if (pixel[0] == 128) {
                uint16_t color = (red >> 3) << 11
                               | (green >> 2) << 5
                               | (blue >> 3);
                write_to_buffer(buf, color);
            }
            buf++;
        }
        img += (256-16)*4;
        buf += FB_WIDTH - 16;
    }


}

void display_text(const char *string, int xPos, int yPos, char red, char green, char blue) {
    int i = 0;
    while (string[i] != '\0') {
        printLetter(string[i], xPos, yPos,red,green,blue);
        switch (string[i]) {
		     case 'l': case 'f': case '!': case '.':
			  case 'i': case 'I': case 'j': case '[':
			  case ']': case ':': case ';': case ' ':
			  case '`': case 'r': case 't': case 'J':
			  case '(': case ')': case '|': xPos += 8; break;
			  case 'W': case 'M': case 'w': case '@':
			  xPos += 14; break;
			  case '%': case 'm': xPos += 16; break;
			  default: xPos += 12; break;
        }
        i++;
    }
}


////////////////////////////////////////////////////////////////////////////////
void display_clear(void) {
	memset((uint16_t *)disp_bufs[DMA_RES], 0xFF, IPULENGTH);
	memset((uint16_t *)hdmi_buf_3, 0xFF, BUFLENGTH);
}

void logo(void) {
////printf("%s: %d\n", __func__,__LINE__);
    // find top left of logo
    int sx = FB_WIDTH / 2 - logo_width / 2;
    int sy = FB_HEIGHT / 2 - logo_height / 2;
    uint16_t * buf = disp_bufs[DMA_RES] + sy * FB_WIDTH + sx;
    char * img = logo_data;
    ////printf("%s: %d\n", __func__,__LINE__);
    // render sos logo
    unsigned char pixel[3];
    for (int y = 0; y < logo_height; y++) {
        for (int x = 0; x < logo_width; x++) {
            HEADER_PIXEL(img, pixel);
            //if (x < 3) //printf("1: %x, 2: %x, 3: %x\n", pixel[0], pixel[1], pixel[2]);
            if (pixel[0] == 128) write_to_buffer(buf,0);
            buf++;
        }
        buf += (FB_WIDTH - logo_width);
    }
////printf("%s: %d\n", __func__,__LINE__);
    for (int i = 0; i < FB_WIDTH; i++) {

    }
}
////////////////////////////////////////////////////////////////////////////////

//
//
//

/*int*/ void display_init(void /*   **virt,
                 uintptr_t *phys,
		         int def_width,
                 int def_height,
                 int def_pixfmt  */)
{
	int err, def_width, def_height, def_pixfmt;
	////printf("%s: %d\n", __func__,__LINE__);
	def_width = DISP_WIDTH;
	def_height = DISP_HEIGHT;
	def_pixfmt = DISP_PIXFMT_RGB565;

	buf_size = def_width * def_height * def_pixfmt;

	D(DBG_LOG, "[disp] initialising display: %dx%dx%d\n",
			DISP_WIDTH, DISP_HEIGHT, DISP_DEPTH);
////printf("%s: %d\n", __func__,__LINE__);
	// Map in memory for peripherals
	//map_device((void *)IPU1_ARB_BASE_ADDR,
	//		IPU1_ARB_END_ADDR - IPU1_ARB_BASE_ADDR + 1);
	//map_device((void *)HDMI_ARB_BASE_ADDR,
	//		HDMI_ARB_END_ADDR - HDMI_ARB_BASE_ADDR + 1);
	//map_device((void *)SRC_BASE_ADDR, SRC_LEN);

	if (!hdmi_detect()) {
		D(DBG_ERR, "[disp] no hdmi display detected!\n");
		return /*-1*/;
	}
////printf("%s: %d\n", __func__,__LINE__);
	// Initialise HDMI TX
	D(DBG_LOG, "[disp] initialising hdmi\n");
	err = hdmi_init();
	if (err) {
		return /*err*/;
	}
////printf("%s: %d\n", __func__,__LINE__);
	// Allocate DMA buffers
	allocate_hdmi_dma_bufs();
	err = allocate_hdmi_bufs(DMA_BUFFERS, DISP_BUFSIZE, dma_phys, disp_bufs);
	//err = allocate_dma_bufs(DMA_BUFFERS, DISP_BUFSIZE, dma_phys, disp_bufs);
	if (err) {
		return /*err*/;
	}
////printf("%s: %d\n", __func__,__LINE__);
	// Initialise IPU
	D(DBG_LOG, "[disp] initialising ipu\n");

	// Channel 0 is enabled, DI clocks set by u-boot
	// Disable IDMAC channel

	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_MEMTOBG, 0);
////printf("%s: %d\n", __func__,__LINE__);
	ipu_cpmem_set(IDMAC_MEMTOBG, DISP_PIXFMT_RGB565,
			DISP_WIDTH, DISP_HEIGHT,
			dma_phys[DMA_DISP], dma_phys[DMA_DISP]);
////printf("%s: %d\n", __func__,__LINE__);
	// Re enable IDMAC channel
	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_MEMTOBG, 1);
	ipu_cpmem_set(IDMAC_RESIP, def_pixfmt,
			def_width, def_height,
			dma_phys[DMA_RES], dma_phys[DMA_RES]);
////printf("%s: %d\n", __func__,__LINE__);
	ipu_cpmem_set(IDMAC_RESOP, DISP_PIXFMT_RGB565,
			DISP_WIDTH, DISP_HEIGHT,
			dma_phys[DMA_DISP], dma_phys[DMA_DISP]);
////printf("%s: %d\n", __func__,__LINE__);
	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_RESIP, 1);
	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_RESOP, 1);

	// Disable image converter
	ipu_ic_enable(IPU_CHANNEL, 0);
////printf("%s: %d\n", __func__,__LINE__);
	// Setup image converter PP task to resize
	ipu_ic_resize_config(IPU_CHANNEL, def_width, def_height,
        DISP_WIDTH, DISP_HEIGHT);

	// Enable IC PP task for resize
	ipu_ic_pp_task_enable(IPU_CHANNEL, 1);
////printf("%s: %d\n", __func__,__LINE__);
	// Enable IC
	ipu_ic_enable(IPU_CHANNEL, 1);
////printf("%s: %d\n", __func__,__LINE__);
	// Prepare resize IDMAC channels
	ipu_channel_buf_ready(IPU_CHANNEL, IDMAC_RESIP, 0);
	ipu_channel_buf_ready(IPU_CHANNEL, IDMAC_RESOP, 0);
	while (ipu_idmac_channel_busy(IPU_CHANNEL, IDMAC_RESIP));
	while (ipu_idmac_channel_busy(IPU_CHANNEL, IDMAC_RESOP));
	ipu_channel_buf_ready(IPU_CHANNEL, IDMAC_MEMTOBG, 0);
	ipu_write_field(IPU_CHANNEL, IPU_FS_PROC_FLOW2, 15 << 12, 0b1001);
	ipu_write_field(IPU_CHANNEL, IPU_FS_PROC_FLOW1, 15 << 12, 0b1011);
	ipu_write_field(IPU_CHANNEL, IPU_FS_DISP_FLOW1, 15, 0b0101);
////printf("%s: %d\n", __func__,__LINE__);
    display_clear();

	if (virt) {
		virt = disp_bufs[DMA_RES];
	}
	if (phys) {
		phys = dma_phys[DMA_RES];
	}
	////printf("%s: %d\n", __func__,__LINE__);
	//return 0;
}

////////////////////////////////////////////////////////////////////////////////

int display_setmode(int width, int height, int pixfmt)
{
    // only support one color mode for now
	assert(pixfmt == DISP_PIXFMT_RGB565);

	// We have a fixed size DMA buffer. Really the DMA allocator should be fixed
	// to allow freeing, then this would be a bit better
	if (width * height * 2 > DISP_BUFSIZE) {
		D(DBG_LOG, "[disp] static framebuffer too small\n");
		return -1;
	}

	// Disable, change, reenabl
	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_RESIP, 0);
	ipu_ic_pp_task_enable(IPU_CHANNEL, 0);
	ipu_cpmem_set(IDMAC_RESIP, pixfmt, width, height,
        dma_phys[DMA_RES], dma_phys[DMA_RES]);
	ipu_ic_resize_config(IPU_CHANNEL, width, height, DISP_WIDTH, DISP_HEIGHT);
	ipu_ic_pp_task_enable(IPU_CHANNEL, 1);
	ipu_idmac_channel_enable(IPU_CHANNEL, IDMAC_RESIP, 1);
	ipu_channel_buf_ready(IPU_CHANNEL, IDMAC_RESIP, 0);
	return 0;
}

////////////////////////////////////////////////////////////////////////

void display_image1(Image img, int posx1, int posy1);

void display_kitty(int num) {

	if (kitty1->x1 != -1 && kitty1->y1 != -1) {
		display_clrimg(kitty1, background);
	}
	if (kitty2->x1 != -1 && kitty2->y1 != -1) {
		display_clrimg(kitty2, background);
	}
	if (kitty3->x1 != -1 && kitty3->y1 != -1) {
		display_clrimg(kitty3, background);
	}
	if (kitty4->x1 != -1 && kitty4->y1 != -1) {
		display_clrimg(kitty4, background);
		display_msg(1, 506, 220);
	} //bootup
	if (swipe->x1 != -1 && swipe->y1 != -1) {
		display_clrimg(swipe, background);
	}

	if (num == 0) {
		display_image(kitty1, 100, 234);
	} else if (num == 1) {
		display_image(kitty2, 100, 234);
	} else if (num == 2) {
		display_image(kitty3, 100, 234);
	} else if (num == 3) {
		display_image(swipe, 100, 234);
	}

}

Image newImage(int width, int height, char* dataLink) {
   Image new = malloc(sizeof(struct image_rep));
   new->x1 = -1;
   new->y1 = -1;
   new->width = width;
   new->height = height;
   new->data = dataLink;
   new->cur = dataLink;
   new->background = 0;
   return new;
}

Image newBackground(char* dataLink) {
   Image new = malloc(sizeof(struct image_rep));
   new->x1 = 0;
   new->y1 = 0;
   new->width = FB_WIDTH;
   new->height = 200;
   new->data = dataLink;
   new->cur = dataLink;
   new->background = 1;
   return new;
}

void freeImage(Image img) {
   free(img->data);
   free(img->cur);
   free(img);
}



void display_image(Image img, int posx1, int posy1) {
   int sx = posx1;
   int sy = posy1;
   img->x1 = posx1;
   img->y1 = posy1;

   uint16_t * buf = disp_bufs[DMA_RES] + (sy) * FB_WIDTH + sx;
   unsigned char pixel[3];
   img->cur = img->data;
   for (int y = 0; y < img->height; y++) {
       for (int x = 0; x < img->width; x++) {

           HEADER_PIXEL(img->cur, pixel);
           uint16_t color = (pixel[0] >> 3) << 11
                          | (pixel[1] >> 2) << 5
                          | (pixel[2] >> 3);
           if (color != 0xFFFF) {
               write_to_buffer(buf, color);
           }
           if (color != 0xFFFF) {
               write_to_buffer(buf, color);
           }
           buf++;
       }
       buf += (FB_WIDTH - img->width);
   }
}

void display_image1(Image img, int posx1, int posy1) {
   int sx = posx1;
   int sy = posy1;
   img->x1 = posx1;
   img->y1 = posy1;

   uint16_t * buf = disp_bufs[DMA_RES] + (sy +img->height) * FB_WIDTH + sx;
   unsigned char pixel[3];
   img->cur = img->data;
   for (int y = 0; y < img->height; y++) {
       for (int x = 0; x < img->width; x++) {
          pixel[0] = img->cur[2] ;
          pixel[1] = img->cur[1];
          pixel[2] = img->cur[0];
          img->cur+=3;
        //
        //    HEADER_PIXEL(img->cur, pixel);
        //    uint16_t color = 0xffff;
           uint16_t color = ((pixel[0] >> 3)&0x1f) << 11
                          | ((pixel[1] >> 2)&0x3f) << 5
                          | ((pixel[2] >> 3)&0x1f);
           if (color != 0xFFFF) {
               write_to_buffer(buf, color);
           }
           if (color != 0xFFFF) {
               write_to_buffer(buf, color);
           }
           buf++;
       }
       img->cur += 2;
       buf -= (FB_WIDTH + img->width);
   }
}

void display_clrimg(Image img, Image background) {
   if (img->x1 == -1 && img->y1 == -1) {
      printf(">> Image has not been displayed\n");
   }
   int sx = img->x1;
   int sy = img->y1;
   img->x1 = -1;
   img->y1 = -1;
   uint16_t * buf = disp_bufs[DMA_RES];
   unsigned char pixel[3];
   uint16_t color;

   background->cur = background->data;
   for (int y = 0; y < FB_HEIGHT; y++) {
       for (int x = 0; x < FB_WIDTH; x++) {
       		if (y < 200) {
           		HEADER_PIXEL(background->cur, pixel);
            	color = (pixel[0] >> 3) << 11
                     | (pixel[1] >> 2) << 5
                     | (pixel[2] >> 3);
            } else {
            	color = 0xFFFF;
            }

            if (x >= sx && x <= (sx+img->width) &&
                y >= sy && y <= (sy+img->height)) {
                write_to_buffer(buf, color);
            }
            buf++;
       }
   }
}

void display_msg(int msg, int x, int y)
{
   if (msg == 1){
      display_image(msg1, x, y);
   }
}

//extern char *image_bytes(void);

void pre_init(void) {
	display_init();
	display_clear();
	background = newBackground(background_data);
	display_image(background, 0, 0);

	//title = newImage(480, 160, title_data);
	//display_image(title, 272, 20);

	kitty4 = newImage(548, 312, kitty4_data);
//    char * bytes = image_bytes() + 54;

	kitty1 = newImage(300, 300, kitty1_data);
    // int i = 0;
    // int j = 0;
    // while (i < ((150*166)/4)) {
    //     for (j = 0; j < 10; j++) {
    //         printf("0x%08x ", ((unsigned int *)bytes)[i+j]);
    //     }
    //     printf("\n");
    //     for (j = 0; j < 10; j++) {
    //         printf("0x%08x ", ((unsigned int *)bytes2)[i+j]);
    //     }
    //     printf("\n");
    //     i+=10;
    //     //printf("0x%08x\t 0x%08x\n", ((unsigned int *)bytes)[i], ((unsigned int *)bytes2)[i]);
    // }
    display_image(kitty1, 238, 234);

	kitty2 = newImage(300, 300, kitty2_data);
	kitty3 = newImage(300, 300, kitty3_data);
	swipe = newImage(300, 300, swipe_data);
	msg1 = newImage(250, 40, msg1_data);
}
////////////////////////////////////////////////////////////////////////////////

int run (void) {
	// printf("Blah\n");
	// printf("%s: %d\n", __func__,__LINE__);
	// printf("%08x, %08x\n", HDMI_ARB_BASE_ADDR, IPU1_ARB_BASE_ADDR);
	// printf("Virtual Addresses:\n0x%08x\n0x%08x\n0x%08x\n0x%08x\n0x%08x\n",
	// 				hdmi_base, ipu_base_0, ipu_base_1, ipu_base_2, ipu_base_3);



	//display_image(kitty2, 0, 0);
	//display_clrimg(kitty1, background);
	//display_image(kitty2, 0, 500);
	//display_clrimg(kitty2, background);
	//display_image(kitty4, 500, 0);
	//display_clrimg(kitty3, background);
	//display_image(kitty4, 500, 500);
   // display_clrimg(kitty4, background);
   //
   // display_text("Kitty v4", 100, 100,50,50,50);
   // splay_text("ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz", 100, 150,0,0,0);
   // splay_text("!@#$%^&*(){}[]|1234567890", 100, 170,0,0,0);
	////printf("%s: %d\n", __func__,__LINE__);
	return 0;
}
