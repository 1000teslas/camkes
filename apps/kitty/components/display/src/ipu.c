/*
 * Modified for SOS
 *
 * Copyright (c) 2011-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "ipu_common.h"
#include "camkes.h"
#include "util.h"

/*!
 * write field of ipu registers, without affecting other bits.
 *
 * @param	ipu_index:	ipu index
 * @param	ID_addr:    register address
 * @param	ID_mask:    fields position
 * @param	data:	    the value of input
 */
 
 ///////////////////////////////////////////////////////////////////////////////
inline uint32_t __raw_readl(uint32_t *addr)
{


	if ((uintptr_t)addr >= HDMI_ARB_BASE_ADDR && (uintptr_t)addr <= HDMI_ARB_END_ADDR) {
		return readl((void*)((uint32_t)addr - HDMI_ARB_BASE_ADDR + (uint32_t)hdmi_base));
	} else if ((uintptr_t)addr >= IPU1_ARB_BASE_ADDR && (uintptr_t)addr <= IPU1_ARB_END_ADDR) {
		switch(((uint32_t)addr-IPU1_ARB_BASE_ADDR)/0x100000) {
			case 0: return readl((unsigned char *)((uint32_t)addr - IPU1_ARB_BASE_ADDR + (uint32_t)ipu_base_0) ); break;
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
////////////////////////////////////////////////////////////////////////////////
 
void ipu_write_field(int ipu_index, uint32_t *ID_addr, uint32_t ID_mask, uint32_t data)
{
    uint32_t rdata;

	uint32_t addr = (uint32_t)ID_addr;

	 //dprintf(0, "**%08x, %08x\n", addr, ID_addr);
	 //printf("ipu_index = %d, REGS = %d / %08x\n", ipu_index, REGS_IPU_BASE(ipu_index), REGS_IPU_BASE(ipu_index));
    addr += (uint32_t)REGS_IPU_BASE(ipu_index);
    //printf("%s: %d\n", __func__,__LINE__);
    //printf("base = %08x\n", IPU1_ARB_BASE_ADDR);
    //printf("addr = %08x\n", addr);
    //if (addr == 0x02608004) rdata = __raw_readl((unsigned int)ipu_dma);
    rdata = __raw_readl((void*)addr);
    //printf("%s: %d\n", __func__,__LINE__);
    rdata &= ~ID_mask;
    rdata |= (data * (ID_mask & -ID_mask)) & ID_mask;
    //printf("%s: %d\n", __func__,__LINE__);
    //if (addr == 0x02608004) __raw_writel(rdata, (unsigned int)ipu_dma);
    __raw_writel(rdata, (uint32_t*)addr);
    //printf("%s: %d\n", __func__,__LINE__);
}

// Enable IC
void ipu_ic_enable(int ipu_index, int enable)
{
	ipu_write_field(ipu_index, IPU_CONF, IPU_CONF_IC_EN, enable);
}

// Enable IC post processing task
void ipu_ic_pp_task_enable(int ipu_index, int enable)
{
	ipu_write_field(ipu_index, IC_CONF, IC_CONF_PP_EN, enable);
}

// Calculate IC resize coefficients
static int ipu_ic_calc_resize_coeffs(int in_size, int out_size,
	int *resize_coeff, int *downsize_coeff)
{
	int32_t tempSize;
    int32_t tempDownsize;

    /* Cannot downsize more than 8:1 */
    if ((out_size << 3) < in_size)
        return -1;

    /* compute downsizing coefficient */
    tempDownsize = 0;
    tempSize = in_size;

    while (((tempSize >= out_size * 2) || (tempSize > 1024))
           && (tempDownsize < 2)) {
        tempSize >>= 1;
        tempDownsize++;
    }

    *downsize_coeff = tempDownsize;

    /* compute resizing coefficient using the following equation:
       resizeCoeff = M*(SI -1)/(SO - 1)
       where M = 2^13, SI - input size, SO - output size    */
    *resize_coeff = (8192L * (tempSize - 1)) / (out_size - 1);

    if (*resize_coeff >= 16384L) {
        D(DBG_LOG, "[disp] overflow on resize coeff %d.\n", *resize_coeff);
        *resize_coeff = 16383;
    }

	return 0;
}

// Configure IC resize task
void ipu_ic_resize_config(int ipu_index, int width_in, int height_in,
		int width_out, int height_out)
{
	ipu_write_field(ipu_index, IC_CONF, IC_CONF_PP_EN, 0);
	ipu_write_field(ipu_index, IC_CONF, IC_CONF_PP_CSC1, 0);
    ipu_write_field(ipu_index, IC_CONF, IC_CONF_PP_CSC2, 0);
    ipu_write_field(ipu_index, IC_CONF, IC_CONF_PP_CMB, 0);

	// Burst length is 16 for widths (stridelines?) of multiple 16, otherwise 8
	if (width_in % 16 == 0) {
		ipu_write_field(ipu_index, IC_IDMAC(1), IC_IDMAC_1_CB2_BURST_16, 1);
		ipu_write_field(ipu_index, IC_IDMAC(1), IC_IDMAC_1_CB5_BURST_16, 1);
	} else {
		ipu_write_field(ipu_index, IC_IDMAC(1), IC_IDMAC_1_CB2_BURST_16, 0);
		ipu_write_field(ipu_index, IC_IDMAC(1), IC_IDMAC_1_CB5_BURST_16, 0);
	}

	ipu_write_field(ipu_index, IC_IDMAC(2), IC_IDMAC_2_T3_FR_HEIGHT, height_out - 1);
	ipu_write_field(ipu_index, IC_IDMAC(3), IC_IDMAC_3_T3_FR_WIDTH, width_out - 1);

	// Calculate resize coefficients. Horizontal and vertical are
	// done separately, aspect ratio not maintained
	int res_coeff, down_coeff;

	ipu_ic_calc_resize_coeffs(width_in, width_out, &res_coeff, &down_coeff);
	ipu_write_field(ipu_index, IC_PP_RSC, IC_PP_RSC_PP_DS_R_H, down_coeff);
	ipu_write_field(ipu_index, IC_PP_RSC, IC_PP_RSC_PP_RS_R_H, res_coeff);

	ipu_ic_calc_resize_coeffs(height_in, height_out, &res_coeff, &down_coeff);
	ipu_write_field(ipu_index, IC_PP_RSC, IC_PP_RSC_PP_DS_R_V, down_coeff);
	ipu_write_field(ipu_index, IC_PP_RSC, IC_PP_RSC_PP_RS_R_V, res_coeff); 
}

// Returns 1 if IDMAC channel is busy
int ipu_idmac_channel_busy(int ipu_index, int channel)
{
	int i = channel / 32;
	uint32_t mask = 1 << (channel % 32);

	return (__raw_readl((void*)(REGS_IPU_BASE(ipu_index) + (uint32_t)IDMAC_CHA_BUSY(i))) & mask) ? 1 : 0;
}

// Set the IPU channel buffer ready signal
void ipu_channel_buf_ready(int ipu_index, int channel, int buf)
{
	uint32_t mask = 1 << (channel % 32);
	uint32_t clear;

	if (buf == 0) {
		clear = (channel / 32) ? IPU_CH_BUF0_RDY1_CLR : IPU_CH_BUF0_RDY0_CLR;
		ipu_write_field(ipu_index, IPU_GPR, clear, 0);
		ipu_write_field(ipu_index, IPU_CHA_BUF0_RDY(channel), mask, 1);
	} else if (buf == 1) {
		clear = (channel / 32) ? IPU_CH_BUF1_RDY1_CLR : IPU_CH_BUF1_RDY0_CLR;
		ipu_write_field(ipu_index, IPU_GPR, clear, 0);
		ipu_write_field(ipu_index, IPU_CHA_BUF1_RDY(channel), mask, 1);

	}
}

// Enable/disable an IDMAC channel
void ipu_idmac_channel_enable(int ipu_index, int channel, int enable)
{
	//printf("%s: %d\n", __func__,__LINE__);
	uint32_t mask = 1 << (channel % 32);
	ipu_write_field(ipu_index, IDMAC_CHA_EN(channel), mask, enable);
}

void ipu_cpmem_set_field(uint32_t base, int32_t w, int32_t bit, int32_t size, uint32_t v)
{
    int32_t i = (bit) / 32;
    int32_t off = (bit) % 32;
    _param_word(base, w)[i] |= (v) << off;
    if (((bit) + (size) - 1) / 32 > i) {
        _param_word(base, w)[i + 1] |= (v) >> (off ? (32 - off) : 0);
    }
}

void ipu_cpmem_mod_field(uint32_t base, int32_t w, int32_t bit, int32_t size, uint32_t v)
{
    int32_t i = (bit) / 32;
    int32_t off = (bit) % 32;
    uint32_t mask = (1UL << size) - 1;
    uint32_t temp = _param_word(base, w)[i];
    temp &= ~(mask << off);
    _param_word(base, w)[i] = temp | (v) << off;
    if (((bit) + (size) - 1) / 32 > i) {
        temp = _param_word(base, w)[i + 1];
        temp &= ~(mask >> (32 - off));
        _param_word(base, w)[i + 1] = temp | ((v) >> (off ? (32 - off) : 0));
    }
}

uint32_t ipu_cpmem_read_field(uint32_t base, int32_t w, int32_t bit, int32_t size)
{
    uint32_t temp2;
    int32_t i = (bit) / 32;
    int32_t off = (bit) % 32;
    uint32_t mask = (1UL << size) - 1;
    uint32_t temp1 = _param_word(base, w)[i];
    temp1 = mask & (temp1 >> off);
    if (((bit) + (size) - 1) / 32 > i) {
        temp2 = _param_word(base, w)[i + 1];
        temp2 &= mask >> (off ? (32 - off) : 0);
        temp1 |= temp2 << (off ? (32 - off) : 0);
    }
    return temp1;
}
