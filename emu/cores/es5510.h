// license:BSD-3-Clause
// copyright-holders:Christian Brunschen
/**********************************************************************************************
 *
 *   es5510.h - Ensoniq ES5510 (ESP) effects DSP
 *   by Christian Brunschen
 *   ported to C for libvgm
 *
 *   This is only the DSP engine (program/register/DRAM state and the execution model).
 *   The libvgm device wrapper lives elsewhere.
 *
 **********************************************************************************************/

#ifndef __ES5510_H__
#define __ES5510_H__

#include "../../stdtype.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ES5510_DRAM_SIZE_DEFAULT	(1 << 20)	/* words; TODO (MAME): not verified, most games use 128KB DRAM */
#define ES5510_INSTR_COUNT			160
#define ES5510_GPR_COUNT			0xC0

typedef struct es5510_state ES5510;

/* dramSizeWords: number of 16-bit DRAM words, rounded up to a power of two. 0 -> default (1<<20). */
ES5510* es5510_create(UINT32 dramSizeWords);
void    es5510_destroy(ES5510* chip);
void    es5510_reset(ES5510* chip);				/* equivalent of MAME device_reset() */

void    es5510_host_w(ES5510* chip, UINT8 offset, UINT8 data);	/* host_w, offset 0x00..0xFF */
UINT8   es5510_host_r(ES5510* chip, UINT8 offset);				/* host_r */

/* external ESPHALT line (esqpump's m_esp_halted). 1 = halted: es5510_run_once() does nothing. */
void    es5510_set_halt_line(ES5510* chip, UINT8 halted);
UINT8   es5510_get_halt_line(const ES5510* chip);

/* serial inputs/outputs 0..7: 0=ser0l 1=ser0r 2=ser1l 3=ser1r 4=ser2l 5=ser2r 6=ser3l 7=ser3r (like MAME ser_r/ser_w) */
void    es5510_ser_w(ES5510* chip, UINT8 idx, INT16 data);
INT16   es5510_ser_r(const ES5510* chip, UINT8 idx);

/* MAME run_once(): release HALT, run 1 instruction, assert HALT, run until the program ends (STATE_HALTED). */
void    es5510_run_once(ES5510* chip);

/* direct access (program image blocks / register pokes) */
void    es5510_write_instr(ES5510* chip, UINT8 pc, UINT64 instr);	/* instr[pc], pc < 160 */
void    es5510_write_gpr(ES5510* chip, UINT8 idx, INT32 value);		/* via write_reg(): idx < 0xC0 = GPR, 0xEA..0xFF = special regs */
UINT64  es5510_read_instr(const ES5510* chip, UINT8 pc);
INT32   es5510_read_gpr(const ES5510* chip, UINT8 idx);				/* via read_reg() */

#ifdef __cplusplus
}
#endif

#endif	// __ES5510_H__
