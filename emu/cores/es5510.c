// license:BSD-3-Clause
// copyright-holders:Christian Brunschen
/***************************************************************************************
 *
 *   es5510.c - Ensoniq ES5510 (ESP) emulation
 *   by Christian Brunschen
 *   ported to C for libvgm
 *
 *   This is a literal translation of MAME's src/devices/cpu/es5510/es5510.cpp
 *   (execution model, ALU/MAC/RAM pipeline, host interface), with all of the
 *   debugger / disassembler / logging / device_t scaffolding removed.
 *
 *   TODO (from MAME):
 *      gunlock and clones: Glitch sound after game over once (MT #07861)
 *      DRAM Size isn't verified, differs per machines?
 *
 ***************************************************************************************/

#include <stdlib.h>
#include <string.h>	// for memset
#include <stddef.h>	// for NULL

#include "../../stdtype.h"
#include "es5510.h"


#define FLAG_N		(1 << 7)
#define FLAG_C		(1 << 6)
#define FLAG_V		(1 << 5)
#define FLAG_LT		(1 << 4)
#define FLAG_Z		(1 << 3)
#define FLAG_NOT	(1 << 2)

#define FLAG_MASK	(FLAG_N | FLAG_C | FLAG_V | FLAG_LT | FLAG_Z)

// The CMP operation is not affected by being skippable
#define OP_CMP		(4)

enum
{
	STATE_RUNNING = 0,
	STATE_HALTED = 1
};

enum
{
	SRC_DST_REG =   1 << 0,
	SRC_DST_DELAY = 1 << 1,
	SRC_DST_BOTH =  (1 << 0) | (1 << 1)
};

enum
{
	RAM_CONTROL_DELAY = 0,
	RAM_CONTROL_TABLE_A,
	RAM_CONTROL_TABLE_B,
	RAM_CONTROL_IO
};

enum
{
	RAM_CYCLE_READ = 0,
	RAM_CYCLE_WRITE = 1,
	RAM_CYCLE_DUMP_FIFO = 2
};

typedef struct
{
	int operands;
} alu_op_t;

typedef struct
{
	UINT8 alu_src;
	UINT8 alu_dst;
	UINT8 mac_src;
	UINT8 mac_dst;
} op_select_t;

typedef struct
{
	UINT8 cycle;	// ram_cycle
	UINT8 access;	// ram_control_access
} ram_control_t;

typedef struct
{
	UINT8 aReg;
	UINT8 bReg;
	UINT8 src;		// op_src_dst
	UINT8 dst;		// op_src_dst
	UINT8 op;
	INT32 aValue;
	INT32 bValue;
	INT32 result;
	UINT8 update_ccr;
	UINT8 write_result;
} alu_t;

typedef struct
{
	UINT8 cReg;
	UINT8 dReg;
	UINT8 src;		// op_src_dst
	UINT8 dst;		// op_src_dst
	UINT8 accumulate;
	INT32 cValue;
	INT32 dValue;
	INT64 product;
	INT64 result;
	UINT8 write_result;
} mulacc_t;

typedef struct
{
	INT32 address;	// up to 20 bits, left-justified within the right 24 bits of the 32-bit word
	UINT8 io;		// I/O space, rather than delay line memory
	UINT8 cycle;	// cycle type (ram_cycle)
} ram_t;

struct es5510_state
{
	int icount;
	UINT8 halt_asserted;	// the ES5510's own HALT input (driven by run_once)
	UINT8 ext_halt;			// external ESPHALT line (esqpump's m_esp_halted); gates run_once
	UINT8 pc;
	UINT8 state;			// STATE_RUNNING / STATE_HALTED
	INT32 gpr[ES5510_GPR_COUNT];	// 24 bits, right justified
	INT16 ser0r;
	INT16 ser0l;
	INT16 ser1r;
	INT16 ser1l;
	INT16 ser2r;
	INT16 ser2l;
	INT16 ser3r;
	INT16 ser3l;
	INT64 machl;			// 48 bits, right justified and sign extended
	UINT8 mac_overflow;		// whether reading the MAC register should return a saturated replacement value
	INT32 dil;
	INT32 memsiz;
	INT32 memmask;
	INT32 memincrement;
	INT8 memshift;
	INT32 dlength;
	INT32 abase;
	INT32 bbase;
	INT32 dbase;
	INT32 sigreg;
	int mulshift;
	INT8 ccr;				// really, 5 bits, left justified
	INT8 cmr;				// really, 6 bits, left justified
	INT32 dol[2];
	int dol_count;

	UINT64 instr[ES5510_INSTR_COUNT];	// 48 bits, right justified
	INT16* dram;			// there are up to 20 address bits (at least 16 expected), left justified within the 24 bits of a gpr or dadr
	UINT32 dram_size;		// in words, power of two
	UINT32 dram_mask;

	// latch registers for host interaction
	INT32  dol_latch;		// 24 bits
	INT32  dil_latch;		// 24 bits
	UINT32 dadr_latch;		// 24 bits
	INT32  gpr_latch;		// 24 bits, holding up to 20 address bits, left justified
	UINT64 instr_latch;		// 48 bits, right justified
	UINT8  ram_sel;			// effectively a boolean
	UINT8  host_control;	// ESP state / host control register
	UINT8  host_serial;		// serial I/O format and control

	// currently executing instruction(s)
	alu_t alu;
	mulacc_t mulacc;
	ram_t ram, ram_p, ram_pp;	// ram operations for cycles N, N-1 and N-2
};


/* ---------------------------------------------------------------------------
 * small helpers (replacements for util::sext, BIT, mul_32x32 and for shifts
 * on possibly-negative values, which are written via unsigned arithmetic so
 * that the bit pattern is the same as in the C++ original on every compiler)
 * --------------------------------------------------------------------------- */

// util::sext(x, 24): sign-extend the low 24 bits of x
static inline INT32 sext24(INT32 x)
{
	return (INT32)((UINT32)x << 8) >> 8;
}

// util::sext(x, 48): sign-extend the low 48 bits of x
static inline INT64 sext48(INT64 x)
{
	return (INT64)((UINT64)x << 16) >> 16;
}

// (INT32)value << shift, bit-exact even when value is negative
static inline INT32 shl32(INT32 value, int shift)
{
	return (INT32)((UINT32)value << shift);
}

// (INT64)value << shift, bit-exact even when value is negative
static inline INT64 shl64(INT64 value, int shift)
{
	return (INT64)((UINT64)value << shift);
}

// mul_32x32(a, b): full 64-bit signed product
static inline INT64 mul_32x32(INT32 a, INT32 b)
{
	return (INT64)a * (INT64)b;
}

static inline UINT8 setFlag(UINT8 ccr, UINT8 flag)
{
	return ccr | flag;
}

static inline UINT8 clearFlag(UINT8 ccr, UINT8 flag)
{
	return ccr & ~flag;
}

static inline UINT8 setFlagTo(UINT8 ccr, UINT8 flag, UINT8 set)
{
	return set ? setFlag(ccr, flag) : clearFlag(ccr, flag);
}

static inline UINT8 isFlagSet(UINT8 ccr, UINT8 flag)
{
	return (ccr & flag) != 0;
}

static inline INT32 alu_add(INT32 a, INT32 b, UINT8* flags)
{
	INT32 aSign = a & 0x00800000;
	INT32 bSign = b & 0x00800000;
	INT32 result = (INT32)((UINT32)a + (UINT32)b);
	INT32 resultSign = result & 0x00800000;
	UINT8 overflow = (aSign == bSign) && (aSign != resultSign);
	UINT8 carry = (result & 0x01000000) != 0;
	UINT8 negative = resultSign != 0;
	UINT8 lessThan = (overflow && !negative) || (!overflow && negative);
	*flags = setFlagTo(*flags, FLAG_C, carry);
	*flags = setFlagTo(*flags, FLAG_N, negative);
	*flags = setFlagTo(*flags, FLAG_Z, (result & 0x00ffffff) == 0);
	*flags = setFlagTo(*flags, FLAG_V, overflow);
	*flags = setFlagTo(*flags, FLAG_LT, lessThan);
	return result & 0x00ffffff;
}

static inline INT32 alu_saturate(INT32 value, UINT8* flags, UINT8 negative)
{
	if (isFlagSet(*flags, FLAG_V))
	{
		*flags = setFlagTo(*flags, FLAG_N, negative);
		*flags = clearFlag(*flags, FLAG_Z);
		return negative ? 0x00800000 : 0x007fffff;
	}
	else
	{
		return value;
	}
}

static inline INT32 alu_negate(INT32 value)
{
	return ((value ^ 0x00ffffff) + 1) & 0x00ffffff;
}

static inline INT32 alu_asl(INT32 value, int shift, UINT8* flags)
{
	const INT32 src24 = value & 0x00ffffff;
	const UINT8 carry = (src24 >> (24 - shift)) & 1;	// BIT(src24, 24 - shift)
	const INT64 shifted = shl64(sext24(src24), shift);
	const UINT8 overflow = (shifted > 0x007fffffLL) || (shifted < -0x00800000LL);
	const INT32 result = overflow
		? ((shifted < 0) ? 0x00800000 : 0x007fffff)
		: ((INT32)shifted & 0x00ffffff);

	*flags = setFlagTo(*flags, FLAG_C, carry);
	*flags = setFlagTo(*flags, FLAG_V, overflow);
	*flags = setFlagTo(*flags, FLAG_N, (result & 0x00800000) != 0);
	*flags = setFlagTo(*flags, FLAG_Z, (result & 0x00ffffff) == 0);

	return result;
}

static INT8 countLowOnes(INT32 x)
{
	INT8 n = 0;
	while ((x & 1) == 1)
	{
		++n;
		x >>= 1;
	}
	return n;
}


/* ---------------------------------------------------------------------------
 * tables
 * --------------------------------------------------------------------------- */

static const alu_op_t ALU_OPS[16] =
{
	{ 2 },	// ADD
	{ 2 },	// SUB
	{ 2 },	// ADDU
	{ 2 },	// SUBU
	{ 2 },	// CMP
	{ 2 },	// AND
	{ 2 },	// OR
	{ 2 },	// XOR
	{ 1 },	// ABS
	{ 1 },	// MOV
	{ 1 },	// ASL2
	{ 1 },	// ASL8
	{ 1 },	// LS15
	{ 1 },	// DIFF
	{ 1 },	// ASR
	{ 0 },	// END
};

static const op_select_t OPERAND_SELECT[16] =
{
	{ SRC_DST_REG,   SRC_DST_REG,   SRC_DST_REG,   SRC_DST_REG   },
	{ SRC_DST_REG,   SRC_DST_REG,   SRC_DST_REG,   SRC_DST_DELAY },
	{ SRC_DST_REG,   SRC_DST_REG,   SRC_DST_REG,   SRC_DST_BOTH  },
	{ SRC_DST_REG,   SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_REG   },
	{ SRC_DST_REG,   SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_BOTH  },
	{ SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_REG   },
	{ SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_DELAY, SRC_DST_REG   },
	{ SRC_DST_REG,   SRC_DST_BOTH,  SRC_DST_REG,   SRC_DST_REG   },
	{ SRC_DST_REG,   SRC_DST_BOTH,  SRC_DST_DELAY, SRC_DST_REG   },
	{ SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_REG,   SRC_DST_REG   },
	{ SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_REG,   SRC_DST_DELAY },
	{ SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_REG,   SRC_DST_BOTH  },
	{ SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_REG   },
	{ SRC_DST_DELAY, SRC_DST_REG,   SRC_DST_DELAY, SRC_DST_BOTH  },
	{ SRC_DST_DELAY, SRC_DST_BOTH,  SRC_DST_REG,   SRC_DST_REG   },
	{ SRC_DST_DELAY, SRC_DST_BOTH,  SRC_DST_DELAY, SRC_DST_REG   },
};

static const ram_control_t RAM_CONTROL[8] =
{
	{ RAM_CYCLE_READ,      RAM_CONTROL_DELAY   },	// Read Delay+%06x
	{ RAM_CYCLE_WRITE,     RAM_CONTROL_DELAY   },	// Write Delay+%06x
	{ RAM_CYCLE_READ,      RAM_CONTROL_TABLE_A },	// Read Table A+%06x
	{ RAM_CYCLE_WRITE,     RAM_CONTROL_TABLE_A },	// Write Table A+%06x
	{ RAM_CYCLE_READ,      RAM_CONTROL_TABLE_B },	// Read Table B+%06x
	{ RAM_CYCLE_DUMP_FIFO, RAM_CONTROL_DELAY   },	// Read Delay+%06x and Dump FIFO
	{ RAM_CYCLE_READ,      RAM_CONTROL_IO      },	// Read from I/O at %06x
	{ RAM_CYCLE_WRITE,     RAM_CONTROL_IO      },	// Write to I/O at %06x
};


/* ---------------------------------------------------------------------------
 * forward declarations
 * --------------------------------------------------------------------------- */

static INT32 read_reg(const ES5510* chip, UINT8 reg);
static void write_reg(ES5510* chip, UINT8 reg, INT32 value);
static void write_to_dol(ES5510* chip, INT32 value);
static INT32 alu_operation(UINT8 op, INT32 a, INT32 b, UINT8* flags);
static void alu_operation_end(ES5510* chip);
static void execute_run(ES5510* chip);


/* ---------------------------------------------------------------------------
 * DRAM access
 * --------------------------------------------------------------------------- */

// TODO (MAME): Masked address?
static inline INT16 dram_r(const ES5510* chip, INT32 addr)
{
	return chip->dram[(UINT32)addr & chip->dram_mask];
}

static inline void dram_w(ES5510* chip, INT32 addr, INT16 data)
{
	chip->dram[(UINT32)addr & chip->dram_mask] = data;
}


/* ---------------------------------------------------------------------------
 * creation / destruction / reset
 * --------------------------------------------------------------------------- */

// Initialize ESP to mostly zeroed, configured for 64k samples of delay line memory, running (not halted)
ES5510* es5510_create(UINT32 dramSizeWords)
{
	ES5510* chip;
	UINT32 size;

	chip = (ES5510*)calloc(1, sizeof(ES5510));
	if (chip == NULL)
		return NULL;

	if (dramSizeWords == 0)
		dramSizeWords = ES5510_DRAM_SIZE_DEFAULT;
	// round up to a power of two so that the DRAM_MASK address masking works like in MAME
	size = 1;
	while (size < dramSizeWords && size < 0x80000000U)
		size <<= 1;
	chip->dram_size = size;
	chip->dram_mask = size - 1;
	chip->dram = (INT16*)calloc(chip->dram_size, sizeof(INT16));
	if (chip->dram == NULL)
	{
		free(chip);
		return NULL;
	}

	// constructor initial values (those not covered by device_reset)
	chip->icount = 0;
	chip->halt_asserted = 0;
	chip->ext_halt = 0;
	chip->pc = 0;
	chip->state = STATE_HALTED;
	chip->ser0r = chip->ser0l = 0;
	chip->ser1r = chip->ser1l = 0;
	chip->ser2r = chip->ser2l = 0;
	chip->ser3r = chip->ser3l = 0;
	chip->machl = 0;
	chip->mac_overflow = 0;
	chip->dil = 0;
	chip->memsiz = 0x00ffffff;
	chip->memmask = 0x00000000;
	chip->memincrement = 0x01000000;
	chip->memshift = 24;
	chip->dlength = 0;
	chip->abase = 0;
	chip->bbase = 0;
	chip->dbase = 0;
	chip->sigreg = 0;
	chip->mulshift = 2;
	chip->ccr = 0;
	chip->cmr = 0;
	chip->dol[0] = chip->dol[1] = 0;
	chip->dol_count = 0;
	chip->dol_latch = 0;
	chip->dil_latch = 0;
	chip->dadr_latch = 0;
	chip->gpr_latch = 0;
	chip->instr_latch = 0;
	chip->ram_sel = 0;
	chip->host_control = 0;
	chip->host_serial = 0;
	memset(&chip->alu, 0, sizeof(chip->alu));
	memset(&chip->mulacc, 0, sizeof(chip->mulacc));

	es5510_reset(chip);

	return chip;
}

void es5510_destroy(ES5510* chip)
{
	if (chip == NULL)
		return;
	free(chip->dram);
	free(chip);
}

// device_reset()
void es5510_reset(ES5510* chip)
{
	chip->pc = 0x00;
	memset(chip->gpr, 0, sizeof(chip->gpr));
	memset(chip->instr, 0, sizeof(chip->instr));
	memset(chip->dram, 0, chip->dram_size * sizeof(INT16));
	chip->state = STATE_RUNNING;
	chip->dil_latch = chip->dol_latch = chip->gpr_latch = 0;
	chip->dadr_latch = 0;
	chip->instr_latch = 0;
	chip->ram_sel = 0;
	chip->host_control = 0x04;	// Signal Host Access not OK
	chip->host_serial = 0;
	chip->sigreg = 0;
	chip->mulshift = 2;
	memset(&chip->ram, 0, sizeof(ram_t));
	memset(&chip->ram_p, 0, sizeof(ram_t));
	memset(&chip->ram_pp, 0, sizeof(ram_t));
}


/* ---------------------------------------------------------------------------
 * host interface
 * --------------------------------------------------------------------------- */

UINT8 es5510_host_r(ES5510* chip, UINT8 offset)
{
	switch (offset)
	{
	case 0x00: return (chip->gpr_latch >> 16) & 0xff;
	case 0x01: return (chip->gpr_latch >>  8) & 0xff;
	case 0x02: return (chip->gpr_latch >>  0) & 0xff;

	case 0x03: return (chip->instr_latch >> 40) & 0xff;
	case 0x04: return (chip->instr_latch >> 32) & 0xff;
	case 0x05: return (chip->instr_latch >> 24) & 0xff;
	case 0x06: return (chip->instr_latch >> 16) & 0xff;
	case 0x07: return (chip->instr_latch >>  8) & 0xff;
	case 0x08: return (chip->instr_latch >>  0) & 0xff;

	case 0x09: return (chip->dil_latch >> 16) & 0xff;
	case 0x0a: return (chip->dil_latch >>  8) & 0xff;
	case 0x0b: return 0;

	case 0x0c: return (chip->dol_latch >> 16) & 0xff;
	case 0x0d: return (chip->dol_latch >>  8) & 0xff;
	case 0x0e: return 0xff;

	case 0x0f: return (chip->dadr_latch >> 16) & 0xff;
	case 0x10: return (chip->dadr_latch >>  8) & 0xff;
	case 0x11: return (chip->dadr_latch >>  0) & 0xff;

	case 0x12: return 0;	// Host Control

	case 0x16: return 0x27;	// Program Counter, for test purposes only
	}

	// default: 0.
	return 0x00;
}

void es5510_host_w(ES5510* chip, UINT8 offset, UINT8 data)
{
	switch (offset)
	{
	case 0x00:
		chip->gpr_latch = (chip->gpr_latch & 0x00ffff) | ((data & 0xff) << 16);
		break;
	case 0x01:
		chip->gpr_latch = (chip->gpr_latch & 0xff00ff) | ((data & 0xff) <<  8);
		break;
	case 0x02:
		chip->gpr_latch = (chip->gpr_latch & 0xffff00) | ((data & 0xff) <<  0);
		break;

		/* 0x03 to 0x08 INSTR Register */
	case 0x03: chip->instr_latch = ((chip->instr_latch & 0x00ffffffffffULL) | ((UINT64)(data & 0xff) << 40)); break;
	case 0x04: chip->instr_latch = ((chip->instr_latch & 0xff00ffffffffULL) | ((UINT64)(data & 0xff) << 32)); break;
	case 0x05: chip->instr_latch = ((chip->instr_latch & 0xffff00ffffffULL) | ((UINT64)(data & 0xff) << 24)); break;
	case 0x06: chip->instr_latch = ((chip->instr_latch & 0xffffff00ffffULL) | ((UINT64)(data & 0xff) << 16)); break;
	case 0x07: chip->instr_latch = ((chip->instr_latch & 0xffffffff00ffULL) | ((UINT64)(data & 0xff) <<  8)); break;
	case 0x08: chip->instr_latch = ((chip->instr_latch & 0xffffffffff00ULL) | ((UINT64)(data & 0xff) <<  0)); break;

		/* 0x09 to 0x0b DIL Register (r/o) */

	case 0x0c:
		chip->dol_latch = (chip->dol_latch & 0x00ffff) | ((data & 0xff) << 16);
		break;
	case 0x0d:
		chip->dol_latch = (chip->dol_latch & 0xff00ff) | ((data & 0xff) <<  8);
		break;
	case 0x0e:
		chip->dol_latch = (chip->dol_latch & 0xffff00) | ((data & 0xff) <<  0);
		break;	// TODO (MAME): docs says that this always returns 0xff

	case 0x0f:
		chip->dadr_latch = (chip->dadr_latch & 0x00ffff) | ((UINT32)(data & 0xff) << 16);
		if (chip->ram_sel)
		{
			chip->dil_latch = shl32(dram_r(chip, (INT32)chip->dadr_latch), 8);
		}
		else
		{
			dram_w(chip, (INT32)chip->dadr_latch, (INT16)(chip->dol_latch >> 8));
		}
		break;

	case 0x10: chip->dadr_latch = (chip->dadr_latch & 0xff00ff) | ((UINT32)(data & 0xff) <<  8); break;
	case 0x11: chip->dadr_latch = (chip->dadr_latch & 0xffff00) | ((UINT32)(data & 0xff) <<  0); break;

		/* 0x12 Host Control */
	case 0x12:
		chip->host_control = (chip->host_control & 0x4) | (data & 0x3);
		if (chip->host_control & 0x2)	// RAM clear
		{
			// TODO (MAME): Timing, MEMSIZ and DLENGTH behavior
			if (chip->state == STATE_HALTED)	// only in halted
			{
				memset(chip->dram, 0, chip->dram_size * sizeof(INT16));
			}
			chip->host_control &= ~0x2;
		}
		// bit 0 is RAM refresh disable flag
		break;

	case 0x14: chip->ram_sel = data & 0x80; /* bit 6 is i/o select, everything else is undefined */ break;

		/* 0x16 Program Counter (test purpose, r/o?) */
		/* 0x17 Internal Refresh counter (test purpose) */
		/* 0x18 Host Serial Control */
	case 0x18:
		chip->host_serial = data;
		break;

		/* 0x1f Halt enable (w) / Frame Counter (r) */
	case 0x1f:
		if (chip->halt_asserted)
		{
			chip->state = STATE_HALTED;
		}
		break;

	case 0x80: /* Read select - GPR + INSTR */
		/* Check if an INSTR address is selected */
		if (data < 0xa0)
		{
			chip->instr_latch = chip->instr[data];
		}
		if (data < 0xc0)
		{
			chip->gpr_latch = chip->gpr[data] & 0xffffff;
		}
		else if (data >= 0xea)
		{
			chip->gpr_latch = read_reg(chip, data);
		}
		break;

	case 0xa0: /* Write select - GPR */
		write_reg(chip, data, chip->gpr_latch);
		break;

	case 0xc0: /* Write select - INSTR */
		if (data < 0xa0)
		{
			chip->instr[data] = chip->instr_latch & 0xffffffffffffULL;
		}
		break;

	case 0xe0: /* Write select - GPR + INSTR */
		if (data < 0xa0)
		{
			chip->instr[data] = chip->instr_latch;
		}
		write_reg(chip, data, chip->gpr_latch);
		break;
	}
}


/* ---------------------------------------------------------------------------
 * serial ports
 * --------------------------------------------------------------------------- */

INT16 es5510_ser_r(const ES5510* chip, UINT8 idx)
{
	switch (idx)
	{
	case 0: return chip->ser0l;
	case 1: return chip->ser0r;
	case 2: return chip->ser1l;
	case 3: return chip->ser1r;
	case 4: return chip->ser2l;
	case 5: return chip->ser2r;
	case 6: return chip->ser3l;
	case 7: return chip->ser3r;
	}
	return 0;
}

void es5510_ser_w(ES5510* chip, UINT8 idx, INT16 data)
{
	switch (idx)
	{
	case 0: chip->ser0l = data; break;
	case 1: chip->ser0r = data; break;
	case 2: chip->ser1l = data; break;
	case 3: chip->ser1r = data; break;
	case 4: chip->ser2l = data; break;
	case 5: chip->ser2r = data; break;
	case 6: chip->ser3l = data; break;
	case 7: chip->ser3r = data; break;
	}
}


/* ---------------------------------------------------------------------------
 * halt line
 * --------------------------------------------------------------------------- */

void es5510_set_halt_line(ES5510* chip, UINT8 halted)
{
	chip->ext_halt = halted ? 1 : 0;
}

UINT8 es5510_get_halt_line(const ES5510* chip)
{
	return chip->ext_halt;
}


/* ---------------------------------------------------------------------------
 * direct access
 * --------------------------------------------------------------------------- */

void es5510_write_instr(ES5510* chip, UINT8 pc, UINT64 instr)
{
	chip->instr[pc % ES5510_INSTR_COUNT] = instr;
}

UINT64 es5510_read_instr(const ES5510* chip, UINT8 pc)
{
	return chip->instr[pc % ES5510_INSTR_COUNT];
}

void es5510_write_gpr(ES5510* chip, UINT8 idx, INT32 value)
{
	write_reg(chip, idx, value);
}

INT32 es5510_read_gpr(const ES5510* chip, UINT8 idx)
{
	return read_reg(chip, idx);
}


/* ---------------------------------------------------------------------------
 * execution
 * --------------------------------------------------------------------------- */

static void execute_run(ES5510* chip)
{
	while (chip->icount > 0)
	{
		if (chip->state == STATE_HALTED)
		{
			// Currently halted, sample the HALT line
			if (chip->halt_asserted)
			{
				// remain halted
				chip->host_control &= ~0x04;	// Signal Host Access OK
			}
			else
			{
				// start from the beginning at PC 0
				chip->state = STATE_RUNNING;
				chip->host_control |= 0x04;	// Signal Host Access not OK
				chip->pc = 0;
			}
		}
		else
		{
			// currently running, execute one instruction.
			UINT64 instr;
			const ram_control_t* ramControl;
			INT32 offset;
			UINT8 operandSelect;
			const op_select_t* opSelect;
			UINT8 skip;
			UINT8 skippable;
			// MAME indexes instr[pc]/gpr[pc] unchecked; pc is kept in range here (see es5510_write_instr)
			const UINT8 ipc = chip->pc % ES5510_INSTR_COUNT;

			chip->ram_pp = chip->ram_p;
			chip->ram_p = chip->ram;

			// *** T0, clock high
			// --- nothing to do!

			// *** T0, clock low
			// --- Read instruction N
			instr = chip->instr[ipc];

			// --- RAM cycle N-2 (if a Read cycle): data read from bus is stored in DIL
			if (chip->ram_pp.cycle != RAM_CYCLE_WRITE)
			{
				if (chip->ram_pp.io)	// read from I/O and store into DIL
				{
					chip->dil = 0;	// read_io(ram_pp.address);
				}
				else	// read from DRAM and store into DIL
				{
					chip->dil = shl32(dram_r(chip, chip->ram_pp.address), 8);
				}
			}

			// --- start of RAM cycle N
			ramControl = &RAM_CONTROL[(instr >> 3) & 0x07];
			chip->ram.cycle = ramControl->cycle;
			chip->ram.io = (ramControl->access == RAM_CONTROL_IO);

			// --- RAM cycle N: read offset N
			offset = chip->gpr[ipc];
			switch (ramControl->access)
			{
			case RAM_CONTROL_DELAY:
				chip->ram.address = (((chip->dbase + offset) % (chip->dlength + chip->memincrement)) & chip->memmask) >> chip->memshift;
				break;
			case RAM_CONTROL_TABLE_A:
				chip->ram.address = ((chip->abase + offset) & chip->memmask) >> chip->memshift;
				break;
			case RAM_CONTROL_TABLE_B:
				chip->ram.address = ((chip->bbase + offset) & chip->memmask) >> chip->memshift;
				break;
			case RAM_CONTROL_IO:
				chip->ram.address = offset & 0x00fffff0;	// mask off the low 4 bits
				break;
			}

			// *** T1, clock high
			// --- Decode instruction N;
			//     we will do this both here and in stages as the different parts of the instruction complete & recommence.

			operandSelect = (UINT8)((instr >> 8) & 0x0f);
			opSelect = &OPERAND_SELECT[operandSelect];
			skippable = (instr & (0x01 << 7)) != 0;	// aka the 'SKIP' bit in the instruction word
			if (skippable)
			{
				UINT8 skipConditionSatisfied = (chip->ccr & chip->cmr & FLAG_MASK) != 0;
				if (isFlagSet((UINT8)chip->cmr, FLAG_NOT))
				{
					skipConditionSatisfied = !skipConditionSatisfied;
				}
				skip = skipConditionSatisfied;
			}
			else
			{
				skip = 0;
			}

			// --- Write Multiplier result N-1
			if (chip->mulacc.write_result)
			{
				INT32 tmp;
				chip->mulacc.product = shl64(mul_32x32(sext24(chip->mulacc.cValue), sext24(chip->mulacc.dValue)), chip->mulshift);
				if (chip->mulacc.accumulate)
				{
					chip->mulacc.result = chip->mulacc.product + chip->machl;
				}
				else
				{
					chip->mulacc.result = chip->mulacc.product;
				}

				if (chip->mulacc.result < -((INT64)1 << 47) || chip->mulacc.result >= ((INT64)1 << 47))
				{
					chip->mac_overflow = 1;
				}
				else
				{
					chip->mac_overflow = 0;
				}
				chip->machl = chip->mulacc.result;
				tmp = chip->mac_overflow ?
					(chip->machl < 0 ? 0x00800000 : 0x007fffff) :
					(INT32)(((UINT64)chip->mulacc.result & 0x0000ffffff000000ULL) >> 24);
				if (chip->mulacc.dst & SRC_DST_REG)
				{
					write_reg(chip, chip->mulacc.cReg, tmp);
				}
				if (chip->mulacc.dst & SRC_DST_DELAY)
				{
					write_to_dol(chip, tmp);
				}
			}

			// *** T1, clock low

			// --- Start of multiplier cycle N
			chip->mulacc.cReg = (UINT8)((instr >> 32) & 0xff);
			chip->mulacc.dReg = (UINT8)((instr >> 40) & 0xff);
			chip->mulacc.src = opSelect->mac_src;
			chip->mulacc.dst = opSelect->mac_dst;
			chip->mulacc.accumulate = ((instr >> 6) & 0x01) != 0;
			chip->mulacc.write_result = !skip;

			// --- Read Multiplier Operands N
			if (chip->mulacc.src == SRC_DST_REG)
			{
				chip->mulacc.cValue = read_reg(chip, chip->mulacc.cReg);
			}
			else	// must be SRC_DST_DELAY
			{
				chip->mulacc.cValue = chip->dil;
			}
			chip->mulacc.dValue = read_reg(chip, chip->mulacc.dReg);

			// *** T2, clock high

			// --- Write ALU Result N-1
			if (chip->alu.write_result)
			{
				UINT8 flags = (UINT8)chip->ccr;
				chip->alu.result = alu_operation(chip->alu.op, chip->alu.aValue, chip->alu.bValue, &flags);
				if (chip->alu.op != OP_CMP)
				{
					if (chip->alu.dst & SRC_DST_REG)
					{
						write_reg(chip, chip->alu.aReg, chip->alu.result);
					}
					if (chip->alu.dst & SRC_DST_DELAY)
					{
						write_to_dol(chip, chip->alu.result);
					}
				}
				if (chip->alu.update_ccr)
				{
					chip->ccr = (INT8)flags;
				}
			}

			// *** T2, clock low

			// --- Start of ALU cycle N
			chip->alu.aReg = (UINT8)((instr >> 16) & 0xff);
			chip->alu.bReg = (UINT8)((instr >> 24) & 0xff);
			chip->alu.op = (UINT8)((instr >> 12) & 0x0f);
			chip->alu.src = opSelect->alu_src;
			chip->alu.dst = opSelect->alu_dst;
			chip->alu.write_result = !skip || (chip->alu.op == OP_CMP);
			chip->alu.update_ccr = !skippable || (chip->alu.op == OP_CMP);

			if (chip->alu.op == 0xf)
			{
				alu_operation_end(chip);
			}
			else
			{
				// --- Read ALU Operands N
				const alu_op_t* aluOp = &ALU_OPS[chip->alu.op];
				if (aluOp->operands == 1)
				{
					if (chip->alu.src == SRC_DST_REG)
					{
						chip->alu.bValue = read_reg(chip, chip->alu.bReg);
					}
					else	// must be SRC_DST_DELAY
					{
						chip->alu.bValue = chip->dil;
					}
				}
				else
				{
					if (chip->alu.src == SRC_DST_REG)
					{
						chip->alu.aValue = read_reg(chip, chip->alu.aReg);
					}
					else	// must be SRC_DST_DELAY
					{
						chip->alu.aValue = chip->dil;
					}
					chip->alu.bValue = read_reg(chip, chip->alu.bReg);
				}
			}

			// --- RAM cycle N-1
			if (chip->ram_p.cycle != RAM_CYCLE_READ)
			{
				if (chip->ram_p.cycle == RAM_CYCLE_WRITE)
				{
					// If this is a write cycle, write the frontmost DOL value to RAM or I/O
					if (chip->ram_p.io)
					{
						// write_io(ram_p.io, dol[0]);
					}
					else
					{
						dram_w(chip, chip->ram_p.address, (INT16)(chip->dol[0] >> 8));
					}
				}
				// If this is a Write or Dump cycle, eject the frontmost DL value.
				chip->dol[0] = chip->dol[1];
				if (chip->dol_count > 0)
				{
					--chip->dol_count;
				}
			}

			++chip->pc;
		}
		--chip->icount;
	}
}

void es5510_run_once(ES5510* chip)
{
	// esq_5505_5510_pump_device::sound_stream_update only calls run_once() while the ESP is not halted
	if (chip->ext_halt)
		return;

	// turn HALT off
	chip->halt_asserted = 0;

	// run for one instruction
	chip->icount = 1;
	execute_run(chip);

	// turn HALT on again
	chip->halt_asserted = 1;

	// run ESP to the end of its program, a few instructions at a time
	while (chip->state != STATE_HALTED)
	{
		chip->icount = 1;
		execute_run(chip);
	}
}


/* ---------------------------------------------------------------------------
 * register file
 * --------------------------------------------------------------------------- */

static INT32 read_reg(const ES5510* chip, UINT8 reg)
{
	if (reg < 0xc0)
	{
		return chip->gpr[reg];
	}
	else
	{
		switch (reg)
		{
		case 234: return shl32(chip->ser0r, 8);
		case 235: return shl32(chip->ser0l, 8);
		case 236: return shl32(chip->ser1r, 8);
		case 237: return shl32(chip->ser1l, 8);
		case 238: return shl32(chip->ser2r, 8);
		case 239: return shl32(chip->ser2l, 8);
		case 240: return shl32(chip->ser3r, 8);
		case 241: return shl32(chip->ser3l, 8);
		case 242: return chip->mac_overflow ? (chip->machl < 0 ? 0x00000000 : 0x00ffffff) : (INT32)((chip->machl >>  0) & 0x00ffffff);	// macl
		case 243: return chip->mac_overflow ? (chip->machl < 0 ? 0x00800000 : 0x007fffff) : (INT32)((chip->machl >> 24) & 0x00ffffff);	// mach
		case 244: return chip->dil;	// DIL when reading
		case 245: return chip->dlength;
		case 246: return chip->abase;
		case 247: return chip->bbase;
		case 248: return chip->dbase;
		case 249: return chip->sigreg;
		case 250: return shl32(chip->ccr, 16);	// ccr is a (signed) int8 in MAME, so this is sign-extended
		case 251: return shl32(chip->cmr, 16);	// cmr likewise
		case 252: return 0x00ffffff;	// minus_one
		case 253: return 0x00800000;	// min
		case 254: return 0x007fffff;	// max
		case 255: return 0;	// zero
		default:
			// unknown SPR
			return 0;
		}
	}
}

static void write_reg(ES5510* chip, UINT8 reg, INT32 value)
{
	value &= 0x00ffffff;
	if (reg < 0xc0)
	{
		chip->gpr[reg] = value;
	}
	else
	{
		switch (reg)
		{
		case 234: chip->ser0r = (INT16)((value >> 8) & 0xffff); break;
		case 235: chip->ser0l = (INT16)((value >> 8) & 0xffff); break;
		case 236: chip->ser1r = (INT16)((value >> 8) & 0xffff); break;
		case 237: chip->ser1l = (INT16)((value >> 8) & 0xffff); break;
		case 238: chip->ser2r = (INT16)((value >> 8) & 0xffff); break;
		case 239: chip->ser2l = (INT16)((value >> 8) & 0xffff); break;
		case 240: chip->ser3r = (INT16)((value >> 8) & 0xffff); break;
		case 241: chip->ser3l = (INT16)((value >> 8) & 0xffff); break;
		case 242: /* macl */
		{
			INT64 masked = chip->machl & ((INT64)0x00ffffff << 24);
			INT64 shifted = (INT64)(value & 0x00ffffff) << 0;
			chip->machl = sext48(masked | shifted);
			break;
		}
		case 243: /* mach */
		{
			INT64 masked = chip->machl & ((INT64)0x00ffffff << 0);
			INT64 shifted = (INT64)(value & 0x00ffffff) << 24;
			chip->machl = sext48(masked | shifted);
			chip->mac_overflow = 0;
			break;
		}
		case 244: /* MEMSIZ when writing */
			chip->memshift = countLowOnes(value);
			chip->memsiz = 0x00ffffff >> (24 - chip->memshift);
			chip->memmask = 0x00ffffff & ~chip->memsiz;
			chip->memincrement = 1 << chip->memshift;
			chip->dbase &= chip->memmask;
			break;
		case 245: chip->dlength = value; break;
		case 246: chip->abase = value; break;
		case 247: chip->bbase = value; break;
		case 248: chip->dbase = value; break;
		case 249:
			chip->sigreg = value;
			chip->mulshift = ((chip->sigreg >> 22) & 1) ? 1 : 2;	// BIT(sigreg, 22)
			break;
		// NOTE: MAME's WRITE_REG(r, x) macro expands to "r = value" and ignores its
		// second argument, so CCR/CMR receive the low 8 bits of the (24-bit) value,
		// not "(value >> 16) & FLAG_MASK". Replicated for bit-exactness.
		case 250: chip->ccr = (INT8)value; break;
		case 251: chip->cmr = (INT8)value; break;
		case 252: break;	// minus_one: no-op
		case 253: break;	// min: no-op
		case 254: break;	// max: no-op
		case 255: break;	// zero: no-op
		default:	// unknown register
			break;
		}
	}
}

static void write_to_dol(ES5510* chip, INT32 value)
{
	if (chip->dol_count >= 2)
	{
		chip->dol[0] = chip->dol[1];
		chip->dol[1] = value;
	}
	else
	{
		chip->dol[chip->dol_count++] = value;
	}
}

static void alu_operation_end(ES5510* chip)
{
	// Handle the END instruction separately
	// sample the HALT line
	if (chip->halt_asserted)
	{
		// halt
		chip->state = STATE_HALTED;
		chip->host_control &= ~0x04;	// Signal Host Access OK
	}
	// update the delay line base pointer
	chip->dbase -= chip->memincrement;
	if (chip->dbase < 0)
	{
		chip->dbase = chip->dlength;
	}
	// Possibly reset the PC
	if (chip->state == STATE_RUNNING)
	{
		chip->pc = 0;
	}
}

static INT32 alu_operation(UINT8 op, INT32 a, INT32 b, UINT8* flags)
{
	INT32 tmp;
	switch (op)
	{
	case 0x0:	// ADD
		tmp = alu_add(a, b, flags);
		return alu_saturate(tmp, flags, (a & 0x00800000) != 0);

	case 0x1:	// SUB
		tmp = alu_add(a, alu_negate(b), flags);
		return alu_saturate(tmp, flags, (a & 0x00800000) != 0);

	case 0x2:	// ADDU
		return alu_add(a, b, flags);

	case 0x3:	// SUBU
		return alu_add(a, alu_negate(b), flags);

	case 0x4:	// CMP
		// perform the subtraction, only for its effect on the flags
		alu_add(a, alu_negate(b), flags);
		return a;

	case 0x5:	// AND
		a &= b;
		*flags = setFlagTo(*flags, FLAG_N, (a & 0x00800000) != 0);
		*flags = setFlagTo(*flags, FLAG_Z, a == 0);
		return a;

	case 0x6:	// OR
		a |= b;
		*flags = setFlagTo(*flags, FLAG_N, (a & 0x00800000) != 0);
		*flags = setFlagTo(*flags, FLAG_Z, a == 0);
		return a;

	case 0x7:	// XOR
		a ^= b;
		*flags = setFlagTo(*flags, FLAG_N, (a & 0x00800000) != 0);
		*flags = setFlagTo(*flags, FLAG_Z, a == 0);
		return a;

	case 0x8:	// ABS
	{
		UINT8 isNegative;
		INT32 result;
		*flags = clearFlag(*flags, FLAG_N);
		isNegative = (b & 0x00800000) != 0;
		*flags = setFlagTo(*flags, FLAG_C, isNegative);
		// Note: the absolute value is calculated by one's complement!
		result = isNegative ? (0x00ffffff ^ b) : b;
		*flags = setFlagTo(*flags, FLAG_Z, (result & 0x00ffffff) == 0);
		return result;
	}

	case 0x9:	// MOV
		return b;

	case 0xa:	// ASL2
		return alu_asl(b, 2, flags);

	case 0xb:	// ASL8
		return alu_asl(b, 8, flags);

	case 0xc:	// LS15
		*flags = clearFlag(*flags, FLAG_N);
		*flags = setFlagTo(*flags, FLAG_C, (b & 0x00800000) != 0);
		return shl32(b, 15) & 0x007fffff;

	case 0xd:	// DIFF
		return alu_add(0x007fffff, alu_negate(b), flags);

	case 0xe:	// ASR
		*flags = setFlagTo(*flags, FLAG_N, (b & 0x00800000) != 0);
		*flags = setFlagTo(*flags, FLAG_C, (b & 1) != 0);
		return (b >> 1) | (b & 0x00800000);

	case 0xf:	// END - handled separately in alu_operation_end()
	default:
		return 0;
	}
}
