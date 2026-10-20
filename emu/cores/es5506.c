// license:BSD-3-Clause
// copyright-holders:Aaron Giles, cam900
/**********************************************************************************************
 *
 *   Ensoniq ES5505/6 driver
 *   by Aaron Giles, rewritten by cam900 (MAME 0.288)
 *   ported to C for libvgm
 *
 *   libvgm additions (VGM "ES55xx extension"):
 *    - 4 sample ROM regions with per-region window offset and region aliasing
 *    - per-voice external bank (ES5505 "exbank", as used by Taito F3/JC/Gun Buster)
 *    - all output channel pairs are rendered, then mixed to stereo with per-channel gains
 *    - optional ES5510 (ESP) effects DSP, fed from the chip's output pairs
 *
 **********************************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "es5506.h"
#include "es5510.h"


static void es550x_pcm_update(void *param, UINT32 samples, DEV_SMPL **outputs);
static UINT8 device_start_es550x(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_es550x(void *info);
static void device_reset_es550x(void *info);

static void es550x_w8(void *info, UINT8 offset, UINT8 data);
static void es550x_w16(void *info, UINT8 offset, UINT16 data);
static void es550x_w32(void *info, UINT8 offset, UINT32 data);
static UINT8 es550x_r8(void *info, UINT8 offset);
static void es550x_ext_w(void *info, UINT16 addr, UINT8 data);
static void es550x_esp_w(void *info, UINT16 addr, UINT8 data);
static void es550x_gain_w(void *info, UINT16 addr, UINT16 data);
static void es550x_alloc_rom(void *info, UINT32 memsize);
static void es550x_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data);

static void es550x_set_mute_mask(void *info, UINT32 MuteMask);
static void es550x_set_srchg_cb(void *info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr);


static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, es550x_w8},
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D16, 0, es550x_w16},
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D32, 0, es550x_w32},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, es550x_r8},
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, ES5506_RWUSER_EXT, es550x_ext_w},
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, ES5506_RWUSER_ESP, es550x_esp_w},
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D16, ES5506_RWUSER_GAIN, es550x_gain_w},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0, es550x_alloc_rom},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, es550x_write_rom},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, es550x_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"ES5506", "MAME", FCC_MAME,

	device_start_es550x,
	device_stop_es550x,
	device_reset_es550x,
	es550x_pcm_update,

	NULL,	// SetOptionBits
	es550x_set_mute_mask,
	NULL,	// SetPanning
	es550x_set_srchg_cb,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && ! (devCfg->flags & ES5506_FLAG_ES5506))
		return "ES5505";
	return "ES5506";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 32;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_ES5506 =
{
	DEVID_ES5506,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};


/**********************************************************************************************

     CONSTANTS

***********************************************************************************************/

#define FINE_FILTER_BIT     16
#define FILTER_BIT          12
#define FILTER_SHIFT        (FINE_FILTER_BIT - FILTER_BIT)

#define ULAW_MAXBITS        8

#define VOLUME_ACC_BIT      20
#define ADDRESS_FRAC_BIT    11

#define CONTROL_BS1         0x8000
#define CONTROL_BS0         0x4000
#define CONTROL_CMPD        0x2000
#define CONTROL_CA2         0x1000
#define CONTROL_CA1         0x0800
#define CONTROL_CA0         0x0400
#define CONTROL_LP4         0x0200
#define CONTROL_LP3         0x0100
#define CONTROL_IRQ         0x0080
#define CONTROL_DIR         0x0040
#define CONTROL_IRQE        0x0020
#define CONTROL_BLE         0x0010
#define CONTROL_LPE         0x0008
#define CONTROL_LEI         0x0004
#define CONTROL_STOP1       0x0002
#define CONTROL_STOP0       0x0001

#define CONTROL_LOOPMASK    (CONTROL_BLE | CONTROL_LPE)
#define CONTROL_STOPMASK    (CONTROL_STOP1 | CONTROL_STOP0)

#define LP3                 1
#define LP4                 2
#define LP_MASK             (LP3 | LP4)

#define MAX_REGIONS         4
#define MAX_OUTCHNS         12  // 6 stereo pairs
#define ESP_INPUTS          6

// mixer channel IDs for es550x_gain_w
#define GAINCH_ESP_L        0x10
#define GAINCH_ESP_R        0x11
#define GAINCH_DIRECT_L     0x12
#define GAINCH_DIRECT_R     0x13

#define ROUTE_NONE          0xFF


typedef struct
{
	// external state
	UINT32  control;    // control register
	UINT64  freqcount;  // frequency count register
	UINT64  start;      // start register
	UINT32  lvol;       // left volume register
	UINT64  end;        // end register
	UINT32  lvramp;     // left volume ramp register
	UINT64  accum;      // accumulator register
	UINT32  rvol;       // right volume register
	UINT32  rvramp;     // right volume ramp register
	UINT32  ecount;     // envelope count register
	UINT32  k2;         // k2 register
	UINT32  k2ramp;     // k2 ramp register
	UINT32  k1;         // k1 register
	UINT32  k1ramp;     // k1 ramp register
	INT32   o4n1;       // filter storage O4(n-1)
	INT32   o3n1;       // filter storage O3(n-1)
	INT32   o3n2;       // filter storage O3(n-2)
	INT32   o2n1;       // filter storage O2(n-1)
	INT32   o2n2;       // filter storage O2(n-2)
	INT32   o1n1;       // filter storage O1(n-1)
	UINT32  exbank;     // external address bank (in words)

	// internal state
	UINT8   index;      // index of this voice
	UINT8   filtcount;  // filter count
	UINT8   Muted;
} ES550X_VOICE;

typedef struct
{
	UINT16* base;       // sample data (16-bit words, host endianness)
	UINT32  size;       // size in words
	UINT32  mask;       // size-1 if size is a power of 2, else 0
	UINT32  window;     // window offset in words (added to every access)
	UINT8   alias;      // region index to read from (identity by default)
} ES550X_REGION;

typedef struct
{
	DEV_DATA _devData;

	UINT8   sndtype;            // 0 = ES5505, 1 = ES5506
	UINT32  master_clock;
	UINT32  sample_rate;
	INT8    address_acc_shift;
	UINT64  address_acc_mask;
	INT8    volume_shift;
	INT8    volume_acc_shift;

	UINT8   current_page;       // current register page
	UINT8   active_voices;      // number of active voices
	UINT16  mode;               // MODE register
	UINT8   irqv;               // IRQV register
	UINT8   auto_ack_irq;       // acknowledge IRQs immediately (no host reads in VGM)

	ES550X_VOICE voice[32];

	INT16   ulaw_lookup[1 << ULAW_MAXBITS];
	UINT32  volume_lookup[1 << 12];

	ES550X_REGION region[MAX_REGIONS];
	UINT32  pending_romsize;    // set by es550x_alloc_rom, applied by the next es550x_write_rom

	// ES5506 specific registers
	UINT32  write_latch;
	UINT32  read_latch;
	UINT8   wst;
	UINT8   wend;
	UINT8   lrend;

	// output mixing
	UINT8   channels;           // number of stereo output pairs (1..6)
	INT32   chn_gain[MAX_OUTCHNS];  // 8.8 fixed point gain per output channel
	UINT8   chn_direct[MAX_OUTCHNS];// output channel is mixed directly to the stereo output
	INT32   direct_gain[2];     // 8.8 gain for the direct mix (L/R)

	// ESP (ES5510) effects processor
	ES5510* esp;
	UINT8   esp_route_in[ESP_INPUTS];   // output channel feeding ESP serial input i (ROUTE_NONE = none)
	UINT8   esp_route_out[2];           // stereo mix target of ESP serial outputs 6/7 (0 = L, 1 = R)
	INT32   esp_gain[2];                // 8.8 gain for ESP outputs
	UINT8   esp_dram_log2;              // DRAM size (log2 words), 0 = default

	DEVCB_SRATE_CHG SmpRateFunc;
	void* SmpRateData;
} ES550XChip;


/**********************************************************************************************

     helpers

***********************************************************************************************/

INLINE INT64 lshift_signed64(INT64 val, INT8 shift)
{
	return (shift >= 0) ? (INT64)((UINT64)val << shift) : (val >> (-shift));
}

INLINE UINT64 lshift_unsigned64(UINT64 val, INT8 shift)
{
	return (shift >= 0) ? (val << shift) : (val >> (-shift));
}

INLINE UINT64 rshift_unsigned64(UINT64 val, INT8 shift)
{
	return (shift >= 0) ? (val >> shift) : (val << (-shift));
}

INLINE INT32 sext32(UINT32 val, UINT8 bits)
{
	UINT32 sign = 1U << (bits - 1);
	return (INT32)((val & (sign - 1)) ^ sign) - (INT32)sign;
}

INLINE UINT64 get_volume(ES550XChip* chip, UINT32 volume)
{
	return chip->volume_lookup[volume >> chip->volume_shift];
}

INLINE UINT64 get_address_acc_shifted_val(ES550XChip* chip, UINT64 val, int bias)
{
	return lshift_unsigned64(val, chip->address_acc_shift - bias);
}

INLINE UINT64 get_address_acc_res(ES550XChip* chip, UINT64 val, int bias)
{
	return rshift_unsigned64(val, chip->address_acc_shift - bias);
}

INLINE UINT64 get_integer_addr(ES550XChip* chip, UINT64 accum, INT32 bias)
{
	return ((accum + ((UINT64)bias << ADDRESS_FRAC_BIT)) & chip->address_acc_mask) >> ADDRESS_FRAC_BIT;
}

INLINE INT64 get_sample(ES550XChip* chip, INT32 sample, UINT32 volume)
{
	return lshift_signed64((INT64)sample * (INT64)get_volume(chip, volume), -chip->volume_acc_shift);
}

INLINE UINT32 get_bank(ES550XChip* chip, UINT32 control)
{
	return chip->sndtype ? ((control >> 14) & 3) : ((control >> 2) & 1);
}

INLINE UINT32 get_ca(ES550XChip* chip, UINT32 control)
{
	return chip->sndtype ? ((control >> 10) & 7) : ((control >> 8) & 3);
}

INLINE UINT32 get_lp(ES550XChip* chip, UINT32 control)
{
	return chip->sndtype ? ((control >> 8) & LP_MASK) : ((control >> 10) & LP_MASK);
}

INLINE UINT16 read_sample(ES550XChip* chip, ES550X_VOICE* voice, UINT64 addr)
{
	ES550X_REGION* rgn = &chip->region[get_bank(chip, voice->control)];
	UINT64 index;

	rgn = &chip->region[rgn->alias & 3];
	if (rgn->base == NULL)
		return 0;
	index = addr + voice->exbank + rgn->window;
	if (rgn->mask)
		return rgn->base[index & rgn->mask];
	if (index >= rgn->size)
		return 0;
	return rgn->base[index];
}


/**********************************************************************************************

     update_irq_state -- update the IRQ state

***********************************************************************************************/

static void update_internal_irq_state(ES550XChip* chip)
{
	// Host (cpu) has just read the voice interrupt vector (voice IRQ ack).
	chip->irqv = 0x80;
}


/**********************************************************************************************

     compute_tables -- compute static tables

***********************************************************************************************/

static void compute_tables(ES550XChip* chip, UINT32 total_volume_bit, UINT32 exponent_bit, UINT32 mantissa_bit)
{
	UINT32 i;
	UINT32 volume_bit, volume_len;
	UINT32 exponent_shift, exponent_mask;
	UINT32 mantissa_len, mantissa_mask, mantissa_shift;

	// generate ulaw lookup table
	for (i = 0; i < (1 << ULAW_MAXBITS); i++)
	{
		const UINT16 rawval = (i << (16 - ULAW_MAXBITS)) | (1 << (15 - ULAW_MAXBITS));
		const UINT8 exponent = rawval >> 13;
		UINT32 mantissa = (rawval << 3) & 0xffff;

		if (exponent == 0)
			chip->ulaw_lookup[i] = (INT16)mantissa >> 7;
		else
		{
			mantissa = (mantissa >> 1) | (~mantissa & 0x8000);
			chip->ulaw_lookup[i] = (INT16)mantissa >> (7 - exponent);
		}
	}

	volume_bit = (exponent_bit + mantissa_bit);
	chip->volume_shift = total_volume_bit - volume_bit;
	volume_len = 1 << volume_bit;

	// generate volume lookup table
	exponent_shift = 1 << exponent_bit;
	exponent_mask = exponent_shift - 1;

	mantissa_len = (1 << mantissa_bit);
	mantissa_mask = (mantissa_len - 1);
	mantissa_shift = exponent_shift - mantissa_bit - 1;

	for (i = 0; i < volume_len; i++)
	{
		const UINT32 exponent = (i >> mantissa_bit) & exponent_mask;
		const UINT32 mantissa = (i & mantissa_mask) | mantissa_len;

		chip->volume_lookup[i] = (mantissa << mantissa_shift) >> (exponent_shift - exponent);
	}
	chip->volume_acc_shift = (16 + exponent_mask) - VOLUME_ACC_BIT;

	// init the voices
	for (i = 0; i < 32; i++)
	{
		chip->voice[i].index = i;
		chip->voice[i].control = CONTROL_STOPMASK;
		chip->voice[i].lvol = (1 << (total_volume_bit - 1));
		chip->voice[i].rvol = (1 << (total_volume_bit - 1));
	}
}


/**********************************************************************************************

     get_accum_mask -- get address accumulator mask

***********************************************************************************************/

static void get_accum_mask(ES550XChip* chip, UINT32 address_integer, UINT32 address_frac)
{
	chip->address_acc_shift = ADDRESS_FRAC_BIT - address_frac;
	chip->address_acc_mask = lshift_unsigned64(((((UINT64)1 << address_integer) - 1) << address_frac) | ((1 << address_frac) - 1), chip->address_acc_shift);
	if (chip->address_acc_shift > 0)
		chip->address_acc_mask |= ((1 << chip->address_acc_shift) - 1);
}


/**********************************************************************************************

     interpolate -- interpolate between two samples

***********************************************************************************************/

INLINE INT32 interpolate(ES550XChip* chip, INT32 sample1, INT32 sample2, UINT64 accum)
{
	const UINT32 shifted = 1 << ADDRESS_FRAC_BIT;
	const UINT32 mask = shifted - 1;
	accum &= mask & chip->address_acc_mask;
	return (sample1 * (INT32)(shifted - accum) +
			sample2 * (INT32)(accum)) >> ADDRESS_FRAC_BIT;
}


/**********************************************************************************************

     apply_filters -- apply the 4-pole digital filter to the sample

***********************************************************************************************/

INLINE INT32 apply_lowpass(INT32 out, INT32 cutoff, INT32 in)
{
	return ((INT32)(cutoff >> FILTER_SHIFT) * (out - in) / (1 << FILTER_BIT)) + in;
}

INLINE INT32 apply_highpass(INT32 out, INT32 cutoff, INT32 in, INT32 prev)
{
	return out - prev + ((INT32)(cutoff >> FILTER_SHIFT) * in) / (1 << (FILTER_BIT + 1)) + in / 2;
}

INLINE void apply_filters(ES550XChip* chip, ES550X_VOICE* voice, INT32* sample)
{
	INT32 s = *sample;

	// pole 1 is always low-pass using K1
	s = apply_lowpass(s, voice->k1, voice->o1n1);
	voice->o1n1 = s;

	// pole 2 is always low-pass using K1
	s = apply_lowpass(s, voice->k1, voice->o2n1);
	voice->o2n2 = voice->o2n1;
	voice->o2n1 = s;

	// remaining poles depend on the current filter setting
	switch (get_lp(chip, voice->control))
	{
		case 0:
			// pole 3 is high-pass using K2
			s = apply_highpass(s, voice->k2, voice->o3n1, voice->o2n2);
			voice->o3n2 = voice->o3n1;
			voice->o3n1 = s;

			// pole 4 is high-pass using K2
			s = apply_highpass(s, voice->k2, voice->o4n1, voice->o3n2);
			voice->o4n1 = s;
			break;

		case LP3:
			// pole 3 is low-pass using K1
			s = apply_lowpass(s, voice->k1, voice->o3n1);
			voice->o3n2 = voice->o3n1;
			voice->o3n1 = s;

			// pole 4 is high-pass using K2
			s = apply_highpass(s, voice->k2, voice->o4n1, voice->o3n2);
			voice->o4n1 = s;
			break;

		case LP4:
			// pole 3 is low-pass using K2
			s = apply_lowpass(s, voice->k2, voice->o3n1);
			voice->o3n2 = voice->o3n1;
			voice->o3n1 = s;

			// pole 4 is low-pass using K2
			s = apply_lowpass(s, voice->k2, voice->o4n1);
			voice->o4n1 = s;
			break;

		case LP3 | LP4:
			// pole 3 is low-pass using K1
			s = apply_lowpass(s, voice->k1, voice->o3n1);
			voice->o3n2 = voice->o3n1;
			voice->o3n1 = s;

			// pole 4 is low-pass using K2
			s = apply_lowpass(s, voice->k2, voice->o4n1);
			voice->o4n1 = s;
			break;
	}
	*sample = s;
}


/**********************************************************************************************

     update_envelopes -- update the envelopes

***********************************************************************************************/

INLINE void update_envelopes(ES550XChip* chip, ES550X_VOICE* voice)
{
	const UINT32 volume_max = (1 << 16) - 1;

	if (! chip->sndtype)
	{
		// no envelopes in ES5505
		voice->ecount = 0;
		return;
	}

	// decrement the envelope counter
	voice->ecount--;

	// ramp left volume
	if (voice->lvramp)
	{
		voice->lvol += (INT8)voice->lvramp;
		if ((INT32)voice->lvol < 0) voice->lvol = 0;
		else if (voice->lvol > volume_max) voice->lvol = volume_max;
	}

	// ramp right volume
	if (voice->rvramp)
	{
		voice->rvol += (INT8)voice->rvramp;
		if ((INT32)voice->rvol < 0) voice->rvol = 0;
		else if (voice->rvol > volume_max) voice->rvol = volume_max;
	}

	// ramp k1 filter constant
	if (voice->k1ramp && ((INT32)voice->k1ramp >= 0 || !(voice->filtcount & 7)))
	{
		voice->k1 += (INT8)voice->k1ramp;
		if ((INT32)voice->k1 < 0) voice->k1 = 0;
		else if (voice->k1 > 0xffff) voice->k1 = 0xffff;
	}

	// ramp k2 filter constant
	if (voice->k2ramp && ((INT32)voice->k2ramp >= 0 || !(voice->filtcount & 7)))
	{
		voice->k2 += (INT8)voice->k2ramp;
		if ((INT32)voice->k2 < 0) voice->k2 = 0;
		else if (voice->k2 > 0xffff) voice->k2 = 0xffff;
	}

	// update the filter constant counter
	voice->filtcount++;
}


/**********************************************************************************************

     check_for_end_forward
     check_for_end_reverse -- check for loop end and loop appropriately

***********************************************************************************************/

INLINE void check_for_end_forward(ES550XChip* chip, ES550X_VOICE* voice, UINT64* accum)
{
	if (chip->sndtype)
	{
		// are we past the end?
		if (*accum > voice->end && !(voice->control & CONTROL_LEI))
		{
			// generate interrupt if required
			if (voice->control & CONTROL_IRQE)
				voice->control |= CONTROL_IRQ;

			// handle the different types of looping
			switch (voice->control & CONTROL_LOOPMASK)
			{
				// non-looping
				case 0:
					voice->control |= CONTROL_STOP0;
					break;

				// uni-directional looping
				case CONTROL_LPE:
					*accum = (voice->start + (*accum - voice->end)) & chip->address_acc_mask;
					break;

				// trans-wave looping
				case CONTROL_BLE:
					*accum = (voice->start + (*accum - voice->end)) & chip->address_acc_mask;
					voice->control = (voice->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
					break;

				// bi-directional looping
				case CONTROL_LPE | CONTROL_BLE:
					*accum = (voice->end - (*accum - voice->end)) & chip->address_acc_mask;
					voice->control ^= CONTROL_DIR;
					break;
			}
		}
	}
	else
	{
		// ES5505 : BLE is ignored when LPE = 0
		if (*accum > voice->end)
		{
			if (voice->control & CONTROL_IRQE)
				voice->control |= CONTROL_IRQ;

			switch (voice->control & CONTROL_LOOPMASK)
			{
				case 0:
				case CONTROL_BLE:
					voice->control |= CONTROL_STOP0;
					break;

				case CONTROL_LPE:
					*accum = (voice->start + (*accum - voice->end)) & chip->address_acc_mask;
					break;

				case CONTROL_LPE | CONTROL_BLE:
					*accum = (voice->end - (*accum - voice->end)) & chip->address_acc_mask;
					voice->control ^= CONTROL_DIR;
					break;
			}
		}
	}
}

INLINE void check_for_end_reverse(ES550XChip* chip, ES550X_VOICE* voice, UINT64* accum)
{
	if (chip->sndtype)
	{
		if (*accum < voice->start && !(voice->control & CONTROL_LEI))
		{
			if (voice->control & CONTROL_IRQE)
				voice->control |= CONTROL_IRQ;

			switch (voice->control & CONTROL_LOOPMASK)
			{
				case 0:
					voice->control |= CONTROL_STOP0;
					break;

				case CONTROL_LPE:
					*accum = (voice->end - (voice->start - *accum)) & chip->address_acc_mask;
					break;

				case CONTROL_BLE:
					*accum = (voice->end - (voice->start - *accum)) & chip->address_acc_mask;
					voice->control = (voice->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
					break;

				case CONTROL_LPE | CONTROL_BLE:
					*accum = (voice->start + (voice->start - *accum)) & chip->address_acc_mask;
					voice->control ^= CONTROL_DIR;
					break;
			}
		}
	}
	else
	{
		if (*accum < voice->start)
		{
			if (voice->control & CONTROL_IRQE)
				voice->control |= CONTROL_IRQ;

			switch (voice->control & CONTROL_LOOPMASK)
			{
				case 0:
				case CONTROL_BLE:
					voice->control |= CONTROL_STOP0;
					break;

				case CONTROL_LPE:
					*accum = (voice->end - (voice->start - *accum)) & chip->address_acc_mask;
					break;

				case CONTROL_LPE | CONTROL_BLE:
					*accum = (voice->start + (voice->start - *accum)) & chip->address_acc_mask;
					voice->control ^= CONTROL_DIR;
					break;
			}
		}
	}
}


/**********************************************************************************************

     generate_voice -- u-law / PCM sample decoding routine

***********************************************************************************************/

static void generate_voice(ES550XChip* chip, ES550X_VOICE* voice, INT32* dest, UINT8 ulaw)
{
	const UINT32 freqcount = (UINT32)voice->freqcount;
	UINT64 accum = voice->accum & chip->address_acc_mask;

	if (!(voice->control & CONTROL_STOPMASK))
	{
		INT32 val1, val2;

		// fetch two samples
		if (ulaw)
		{
			val1 = read_sample(chip, voice, get_integer_addr(chip, accum, 0));
			val2 = read_sample(chip, voice, get_integer_addr(chip, accum, 1));

			// decompress u-law
			val1 = chip->ulaw_lookup[val1 >> (16 - ULAW_MAXBITS)];
			val2 = chip->ulaw_lookup[val2 >> (16 - ULAW_MAXBITS)];
		}
		else
		{
			val1 = (INT16)read_sample(chip, voice, get_integer_addr(chip, accum, 0));
			val2 = (INT16)read_sample(chip, voice, get_integer_addr(chip, accum, 1));
		}

		// interpolate
		val1 = interpolate(chip, val1, val2, accum);
		if (!(voice->control & CONTROL_DIR))
			accum = (accum + freqcount) & chip->address_acc_mask;
		else
			accum = (accum - freqcount) & chip->address_acc_mask;

		// apply filters
		apply_filters(chip, voice, &val1);

		// update filters/volumes
		if (voice->ecount != 0)
			update_envelopes(chip, voice);

		// apply volumes and add
		if (! voice->Muted)
		{
			dest[0] += (INT32)get_sample(chip, val1, voice->lvol);
			dest[1] += (INT32)get_sample(chip, val1, voice->rvol);
		}

		// check for loop end
		if (!(voice->control & CONTROL_DIR))
			check_for_end_forward(chip, voice, &accum);
		else
			check_for_end_reverse(chip, voice, &accum);
	}
	else
	{
		// if we stopped, process any additional envelope
		if (voice->ecount != 0)
			update_envelopes(chip, voice);
	}

	voice->accum = accum;
}


/**********************************************************************************************

     generate_irq -- general interrupt handling routine

***********************************************************************************************/

INLINE void generate_irq(ES550XChip* chip, ES550X_VOICE* voice, int v)
{
	// does this voice have it's IRQ bit raised?
	if (voice->control & CONTROL_IRQ)
	{
		// only update voice vector if existing IRQ is acked by host
		if (chip->irqv & 0x80)
		{
			// latch voice number into vector, and set high bit low
			chip->irqv = v & 0x1f;

			// take down IRQ bit on voice
			voice->control &= ~CONTROL_IRQ;

			// a VGM contains no host reads - emulate an immediate acknowledge if requested
			if (chip->auto_ack_irq)
				update_internal_irq_state(chip);
		}
	}
}


/**********************************************************************************************

     es550x_pcm_update -- generate samples, mix output pairs, run the ESP

***********************************************************************************************/

INLINE INT32 clamp_int(INT32 val, INT32 limit)
{
	if (val >= limit)
		return limit - 1;
	if (val < -limit)
		return -limit;
	return val;
}

static void es550x_pcm_update(void *param, UINT32 samples, DEV_SMPL **outputs)
{
	ES550XChip* chip = (ES550XChip*)param;
	UINT32 sampindex;
	DEV_SMPL* outL = outputs[0];
	DEV_SMPL* outR = outputs[1];

	for (sampindex = 0; sampindex < samples; sampindex++)
	{
		INT32 cursample[MAX_OUTCHNS];
		INT32 mixL = 0, mixR = 0;
		int v, c;

		memset(cursample, 0, sizeof(cursample));

		// loop over voices
		for (v = 0; v <= chip->active_voices; v++)
		{
			ES550X_VOICE* voice = &chip->voice[v];
			UINT32 voice_channel, channel;

			// special case: if end == start, stop the voice (ES5506 only, the ES5505
			// in Ensoniq synthesizers relies on such voices running)
			if (chip->sndtype && voice->start == voice->end)
				voice->control |= CONTROL_STOP0;

			voice_channel = get_ca(chip, voice->control);
			channel = voice_channel % chip->channels;

			// generate from the appropriate source (no compressed sample support on ES5505)
			generate_voice(chip, voice, &cursample[channel << 1], chip->sndtype && (voice->control & CONTROL_CMPD));

			// does this voice have it's IRQ bit raised?
			generate_irq(chip, voice, v);
		}

		// generated samples are 20-bit signed, range is [-2^19 .. 2^19-1]
		// scale to 16-bit and apply the per-channel gains
		for (c = 0; c < MAX_OUTCHNS; c++)
		{
			INT32 s = clamp_int(cursample[c], 1 << 19) >> 4;
			s = (INT32)(((INT64)s * chip->chn_gain[c]) >> 8);
			cursample[c] = s;
			if (chip->chn_direct[c])
			{
				if (c & 1)
					mixR += s;
				else
					mixL += s;
			}
		}
		mixL = (INT32)(((INT64)mixL * chip->direct_gain[0]) >> 8);
		mixR = (INT32)(((INT64)mixR * chip->direct_gain[1]) >> 8);

		// effects processor
		if (chip->esp != NULL)
		{
			INT32 espOut[2];
			for (c = 0; c < ESP_INPUTS; c++)
			{
				UINT8 src = chip->esp_route_in[c];
				INT16 in = (src < MAX_OUTCHNS) ? (INT16)clamp_int(cursample[src], 0x8000) : 0;
				es5510_ser_w(chip->esp, (UINT8)c, in);
			}
			es5510_run_once(chip->esp);
			espOut[0] = (INT32)(((INT64)es5510_ser_r(chip->esp, 6) * chip->esp_gain[0]) >> 8);
			espOut[1] = (INT32)(((INT64)es5510_ser_r(chip->esp, 7) * chip->esp_gain[1]) >> 8);
			for (c = 0; c < 2; c++)
			{
				if (chip->esp_route_out[c] & 1)
					mixR += espOut[c];
				else
					mixL += espOut[c];
			}
		}

		outL[sampindex] = mixL;
		outR[sampindex] = mixR;
	}
}


/**********************************************************************************************

     routing presets

***********************************************************************************************/

static void es550x_apply_route_preset(ES550XChip* chip, UINT8 preset)
{
	int c;

	for (c = 0; c < MAX_OUTCHNS; c++)
		chip->chn_direct[c] = 1;
	for (c = 0; c < ESP_INPUTS; c++)
		chip->esp_route_in[c] = ROUTE_NONE;
	chip->esp_route_out[0] = 0;
	chip->esp_route_out[1] = 1;

	switch (preset)
	{
		case 0:	// Taito Ensoniq board (esq_5505_5510_pump): pair 0 -> "aux" (direct),
				// pairs 1..3 -> ESP serial inputs 0..5, ESP serial outputs 6/7 -> L/R
			for (c = 2; c < 8; c++)
			{
				chip->chn_direct[c] = 0;
				chip->esp_route_in[c - 2] = (UINT8)c;
			}
			break;
		case 1:	// Ensoniq synthesizers (VFX/SD-1/ASR): pairs 0..2 through the ESP, nothing direct
			for (c = 0; c < 6; c++)
			{
				chip->chn_direct[c] = 0;
				chip->esp_route_in[c] = (UINT8)c;
			}
			break;
		default:	// explicit routing: everything direct until told otherwise
			break;
	}
}

static void es550x_ensure_esp(ES550XChip* chip)
{
	if (chip->esp != NULL)
		return;
	chip->esp = es5510_create(chip->esp_dram_log2 ? (1U << chip->esp_dram_log2) : 0);
	if (chip->esp == NULL)
		return;
	es5510_reset(chip->esp);
	es5510_set_halt_line(chip->esp, 1);	// ESPHALT is asserted at power-up
	es550x_apply_route_preset(chip, 0);
}


/**********************************************************************************************

     device start / stop / reset

***********************************************************************************************/

static UINT8 device_start_es550x(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	ES550XChip* chip;
	int c;

	chip = (ES550XChip*)calloc(1, sizeof(ES550XChip));
	if (chip == NULL)
		return 0xFF;

	chip->sndtype = (cfg->flags & ES5506_FLAG_ES5506) ? 1 : 0;
	chip->master_clock = cfg->clock;
	chip->channels = cfg->flags & ES5506_FLAG_CHNMASK;
	if (chip->channels < 1)
		chip->channels = 1;
	if (chip->channels > (chip->sndtype ? 6 : 4))
		chip->channels = chip->sndtype ? 6 : 4;
	chip->auto_ack_irq = 1;

	for (c = 0; c < MAX_REGIONS; c++)
		chip->region[c].alias = (UINT8)c;
	for (c = 0; c < MAX_OUTCHNS; c++)
		chip->chn_gain[c] = 0x100;
	chip->direct_gain[0] = chip->direct_gain[1] = 0x100;
	chip->esp_gain[0] = chip->esp_gain[1] = 0x100;
	es550x_apply_route_preset(chip, 0xFF);

	if (chip->sndtype)
	{
		compute_tables(chip, 16, 4, 8);	// 4 bit exponent, 8 bit mantissa
		get_accum_mask(chip, 21, 11);	// 21 bit integer and 11 bit fraction
	}
	else
	{
		compute_tables(chip, 8, 4, 4);	// 4 bit exponent, 4 bit mantissa
		get_accum_mask(chip, 20, 9);	// 20 bit integer and 9 bit fraction
	}

	chip->active_voices = 0x1f;
	chip->sample_rate = chip->master_clock / (16 * (chip->active_voices + 1));

	es550x_set_mute_mask(chip, 0x00000000);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, chip->sample_rate, &devDef);

	return 0x00;
}

static void device_stop_es550x(void *info)
{
	ES550XChip* chip = (ES550XChip*)info;
	int c;

	for (c = 0; c < MAX_REGIONS; c++)
		free(chip->region[c].base);
	if (chip->esp != NULL)
		es5510_destroy(chip->esp);
	free(chip);
}

static void device_reset_es550x(void *info)
{
	ES550XChip* chip = (ES550XChip*)info;
	int v;

	chip->current_page = 0;
	chip->irqv = 0x80;
	chip->write_latch = 0;
	chip->read_latch = 0;
	chip->wst = chip->wend = chip->lrend = 0;

	for (v = 0; v < 32; v++)
	{
		ES550X_VOICE* voice = &chip->voice[v];
		UINT8 muted = voice->Muted;
		memset(voice, 0, sizeof(ES550X_VOICE));
		voice->index = (UINT8)v;
		voice->control = CONTROL_STOPMASK;
		voice->lvol = (1 << ((chip->sndtype ? 16 : 8) - 1));
		voice->rvol = voice->lvol;
		voice->Muted = muted;
	}

	chip->active_voices = 0x1f;	// 32 voices at reset
	chip->mode = chip->sndtype ? 0x17 : 0;	// Dual, Slave, BCLK_EN high, WCLK_EN high, LRCLK_EN high
	chip->sample_rate = chip->master_clock / (16 * (chip->active_voices + 1));
	if (chip->SmpRateFunc != NULL)
		chip->SmpRateFunc(chip->SmpRateData, chip->sample_rate);

	if (chip->esp != NULL)
	{
		es5510_reset(chip->esp);
		es5510_set_halt_line(chip->esp, 1);
	}
}

static void es550x_update_sample_rate(ES550XChip* chip)
{
	chip->sample_rate = chip->master_clock / (16 * (chip->active_voices + 1));
	if (chip->SmpRateFunc != NULL)
		chip->SmpRateFunc(chip->SmpRateData, chip->sample_rate);
}


/**********************************************************************************************

     ES5506 register writes

***********************************************************************************************/

static void es5506_reg_write_low(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT32 data)
{
	switch (offset)
	{
		case 0x00/8:    // CR
			voice->control = data & 0xffff;
			break;

		case 0x08/8:    // FC
			voice->freqcount = get_address_acc_shifted_val(chip, data & 0x1ffff, 0);
			break;

		case 0x10/8:    // LVOL
			voice->lvol = data & 0xffff; // low 4 bit is used for finer envelope control
			break;

		case 0x18/8:    // LVRAMP
			voice->lvramp = (data & 0xff00) >> 8;
			break;

		case 0x20/8:    // RVOL
			voice->rvol = data & 0xffff; // low 4 bit is used for finer envelope control
			break;

		case 0x28/8:    // RVRAMP
			voice->rvramp = (data & 0xff00) >> 8;
			break;

		case 0x30/8:    // ECOUNT
			voice->ecount = data & 0x1ff;
			voice->filtcount = 0;
			break;

		case 0x38/8:    // K2
			voice->k2 = data & 0xffff; // low 4 bit is used for finer envelope control
			break;

		case 0x40/8:    // K2RAMP
			voice->k2ramp = ((data & 0xff00) >> 8) | ((data & 0x0001) << 31);
			break;

		case 0x48/8:    // K1
			voice->k1 = data & 0xffff; // low 4 bit is used for finer envelope control
			break;

		case 0x50/8:    // K1RAMP
			voice->k1ramp = ((data & 0xff00) >> 8) | ((data & 0x0001) << 31);
			break;

		case 0x58/8:    // ACTV
			chip->active_voices = data & 0x1f;
			es550x_update_sample_rate(chip);
			break;

		case 0x60/8:    // MODE
			chip->mode = data & 0x1f; // MODE1[4], MODE0[3], BCLK_EN[2], WCLK_EN[1], LRCLK_EN[0]
			break;

		case 0x68/8:    // PAR - read only
		case 0x70/8:    // IRQV - read only
			break;

		case 0x78/8:    // PAGE
			chip->current_page = data & 0x7f;
			break;
	}
}

static void es5506_reg_write_high(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT32 data)
{
	switch (offset)
	{
		case 0x00/8:    // CR
			voice->control = data & 0xffff;
			break;

		case 0x08/8:    // START
			voice->start = get_address_acc_shifted_val(chip, data & 0xfffff800, 0);
			break;

		case 0x10/8:    // END
			voice->end = get_address_acc_shifted_val(chip, data & 0xffffff80, 0);
			break;

		case 0x18/8:    // ACCUM
			voice->accum = get_address_acc_shifted_val(chip, data, 0);
			break;

		case 0x20/8:    // O4(n-1)
			voice->o4n1 = sext32(data, 18);
			break;

		case 0x28/8:    // O3(n-1)
			voice->o3n1 = sext32(data, 18);
			break;

		case 0x30/8:    // O3(n-2)
			voice->o3n2 = sext32(data, 18);
			break;

		case 0x38/8:    // O2(n-1)
			voice->o2n1 = sext32(data, 18);
			break;

		case 0x40/8:    // O2(n-2)
			voice->o2n2 = sext32(data, 18);
			break;

		case 0x48/8:    // O1(n-1)
			voice->o1n1 = sext32(data, 18);
			break;

		case 0x50/8:    // W_ST
			chip->wst = data & 0x7f;
			break;

		case 0x58/8:    // W_END
			chip->wend = data & 0x7f;
			break;

		case 0x60/8:    // LR_END
			chip->lrend = data & 0x7f;
			break;

		case 0x68/8:    // PAR - read only
		case 0x70/8:    // IRQV - read only
			break;

		case 0x78/8:    // PAGE
			chip->current_page = data & 0x7f;
			break;
	}
}

static void es5506_reg_write_test(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT32 data)
{
	switch (offset)
	{
		case 0x78/8:    // PAGE
			chip->current_page = data & 0x7f;
			break;
		default:        // channel test writes, EMPTY, PAR, IRQV
			break;
	}
}

static void es5506_reg_write(ES550XChip* chip, UINT8 reg, UINT32 data)
{
	ES550X_VOICE* voice = &chip->voice[chip->current_page & 0x1f];

	// switch off the page and register
	if (chip->current_page < 0x20)
		es5506_reg_write_low(chip, voice, reg, data);
	else if (chip->current_page < 0x40)
		es5506_reg_write_high(chip, voice, reg, data);
	else
		es5506_reg_write_test(chip, voice, reg, data);
}

static void es5506_w(ES550XChip* chip, UINT8 offset, UINT8 data)
{
	int shift = 8 * (offset & 3);

	// accumulate the data
	chip->write_latch = (chip->write_latch & ~(0xff000000 >> shift)) | ((UINT32)data << (24 - shift));

	// wait for a write to complete
	if (shift != 24)
		return;

	es5506_reg_write(chip, offset / 4, chip->write_latch);

	// clear the write latch when done
	chip->write_latch = 0;
}


/**********************************************************************************************

     ES5505 register writes

***********************************************************************************************/

#define ACCESSING_BITS_0_7    (mem_mask & 0x00ff)
#define ACCESSING_BITS_8_15   (mem_mask & 0xff00)

static void es5505_reg_write_low(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT16 data, UINT16 mem_mask)
{
	switch (offset)
	{
		case 0x00:  // CR
			voice->control |= 0xf000; // bit 15-12 always 1
			if (ACCESSING_BITS_0_7)
			{
				voice->control &= ~0x00ff;
				voice->control |= (data & 0x00ff);
			}
			if (ACCESSING_BITS_8_15)
				voice->control = (voice->control & ~0x0f00) | (data & 0x0f00);
			break;

		case 0x01:  // FC
			if (ACCESSING_BITS_0_7)
				voice->freqcount = (voice->freqcount & ~get_address_acc_shifted_val(chip, 0x00fe, 1)) | (get_address_acc_shifted_val(chip, (data & 0x00fe), 1));
			if (ACCESSING_BITS_8_15)
				voice->freqcount = (voice->freqcount & ~get_address_acc_shifted_val(chip, 0xff00, 1)) | (get_address_acc_shifted_val(chip, (data & 0xff00), 1));
			break;

		case 0x02:  // STRT (hi)
			if (ACCESSING_BITS_0_7)
				voice->start = (voice->start & ~get_address_acc_shifted_val(chip, 0x00ff0000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x00ff) << 16, 0));
			if (ACCESSING_BITS_8_15)
				voice->start = (voice->start & ~get_address_acc_shifted_val(chip, 0x1f000000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x1f00) << 16, 0));
			break;

		case 0x03:  // STRT (lo)
			if (ACCESSING_BITS_0_7)
				voice->start = (voice->start & ~get_address_acc_shifted_val(chip, 0x000000e0, 0)) | (get_address_acc_shifted_val(chip, data & 0x00e0, 0));
			if (ACCESSING_BITS_8_15)
				voice->start = (voice->start & ~get_address_acc_shifted_val(chip, 0x0000ff00, 0)) | (get_address_acc_shifted_val(chip, data & 0xff00, 0));
			break;

		case 0x04:  // END (hi)
			if (ACCESSING_BITS_0_7)
				voice->end = (voice->end & ~get_address_acc_shifted_val(chip, 0x00ff0000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x00ff) << 16, 0));
			if (ACCESSING_BITS_8_15)
				voice->end = (voice->end & ~get_address_acc_shifted_val(chip, 0x1f000000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x1f00) << 16, 0));
			break;

		case 0x05:  // END (lo)
			if (ACCESSING_BITS_0_7)
				voice->end = (voice->end & ~get_address_acc_shifted_val(chip, 0x000000e0, 0)) | (get_address_acc_shifted_val(chip, data & 0x00e0, 0));
			if (ACCESSING_BITS_8_15)
				voice->end = (voice->end & ~get_address_acc_shifted_val(chip, 0x0000ff00, 0)) | (get_address_acc_shifted_val(chip, data & 0xff00, 0));
			break;

		case 0x06:  // K2
			if (ACCESSING_BITS_0_7)
				voice->k2 = (voice->k2 & ~0x00f0) | (data & 0x00f0);
			if (ACCESSING_BITS_8_15)
				voice->k2 = (voice->k2 & ~0xff00) | (data & 0xff00);
			break;

		case 0x07:  // K1
			if (ACCESSING_BITS_0_7)
				voice->k1 = (voice->k1 & ~0x00f0) | (data & 0x00f0);
			if (ACCESSING_BITS_8_15)
				voice->k1 = (voice->k1 & ~0xff00) | (data & 0xff00);
			break;

		case 0x08:  // LVOL
			if (ACCESSING_BITS_8_15)
				voice->lvol = (voice->lvol & ~0xff) | ((data & 0xff00) >> 8);
			break;

		case 0x09:  // RVOL
			if (ACCESSING_BITS_8_15)
				voice->rvol = (voice->rvol & ~0xff) | ((data & 0xff00) >> 8);
			break;

		case 0x0a:  // ACC (hi)
			if (ACCESSING_BITS_0_7)
				voice->accum = (voice->accum & ~get_address_acc_shifted_val(chip, 0x00ff0000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x00ff) << 16, 0));
			if (ACCESSING_BITS_8_15)
				voice->accum = (voice->accum & ~get_address_acc_shifted_val(chip, 0x1f000000, 0)) | (get_address_acc_shifted_val(chip, (UINT32)(data & 0x1f00) << 16, 0));
			break;

		case 0x0b:  // ACC (lo)
			if (ACCESSING_BITS_0_7)
				voice->accum = (voice->accum & ~get_address_acc_shifted_val(chip, 0x000000ff, 0)) | (get_address_acc_shifted_val(chip, data & 0x00ff, 0));
			if (ACCESSING_BITS_8_15)
				voice->accum = (voice->accum & ~get_address_acc_shifted_val(chip, 0x0000ff00, 0)) | (get_address_acc_shifted_val(chip, data & 0xff00, 0));
			break;

		case 0x0c:  // unused
			break;

		case 0x0d:  // ACT
			if (ACCESSING_BITS_0_7)
			{
				chip->active_voices = data & 0x1f;
				es550x_update_sample_rate(chip);
			}
			break;

		case 0x0e:  // IRQV - read only
			break;

		case 0x0f:  // PAGE
			if (ACCESSING_BITS_0_7)
				chip->current_page = data & 0x7f;
			break;
	}
}

static void es5505_reg_write_high(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT16 data, UINT16 mem_mask)
{
	switch (offset)
	{
		case 0x00:  // CR
			voice->control |= 0xf000; // bit 15-12 always 1
			if (ACCESSING_BITS_0_7)
				voice->control = (voice->control & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->control = (voice->control & ~0x0f00) | (data & 0x0f00);
			break;

		case 0x01:  // O4(n-1)
			if (ACCESSING_BITS_0_7)
				voice->o4n1 = (voice->o4n1 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o4n1 = (INT16)((voice->o4n1 & ~0xff00) | (data & 0xff00));
			break;

		case 0x02:  // O3(n-1)
			if (ACCESSING_BITS_0_7)
				voice->o3n1 = (voice->o3n1 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o3n1 = (INT16)((voice->o3n1 & ~0xff00) | (data & 0xff00));
			break;

		case 0x03:  // O3(n-2)
			if (ACCESSING_BITS_0_7)
				voice->o3n2 = (voice->o3n2 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o3n2 = (INT16)((voice->o3n2 & ~0xff00) | (data & 0xff00));
			break;

		case 0x04:  // O2(n-1)
			if (ACCESSING_BITS_0_7)
				voice->o2n1 = (voice->o2n1 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o2n1 = (INT16)((voice->o2n1 & ~0xff00) | (data & 0xff00));
			break;

		case 0x05:  // O2(n-2)
			if (ACCESSING_BITS_0_7)
				voice->o2n2 = (voice->o2n2 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o2n2 = (INT16)((voice->o2n2 & ~0xff00) | (data & 0xff00));
			break;

		case 0x06:  // O1(n-1)
			if (ACCESSING_BITS_0_7)
				voice->o1n1 = (voice->o1n1 & ~0x00ff) | (data & 0x00ff);
			if (ACCESSING_BITS_8_15)
				voice->o1n1 = (INT16)((voice->o1n1 & ~0xff00) | (data & 0xff00));
			break;

		case 0x0d:  // ACT
			if (ACCESSING_BITS_0_7)
			{
				chip->active_voices = data & 0x1f;
				es550x_update_sample_rate(chip);
			}
			break;

		case 0x0f:  // PAGE
			if (ACCESSING_BITS_0_7)
				chip->current_page = data & 0x7f;
			break;

		default:    // 0x07..0x0c unused, 0x0e IRQV read only
			break;
	}
}

static void es5505_reg_write_test(ES550XChip* chip, ES550X_VOICE* voice, UINT8 offset, UINT16 data, UINT16 mem_mask)
{
	switch (offset)
	{
		case 0x08:  // SERMODE
			chip->mode |= 0x7f8; // bit 10-3 always 1
			if (ACCESSING_BITS_8_15)
				chip->mode = (chip->mode & ~0xf800) | (data & 0xf800); // MSB[4:0] (unknown purpose)
			if (ACCESSING_BITS_0_7)
				chip->mode = (chip->mode & ~0x0007) | (data & 0x0007); // SONY/BB, TEST, A/D
			break;

		case 0x0d:  // ACT
			if (ACCESSING_BITS_0_7)
			{
				chip->active_voices = data & 0x1f;
				es550x_update_sample_rate(chip);
			}
			break;

		case 0x0f:  // PAGE
			if (ACCESSING_BITS_0_7)
				chip->current_page = data & 0x7f;
			break;

		default:    // channel test registers, PAR, IRQV
			break;
	}
}

static void es5505_w(ES550XChip* chip, UINT8 offset, UINT16 data, UINT16 mem_mask)
{
	ES550X_VOICE* voice = &chip->voice[chip->current_page & 0x1f];

	// switch off the page and register
	if (chip->current_page < 0x20)
		es5505_reg_write_low(chip, voice, offset, data, mem_mask);
	else if (chip->current_page < 0x40)
		es5505_reg_write_high(chip, voice, offset, data, mem_mask);
	else
		es5505_reg_write_test(chip, voice, offset, data, mem_mask);
}


/**********************************************************************************************

     VGM-facing write functions

***********************************************************************************************/

// 0xBE aa dd: byte write to byte offset aa
//   ES5506: 32-bit registers, 4 byte lanes per register (big endian), register = aa/4
//   ES5505: 16-bit registers, 2 byte lanes per register (big endian), register = aa/2
//   aa >= 0x40: legacy VGMPlay voice bank write (voice = aa & 0x1F, bank = dd << 20 words)
static void es550x_w8(void *info, UINT8 offset, UINT8 data)
{
	ES550XChip* chip = (ES550XChip*)info;

	if (offset < 0x40)
	{
		if (chip->sndtype)
			es5506_w(chip, offset, data);
		else
			es5505_w(chip, offset >> 1, (offset & 1) ? data : ((UINT16)data << 8), (offset & 1) ? 0x00ff : 0xff00);
	}
	else
	{
		chip->voice[offset & 0x1f].exbank = (UINT32)data << 20;
	}
}

// 0xD6 aa ddee: 16-bit write to byte offset aa (big endian data, i.e. byte aa|0 = dd, aa|1 = ee)
static void es550x_w16(void *info, UINT8 offset, UINT16 data)
{
	ES550XChip* chip = (ES550XChip*)info;

	if (offset < 0x40)
	{
		if (chip->sndtype)
		{
			es5506_w(chip, offset & ~1, (data & 0xFF00) >> 8);
			es5506_w(chip, offset | 1, (data & 0x00FF) >> 0);
		}
		else
		{
			es5505_w(chip, offset >> 1, data, 0xffff);
		}
	}
	else
	{
		chip->voice[offset & 0x1f].exbank = (UINT32)data << 20;
	}
}

// 0xE2 aa dddddddd: 32-bit write to register aa/4 of the current page (ES5506)
static void es550x_w32(void *info, UINT8 offset, UINT32 data)
{
	ES550XChip* chip = (ES550XChip*)info;

	if (offset >= 0x40)
	{
		chip->voice[offset & 0x1f].exbank = data;	// bank in words
		return;
	}
	if (chip->sndtype)
	{
		es5506_reg_write(chip, offset / 4, data);
		chip->write_latch = 0;
	}
	else
	{
		es5505_w(chip, offset >> 1, (UINT16)data, 0xffff);
	}
}

static UINT8 es550x_r8(void *info, UINT8 offset)
{
	ES550XChip* chip = (ES550XChip*)info;

	// only the IRQ vector / page registers are useful without host interaction
	if (chip->sndtype)
	{
		int shift = 8 * (offset & 3);
		if (shift != 0)
			return chip->read_latch >> (24 - shift);
		switch (offset / 4)
		{
			case 0x70/8:	// IRQV
				chip->read_latch = chip->irqv;
				update_internal_irq_state(chip);
				break;
			case 0x78/8:	// PAGE
				chip->read_latch = chip->current_page;
				break;
			default:
				chip->read_latch = 0;
				break;
		}
		return chip->read_latch >> 24;
	}
	else
	{
		UINT16 result = 0;
		switch (offset >> 1)
		{
			case 0x0e:	// IRQV
				result = chip->irqv;
				if (! (offset & 1))
					update_internal_irq_state(chip);
				break;
			case 0x0f:	// PAGE
				result = chip->current_page;
				break;
		}
		return (offset & 1) ? (result & 0xff) : (result >> 8);
	}
}

// 0xD7 pp aa dd: ES5506 extension commands, addr = (pp << 8) | aa
static void es550x_ext_w(void *info, UINT16 addr, UINT8 data)
{
	ES550XChip* chip = (ES550XChip*)info;
	UINT8 sub = (addr >> 8) & 0x7F;
	UINT8 aa = addr & 0xFF;

	switch (sub)
	{
		case ES5506_EXT_VOICE_BANK:	// voice external bank, in units of 1M words
			chip->voice[aa & 0x1f].exbank = (UINT32)data << 20;
			break;
		case ES5506_EXT_REGION_WINDOW:	// region window base, in units of 1M words
			chip->region[aa & 3].window = (UINT32)data << 20;
			break;
		case ES5506_EXT_REGION_ALIAS:	// region aa reads from region dd
			chip->region[aa & 3].alias = data & 3;
			break;
		case ES5506_EXT_IRQ_ACK:	// host read IRQV
			update_internal_irq_state(chip);
			break;
		case ES5506_EXT_IRQ_AUTOACK:	// 0 = IRQ acks are logged (0xD7 03), 1 = acknowledge immediately
			chip->auto_ack_irq = data ? 1 : 0;
			break;
	}
}

// 0xD8 pp aa dd: ES5510 (ESP) commands, addr = (pp << 8) | aa
static void es550x_esp_w(void *info, UINT16 addr, UINT8 data)
{
	ES550XChip* chip = (ES550XChip*)info;
	UINT8 sub = (addr >> 8) & 0x7F;
	UINT8 aa = addr & 0xFF;

	if (sub == ES5510_CMD_DRAM_SIZE)
	{
		chip->esp_dram_log2 = data;	// must precede the first ESP access
		return;
	}
	es550x_ensure_esp(chip);
	if (chip->esp == NULL)
		return;

	switch (sub)
	{
		case ES5510_CMD_HOST_WRITE:	// host register write
			es5510_host_w(chip->esp, aa, data);
			break;
		case ES5510_CMD_HALT:	// ESPHALT line
			es5510_set_halt_line(chip->esp, data ? 1 : 0);
			break;
		case ES5510_CMD_ROUTE_IN:	// serial input aa <- output channel dd (0xFF = none)
			if (aa < ESP_INPUTS)
			{
				chip->esp_route_in[aa] = data;
				if (data < MAX_OUTCHNS)
					chip->chn_direct[data] = 0;
			}
			break;
		case ES5510_CMD_ROUTE_OUT:	// serial output 6/7 -> stereo mix side dd
			if (aa >= 6 && aa <= 7)
				chip->esp_route_out[aa - 6] = data & 1;
			break;
		case ES5510_CMD_DIRECT:	// output channel aa is mixed directly (dd != 0) or not
			if (aa < MAX_OUTCHNS)
				chip->chn_direct[aa] = data ? 1 : 0;
			break;
		case ES5510_CMD_ROUTE_PRESET:	// routing preset
			es550x_apply_route_preset(chip, aa);
			break;
	}
}

// 0xE3 pp cc gggg: output gain, addr = (pp << 8) | cc, data = gain in 8.8 fixed point
static void es550x_gain_w(void *info, UINT16 addr, UINT16 data)
{
	ES550XChip* chip = (ES550XChip*)info;
	UINT8 cc = addr & 0xFF;

	if (cc < MAX_OUTCHNS)
		chip->chn_gain[cc] = data;
	else if (cc == GAINCH_ESP_L || cc == GAINCH_ESP_R)
		chip->esp_gain[cc & 1] = data;
	else if (cc == GAINCH_DIRECT_L || cc == GAINCH_DIRECT_R)
		chip->direct_gain[cc & 1] = data;
}


/**********************************************************************************************

     sample ROM handling

***********************************************************************************************/

static void es550x_alloc_rom(void *info, UINT32 memsize)
{
	ES550XChip* chip = (ES550XChip*)info;

	// the region number is only known from the start offset of the following ROM write
	chip->pending_romsize = memsize;
}

static void es550x_resize_region(ES550X_REGION* rgn, UINT32 sizeWords)
{
	if (rgn->size == sizeWords)
		return;
	free(rgn->base);
	rgn->base = NULL;
	rgn->size = 0;
	rgn->mask = 0;
	if (! sizeWords)
		return;
	rgn->base = (UINT16*)calloc(sizeWords, sizeof(UINT16));
	if (rgn->base == NULL)
		return;
	rgn->size = sizeWords;
	rgn->mask = (sizeWords & (sizeWords - 1)) ? 0 : (sizeWords - 1);
}

// offset bits 28-29: region number, bit 31: 8-bit sample data (converted to 16-bit)
static void es550x_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data)
{
	ES550XChip* chip = (ES550XChip*)info;
	ES550X_REGION* rgn = &chip->region[(offset >> 28) & 3];
	UINT8 is8bit = (offset >> 31) & 1;
	UINT32 romSize = chip->pending_romsize;
	UINT32 curPos;

	offset &= 0x0FFFFFFF;
	chip->pending_romsize = 0;
	if (is8bit)
	{
		romSize *= 2;
		offset *= 2;
		length *= 2;
	}
	if (romSize)
		es550x_resize_region(rgn, romSize / 2);
	if (rgn->base == NULL)
		return;

	if (offset >= rgn->size * 2)
		return;
	if (offset + length > rgn->size * 2)
		length = rgn->size * 2 - offset;

	if (is8bit)
	{
		for (curPos = 0; curPos < length / 2; curPos++)
			rgn->base[offset / 2 + curPos] = (UINT16)data[curPos] << 8;
	}
	else
	{
		// 16-bit sample words, stored little endian in the VGM
		UINT16* dst = &rgn->base[offset / 2];
		for (curPos = 0; curPos + 1 < length; curPos += 2)
			*dst++ = data[curPos] | ((UINT16)data[curPos + 1] << 8);
	}
}


/**********************************************************************************************

     misc

***********************************************************************************************/

static void es550x_set_mute_mask(void *info, UINT32 MuteMask)
{
	ES550XChip* chip = (ES550XChip*)info;
	UINT8 CurChn;

	for (CurChn = 0; CurChn < 32; CurChn++)
		chip->voice[CurChn].Muted = (MuteMask >> CurChn) & 0x01;
}

static void es550x_set_srchg_cb(void *info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr)
{
	ES550XChip* chip = (ES550XChip*)info;

	chip->SmpRateFunc = CallbackFunc;
	chip->SmpRateData = DataPtr;
}
