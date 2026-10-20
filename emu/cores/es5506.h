#ifndef __ES5506_H__
#define __ES5506_H__

#include "../EmuStructs.h"

// cfg.flags:
//   bits 0-3: number of stereo output pairs (ES5505: 1..4, ES5506: 1..6), 0 = 1
//   bit 7: chip type, 0 = ES5505, 1 = ES5506
#define ES5506_FLAG_CHNMASK     0x0F
#define ES5506_FLAG_ES5506      0x80

// rwFuncs "user" IDs for the DEVRW_A16D8 / DEVRW_A16D16 register writes
#define ES5506_RWUSER_EXT       0   // VGM command 0xD7: ES5506 extension, addr = (sub << 8) | aa
#define ES5506_RWUSER_ESP       1   // VGM command 0xD8: ES5510 (ESP) commands, addr = (sub << 8) | aa
#define ES5506_RWUSER_GAIN      2   // VGM command 0xE3: output gain, addr = (pp << 8) | channel, data = 8.8 gain

// 0xD7 sub-commands
#define ES5506_EXT_VOICE_BANK       0x00    // aa = voice, dd = external bank (1M word units)
#define ES5506_EXT_REGION_WINDOW    0x01    // aa = region, dd = window base (1M word units)
#define ES5506_EXT_REGION_ALIAS     0x02    // aa = region, dd = source region
#define ES5506_EXT_IRQ_ACK          0x03    // host read of IRQV
#define ES5506_EXT_IRQ_AUTOACK      0x04    // dd = 1: acknowledge IRQs immediately (default)

// 0xD8 sub-commands
#define ES5510_CMD_HOST_WRITE       0x00    // aa = host register, dd = data
#define ES5510_CMD_HALT             0x01    // dd = ESPHALT line (1 = halted)
#define ES5510_CMD_ROUTE_IN         0x02    // aa = ESP serial input (0..5), dd = ES5506 output channel (0xFF = none)
#define ES5510_CMD_ROUTE_OUT        0x03    // aa = ESP serial output (6..7), dd = mix side (0 = L, 1 = R)
#define ES5510_CMD_DIRECT           0x04    // aa = ES5506 output channel, dd = mixed directly (1) or not (0)
#define ES5510_CMD_ROUTE_PRESET     0x05    // aa = preset (0 = Taito EN, 1 = Ensoniq synth, 0xFF = all direct)
#define ES5510_CMD_DRAM_SIZE        0x06    // dd = log2 of DRAM size in words (0 = default 1M)

// 0xE3 channel IDs: 0x00-0x0B = ES5506 output channels, 0x10/0x11 = ESP output L/R, 0x12/0x13 = direct mix L/R

extern const DEV_DECL sndDev_ES5506;

#endif	// __ES5506_H__
