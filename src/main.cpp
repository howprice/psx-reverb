/*

Digital Signal Processing
=========================

x[n] input signal
y[n] output signal
h[n] impulse response of the system
Convolution takes two signals and produces a third signal
x[n] * h[n] = y[n]  n.b. * denotes convolution, not multiplication
x[n] is convolved with h[n] to produce y[n].

Note that n just means some index into the signal. It could be written as i.

For full convolution, the length of the output signal is equal to the length of the input signal, plus the length of the impulse response, minus one.

FIR Finite Impulse Response
An impulse response that is of finite length. The output depends only on the current and a finite number of previous input samples.
i.e. FIR filters do not use feedback.

IIR Infinite Impulse Response
An impulse response that is of infinite length. The output depends on the current and all previous input samples, as well as previous output samples.
i.e. IIR filters use feedback.

TODO
====
- All pass filters
- Add or identify BIOS reverb preset
- vIIR -8000h bug https://psx-spx.consoledev.net/soundprocessingunitspu/#bug
*/

#include "Parse.h"
#include "Log.h"
#include "hp_assert.h"
#include "StringHelpers.h"
#include "MathsHelpers.h"
#include "ArrayHelpers.h"
#include "Helpers.h" // HP_UNUSED
#include "Types.h"

#include <stdlib.h> // EXIT_SUCCESS, EXIT_FAILURE
#include <string.h> // strcmp

// 39-tap PSX SPU FIR filter
// https://psx-spx.consoledev.net/soundprocessingunitspu/#reverb-buffer-resampling
// Each coefficient is a volume multipler N/0x8000, so after multiplying by the input sample, the result needs to be shift right by 15 to rescale.
// Note that every other sample except the peak is zero, so this is effectively a 20-tap filter with zeros interleaved for downsampling by 2.
// 
// The filter is also used for upsampling!
//
static const s16 kFiniteImpulseResponse[] =
{
	-0x0001,  0x0000,  0x0002,  0x0000, -0x000A,  0x0000,  0x0023,  0000,
	-0x0067,  0x0000,  0x010A,  0x0000, -0x0268,  0x0000,  0x0534,  0000,
	-0x0B90,  0x0000,  0x2806,  0x4000,  0x2806,  0x0000, -0x0B90,  0000,
	 0x0534,  0x0000, -0x0268,  0x0000,  0x010A,  0x0000, -0x0067,  0000,
	 0x0023,  0x0000, -0x000A,  0x0000,  0x0002,  0x0000, -0x0001,
};
static const unsigned int kFIRFilterSize = 39;
static_assert(COUNTOF_ARRAY(kFiniteImpulseResponse) == kFIRFilterSize);

// Reverb Volume and Address Registers (R/W)
//
//   Port      Reg   Name    Type    Expl.
//   1F801D84h spu   vLOUT   volume  Reverb Output Volume Left
//   1F801D86h spu   vROUT   volume  Reverb Output Volume Right
//   1F801DA2h spu   mBASE   base    Reverb Work Area Start Address in Sound RAM
//   1F801DC0h rev00 dAPF1   disp    Reverb APF Offset 1
//   1F801DC2h rev01 dAPF2   disp    Reverb APF Offset 2
//   1F801DC4h rev02 vIIR    volume  Reverb Reflection Volume 1
//   1F801DC6h rev03 vCOMB1  volume  Reverb Comb Volume 1
//   1F801DC8h rev04 vCOMB2  volume  Reverb Comb Volume 2
//   1F801DCAh rev05 vCOMB3  volume  Reverb Comb Volume 3
//   1F801DCCh rev06 vCOMB4  volume  Reverb Comb Volume 4
//   1F801DCEh rev07 vWALL   volume  Reverb Reflection Volume 2
//   1F801DD0h rev08 vAPF1   volume  Reverb APF Volume 1
//   1F801DD2h rev09 vAPF2   volume  Reverb APF Volume 2
//   1F801DD4h rev0A mLSAME  src/dst Reverb Same Side Reflection Address 1 Left
//   1F801DD6h rev0B mRSAME  src/dst Reverb Same Side Reflection Address 1 Right
//   1F801DD8h rev0C mLCOMB1 src     Reverb Comb Address 1 Left
//   1F801DDAh rev0D mRCOMB1 src     Reverb Comb Address 1 Right
//   1F801DDCh rev0E mLCOMB2 src     Reverb Comb Address 2 Left
//   1F801DDEh rev0F mRCOMB2 src     Reverb Comb Address 2 Right
//   1F801DE0h rev10 dLSAME  src     Reverb Same Side Reflection Address 2 Left
//   1F801DE2h rev11 dRSAME  src     Reverb Same Side Reflection Address 2 Right
//   1F801DE4h rev12 mLDIFF  src/dst Reverb Different Side Reflect Address 1 Left
//   1F801DE6h rev13 mRDIFF  src/dst Reverb Different Side Reflect Address 1 Right
//   1F801DE8h rev14 mLCOMB3 src     Reverb Comb Address 3 Left
//   1F801DEAh rev15 mRCOMB3 src     Reverb Comb Address 3 Right
//   1F801DECh rev16 mLCOMB4 src     Reverb Comb Address 4 Left
//   1F801DEEh rev17 mRCOMB4 src     Reverb Comb Address 4 Right
//   1F801DF0h rev18 dLDIFF  src     Reverb Different Side Reflect Address 2 Left
//   1F801DF2h rev19 dRDIFF  src     Reverb Different Side Reflect Address 2 Right
//   1F801DF4h rev1A mLAPF1  src/dst Reverb APF Address 1 Left
//   1F801DF6h rev1B mRAPF1  src/dst Reverb APF Address 1 Right
//   1F801DF8h rev1C mLAPF2  src/dst Reverb APF Address 2 Left
//   1F801DFAh rev1D mRAPF2  src/dst Reverb APF Address 2 Right
//   1F801DFCh rev1E vLIN    volume  Reverb Input Volume Left
//   1F801DFEh rev1F vRIN    volume  Reverb Input Volume Right
//
// All volume registers are signed 16bit (range -8000h..+7FFFh).
// - INT16_MAX 0x7fff is max volume
// - INT16_MIN 0x8000 -32768 is max negative volume and will invert the signal
// 
// All src/dst/disp/base registers are addresses in SPU memory *divided by 8*.
// src/dst are relative to the current buffer address
// disp registers are relative to src registers
// The base register defines the start address of the reverb buffer (the end address is fixed, at 7FFFEh).
// Writing a value to mBASE does additionally set the current buffer address to that value.
// 
// https://psx-spx.consoledev.net/soundprocessingunitspu/#spu-reverb-registers
//

struct SPU
{
	static const unsigned int kRamSizeBytes = 512 * 1024; // 512 KB of SPU RAM
	u8 ram[kRamSizeBytes];

	s16 vLOUT;
	s16 vROUT;
	u16 mBASE; // Reverb buffer start address. Extends to end of ram

	u32 reverbBufferStart; // for convenience
	u32 currentReverbBufferHead; // current reverb buffer head address. This is incremented after each pass of the reverb.
	unsigned int reverbBufferSizeBytes; // extends from mBASE to end of RAM
};

// Reverb unit registers
struct ReverbRegisters
{
	u16 dAPF1;
	u16 dAPF2;
	u16 vIIR;
	u16 vCOMB1;
	u16 vCOMB2;
	u16 vCOMB3;
	u16 vCOMB4;
	u16 vWALL;
	u16 vAPF1;
	u16 vAPF2;
	u16 mLSAME;
	u16 mRSAME;
	u16 mLCOMB1;
	u16 mRCOMB1;
	u16 mLCOMB2;
	u16 mRCOMB2;
	u16 dLSAME;
	u16 dRSAME;
	u16 mLDIFF;
	u16 mRDIFF;
	u16 mLCOMB3;
	u16 mRCOMB3;
	u16 mLCOMB4;
	u16 mRCOMB4;
	u16 dLDIFF;
	u16 dRDIFF;
	u16 mLAPF1;
	u16 mRAPF1;
	u16 mLAPF2;
	u16 mRAPF2;
	u16 vLIN;
	u16 vRIN;
};

// Reverb presets from https://psx-spx.consoledev.net/soundprocessingunitspu/#spu-reverb-examples

//   dAPF1  dAPF2  vIIR   vCOMB1 vCOMB2  vCOMB3  vCOMB4  vWALL   ;1F801DC0h..CEh
//   vAPF1  vAPF2  mLSAME mRSAME mLCOMB1 mRCOMB1 mLCOMB2 mRCOMB2 ;1F801DD0h..DEh
//   dLSAME dRSAME mLDIFF mRDIFF mLCOMB3 mRCOMB3 mLCOMB4 mRCOMB4 ;1F801DE0h..EEh
//   dLDIFF dRDIFF mLAPF1 mRAPF1 mLAPF2  mRAPF2  vLIN    vRIN    ;1F801DF0h..FEh

// Required memory size = 0x80000 - mBASE*8

// Room (size=26C0h bytes)
static const ReverbRegisters kReverbPreset_Room =
{
	0x007D, 0x005B, 0x6D80, 0x54B8, 0xBED0, 0x0000, 0x0000, 0xBA80,
	0x5800, 0x5300, 0x04D6, 0x0333, 0x03F0, 0x0227, 0x0374, 0x01EF,
	0x0334, 0x01B5, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x01B4, 0x0136, 0x00B8, 0x005C, 0x8000, 0x8000,
};

// Studio Small (size=1F40h bytes)
static const ReverbRegisters kReverbPreset_StudioSmall =
{
	0x0033, 0x0025, 0x70F0, 0x4FA8, 0xBCE0, 0x4410, 0xC0F0, 0x9C00,
	0x5280, 0x4EC0, 0x03E4, 0x031B, 0x03A4, 0x02AF, 0x0372, 0x0266,
	0x031C, 0x025D, 0x025C, 0x018E, 0x022F, 0x0135, 0x01D2, 0x00B7,
	0x018F, 0x00B5, 0x00B4, 0x0080, 0x004C, 0x0026, 0x8000, 0x8000,
};

// Studio Medium (size=4840h bytes)
static const ReverbRegisters kReverbPreset_StudioMedium =
{
	0x00B1, 0x007F, 0x70F0, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0xB4C0,
	0x5280, 0x4EC0, 0x0904, 0x076B, 0x0824, 0x065F, 0x07A2, 0x0616,
	0x076C, 0x05ED, 0x05EC, 0x042E, 0x050F, 0x0305, 0x0462, 0x02B7,
	0x042F, 0x0265, 0x0264, 0x01B2, 0x0100, 0x0080, 0x8000, 0x8000,
};

// Studio Large (size=6FE0h bytes)
static const ReverbRegisters kReverbPreset_StudioLarge =
{
	0x00E3, 0x00A9, 0x6F60, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0xA680,
	0x5680, 0x52C0, 0x0DFB, 0x0B58, 0x0D09, 0x0A3C, 0x0BD9, 0x0973,
	0x0B59, 0x08DA, 0x08D9, 0x05E9, 0x07EC, 0x04B0, 0x06EF, 0x03D2,
	0x05EA, 0x031D, 0x031C, 0x0238, 0x0154, 0x00AA, 0x8000, 0x8000,
};

// Hall (size=ADE0h bytes)
static const ReverbRegisters kReverbPreset_Hall =
{
	0x01A5, 0x0139, 0x6000, 0x5000, 0x4C00, 0xB800, 0xBC00, 0xC000,
	0x6000, 0x5C00, 0x15BA, 0x11BB, 0x14C2, 0x10BD, 0x11BC, 0x0DC1,
	0x11C0, 0x0DC3, 0x0DC0, 0x09C1, 0x0BC4, 0x07C1, 0x0A00, 0x06CD,
	0x09C2, 0x05C1, 0x05C0, 0x041A, 0x0274, 0x013A, 0x8000, 0x8000,
};

// Half Echo (size=3C00h bytes)
static const ReverbRegisters kReverbPreset_HalfEcho =
{
	0x0017, 0x0013, 0x70F0, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0x8500,
	0x5F80, 0x54C0, 0x0371, 0x02AF, 0x02E5, 0x01DF, 0x02B0, 0x01D7,
	0x0358, 0x026A, 0x01D6, 0x011E, 0x012D, 0x00B1, 0x011F, 0x0059,
	0x01A0, 0x00E3, 0x0058, 0x0040, 0x0028, 0x0014, 0x8000, 0x8000,
};

// Space Echo (size=F6C0h bytes)
static const ReverbRegisters kReverbPreset_SpaceEcho =
{
	0x033D, 0x0231, 0x7E00, 0x5000, 0xB400, 0xB000, 0x4C00, 0xB000,
	0x6000, 0x5400, 0x1ED6, 0x1A31, 0x1D14, 0x183B, 0x1BC2, 0x16B2,
	0x1A32, 0x15EF, 0x15EE, 0x1055, 0x1334, 0x0F2D, 0x11F6, 0x0C5D,
	0x1056, 0x0AE1, 0x0AE0, 0x07A2, 0x0464, 0x0232, 0x8000, 0x8000,
};

// Chaos Echo (almost infinite) (size=18040h bytes)
static const ReverbRegisters kReverbPreset_ChaosEcho =
{
	0x0001, 0x0001, 0x7FFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x8100,
	0x0000, 0x0000, 0x1FFF, 0x0FFF, 0x1005, 0x0005, 0x0000, 0x0000,
	0x1005, 0x0005, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x1004, 0x1002, 0x0004, 0x0002, 0x8000, 0x8000,
};

// Delay (one-shot echo) (size=18040h bytes)
static const ReverbRegisters kReverbPreset_Delay =
{
	0x0001, 0x0001, 0x7FFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x1FFF, 0x0FFF, 0x1005, 0x0005, 0x0000, 0x0000,
	0x1005, 0x0005, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x1004, 0x1002, 0x0004, 0x0002, 0x8000, 0x8000,
};

// Off (size=10h dummy bytes)
static const ReverbRegisters kReverbPreset_Off =
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001,
  0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001,
  0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000,
};

enum class ReverbPreset
{
	Room,
	StudioSmall,
	StudioMedium,
	StudioLarge,
	Hall,
	HalfEcho,
	SpaceEcho,
	ChaosEcho,
	Delay,
	Off,

	Max = Off
};

static const char* kReverbPresetNames[] =
{
	"Room",
	"StudioSmall",
	"StudioMedium",
	"StudioLarge",
	"Hall",
	"HalfEcho",
	"SpaceEcho",
	"ChaosEcho",
	"Delay",
	"Off",
};
static_assert(COUNTOF_ARRAY(kReverbPresetNames) == ENUM_COUNT(ReverbPreset));

static const ReverbRegisters kReverbPresets[] =
{
	kReverbPreset_Room,
	kReverbPreset_StudioSmall,
	kReverbPreset_StudioMedium,
	kReverbPreset_StudioLarge,
	kReverbPreset_Hall,
	kReverbPreset_HalfEcho,
	kReverbPreset_SpaceEcho,
	kReverbPreset_ChaosEcho,
	kReverbPreset_Delay,
	kReverbPreset_Off,
};
static_assert(COUNTOF_ARRAY(kReverbPresets) == ENUM_COUNT(ReverbPreset));

static unsigned int kReverbPresetBufferSizeBytes[] =
{
	0x26C0,  // Room
	0x1F40,  // StudioSmall
	0x4840,  // StudioMedium
	0x6FE0,  // StudioLarge
	0xADE0,  // Hall
	0x3C00,  // HalfEcho
	0xF6C0,  // SpaceEcho
	0x18040, // ChaosEcho
	0x18040, // Delay
	0x0010,  // Off
};
static_assert(COUNTOF_ARRAY(kReverbPresetBufferSizeBytes) == ENUM_COUNT(ReverbPreset));

//--------------------------------------------------------------------------------------------------------

static bool loadInput(const char* filename, s16*& buffer, size_t& numElements)
{
	FILE* file = fopen(filename, "rb");
	if (!file)
	{
		LOG_ERROR("Failed to open file: %s\n", filename);
		return false;
	}
	fseek(file, 0, SEEK_END);
	long fileSize = ftell(file);

	if ((fileSize % 2) != 0)
	{
		LOG_ERROR("Expect signed 16-bit input data. File size is not a multiple of 2 bytes: %s\n", filename);
		fclose(file);
		return false;
	}

	fseek(file, 0, SEEK_SET); // Seek back to beginning before reading
	numElements = fileSize / 2; // Each sample is 2 bytes (16 bits)
	buffer = new s16[numElements];
	if (fread(buffer, 2, numElements, file) != numElements)
	{
		LOG_ERROR("Failed to read file: %s\n", filename);
		fclose(file);
		return false;
	}

	fclose(file);
	file = nullptr;
	LOG_INFO("Read %s size %zu samples\n", filename, numElements);
	return true;
}

static bool saveBuffer(const char* filename, s16* buffer, size_t numElements)
{
	FILE* file = fopen(filename, "wb");
	if (!file)
	{
		LOG_ERROR("Failed to open file for write: %s\n", filename);
		return false;
	}

	if (fwrite(buffer, 2, numElements, file) != numElements)
	{
		LOG_ERROR("Failed to write to file: %s\n", filename);
		fclose(file);
		return false;
	}

	fclose(file);
	file = nullptr;

	LOG_INFO("Wrote %s size %zu samples\n", filename, numElements);
	return true;
}

//
// Saturates signed 32-bit value to signed 16-bit range
//
static s16 saturateS32toS16(s32 val)
{
	return (s16)Clamp(val, (s32)INT16_MIN, (s32)INT16_MAX);
}

static inline u32 reverbAddrToRamAddr(const SPU& spu, u16 addressDiv8)
{
	u32 addr = spu.currentReverbBufferHead + (addressDiv8 * 8u);

	// The PSX SPU reverb buffer always extends to the end of RAM.
	if (addr >= SPU::kRamSizeBytes)
		addr -= spu.reverbBufferSizeBytes;

	HP_ASSERT(addr >= spu.reverbBufferStart && addr < SPU::kRamSizeBytes);

	return addr;
}

static s16 readReverbBuffer(const SPU& spu, u16 addressDiv8)
{
	u32 address = reverbAddrToRamAddr(spu, addressDiv8);
	HP_ASSERT(address >= spu.reverbBufferStart && address + 2 <= SPU::kRamSizeBytes);
	return *(s16*)(spu.ram + address);
}

[[maybe_unused]]
static void writeReverbBuffer(SPU& spu, u16 addressDiv8, s16 val)
{
	u32 address = reverbAddrToRamAddr(spu, addressDiv8);
	HP_ASSERT(address >= spu.reverbBufferStart && address + 2 <= SPU::kRamSizeBytes);
	u16* pDst = (u16*)(spu.ram + address);
	*pDst = val;
}

static s16 readRAM(const SPU& spu, u32 address)
{
	HP_ASSERT(address + 2 <= SPU::kRamSizeBytes);
	s16 val = *(s16*)(spu.ram + address);
	return val;
}

static void writeRAM(SPU& spu, u32 address, s16 val)
{
	HP_ASSERT(address + 2 <= SPU::kRamSizeBytes);
	u16* pDst = (u16*)(spu.ram + address);
	*pDst = val;
}

// Delay and filter. And mix the two together.
// 
// A delayed version of the input is added to the input and then blended with previous sample.
//
//     buf[m_addr] = (input + buf[d_addr]*vWALL - buf[m_addr-2])*vIIR + buf[m_addr-2]
//
// In DSP notation this is:
// 
//     y[n] = (x[n] + vWALL*y[n-D] - y[n-1])*vIIR + y[n-1]
//
// Where [m_addr-2] is the previous sample in the delay line.
//
// This can be rewritten to show that this is a weighted average of the new sample an the previous sample:
//
// buf[m_addr] = vIIR*(input + buf[d_addr]*vWALL) - vIIR*buf[m_addr-2] + buf[m_addr-2]
// buf[m_addr] = vIIR*(input + buf[d_addr]*vWALL) + 1*buf[m_addr-2] - vIIR*buf[m_addr-2]
// buf[m_addr] = vIIR*(input + buf[d_addr]*vWALL) + (1-vIIR)*buf[m_addr-2]
//
// In DSP notation this is:
// 
//     y[n] = vIIR*(x[n] + y[n-D]*vWALL) - (1-vIIR)*y[n-1]
//
// In jsgroth's blog post, 1-vIIR is written as 0x8000 - vIIR because volume is represented as a signed 16-bit value.
//
// High vIIR makes the reverb responsive to new inputs.
// Low vIIR makes the reverb less responsive to new inputs, which creates a longer decay time.  This is a "Leaky Integrator"
// which acts as a low pass filter by averaging inputs over time.
//
// It is an *Infinite* Impulse Response filter because the output depends on all previous inputs, as well as previous outputs (feedback).
//
// Note that the vIIR coefficient also applies to the tapped echo. This prevents the energy in the system from growing continually via feedback.
//
static s32 applyReflection(
	SPU& spu,
	s32 input,
	u16 m_addr,
	u16 d_addr, // delayed tap taken from buffer
	s16 vIIR, // Feedback coefficient. Controls decay time
	s16 vWALL) // Reflection coefficient. The "hardness" of the room's walls
{
	u32 outputAddr = reverbAddrToRamAddr(spu, m_addr);

	// previous output sample y[n-1]
	u32 prevAddr = outputAddr > spu.reverbBufferStart ? outputAddr - 2 : SPU::kRamSizeBytes - 2; // the reverb buffer always extends to the end of RAM, so wrap around to there
	s32 prev = (s32)readRAM(spu, prevAddr);

	// tap sample for echo
	u32 tapAddr = reverbAddrToRamAddr(spu, d_addr);
	s32 tap = (s32)readRAM(spu, tapAddr); // tap a previous sample to generate delay

	s32 reflection = (tap * vWALL) >> 15; // Apply volume, and rescale back down to signed 16-bit range

	// Lerp between new input+delay and previous output
	s32 output = ((input + reflection - prev) * vIIR) >> 15; // Rescale back down after multiplying by volume
	output += prev;

	writeRAM(spu, outputAddr, saturateS32toS16(output));

	return output;
}

// Comb filter to simulate hearing the same sound arriving at different times from different paths through the room.
// It is called a comb filter because of the characterisic shape of the frequency response, which has regularly spaced peaks and troughs like a comb.
// The delay time determines the spacing of the peaks and troughs, and the feedback volume determines how pronounced they are.
// The peaks and troughs are generated due to constructive and destructive interference between the direct sound and the delayed sound.
//
// out=vCOMB1*[mCOMB1] + vCOMB2*[mCOMB2] + vCOMB3*[mCOMB3] + vCOMB4*[mCOMB4]
//
static s32 applyEarlyEcho(
	const SPU& spu,
	u16 mComb1, u16 mComb2, u16 mComb3, u16 mComb4,
	s16 vComb1, s16 vComb2, s16 vComb3, s16 vComb4)
{
	s32 output = 0;
	output += (s32)vComb1 * (s32)readReverbBuffer(spu, mComb1);
	output += (s32)vComb2 * (s32)readReverbBuffer(spu, mComb2);
	output += (s32)vComb3 * (s32)readReverbBuffer(spu, mComb3);
	output += (s32)vComb4 * (s32)readReverbBuffer(spu, mComb4);
	output >>= 15; // Rescale back down after multiplying by volume

	return output;
}

static void printUsage(const char* programName)
{
	LOG_INFO("Usage: %s [options] <input file>\n", programName);
	LOG_INFO("Options:\n");
	LOG_INFO("  --help               Show this help message\n");
	LOG_INFO("  --log-level <level>  Set log level: 2=trace, 1=debug, 0=info (default), -1=warn), -2=error -3=none\n");
	LOG_INFO("  --preset <name>      Set reverb preset (default: Room)\n");
}

int main(int argc, char** argv)
{
	// Parse command line arguments
	ReverbPreset preset = ReverbPreset::Room;
	const char* inputFilename = nullptr;

	for (int i = 1; i < argc; i++)
	{
		const char* arg = argv[i];

		if (strcmp(arg, "--help") == 0)
		{
			printUsage(argv[0]);
			exit(EXIT_SUCCESS);
		}
		else if (strcmp(arg, "--log-level") == 0)
		{
			if (i + 1 == argc)
			{
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}

			arg = argv[++i];
			if (arg[0] == '-')
			{
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}

			int logLevel;
			if (!ParseInt(arg, logLevel) || logLevel < LOG_LEVEL_MIN || logLevel > LOG_LEVEL_MAX)
			{
				LOG_ERROR("Invalid log-level value\n");
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}
			SetLogLevel(logLevel);
		}
		else if (strcmp(arg, "--preset") == 0)
		{
			if (i + 1 == argc)
			{
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}
			arg = argv[++i];
			unsigned int presetIndex;
			for (presetIndex = 0; presetIndex < ENUM_COUNT(ReverbPreset); presetIndex++)
			{
				if (Stricmp(arg, kReverbPresetNames[presetIndex]) == 0)
				{
					preset = (ReverbPreset)presetIndex;
					break;
				};
			}
			if (presetIndex == ENUM_COUNT(ReverbPreset))
			{
				LOG_ERROR("Invalid preset name: %s\n", arg);
				LOG_INFO("Valid preset names are:\n");
				for (presetIndex = 0; presetIndex < ENUM_COUNT(ReverbPreset); presetIndex++)
				{
					LOG_INFO("  %s\n", kReverbPresetNames[presetIndex]);
				}
				exit(EXIT_FAILURE);
			}
		}
		else if (!inputFilename)
		{
			inputFilename = arg;
		}
		else
		{
			LOG_ERROR("Unrecognised command line arg: %s\n", arg);
			printUsage(argv[0]);
			exit(EXIT_FAILURE);
		}
	}

	s16* input;
	size_t inputLengthSamples;
	if (!loadInput(inputFilename, /*out*/input, /*out*/inputLengthSamples))
	{
		LOG_ERROR("Failed to load input file: %s\n", inputFilename);
		return EXIT_FAILURE;
	}
	if (inputLengthSamples & 1)
	{
		LOG_ERROR("Expect stereo input data. Number of samples is not even: %zu\n", inputLengthSamples);
		return EXIT_FAILURE;
	}
	size_t inputLengthFrames = inputLengthSamples >> 1; // stereo

	const ReverbRegisters& reverb = kReverbPresets[(int)preset];
	LOG_INFO("Reverb preset: %s\n", kReverbPresetNames[(int)preset]);

	// All of the reverb presets seem to use 0x8000 for vLIN and vRIN.
	// This is maximum negative volume and will invert the signal.
	// So perform the same operation at the output to flip the signal back up again.
	SPU spu{};
	spu.vLOUT = INT16_MIN;
	spu.vROUT = INT16_MIN;
	spu.mBASE = (u16)(0x60000 >> 3); // Representative value for testing

	spu.reverbBufferStart = spu.mBASE * 8; // Convert from address in sound ram (divided by 8) to byte address

	// Writing to mBASE set the current buffer address to that value.
	spu.currentReverbBufferHead = spu.reverbBufferStart;

	// Calculate the reverb buffer size, which is required for accessing reverb buffer memory
	HP_ASSERT(spu.currentReverbBufferHead <= SPU::kRamSizeBytes);
	spu.reverbBufferSizeBytes = SPU::kRamSizeBytes - spu.currentReverbBufferHead;
	HP_ASSERT(spu.reverbBufferSizeBytes >= kReverbPresetBufferSizeBytes[(int)preset]); // Reverb buffer size must fit in available ram

	s16 downsamplerRingbufferL[kFIRFilterSize]{};
	s16 downsamplerRingbufferR[kFIRFilterSize]{};
	unsigned int downsamplerRingbufferIndex = 0; // Circular buffer index for downsampler input

	s16 upsamplerRingbufferL[kFIRFilterSize]{};
	s16 upsamplerRingbufferR[kFIRFilterSize]{};
	unsigned int upsamplerRingbufferIndex = 0; // Circular buffer index for upsampler input

	// Create some intermediate buffers for debugging

	// For full convolution, the length of the output signal is equal to the length of the input signal, plus the length of the impulse response, minus one.
	// However we are downsampling by 2, so the output length is half of the input length. We can calculate the output capacity based on this.
	const size_t downsampledBufferCapacitySamples = inputLengthSamples / 2; // Downsample by 2, so output capacity is half of input length
	s16* downsampledBuffer = new s16[downsampledBufferCapacitySamples]; // single buffer containing stereo samplees
	size_t downsampledBufferLenSamples = 0; // Will be set by downsample function

	const size_t sameSideReflectionBufferCapacitySamples = downsampledBufferCapacitySamples;
	s16* sameSideReflectionBuffer = new s16[sameSideReflectionBufferCapacitySamples]; // single buffer containing stereo samplees
	size_t sameSideReflectionBufferLenSamples = 0;

	const size_t differentSideReflectionBufferCapacitySamples = downsampledBufferCapacitySamples;
	s16* differentSideReflectionBuffer = new s16[differentSideReflectionBufferCapacitySamples]; // single buffer containing stereo samplees
	size_t differentSideReflectionBufferLenSamples = 0;

	const size_t earlyEchoBufferCapacitySamples = downsampledBufferCapacitySamples;
	s16* earlyEchoBuffer = new s16[earlyEchoBufferCapacitySamples]; // single buffer containing stereo samplees
	size_t earlyEchoBufferLenSamples = 0;

	const size_t reverbOutputBufferCapacitySamples = downsampledBufferCapacitySamples; // Reverb is performed at 22050 Hz
	s16* reverbOutputBuffer = new s16[reverbOutputBufferCapacitySamples];
	size_t reverbOutputBufferLenSamples = 0;

	// Final output from reverb output upsampled back to 44100 Hz
	const size_t upsampledBufferCapacitySamples = inputLengthSamples; // Upsample by 2, so same length as input buffer
	s16* upsampledBuffer = new s16[upsampledBufferCapacitySamples];
	size_t upsampledBufferLenSamples = 0;

	// Mixed wet output
	const size_t mixedOutputBufferCapacitySamples = inputLengthSamples; // same length as input buffer
	s16* mixedOutputBuffer = new s16[mixedOutputBufferCapacitySamples];
	size_t mixedOutputBufferLenSamples = 0;

	// This would be called at 44100 Hz, every 768 cycles on PSX
	for (unsigned int inputFrameIndex = 0; inputFrameIndex < inputLengthFrames; inputFrameIndex++)
	{
		const s16 inputSampleL = input[2 * inputFrameIndex]; // Left sample of current stereo frame
		const s16 inputSampleR = input[2 * inputFrameIndex + 1]; // Right sample of current stereo frame

		// The downsampler ring buffers must be fed every tick.
		downsamplerRingbufferL[downsamplerRingbufferIndex] = inputSampleL; // Left sample of current stereo frame
		downsamplerRingbufferR[downsamplerRingbufferIndex] = inputSampleR; // Right sample of current stereo frame

		// Reverb is processed every other cycle i.e. at 22050 Hz
		if ((inputFrameIndex & 1) == 0)
		{
			// Downsample from 44100 Hz to 22050 Hz to calculate the reverb unit input.
			s32 LeftInput = 0; // input to reverb unit
			s32 RightInput = 0;
			{
				for (size_t j = 0; j < COUNTOF_ARRAY(kFiniteImpulseResponse); j++)
				{
					// n.b. The filter is not centered so introduces a delay. To center the filter input index would need to
					// be offset by 19 (half of 39) but this would cause negative input index for the first 19 output samples,
					// so we start with input index of 0 and let the filter introduce the delay.
					int inputIndex = (int)downsamplerRingbufferIndex - (int)j;
					if (inputIndex < 0)
						inputIndex += COUNTOF_ARRAY(downsamplerRingbufferL); // Wrap around; circular buffer

					s32 h = (s32)kFiniteImpulseResponse[j]; // impulse response
					LeftInput += (s32)downsamplerRingbufferL[inputIndex] * h;
					RightInput += (s32)downsamplerRingbufferR[inputIndex] * h;
				}

				LeftInput >>= 15; // Rescale by 0x8000
				RightInput >>= 15; // Rescale by 0x8000

				HP_DEBUG_ASSERT(downsampledBufferLenSamples + 2 <= downsampledBufferCapacitySamples); // should never fail by construction
				downsampledBuffer[downsampledBufferLenSamples++] = saturateS32toS16(LeftInput);
				downsampledBuffer[downsampledBufferLenSamples++] = saturateS32toS16(RightInput);
			}

			// Reverb

			// Apply input volume vLIN, vRIN
			s32 Lin = ((s32)(s16)reverb.vLIN * LeftInput) >> 15; // / 0x8000;
			s32 Rin = ((s32)(s16)reverb.vRIN * RightInput) >> 15; // / 0x8000;
			
			// Reverb chain
			
			// Same Side Reflection (left-to-left and right-to-right)
			//   [mLSAME] = (Lin + [dLSAME]*vWALL - [mLSAME-2])*vIIR + [mLSAME-2]  ;L-to-L
			//   [mRSAME] = (Rin + [dRSAME]*vWALL - [mRSAME-2])*vIIR + [mRSAME-2]  ;R-to-R
			s32 LSAME = applyReflection(spu, Lin, reverb.mLSAME, reverb.dLSAME, reverb.vIIR, reverb.vWALL);
			s32 RSAME = applyReflection(spu, Rin, reverb.mRSAME, reverb.dRSAME, reverb.vIIR, reverb.vWALL);
			HP_DEBUG_ASSERT(sameSideReflectionBufferLenSamples + 2 <= sameSideReflectionBufferCapacitySamples); // should never fail by construction
			// The outputs aren't used directly. They are written to memory and read by the subsequent comb filters.
			sameSideReflectionBuffer[sameSideReflectionBufferLenSamples++] = saturateS32toS16(LSAME);
			sameSideReflectionBuffer[sameSideReflectionBufferLenSamples++] = saturateS32toS16(RSAME);

			// Different Side Reflection (left-to-right and right-to-left)
			//   [mLDIFF] = (Lin + [dRDIFF]*vWALL - [mLDIFF-2])*vIIR + [mLDIFF-2]  ;R-to-L   n.b. This uses the *right* delay tap dRDIFF to bounce the signal from left to right
			//   [mRDIFF] = (Rin + [dLDIFF]*vWALL - [mRDIFF-2])*vIIR + [mRDIFF-2]  ;L-to-R   n.b. This uses the *left* delay tap dLDIFF to bounce the signal from right to left
			s32 LDIFF = applyReflection(spu, Lin, reverb.mLDIFF, reverb.dRDIFF, reverb.vIIR, reverb.vWALL);
			s32 RDIFF = applyReflection(spu, Rin, reverb.mRDIFF, reverb.dLDIFF, reverb.vIIR, reverb.vWALL);
			// The outputs aren't used directly. They are written to memory and read by the subsequent comb filters.
			HP_DEBUG_ASSERT(differentSideReflectionBufferLenSamples + 2 <= differentSideReflectionBufferCapacitySamples); // should never fail by construction
			differentSideReflectionBuffer[differentSideReflectionBufferLenSamples++] = saturateS32toS16(LDIFF);
			differentSideReflectionBuffer[differentSideReflectionBufferLenSamples++] = saturateS32toS16(RDIFF);

			// Early Echo (Comb Filter, with input from buffer)
			//   Lout=vCOMB1*[mLCOMB1]+vCOMB2*[mLCOMB2]+vCOMB3*[mLCOMB3]+vCOMB4*[mLCOMB4]
			//   Rout=vCOMB1*[mRCOMB1]+vCOMB2*[mRCOMB2]+vCOMB3*[mRCOMB3]+vCOMB4*[mRCOMB4]
			s32 Lout = applyEarlyEcho(spu, reverb.mLCOMB1, reverb.mLCOMB2, reverb.mLCOMB3, reverb.mLCOMB4, reverb.vCOMB1, reverb.vCOMB2, reverb.vCOMB3, reverb.vCOMB4);
			s32 Rout = applyEarlyEcho(spu, reverb.mRCOMB1, reverb.mRCOMB2, reverb.mRCOMB3, reverb.mRCOMB4, reverb.vCOMB1, reverb.vCOMB2, reverb.vCOMB3, reverb.vCOMB4);
			HP_DEBUG_ASSERT(earlyEchoBufferLenSamples + 2 <= earlyEchoBufferCapacitySamples); // should never fail by construction
			earlyEchoBuffer[earlyEchoBufferLenSamples++] = saturateS32toS16(Lout);
			earlyEchoBuffer[earlyEchoBufferLenSamples++] = saturateS32toS16(Rout);

			// #TODO: Late Reverb APF1 (All Pass Filter 1, with input from COMB)
			// #TODO: Late Reverb APF2 (All Pass Filter 2, with input from APF1)

			// Apply output volume vLOUT, vROUT
			s32 LeftOutput = (Lout * (s32)spu.vLOUT) >> 15; // / 0x8000;
			s32 RightOutput = (Rout * (s32)spu.vROUT) >> 15; // / 0x8000;

			// Write the new reverb output values into the upsampler ring buffers
			upsamplerRingbufferL[upsamplerRingbufferIndex] = saturateS32toS16(LeftOutput);
			upsamplerRingbufferR[upsamplerRingbufferIndex] = saturateS32toS16(RightOutput);

			// Write to debug reverb output buffer at 22050 Hz
			HP_DEBUG_ASSERT(reverbOutputBufferLenSamples + 2 <= reverbOutputBufferCapacitySamples); // should never fail by construction
			reverbOutputBuffer[reverbOutputBufferLenSamples++] = upsamplerRingbufferL[upsamplerRingbufferIndex];
			reverbOutputBuffer[reverbOutputBufferLenSamples++] = upsamplerRingbufferR[upsamplerRingbufferIndex];

			// Increment and wrap reverb buffer current head position.
			// Note that each reverb stage has separate buffers for L and R so only need to advance by 1 sample (2 bytes) per cycle, not 2 samples for stereo.
			spu.currentReverbBufferHead += 2;
			if (spu.currentReverbBufferHead >= SPU::kRamSizeBytes)
				spu.currentReverbBufferHead -= spu.reverbBufferSizeBytes;
		}
		else
		{
			// Zero stuffing for upsampling: every other sample is zero, so write zeros to the ring buffer for the odd cycles where reverb is not processed.
			upsamplerRingbufferL[upsamplerRingbufferIndex] = 0;
			upsamplerRingbufferR[upsamplerRingbufferIndex] = 0;
		}

		// Upsample from 22050 Hz back to 44100 Hz
		// In the PSX SPU reverb unit, this is achieved with "zero stuffing" every other element and convolving with the same FIR filter.
		s32 upsampledL = 0;
		s32 upsampledR = 0;
		for (size_t j = 0; j < COUNTOF_ARRAY(kFiniteImpulseResponse); j++)
		{
			int inputIndex = (int)upsamplerRingbufferIndex - (int)j;
			if (inputIndex < 0)
				inputIndex += COUNTOF_ARRAY(upsamplerRingbufferL); // Wrap around; circular buffer

			s32 h = (s32)kFiniteImpulseResponse[j]; // impulse response
			upsampledL += (s32)upsamplerRingbufferL[inputIndex] * h;
			upsampledR += (s32)upsamplerRingbufferR[inputIndex] * h;
		}

		// The coefficients in the FIR table represent volume multipler N/0x8000 so should rescale by dividing by 0x8000 (shifting right by 15) to compensate for this.
		// However, because of the zero stuffing, every other sample in the input is zero, the volume will be halved compared to the downsampled signal,
		// so instead of rescaling by 0x8000, we can rescale by 0x4000 (shifting right by 14)
		upsampledL >>= 14;
		upsampledR >>= 14;

		HP_DEBUG_ASSERT(upsampledBufferLenSamples + 2 <= upsampledBufferCapacitySamples); // should never fail by construction
		upsampledL = saturateS32toS16(upsampledL);
		upsampledR = saturateS32toS16(upsampledR);
		upsampledBuffer[upsampledBufferLenSamples++] = (s16)upsampledL;
		upsampledBuffer[upsampledBufferLenSamples++] = (s16)upsampledR;

		// Mix the dry input with the reverb output. The reverb output is already scaled by vLOUT and vROUT, so just need to add them together.
		s32 mixedL = (s32)inputSampleL + upsampledL;
		s32 mixedR = (s32)inputSampleR + upsampledR;
		HP_DEBUG_ASSERT(mixedOutputBufferLenSamples + 2 <= mixedOutputBufferCapacitySamples); // should never fail by construction
		mixedOutputBuffer[mixedOutputBufferLenSamples++] = saturateS32toS16(mixedL);
		mixedOutputBuffer[mixedOutputBufferLenSamples++] = saturateS32toS16(mixedR);

		// Increment this after calculating the output sample for the current input, so that the current input is included in the FIR filter calculation for the next output sample.
		downsamplerRingbufferIndex++;
		if (downsamplerRingbufferIndex == COUNTOF_ARRAY(downsamplerRingbufferL))
			downsamplerRingbufferIndex = 0;

		// Increment this after calculating the output sample for the current input, so that the current input is included in the FIR filter calculation for the next output sample.
		upsamplerRingbufferIndex++;
		if (upsamplerRingbufferIndex == COUNTOF_ARRAY(upsamplerRingbufferL))
			upsamplerRingbufferIndex = 0;
	}

	delete[] input;
	input = nullptr;

	saveBuffer("downsampled.raw", downsampledBuffer, downsampledBufferLenSamples);
	delete[] downsampledBuffer;
	downsampledBuffer = nullptr;

	saveBuffer("same_side_reflection.raw", sameSideReflectionBuffer, sameSideReflectionBufferLenSamples);
	delete[] sameSideReflectionBuffer;
	sameSideReflectionBuffer = nullptr;

	saveBuffer("different_side_reflection.raw", differentSideReflectionBuffer, differentSideReflectionBufferLenSamples);
	delete[] differentSideReflectionBuffer;
	differentSideReflectionBuffer = nullptr;

	saveBuffer("early_echo.raw", earlyEchoBuffer, earlyEchoBufferLenSamples);
	delete[] earlyEchoBuffer;
	earlyEchoBuffer = nullptr;

	saveBuffer("reverb_out.raw", reverbOutputBuffer, reverbOutputBufferLenSamples);
	delete[] reverbOutputBuffer;
	reverbOutputBuffer = nullptr;

	saveBuffer("upsampled.raw", upsampledBuffer, upsampledBufferLenSamples);
	delete[] upsampledBuffer;
	upsampledBuffer = nullptr;

	saveBuffer("mixed_output.raw", mixedOutputBuffer, mixedOutputBufferLenSamples);
	delete[] mixedOutputBuffer;
	mixedOutputBuffer = nullptr;

    return EXIT_SUCCESS;
}
