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

TODO
====
- Reflection filters
- Comb filters
- All pass filters
- Save mixed (dry + wet) output

*/

#include "Log.h"
#include "hp_assert.h"
#include "MathsHelpers.h"
#include "ArrayHelpers.h"
#include "Helpers.h" // HP_UNUSED
#include "Types.h"

#include <stdlib.h> // EXIT_SUCCESS, EXIT_FAILURE

static void printUsage(const char* programName)
{
	LOG_INFO("Usage: %s <filename>\n", programName);
}

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
// All src/dst/disp/base registers are addresses in SPU memory (divided by 8)
// src/dst are relative to the current buffer address
// disp registers are relative to src registers
// The base register defines the start address of the reverb buffer (the end address is fixed, at 7FFFEh).
// Writing a value to mBASE does additionally set the current buffer address to that value.
// 
// https://psx-spx.consoledev.net/soundprocessingunitspu/#spu-reverb-registers
//
struct ReverbRegisters
{
	// SPU registers
	s16 vLOUT;
	s16 vROUT;
	u16 mBASE;

	// Reverb unit registers
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
	s16 vLIN;
	s16 vRIN;
};

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

int main(int argc, char** argv)
{
	// Parse filename from command line arguments
	if (argc < 2)
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	const char* inputFilename = argv[1];
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

	ReverbRegisters regs{};

	// #TODO: Set reg values to proper values e.g. from https://psx-spx.consoledev.net/soundprocessingunitspu/#reverb-examples or BIOS.

	// For volumes:
	// - INT16_MAX 0x7fff is max volume
	// - INT16_MIN 0x8000 -32768 is max negative volume and will invert the signal
	regs.vLIN = 0x7fff;
	regs.vRIN = 0x7fff;
	regs.vLOUT = 0x7fff;
	regs.vROUT = 0x7fff;

	s16 downsamplerRingbufferL[kFIRFilterSize]{};
	s16 downsamplerRingbufferR[kFIRFilterSize]{};
	unsigned int downsamplerRingbufferIndex = 0; // Circular buffer index for downsampler input

	s16 upsamplerRingbufferL[kFIRFilterSize]{};
	s16 upsamplerRingbufferR[kFIRFilterSize]{};
	unsigned int upsamplerRingbufferIndex = 0; // Circular buffer index for upsampler input

	// Create some intermediate buffers for debugging

	// Debug buffer
	// For full convolution, the length of the output signal is equal to the length of the input signal, plus the length of the impulse response, minus one.
	// However we are downsampling by 2, so the output length is half of the input length. We can calculate the output capacity based on this.
	const size_t downsampledBufferCapacitySamples = inputLengthSamples / 2; // Downsample by 2, so output capacity is half of input length
	s16* downsampledBuffer = new s16[downsampledBufferCapacitySamples]; // single buffer containing stereo samplees
	size_t downsampledBufferLenSamples = 0; // Will be set by downsample function

	// Debug buffer
	// Reverb is performed at 22050 Hz
	const size_t reverbOutputBufferCapacitySamples = downsampledBufferCapacitySamples;
	s16* reverbOutputBuffer = new s16[reverbOutputBufferCapacitySamples];
	size_t reverbOutputBufferLenSamples = 0;

	// Debug buffer
	// Final output from reverb output upsampled back to 44100 Hz
	const size_t upsampledBufferCapacitySamples = inputLengthSamples; // Upsample by 2, so output capacity is double the downsampled length
	s16* upsampledBuffer = new s16[upsampledBufferCapacitySamples]; // Upsample by 2, so output capacity is double the downsampled length
	size_t upsampledBufferLenSamples = 0;

	// This would be called at 44100 Hz, every 768 cycles on PSX
	for (unsigned int inputFrameIndex = 0; inputFrameIndex < inputLengthFrames; inputFrameIndex++)
	{
		// The downsampler ring buffers must be fed every tick.
		downsamplerRingbufferL[downsamplerRingbufferIndex] = input[2 * inputFrameIndex]; // Left sample of current stereo frame
		downsamplerRingbufferR[downsamplerRingbufferIndex] = input[2 * inputFrameIndex + 1]; // Right sample of current stereo frame

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
			s32 Lin = ((s32)regs.vLIN * LeftInput) >> 15; // / 0x8000;
			s32 Rin = ((s32)regs.vRIN * RightInput) >> 15; // / 0x8000;
			
			// #TEMP: Just copy the downsampled buffer into the reverb output buffer for now, will implement actual reverb processing later.
			// #TODO: Implement reverb chain
			s32 Lout = Lin;
			s32 Rout = Rin;

			// Apply output volume vLOUT, vROUT
			s32 LeftOutput = (Lout * (s32)regs.vLOUT) >> 15; // / 0x8000;
			s32 RightOutput = (Rout * (s32)regs.vROUT) >> 15; // / 0x8000;

			// Write the new reverb output values into the upsampler ring buffers
			upsamplerRingbufferL[upsamplerRingbufferIndex] = saturateS32toS16(LeftOutput);
			upsamplerRingbufferR[upsamplerRingbufferIndex] = saturateS32toS16(RightOutput);

			// Write to debug reverb output buffer at 22050 Hz
			HP_DEBUG_ASSERT(reverbOutputBufferLenSamples + 2 <= reverbOutputBufferCapacitySamples); // should never fail by construction
			reverbOutputBuffer[reverbOutputBufferLenSamples++] = upsamplerRingbufferL[upsamplerRingbufferIndex];
			reverbOutputBuffer[reverbOutputBufferLenSamples++] = upsamplerRingbufferR[upsamplerRingbufferIndex];
		}
		else
		{
			// Zero stuffing for upsampling: every other sample is zero, so write zeros to the ring buffer for the odd cycles where reverb is not processed.
			upsamplerRingbufferL[upsamplerRingbufferIndex] = 0;
			upsamplerRingbufferR[upsamplerRingbufferIndex] = 0;
		}

		// Upsample from 22050 Hz back to 44100 Hz
		// In the PSX SPU reverb unit, this is achieved with "zero stuffing" every other element and convolving with the same FIR filter.
		{
			s32 sumL = 0;
			s32 sumR = 0;
			for (size_t j = 0; j < COUNTOF_ARRAY(kFiniteImpulseResponse); j++)
			{
				int inputIndex = (int)upsamplerRingbufferIndex - (int)j;
				if (inputIndex < 0)
					inputIndex += COUNTOF_ARRAY(upsamplerRingbufferL); // Wrap around; circular buffer

				s32 h = (s32)kFiniteImpulseResponse[j]; // impulse response
				sumL += (s32)upsamplerRingbufferL[inputIndex] * h;
				sumR += (s32)upsamplerRingbufferR[inputIndex] * h;
			}

			// The coefficients in the FIR table represent volume multipler N/0x8000 so should rescale by dividing by 0x8000 (shifting right by 15) to compensate for this.
			// However, because of the zero stuffing, every other sample in the input is zero, the volume will be halved compared to the downsampled signal,
			// so instead of rescaling by 0x8000, we can rescale by 0x4000 (shifting right by 14)
			sumL >>= 14;
			sumR >>= 14;

			HP_DEBUG_ASSERT(upsampledBufferLenSamples + 2 <= upsampledBufferCapacitySamples); // should never fail by construction
			upsampledBuffer[upsampledBufferLenSamples++] = saturateS32toS16(sumL);
			upsampledBuffer[upsampledBufferLenSamples++] = saturateS32toS16(sumR);
		}

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

	saveBuffer("reverb_out.raw", reverbOutputBuffer, reverbOutputBufferLenSamples);
	delete[] reverbOutputBuffer;
	reverbOutputBuffer = nullptr;

	saveBuffer("upsampled.raw", upsampledBuffer, upsampledBufferLenSamples);
	delete[] upsampledBuffer;
	upsampledBuffer = nullptr;

    return EXIT_SUCCESS;
}
