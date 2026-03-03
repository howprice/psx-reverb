/*

x[n] input signal
y[n] output signal
h[n] impulse response of the system
Convolution takes two signals and produces a third signal
x[n] * h[n] = y[n]  n.b. * denotes convolution, not multiplication
x[n] is convolved with h[n] to produce y[n].

Note that n just means some index into the signal. It could be written as i.

For full convolution, the length of the output signal is equal to the length of the input signal, plus the length of the impulse response, minus one.

FIR Finite Impulse Response

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

static bool downsample(
	const s16* input, size_t inputLength,
	s16* output, size_t outputCapacity, size_t& outputLength)
{
	// 39-tap PSX SPU FIR filter
	// https://psx-spx.consoledev.net/soundprocessingunitspu/#reverb-buffer-resampling
	// Each coefficient is a volume multipler N/0x8000, so after multiplying by the input sample, the result needs to be shift right by 15 to rescale.
	// Note that every other sample except the peak is zero, so this is effectively a 20-tap filter with zeros interleaved for downsampling by 2.
	static const s16 kImpulseResponse[] =
	{
		-0x0001,  0x0000,  0x0002,  0x0000, -0x000A,  0x0000,  0x0023,  0000,
		-0x0067,  0x0000,  0x010A,  0x0000, -0x0268,  0x0000,  0x0534,  0000,
		-0x0B90,  0x0000,  0x2806,  0x4000,  0x2806,  0x0000, -0x0B90,  0000,
		 0x0534,  0x0000, -0x0268,  0x0000,  0x010A,  0x0000, -0x0067,  0000,
		 0x0023,  0x0000, -0x000A,  0x0000,  0x0002,  0x0000, -0x0001,
	};
	static_assert(COUNTOF_ARRAY(kImpulseResponse) == 39);

	outputLength = inputLength / 2; // Downsample by 2, so output length is half of input length
	if (outputCapacity < outputLength)
		return false;

	// Resample from 44100 to 22050 Hz by convolving with impulse response and downsampling by 2.
	for (size_t i = 0; i < outputLength; i++)
	{
		s32 sum = 0;
		for (size_t j = 0; j < COUNTOF_ARRAY(kImpulseResponse); j++)
		{
			// Step input by 2 for downsampling.
			// 
			// n.b. The filter is not centered so introduced a delay. To center the filter input index would need to
			// be offset by 19 (half of 39) but this would cause negative input index for the first 19 output samples,
			// so we start with input index of 0 and let the filter introduce the delay.
			s64 inputIndex = (s64)(2 * i) - (s64)j;

			s16 inputSample = (inputIndex >= 0 && inputIndex < (s64)inputLength) ? input[inputIndex] : 0;
			sum += (s32)inputSample * (s32)kImpulseResponse[j];
		}

		sum >>= 15; // Rescale by 0x8000
		output[i] = (s16)Clamp(sum, (s32)INT16_MIN, (s32)INT16_MAX); // Saturate to 16-bit signed range
	}

	return true;
}

int main(int argc, char** argv)
{
	// Parse filename from command line arguments
	if (argc < 2)
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	const char* filename = argv[1];
	FILE* inputFile = fopen(filename, "rb");
	if (!inputFile)
	{
		LOG_ERROR("Failed to open file: %s\n", filename);
		return EXIT_FAILURE;
	}

	// Check file size matches expected memory card size
	fseek(inputFile, 0, SEEK_END);
	long fileSize = ftell(inputFile);
	fseek(inputFile, 0, SEEK_SET); // Seek back to beginning before reading

	if ((fileSize % 2) != 0)
	{
		LOG_ERROR("Expect signed 16-bit input data. File size is not a multiple of 2 bytes: %s\n", filename);
		fclose(inputFile);
		return EXIT_FAILURE;
	}

	size_t inputLength = fileSize / 2; // Each sample is 2 bytes (16 bits)

	s16* input = new s16[inputLength];

	if (fread(input, 2, inputLength, inputFile) != inputLength)
	{
		LOG_ERROR("Failed to read file: %s\n", filename);
		fclose(inputFile);
		return EXIT_FAILURE;
	}
	fclose(inputFile);
	inputFile = nullptr;

	// For full convolution, the length of the output signal is equal to the length of the input signal, plus the length of the impulse response, minus one.
	// However we are downsampling by 2, so the output length is half of the input length. We can calculate the output capacity based on this.
	size_t outputCapacity = inputLength / 2; // Downsample by 2, so output capacity is half of input length
	s16* output = new s16[outputCapacity];

	size_t outputLength;
	if (!downsample(input, inputLength, output, outputCapacity, /*out*/outputLength))
	{
		LOG_ERROR("Downsampling failed\n");
		return EXIT_FAILURE;
	}

	// Save output
	FILE* outputFile = fopen("output.bin", "wb");
	if (!outputFile)
	{
		LOG_ERROR("Failed to open output file for writing: output.bin\n");
		return EXIT_FAILURE;
	}

	if (fwrite(output, 2, outputLength, outputFile) != outputLength)
	{
		LOG_ERROR("Failed to write output file: output.bin\n");
		fclose(outputFile);
		return EXIT_FAILURE;
	}

	LOG_INFO("Output written to output.bin\n");

	fclose(outputFile);
	outputFile = nullptr;

	delete[] input;
	input = nullptr;

	delete[] output;
	output = nullptr;

    return EXIT_SUCCESS;
}
