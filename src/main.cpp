/*

x[n] input signal
y[n] output signal
h[n] impulse response of the system
Convolution takes two signals and produces a third signal
x[n] * h[n] = y[n]  n.b. * denotes convolution, not multiplication
x[n] is convolved with h[n] to produce y[n].

Note that n just means some index into the signal. It could be written as i.

FIR Finite Impulse Response

*/

#include "Log.h"
#include "Helpers.h" // HP_UNUSED
#include "Types.h"

#include <stdlib.h> // EXIT_SUCCESS, EXIT_FAILURE

static void printUsage(const char* programName)
{
	LOG_INFO("Usage: %s <filename>\n", programName);
}

static void process(const s16* input, size_t inputLength, s16* output, size_t outputLength)
{
	// Placeholder: Copy input to output (identity operation)
	for (size_t i = 0; i < outputLength; ++i)
	{
		output[i] = (i < inputLength) ? input[i] : 0; // Zero-pad if output is longer than input
	}
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

	size_t outputLength = inputLength; // #TODO: Determine output length based on convolution parameters
	s16* output = new s16[outputLength];

	process(input, inputLength, output, outputLength);

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
