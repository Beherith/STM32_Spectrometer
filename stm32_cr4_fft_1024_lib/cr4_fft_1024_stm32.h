/*

x[N] be the time signal samples. To use the FFT functions of the DSP library, the
following conditions must be satisfied:
? N is a power of 4
? All the signal samples must be 32-bit data containing the 16-bit real part followed by the
16-bit imaginary part (in the little Endian order: imaginary_real).


*/


/* 1024 points */
extern "C" {
	void cr4_fft_1024_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
}

