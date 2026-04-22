/**
 * @file lowpass_filter.cpp
 * @brief CMSIS-DSP 2nd-order low-pass filter implementation
 *
 * Drop-in replacement for math::LowPassFilter2p<float> using ARM CMSIS-DSP
 * for hardware acceleration on ARM Cortex processors.
 */


#if defined(__PX4_FREERTOS)

#include "cmsis_dsp_filters.hpp"
#include <math.h>
#include <float.h>

namespace cmsis_dsp
{

LowPassFilter2pDSP::LowPassFilter2pDSP(float sample_freq, float cutoff_freq)
{
	set_cutoff_frequency(sample_freq, cutoff_freq);
}

void LowPassFilter2pDSP::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
	// Validate inputs (same logic as original LowPassFilter2p)
	if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.0f) ||
	    !isfinite(sample_freq) || !isfinite(cutoff_freq)) {

		disable();
		return;
	}

	// Store parameters
	_sample_freq = sample_freq;
	_cutoff_freq = fmaxf(cutoff_freq, sample_freq * 0.001f);  // Ensure reasonable cutoff

	// Calculate 2nd-order Butterworth low-pass filter coefficients
	// This uses the bilinear transform method, same as original implementation

	const float fr = _sample_freq / _cutoff_freq;
	const float ohm = tanf(M_PI_F / fr);
	const float c = 1.f + 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm;

	// Feedforward coefficients
	const float b0 = ohm * ohm / c;
	const float b1 = 2.f * b0;
	const float b2 = b0;

	// Feedback coefficients (normalized, a0 = 1)
	const float a1 = 2.f * (ohm * ohm - 1.f) / c;
	const float a2 = (1.f - 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm) / c;

	// Check for valid coefficients
	if (!isfinite(b0) || !isfinite(b1) || !isfinite(b2) || !isfinite(a1) || !isfinite(a2)) {
		disable();
		return;
	}

	// Set coefficients in the underlying biquad filter
	_filter.setCoefficients(b0, b1, b2, a1, a2);

	// Reset delay elements on filter change (same as original)
	_filter.reset(0.0f);
}

float LowPassFilter2pDSP::reset(float sample)
{
	// Ensure finite sample
	const float input = isfinite(sample) ? sample : 0.0f;

	// Reset filter state to steady-state for this input
	_filter.reset(input);

	// Return filtered value (first sample after reset)
	return apply(input);
}

void LowPassFilter2pDSP::disable()
{
	// Disable filtering by setting passthrough coefficients
	_sample_freq = 0.f;
	_cutoff_freq = 0.f;

	// Passthrough: y[n] = x[n]
	_filter.setCoefficients(
		1.f,  // b0 = 1
		0.f,  // b1 = 0
		0.f,  // b2 = 0
		0.f,  // a1 = 0
		0.f   // a2 = 0
	);

	_filter.reset(0.0f);
}

float LowPassFilter2pDSP::getMagnitudeResponse(float frequency) const
{
	return _filter.getMagnitudeResponse(frequency, _sample_freq);
}

} // namespace cmsis_dsp

#endif /* __PX4_FREERTOS */
