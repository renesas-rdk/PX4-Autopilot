/**
 * @file notch_filter.cpp
 * @brief CMSIS-DSP notch filter implementation
 *
 * Drop-in replacement for math::NotchFilter<float> using ARM CMSIS-DSP
 * for hardware acceleration on ARM Cortex processors.
 */


#if defined(__PX4_FREERTOS)

#include "cmsis_dsp_filters.hpp"
#include <math.h>
#include <float.h>

namespace cmsis_dsp
{

bool NotchFilterDSP::setParameters(float sample_freq, float notch_freq, float bandwidth)
{
	// Validate inputs (same logic as original NotchFilter)
	if ((sample_freq <= 0.f) || (notch_freq <= 0.f) || (bandwidth <= 0.f) || (notch_freq >= sample_freq / 2.0f) ||
	    !isfinite(sample_freq) || !isfinite(notch_freq) || !isfinite(bandwidth)) {

		reset();
		return false;
	}

	// Ensure minimum frequencies
	const float freq_min = sample_freq * 0.001f;
	const float notch_freq_new = fmaxf(notch_freq, freq_min);
	const float bandwidth_new = fmaxf(bandwidth, freq_min);

	// Store parameters
	_sample_freq = sample_freq;
	_notch_freq = notch_freq_new;
	_bandwidth = bandwidth_new;

	// Calculate notch filter coefficients using the same algorithm as original
	// This is a 2nd-order IIR notch filter designed using the bilinear transform

	const float alpha = tanf(M_PI_F * _bandwidth / _sample_freq);
	const float beta = -cosf(2.f * M_PI_F * _notch_freq / _sample_freq);
	const float a0_inv = 1.f / (alpha + 1.f);

	// Feedforward coefficients
	_b0 = a0_inv;
	_b1 = 2.f * beta * a0_inv;
	_b2 = a0_inv;

	// Feedback coefficients (normalized, a0 = 1)
	_a1 = _b1;  // For notch filter, a1 = b1
	_a2 = (1.f - alpha) * a0_inv;

	// Validate coefficients
	if (!isfinite(_b0) || !isfinite(_b1) || !isfinite(_b2) || !isfinite(_a1) || !isfinite(_a2)) {
		reset();
		return false;
	}

	// Set coefficients in the underlying biquad filter
	_filter.setCoefficients(_b0, _b1, _b2, _a1, _a2);

	return true;
}

void NotchFilterDSP::reset(float sample)
{
	const float input = isfinite(sample) ? sample : 0.0f;

	// Reset filter state to steady-state for this input
	_filter.reset(input);
}

float NotchFilterDSP::getMagnitudeResponse(float frequency) const
{
	if (_sample_freq <= 0.0f) {
		return 1.0f;
	}

	// Calculate frequency response using the same method as original
	float w = 2.f * M_PI_F * frequency / _sample_freq;

	float numerator = _b0 * _b0 + _b1 * _b1 + _b2 * _b2
			  + 2.f * (_b0 * _b1 + _b1 * _b2) * cosf(w)
			  + 2.f * _b0 * _b2 * cosf(2.f * w);

	float denominator = 1.f + _a1 * _a1 + _a2 * _a2
			    + 2.f * (_a1 + _a1 * _a2) * cosf(w)
			    + 2.f * _a2 * cosf(2.f * w);

	return (denominator > 1e-6f) ? sqrtf(numerator / denominator) : 0.0f;
}

void NotchFilterDSP::getCoefficients(float &b0, float &b1, float &b2, float &a1, float &a2) const
{
	b0 = _b0;
	b1 = _b1;
	b2 = _b2;
	a1 = _a1;
	a2 = _a2;
}

void NotchFilterDSP::setCoefficients(float b0, float b1, float b2, float a1, float a2)
{
	_b0 = b0;
	_b1 = b1;
	_b2 = b2;
	_a1 = a1;
	_a2 = a2;

	_filter.setCoefficients(b0, b1, b2, a1, a2);
}

} // namespace cmsis_dsp

#endif /* __PX4_FREERTOS */
