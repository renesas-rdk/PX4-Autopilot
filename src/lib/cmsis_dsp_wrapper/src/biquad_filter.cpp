/**
 * @file biquad_filter.cpp
 * @brief CMSIS-DSP biquad filter wrapper implementation
 */


#if defined(__PX4_FREERTOS)

#include "cmsis_dsp_filters.hpp"
#include <string.h>
#include <math.h>

namespace cmsis_dsp
{

BiquadFilterDSP::BiquadFilterDSP()
	: _numStages(1)
{
	// Initialize with passthrough coefficients
	_coeffs[0] = 1.0f;  // b0
	_coeffs[1] = 0.0f;  // b1
	_coeffs[2] = 0.0f;  // b2
	_coeffs[3] = 0.0f;  // a1
	_coeffs[4] = 0.0f;  // a2

	// Zero state
	memset(_state, 0, sizeof(_state));

	// Initialize CMSIS-DSP instance
	arm_biquad_cascade_df2T_init_f32(&_instance, _numStages, _coeffs, _state);
}

void BiquadFilterDSP::setCoefficients(float b0, float b1, float b2, float a1, float a2)
{
	// Store coefficients in CMSIS-DSP format
	_coeffs[0] = b0;
	_coeffs[1] = b1;
	_coeffs[2] = b2;
	_coeffs[3] = a1;
	_coeffs[4] = a2;

	// Reinitialize the filter instance with new coefficients
	// Note: This preserves the state, allowing coefficient updates without reset
	arm_biquad_cascade_df2T_init_f32(&_instance, _numStages, _coeffs, _state);
}

float BiquadFilterDSP::apply(float input)
{
	float output;
	// Process single sample using CMSIS-DSP
	arm_biquad_cascade_df2T_f32(&_instance, &input, &output, 1);
	return output;
}

void BiquadFilterDSP::applyArray(const float *input, float *output, uint32_t numSamples)
{
	// CMSIS-DSP expects non-const input pointer, but doesn't modify it
	arm_biquad_cascade_df2T_f32(&_instance, const_cast<float *>(input), output, numSamples);
}

void BiquadFilterDSP::reset(float value)
{
	// Reset state to steady-state value based on coefficients
	// For steady-state: y[n] = y[n-1] = y[n-2] = value
	// From difference equation: y = (b0 + b1 + b2) / (1 + a1 + a2) * x

	float denominator = 1.0f + _coeffs[3] + _coeffs[4];  // 1 + a1 + a2

	if (fabsf(denominator) > 1e-6f) {
		// Calculate steady-state for given input
		float steadyState = value * (_coeffs[0] + _coeffs[1] + _coeffs[2]) / denominator;

		// For Direct Form II Transpose, state variables represent:
		// state[0] = b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
		// state[1] = b2*x[n-1] - a2*y[n-1]

		// At steady state with x = y = steadyState:
		_state[0] = (_coeffs[1] + _coeffs[2] - _coeffs[3] - _coeffs[4]) * steadyState;
		_state[1] = (_coeffs[2] - _coeffs[4]) * steadyState;
		_state[2] = 0.0f;  // Not used for single stage
		_state[3] = 0.0f;  // Not used for single stage
	} else {
		// Denominator near zero, just zero the state
		memset(_state, 0, sizeof(_state));
	}
}

float BiquadFilterDSP::getMagnitudeResponse(float frequency, float sample_freq) const
{
	if (sample_freq <= 0.0f || frequency < 0.0f) {
		return 1.0f;
	}

	// Calculate normalized frequency
	float omega = 2.0f * M_PI_F * frequency / sample_freq;
	float cos_omega = cosf(omega);
	float sin_omega = sinf(omega);

	// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
	// z = e^(j*omega) where omega = 2*pi*f/fs

	// Numerator: b0 + b1*cos(omega) - j*b1*sin(omega) + b2*cos(2*omega) - j*b2*sin(2*omega)
	float b_real = _coeffs[0] + _coeffs[1] * cos_omega + _coeffs[2] * cosf(2.0f * omega);
	float b_imag = -_coeffs[1] * sin_omega - _coeffs[2] * sinf(2.0f * omega);

	// Denominator: 1 + a1*cos(omega) - j*a1*sin(omega) + a2*cos(2*omega) - j*a2*sin(2*omega)
	float a_real = 1.0f + _coeffs[3] * cos_omega + _coeffs[4] * cosf(2.0f * omega);
	float a_imag = -_coeffs[3] * sin_omega - _coeffs[4] * sinf(2.0f * omega);

	// |H| = |numerator| / |denominator|
	float b_mag = sqrtf(b_real * b_real + b_imag * b_imag);
	float a_mag = sqrtf(a_real * a_real + a_imag * a_imag);

	return (a_mag > 1e-6f) ? (b_mag / a_mag) : 0.0f;
}

} // namespace cmsis_dsp

#endif /* __PX4_FREERTOS */
