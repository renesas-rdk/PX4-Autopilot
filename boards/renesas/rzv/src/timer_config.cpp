/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include <px4_platform_common/px4_config.h>
#include <board_config.h>

// Dummy timer_config for generate_params.py to parse. 
// RZ/V2H doesn't use this file for actual PWM generation, it uses board_pwm_out.cpp.
// However, the module.yaml specifies "generator: pwm" which requires this file
// to exist so QGC's Actuator UI can get the correct JSON metadata.

io_timers_t io_timers[1] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index0, DMA::Stream0, DMA::Channel0}),
};

timer_io_channels_t timer_io_channels[4] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),
};

#endif /* __PX4_FREERTOS */
