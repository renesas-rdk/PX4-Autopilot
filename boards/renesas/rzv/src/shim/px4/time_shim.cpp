/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file time_shim.cpp
 * @brief Time port shim implementation for Renesas RZ/V2H
 */


#if defined(__PX4_FREERTOS)

#include <ctime>
#include <cstdint>

namespace
{
bool is_leap_year(int year)
{
	return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

int64_t days_since_epoch_utc(int year, int month, int day)
{
	/* Normalize month into range [0, 11] and adjust year accordingly */
	int y = year;
	int m = month;

	if (m < 0 || m > 11) {
		int adjust = m / 12;
		int rem = m % 12;

		if (rem < 0) {
			rem += 12;
			adjust -= 1;
		}

		y += adjust;
		m = rem;
	}

	static const int days_before_month[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

	int64_t days = 0;

	if (y >= 1970) {
		for (int yr = 1970; yr < y; yr++) {
			days += is_leap_year(yr) ? 366 : 365;
		}

	} else {
		for (int yr = y; yr < 1970; yr++) {
			days -= is_leap_year(yr) ? 366 : 365;
		}
	}

	days += days_before_month[m];

	if ((m > 1) && is_leap_year(y)) {
		days += 1;
	}

	days += (int64_t)day - 1;
	return days;
}
} // namespace

time_t mktime(struct tm *tm)
{
	if (tm == nullptr) {
		return static_cast<time_t>(-1);
	}

	const int year = tm->tm_year + 1900;
	const int month = tm->tm_mon;
	const int day = tm->tm_mday;

	const int64_t days = days_since_epoch_utc(year, month, day);
	const int64_t seconds = (int64_t)tm->tm_sec +
				((int64_t)tm->tm_min * 60) +
				((int64_t)tm->tm_hour * 3600) +
				(days * 86400);

	/* Update derived fields */
	tm->tm_yday = (int)(days - days_since_epoch_utc(year, 0, 1));
	int wday = (int)((days + 4) % 7); // Jan 1 1970 was a Thursday (4)

	if (wday < 0) {
		wday += 7;
	}

	tm->tm_wday = wday;
	tm->tm_isdst = 0;

	return (time_t)seconds;
}

extern "C" {

struct tm *gmtime_r(const time_t *timer, struct tm *result)
{
	if (!timer || !result) {
		return nullptr;
	}

	struct tm *tmp = gmtime(timer);

	if (tmp == nullptr) {
		return nullptr;
	}

	*result = *tmp;
	return result;
}

struct tm *localtime_r(const time_t *timer, struct tm *result)
{
	if (!timer || !result) {
		return nullptr;
	}

	struct tm *tmp = localtime(timer);

	if (tmp == nullptr) {
		return nullptr;
	}

	*result = *tmp;
	return result;
}

} // extern "C"

#endif /* __PX4_FREERTOS */
