#pragma once

//From: https://bitbashing.io/comparing-floats.html
//Returns the units of least precision between two floats
int32_t ulps_distance(const float a, const float b);

//Check for nearly equal float values
bool nearly_equal(const float a, const float b, const float fixedEpsilon = 0.0000001f, const int ulpsEpsilon = 3);

bool nearly_less(const float a, const float b);

bool nearly_greater(const float a, const float b);

bool nearly_greater_or_equal(const float a, const float b);

bool nearly_less_or_equal(const float a, const float b);
