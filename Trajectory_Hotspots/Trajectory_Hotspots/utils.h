#pragma once

//From: https://bitbashing.io/comparing-floats.html
//Returns the units of least precision between two floats
int32_t ulpsDistance(const float a, const float b);

//Check for nearly equal float values
bool nearlyEqual(float a, float b, float fixedEpsilon = 0.0000001f, int ulpsEpsilon = 3);