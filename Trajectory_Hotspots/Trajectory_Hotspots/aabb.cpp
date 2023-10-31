#include "pch.h"
#include "aabb.h"

AABB AABB::operator+(const AABB& other) const { return {min + other.min, max + other.max}; }
AABB AABB::operator-(const AABB& other) const { return {min - other.min, max - other.max}; }
AABB AABB::operator*(const Float& scalar) const { return {min * scalar, max * scalar}; }
AABB AABB::operator/(const Float& scalar) const { return {min / scalar, max / scalar}; }

AABB AABB::operator-() const { return {-min, -max}; }

AABB& AABB::operator+=(const AABB& other)
{
    min += other.min;
    max += other.max;
    return *this;
}

AABB& AABB::operator-=(const AABB& other)
{
    min -= other.min;
    max -= other.max;
    return *this;
}

AABB& AABB::operator*=(const Float& scalar)
{
    min *= scalar;
    max *= scalar;
    return *this;
}

AABB& AABB::operator/=(const Float& scalar)
{
    //TODO: perhaps check for division by zero?
    min /= scalar;
    max /= scalar;
    return *this;
}
