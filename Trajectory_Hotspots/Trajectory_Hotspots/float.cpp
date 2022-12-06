#include "pch.h"
#include "float.h"


//From: https://bitbashing.io/comparing-floats.html
//Returns the units of least precision between two floats
int32_t ulps_distance(const float a, const float b)
{
    // Save work if the floats are equal.
    // Also handles +0 == -0
    if (a == b) return 0;

    constexpr auto max = std::numeric_limits<int32_t>::max();

    // Max distance for NaN
    if (isnan(a) || isnan(b)) return max;

    // If one's infinite and they're not equal, max distance.
    if (isinf(a) || isinf(b)) return max;

    int32_t ia, ib;
    memcpy(&ia, &a, sizeof(float));
    memcpy(&ib, &b, sizeof(float));

    // Don't compare differently-signed floats.
    if ((ia < 0) != (ib < 0)) return max;

    // Return the absolute value of the distance in ULPs.
    int32_t distance = ia - ib;
    if (distance < 0) distance = -distance;
    return distance;
}



//Check for nearly equal float values
bool Float::nearly_equal(const float a, const float b, const float fixedEpsilon, const int ulpsEpsilon)
{
    // Handle the near-zero case.
    const float difference = fabs(a - b);
    if (difference <= fixedEpsilon) return true;

    return ulps_distance(a, b) <= ulpsEpsilon;
}

bool Float::nearly_less(const float a, const float b)
{
    //If equal
    if (nearly_equal(a, b))
    {
        return false;
    }
    //If less then
    else if (a < b)
    {
        return true;
    }

    //Greater then
    return false;
}

bool Float::nearly_greater(const float a, const float b)
{
    return (!nearly_less(a, b) && !nearly_equal(a, b));
}

bool Float::nearly_greater_or_equal(const float a, const float b)
{
    return nearly_equal(a, b) || nearly_greater(a, b);
}

bool Float::nearly_less_or_equal(const float a, const float b)
{
    return nearly_equal(a, b) || nearly_less(a, b);
}

Float& Float::operator=(const Float& other)
{
    // TODO: insert return statement here
}

Float& Float::operator=(float value)
{
    // TODO: insert return statement here
}

Float::operator float() const
{
}

bool Float::operator==(const Float& other) const
{
    return false;
}

bool Float::operator==(float value) const
{
    return false;
}

bool Float::operator!=(const Float& other) const
{
    return false;
}

bool Float::operator!=(float value) const
{
    return false;
}

bool Float::operator<(const Float& other) const
{
    return false;
}

bool Float::operator<(float value) const
{
    return false;
}

bool Float::operator>(const Float& other) const
{
    return false;
}

bool Float::operator>(float value) const
{
    return false;
}

bool Float::operator<=(const Float& other) const
{
    return false;
}

bool Float::operator<=(float value) const
{
    return false;
}

bool Float::operator>=(const Float& other) const
{
    return false;
}

bool Float::operator>=(float value) const
{
    return false;
}

Float Float::operator+(const Float& other) const
{
    return Float();
}

Float Float::operator+(float value) const
{
    return Float();
}

Float Float::operator-(const Float& other) const
{
    return Float();
}

Float Float::operator-(float value) const
{
    return Float();
}

Float Float::operator*(const Float& other) const
{
    return Float();
}

Float Float::operator*(float value) const
{
    return Float();
}

Float Float::operator/(const Float& other) const
{
    return Float();
}

Float Float::operator/(float value) const
{
    return Float();
}

Float& Float::operator+=(const Float& other)
{
    // TODO: insert return statement here
}

Float& Float::operator+=(float value)
{
    // TODO: insert return statement here
}

Float& Float::operator-=(const Float& other)
{
    // TODO: insert return statement here
}

Float& Float::operator-=(float value)
{
    // TODO: insert return statement here
}

Float& Float::operator*=(const Float& other)
{
    // TODO: insert return statement here
}

Float& Float::operator*=(float value)
{
    // TODO: insert return statement here
}

Float& Float::operator/=(const Float& other)
{
    // TODO: insert return statement here
}

Float& Float::operator/=(float value)
{
    // TODO: insert return statement here
}

Float Float::operator-() const
{
    return Float();
}


