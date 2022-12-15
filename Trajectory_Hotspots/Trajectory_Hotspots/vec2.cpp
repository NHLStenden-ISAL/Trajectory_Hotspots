#include "pch.h"
#include "Vec2.h"

Float Vec2::dot(const Vec2& other) const
{
    return x * other.x + y * other.y;
}

Float Vec2::squared_length() const
{
    return x * x + y * y;
}

Float Vec2::length() const
{
    return sqrtf((x * x + y * y).get_value());
}

void Vec2::normalize()
{
    //Multiply by reciprocal to prevent two divides
    Float rl = 1.f / length();
    x *= rl;
    y *= rl;
}

Vec2 Vec2::normalized() const
{
    //Multiply by reciprocal to prevent two divides
    Float rl = 1.f / length();
    return Vec2(x * rl, y * rl);
}

Vec2 Vec2::operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*(const Float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/(const Float& scalar) const { return Vec2(x / scalar, y / scalar); }

Vec2 Vec2::operator-() const { return Vec2(-x, -y); }

Vec2 Vec2::operator+=(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-=(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*=(const Float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/=(const Float& scalar) const { return Vec2(x / scalar, y / scalar); }

bool Vec2::operator==(const Vec2& operand) const { return nearly_equal(x.get_value(), operand.x.get_value()) && nearly_equal(y.get_value(), operand.y.get_value()); }
bool Vec2::operator!=(const Vec2& operand) const { return !(*this == operand); }

Vec2 operator*(const Float& scalar, const Vec2& vec) { return Vec2(vec.x * scalar, vec.y * scalar); }
Vec2 operator/(const Float& scalar, const Vec2& vec) { return Vec2(vec.x / scalar, vec.y / scalar); }