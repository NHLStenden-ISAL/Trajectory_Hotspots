#include "pch.h"
#include "Vec2.h"

Float Vec2::dot(const Vec2& other) const
{
    return x * other.x + y * other.y;
}

Float Vec2::cross(const Vec2& other) const
{
    return (x * other.y) - (y * other.x);
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
    const Float rl = 1.f / length();
    return Vec2(x * rl, y * rl);
}

Vec2 Vec2::operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator+(const Float& n) const { return Vec2(x + n, y + n); }

Vec2 Vec2::operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator-(const Float& n) const { return Vec2(x - n, y - n); }

Vec2 Vec2::operator*(const Float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/(const Float& scalar) const { return Vec2(x / scalar, y / scalar); }

Vec2 Vec2::operator-() const { return Vec2(-x, -y); }

Vec2& Vec2::operator+=(const Vec2& other) 
{
    x += other.x;
    y += other.y;
    return *this;
}

Vec2& Vec2::operator-=(const Vec2& other) 
{
    x -= other.x;
    y -= other.y;
    return *this;
}

Vec2& Vec2::operator*=(const Float& scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vec2& Vec2::operator/=(const Float& scalar)
{
    if (scalar == 0.f)
    {
        throw std::runtime_error("Can't divide by zero");
    }
    x /= scalar;
    y /= scalar;
    return *this;
}
bool Vec2::operator==(const Vec2& operand) const { return x == operand.x && y == operand.y; }
bool Vec2::operator!=(const Vec2& operand) const { return !(*this == operand); }

bool Vec2::operator<(const Vec2& operand) const { return x < operand.x && y < operand.y; }
bool Vec2::operator<=(const Vec2& operand) const { return x <= operand.x && y <= operand.y; }
bool Vec2::operator>(const Vec2& operand) const { return x > operand.x && y > operand.y; }
bool Vec2::operator>=(const Vec2& operand) const { return x >= operand.x && y >= operand.y; }

Vec2 operator*(const Float& scalar, const Vec2& vec) { return Vec2(vec.x * scalar, vec.y * scalar); }
Vec2 operator/(const Float& scalar, const Vec2& vec) { return Vec2(vec.x / scalar, vec.y / scalar); }