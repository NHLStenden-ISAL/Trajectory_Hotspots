#pragma once

//Represents a 2d vector
class Vec2
{
public:

    Vec2() = default;

    Vec2(const Float x, const Float y) : x(x), y(y)
    {
    }

    Float dot(const Vec2& other) const;

    Float cross(const Vec2& other) const;

    Float squared_length() const;

    Float length() const;

    void normalize();

    Vec2 normalized() const;

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(const Float& scalar) const;
    Vec2 operator/(const Float& scalar) const;

    Vec2 operator-() const;

    Vec2& operator+=(const Vec2& other);
    Vec2& operator-=(const Vec2& other);
    Vec2& operator*=(const Float& scalar);
    Vec2& operator/=(const Float& scalar);
    
    bool operator==(const Vec2& operand) const;
    bool operator!=(const Vec2& operand) const;
    
    bool operator<(const Vec2& operand) const;
    bool operator<=(const Vec2& operand) const;
    bool operator>(const Vec2& operand) const;
    bool operator>=(const Vec2& operand) const;

    Float x;
    Float y;

private:

};


Vec2 operator*(const Float& scalar, const Vec2& vec);
Vec2 operator/(const Float& scalar, const Vec2& vec);