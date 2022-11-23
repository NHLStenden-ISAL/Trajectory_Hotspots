#pragma once

//Represents a 2d vector
class Vec2
{
public:

    Vec2() = default;

    constexpr Vec2(const float x, const float y) : x(x), y(y)
    {
    }

    float dot(const Vec2& other) const;

    float squared_length() const;

    float length() const;
    
    float cross(const Vec2& other) const;

    void normalize();

    Vec2 normalized() const;

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(const float& scalar) const;
    Vec2 operator/(const float& scalar) const;

    Vec2 operator-() const;

    Vec2 operator+=(const Vec2& other) const;
    Vec2 operator-=(const Vec2& other) const;
    Vec2 operator*=(const float& scalar) const;
    Vec2 operator/=(const float& scalar) const;

    bool operator==(const Vec2& operand) const;
    bool operator!=(const Vec2& operand) const;

    bool operator<(const Vec2& operand) const;
    bool operator>(const Vec2& operand) const;


    float x;
    float y;

private:

};


Vec2 operator*(const float& scalar, const Vec2& vec);
Vec2 operator/(const float& scalar, const Vec2& vec);