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

    Vec2 operator+=(const Vec2& other) const;
    Vec2 operator-=(const Vec2& other) const;
    Vec2 operator*=(const Float& scalar) const;
    Vec2 operator/=(const Float& scalar) const;

    bool operator==(const Vec2& operand) const;
    bool operator!=(const Vec2& operand) const;

    bool operator<(const Vec2& operand) const;
    bool operator>(const Vec2& operand) const;

    Float x;
    Float y;

    //Returns if a given vector a lies to the right (positive) or left (negative) of given vector b around a center vertex
    [[nodiscard]]
    static Float order_around_center(const Vec2& center, const Vec2& a, const Vec2& b)
    {
        return (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    };

private:

};


Vec2 operator*(const Float& scalar, const Vec2& vec);
Vec2 operator/(const Float& scalar, const Vec2& vec);