#pragma once

//Reprecents a 2d vector
class Vec2
{
public:

    Vec2() = default;

    Vec2(float x, float y) : x(x), y(y)
    {
    }

    float x;
    float y;

    float dot(const Vec2& other)
    {
        return x * other.x + y * other.y;
    }

    float squared_length()
    {
        return x * x + y * y;
    }

    float length()
    {
        return sqrt(x * x + y * y);
    }

    void normalize()
    {
        //Multiply by reciprical to prevent two divides
        float rl = 1.f / length();
        x *= rl;
        y *= rl;
    }

    Vec2 normalized()
    {
        //Multiply by reciprical to prevent two divides
        float rl = 1.f / length();
        return Vec2(x * rl, y * rl);
    }

    Vec2 operator+(const Vec2& other) { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) { return Vec2(x - other.x, y - other.y); }



    Vec2 operator-() const { return Vec2(-x, -y); }



private:

};