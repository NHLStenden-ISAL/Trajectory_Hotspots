#pragma once

class Float
{
public:
    Float() {};
    Float(const float value) : value(value) {};
    Float(const Float& other) : value(other.value) {};
    ~Float() {};

    static inline float fixed_epsilon = 0.0000001f;

    float get_value() const { return value; }
    void set_value(const float value) { this->value = value; }

    Float& operator=(const Float& other);
    Float& operator=(float value);

    bool operator==(const Float& other) const;
    bool operator==(float value) const;

    bool operator!=(const Float& other) const;
    bool operator!=(float value) const;

    bool operator<(const Float& other) const;
    bool operator<(float value) const;

    bool operator>(const Float& other) const;
    bool operator>(float value) const;

    bool operator<=(const Float& other) const;
    bool operator<=(float value) const;

    bool operator>=(const Float& other) const;
    bool operator>=(float value) const;

    Float operator+(const Float& other) const;
    Float operator+(float value) const;

    Float operator-(const Float& other) const;
    Float operator-(float value) const;

    Float operator*(const Float& other) const;
    Float operator*(float value) const;

    Float operator/(const Float& other) const;
    Float operator/(float value) const;

    Float& operator+=(const Float& other);
    Float& operator+=(float value);

    Float& operator-=(const Float& other);
    Float& operator-=(float value);

    Float& operator*=(const Float& other);
    Float& operator*=(float value);

    Float& operator/=(const Float& other);
    Float& operator/=(float value);

    Float operator-() const;

    bool is_inf() const;
    //Implicit conversion to float
    //operator float() const;

private:

    //From: https://bitbashing.io/comparing-floats.html
    //Returns the units of least precision between two floats
    int32_t ulps_distance(const float a, const float b) const;

    //Check for nearly equal float values
    bool nearly_equal(const float a, const float b, const int ulpsEpsilon = 3) const;

    bool nearly_less(const float a, const float b) const;

    bool nearly_greater(const float a, const float b) const;

    bool nearly_greater_or_equal(const float a, const float b) const;

    bool nearly_less_or_equal(const float a, const float b) const;

    float value = 0.0f;
};

Float operator/(const float& value, const Float& divider);
