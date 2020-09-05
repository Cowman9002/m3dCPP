#include "m3d/vec2.h"
#include "m3d/math1D.h"
#include <math.h>

namespace m3d
{
    vec2::vec2() : vec2(0) {}
    vec2::vec2(const float& v) : vec2(v, v) {}
    vec2::vec2(const float& x, const float& y) : x(x), y(y) {}

    ///////////////////////////////////////
    //              STATIC               //
    ///////////////////////////////////////

    float vec2::angle(const vec2& a, const vec2& b)
    {
        float numerator = vec2::dot(a, b);
        float denominator = vec2::length(a) * vec2::length(b);

        return acos(numerator / denominator);
    }

    float vec2::distance(const vec2& a, const vec2& b)
    {
        return vec2::length(b - a);
    }

    float vec2::distanceSqr(const vec2& a, const vec2& b)
    {
        return vec2::lengthSqr(b - a);
    }

    float vec2::dot(const vec2& a, const vec2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float vec2::length(const vec2& v)
    {
        return sqrt(vec2::lengthSqr(v));
    }

    float vec2::lengthSqr(const vec2& v)
    {
        return v.x * v.x + v.y * v.y;
    }

    vec2 vec2::lerp(const vec2& a, const vec2& b, const float& t)
    {
        vec2 res;
        res.x = m3d::lerp(a.x, b.x, t);
        res.y = m3d::lerp(a.y, b.y, t);
        return res;
    }

    vec2 vec2::max(const vec2& a, const vec2& b)
    {
        vec2 res;
        res.x = fmax(a.x, b.x);
        res.y = fmax(a.y, b.y);

        return res;
    }

    vec2 vec2::min(const vec2& a, const vec2& b)
    {
        vec2 res;
        res.x = fmin(a.x, b.x);
        res.y = fmin(a.y, b.y);

        return res;
    }

    vec2 vec2::normalized(const vec2& v)
    {
        float length = vec2::length(v);

        if(length != 0)
            return v / length;
        else return v;
    }

    vec2 vec2::reflect(const vec2& v, const vec2& normal)
    {
        float numerator = vec2::dot(v * 2, normal);
        vec2 a = normal * (numerator / vec2::lengthSqr(normal));

        return v - a;
    }

    vec2 vec2::slerp(const vec2& a, const vec2& b, const float& t)
    {
        float dot = a.dot(b);
        dot = m3d::clamp(dot, -1.0f, 1.0f);
        float theta = acos(dot) * t;
        vec2 offset = b - a * dot;
        offset = vec2::normalized(offset);
        return a * cos(theta) + offset * sin(theta);
    }

    vec2 vec2::add(const vec2& a, const vec2& b)
    {
        vec2 res;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        return res;
    }

    vec2 vec2::add(const vec2& a, const float& b)
    {
        vec2 res;
        res.x = a.x + b;
        res.y = a.y + b;
        return res;
    }

    vec2 vec2::sub(const vec2& a, const vec2& b)
    {
        vec2 res;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        return res;
    }

    vec2 vec2::sub(const vec2& a, const float& b)
    {
        vec2 res;
        res.x = a.x - b;
        res.y = a.y - b;
        return res;
    }

    vec2 vec2::mul(const vec2& a, const float& b)
    {
        vec2 res;
        res.x = a.x * b;
        res.y = a.y * b;
        return res;
    }

    vec2 vec2::div(const vec2& a, const float& b)
    {
        vec2 res;
        res.x = a.x / b;
        res.y = a.y / b;
        return res;
    }

    bool vec2::equals(const vec2& a, const vec2& b)
    {
        return a.x == b.x && a.y == b.y;
    }

    ///////////////////////////////////////
    //              METHODS              //
    ///////////////////////////////////////

    vec2 vec2::normalized() const
    {
        return vec2::normalized(*this);
    }

    float vec2::length() const
    {
        return vec2::length(*this);
    }

    float vec2::lengthSqr() const
    {
        return vec2::lengthSqr(*this);
    }

    float vec2::dot(const vec2& b) const
    {
        return vec2::dot(*this, b);
    }
}

bool operator==(const m3d::vec2& a, const m3d::vec2&b)
{
    return m3d::vec2::equals(a, b);
}

bool operator!=(const m3d::vec2& a, const m3d::vec2&b)
{
    return !m3d::vec2::equals(a, b);
}

m3d::vec2 operator-(const m3d::vec2& a)
{
    return m3d::vec2::mul(a, -1);
}

m3d::vec2 operator+(const m3d::vec2& a, const m3d::vec2& b)
{
    return m3d::vec2::add(a, b);
}

m3d::vec2 operator+(const m3d::vec2& a, const float& b)
{
    return m3d::vec2::add(a, b);
}

m3d::vec2 operator-(const m3d::vec2& a, const m3d::vec2& b)
{
    return m3d::vec2::sub(a, b);
}

m3d::vec2 operator-(const m3d::vec2& a, const float& b)
{
    return m3d::vec2::sub(a, b);
}

m3d::vec2 operator*(const m3d::vec2& a, const float& b)
{
    return m3d::vec2::mul(a, b);
}

m3d::vec2 operator/(const m3d::vec2& a, const float& b)
{
    return m3d::vec2::div(a, b);
}

m3d::vec2& operator+=(m3d::vec2& a, const m3d::vec2& b)
{
    a = m3d::vec2::add(a, b);
    return a;
}

m3d::vec2& operator+=(m3d::vec2& a, const float& b)
{
    a = m3d::vec2::add(a, b);
    return a;
}

m3d::vec2& operator-=(m3d::vec2& a, const m3d::vec2& b)
{
    a = m3d::vec2::sub(a, b);
    return a;
}

m3d::vec2& operator-=(m3d::vec2& a, const float& b)
{
    a = m3d::vec2::sub(a, b);
    return a;
}

m3d::vec2& operator*=(m3d::vec2& a, const float& b)
{
    a = m3d::vec2::mul(a, b);
    return a;
}

m3d::vec2& operator/=(m3d::vec2& a, const float& b)
{
    a = m3d::vec2::div(a, b);
    return a;
}
