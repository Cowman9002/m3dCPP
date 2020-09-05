#include "m3d/vec3.h"
#include "m3d/vec2.h"
#include "m3d/math1D.h"
#include <math.h>

namespace m3d
{
    vec3::vec3() : vec3(0) {};
    vec3::vec3(const float& v) : vec3(v, v, v) {};
    vec3::vec3(const vec2& v, const float& z) : vec3(v.x, v.y, z) {};
    vec3::vec3(const float& x, const vec2& v) : vec3(x, v.x, v.y) {};
    vec3::vec3(const float& x, const float& y, const float& z) : x(x), y(y), z(z) {};

    vec2 vec3::xy() const
    {
        return vec2(x, y);
    }

    vec2 vec3::yz() const
    {
        return vec2(y, z);
    }

    vec2 vec3::xz() const
    {
        return vec2(x, z);
    }

    vec2 vec3::yx() const
    {
        return vec2(y, x);
    }

    vec2 vec3::zy() const
    {
        return vec2(z, y);
    }

    vec2 vec3::zx() const
    {
        return vec2(z, x);
    }

    ///////////////////////////////////////
    //              STATIC               //
    ///////////////////////////////////////

    float vec3::angle(const vec3& a, const vec3& b)
    {
        float numerator = vec3::dot(a, b);
        float denominator = vec3::length(a) * length(b);

        return acos(numerator / denominator);
    }

    vec3 vec3::cross(const vec3& a, const vec3& b)
    {
        vec3 res;
        res.x = a.y * b.z - a.z * b.y;
        res.y = a.z * b.x - a.x * b.z;
        res.z = a.x * b.y - a.y * b.x;
        return res;
    }

    float vec3::distance(const vec3& a, const vec3& b)
    {
        return vec3::length(b - a);
    }

    float distanceSqr(const vec3& a, const vec3& b)
    {
        return vec3::lengthSqr(b - a);
    }

    float vec3::dot(const vec3& a, const vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    float vec3::length(const vec3& v)
    {
        return sqrt(vec3::lengthSqr(v));
    }

    float vec3::lengthSqr(const vec3& v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    vec3 vec3::lerp(const vec3& a, const vec3& b, const float& t)
    {
        vec3 res;
        res.x = m3d::lerp(a.x, b.x, t);
        res.y = m3d::lerp(a.y, b.y, t);
        res.z = m3d::lerp(a.z, b.z, t);
        return res;
    }

    vec3 vec3::max(const vec3& a, const vec3& b)
    {
        vec3 res;
        res.x = fmaxf(a.x, b.x);
        res.y = fmaxf(a.y, b.y);
        res.z = fmaxf(a.z, b.z);

        return res;
    }

    vec3 vec3::min(const vec3& a, const vec3& b)
    {
        vec3 res;
        res.x = fminf(a.x, b.x);
        res.y = fminf(a.y, b.y);
        res.z = fminf(a.z, b.z);

        return res;
    }

    vec3 vec3::normalized(const vec3& v)
    {
        float length = vec3::length(v);

        if(length != 0)
            return v / length;
        else return v;
    }

    vec3 vec3::reflect(const vec3& v, const vec3& normal)
    {
        float numerator = vec3::dot(v * 2, normal);
        vec3 a = normal * (numerator / vec3::lengthSqr(normal));

        return v - a;
    }

    vec3 vec3::slerp(const vec3& a, const vec3& b, const float& t)
    {
        float dot = vec3::dot(a, b);
        dot = m3d::clamp(dot, -1.0f, 1.0f);
        float theta = acos(dot)* t;
        vec3 offset = b - a * dot;
        offset = vec3::normalized(offset);
        return a * cos(theta) + offset * sin(theta);
    }

    vec3 vec3::add(const vec3& a, const vec3& b)
    {
        vec3 res;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        res.z = a.z + b.z;
        return res;
    }

    vec3 vec3::add(const vec3& a, const float& b)
    {
        vec3 res;
        res.x = a.x + b;
        res.y = a.y + b;
        res.z = a.z + b;
        return res;
    }

    vec3 vec3::sub(const vec3& a, const vec3& b)
    {
        vec3 res;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        return res;
    }

    vec3 vec3::sub(const vec3& a, const float& b)
    {
        vec3 res;
        res.x = a.x - b;
        res.y = a.y - b;
        res.z = a.z - b;
        return res;
    }

    vec3 vec3::mul(const vec3& a, const float& b)
    {
        vec3 res;
        res.x = a.x * b;
        res.y = a.y * b;
        res.z = a.z * b;
        return res;
    }

    vec3 vec3::div(const vec3& a, const float& b)
    {
        vec3 res;
        res.x = a.x / b;
        res.y = a.y / b;
        res.z = a.z / b;
        return res;
    }

    bool vec3::equals(const vec3& a, const vec3& b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    ///////////////////////////////////////
    //              METHODS              //
    ///////////////////////////////////////
    vec3 vec3::normalized() const
    {
        return vec3::normalized(*this);
    }

    float vec3::length() const
    {
        return vec3::length(*this);
    }

    float vec3::lengthSqr() const
    {
        return vec3::lengthSqr(*this);
    }

    float vec3::dot(const vec3& b) const
    {
        return vec3::dot(*this, b);
    }
}

bool operator==(const m3d::vec3& a, const m3d::vec3&b)
{
    return m3d::vec3::equals(a, b);
}

bool operator!=(const m3d::vec3& a, const m3d::vec3&b)
{
    return !m3d::vec3::equals(a, b);
}

m3d::vec3 operator-(const m3d::vec3& a)
{
    return m3d::vec3::mul(a, -1);
}

m3d::vec3 operator+(const m3d::vec3& a, const m3d::vec3& b)
{
    return m3d::vec3::add(a, b);
}

m3d::vec3 operator+(const m3d::vec3& a, const float& b)
{
    return m3d::vec3::add(a, b);
}

m3d::vec3 operator-(const m3d::vec3& a, const m3d::vec3& b)
{
    return m3d::vec3::sub(a, b);
}

m3d::vec3 operator-(const m3d::vec3& a, const float& b)
{
    return m3d::vec3::sub(a, b);
}

m3d::vec3 operator*(const m3d::vec3& a, const float& b)
{
    return m3d::vec3::mul(a, b);
}

m3d::vec3 operator/(const m3d::vec3& a, const float& b)
{
    return m3d::vec3::div(a, b);
}

m3d::vec3& operator+=(m3d::vec3& a, const m3d::vec3& b)
{
    a = m3d::vec3::add(a, b);
    return a;
}

m3d::vec3& operator+=(m3d::vec3& a, const float& b)
{
    a = m3d::vec3::add(a, b);
    return a;
}

m3d::vec3& operator-=(m3d::vec3& a, const m3d::vec3& b)
{
    a = m3d::vec3::sub(a, b);
    return a;
}

m3d::vec3& operator-=(m3d::vec3& a, const float& b)
{
    a = m3d::vec3::sub(a, b);
    return a;
}

m3d::vec3& operator*=(m3d::vec3& a, const float& b)
{
    a = m3d::vec3::mul(a, b);
    return a;
}

m3d::vec3& operator/=(m3d::vec3& a, const float& b)
{
    a = m3d::vec3::div(a, b);
    return a;
}
