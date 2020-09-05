#include "m3d/vec4.h"
#include "m3d/vec3.h"
#include "m3d/math1D.h"

namespace m3d
{
    vec4::vec4() : vec4(0.0f) {};
    vec4::vec4(const float& v) : vec4(v, v, v, v) {};
    vec4::vec4(const vec3& v) : vec4(v, 1.0f) {};
    vec4::vec4(const vec3& v, const float& w) : vec4(v.x, v.y, v.z, w) {};
    vec4::vec4(const float& x, const float& y, const float& z, const float& w) :
        x(x), y(y), z(z), w(w) {};

    vec3 vec4::xyz() const
    {
        return vec3(x, y, z);
    }

    vec4 vec4::add(const vec4& a, const vec4& b)
    {
        vec4 res;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        res.z = a.z + b.z;
        res.w = a.w + b.w;
        return res;
    }

    vec4 vec4::add(const vec4& a, const float& b)
    {
        vec4 res;
        res.x = a.x + b;
        res.y = a.y + b;
        res.z = a.z + b;
        res.w = a.w + b;
        return res;
    }

    vec4 vec4::sub(const vec4& a, const vec4& b)
    {
        vec4 res;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        res.w = a.w - b.w;
        return res;
    }

    vec4 vec4::sub(const vec4& a, const float& b)
    {
        vec4 res;
        res.x = a.x - b;
        res.y = a.y - b;
        res.z = a.z - b;
        res.w = a.w - b;
        return res;
    }

    vec4 vec4::mul(const vec4& a, const float& b)
    {
        vec4 res;
        res.x = a.x * b;
        res.y = a.y * b;
        res.z = a.z * b;
        res.w = a.w * b;
        return res;
    }

    vec4 vec4::div(const vec4& a, const float& b)
    {
        vec4 res;
        res.x = a.x / b;
        res.y = a.y / b;
        res.z = a.z / b;
        res.w = a.w / b;
        return res;
    }

    bool vec4::equals(const vec4& a, const vec4& b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
    }
}

bool operator==(const m3d::vec4& a, const m3d::vec4&b)
{
    return m3d::vec4::equals(a, b);
}

bool operator!=(const m3d::vec4& a, const m3d::vec4&b)
{
    return !m3d::vec4::equals(a, b);
}

m3d::vec4 operator-(const m3d::vec4& a)
{
    return m3d::vec4::mul(a, -1);
}

m3d::vec4 operator+(const m3d::vec4& a, const m3d::vec4& b)
{
    return m3d::vec4::add(a, b);
}

m3d::vec4 operator+(const m3d::vec4& a, const float& b)
{
    return m3d::vec4::add(a, b);
}

m3d::vec4 operator-(const m3d::vec4& a, const m3d::vec4& b)
{
    return m3d::vec4::sub(a, b);
}

m3d::vec4 operator-(const m3d::vec4& a, const float& b)
{
    return m3d::vec4::sub(a, b);
}

m3d::vec4 operator*(const m3d::vec4& a, const float& b)
{
    return m3d::vec4::mul(a, b);
}

m3d::vec4 operator/(const m3d::vec4& a, const float& b)
{
    return m3d::vec4::div(a, b);
}

m3d::vec4& operator+=(m3d::vec4& a, const m3d::vec4& b)
{
    a = m3d::vec4::add(a, b);
    return a;
}

m3d::vec4& operator+=(m3d::vec4& a, const float& b)
{
    a = m3d::vec4::add(a, b);
    return a;
}

m3d::vec4& operator-=(m3d::vec4& a, const m3d::vec4& b)
{
    a = m3d::vec4::sub(a, b);
    return a;
}

m3d::vec4& operator-=(m3d::vec4& a, const float& b)
{
    a = m3d::vec4::sub(a, b);
    return a;
}

m3d::vec4& operator*=(m3d::vec4& a, const float& b)
{
    a = m3d::vec4::mul(a, b);
    return a;
}

m3d::vec4& operator/=(m3d::vec4& a, const float& b)
{
    a = m3d::vec4::mul(a, b);
    return a;
}
