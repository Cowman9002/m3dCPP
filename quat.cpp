#include "m3d/quat.h"
#include "m3d/vec3.h"
#include "m3d/math1D.h"
#include <math.h>

namespace m3d
{
    quat::quat() : quat(0.0f, 0.0f, 0.0f, 1.0f) {};
    quat::quat(const float& i, const float& j, const float& k, const float& w) : i(i), j(j), k(k), w(w) {};
    quat::quat(const float& angle, const vec3& axis)
    {
        i = axis.x * sin(angle / 2.0f);
        j = axis.y * sin(angle / 2.0f);
        k = axis.z * sin(angle / 2.0f);
        w = cos(angle / 2.0f);
    }

    ///////////////////////////////////////
    //              STATIC               //
    ///////////////////////////////////////

    //https://www.mathworks.com/matlabcentral/answers/415936-angle-between-2-quaternions
    float quat::angle(const quat& a, const quat& b)
    {
        return 2.0f * acos((quat::conjugate(a) * b).w);
    }

    //https://stackoverflow.com/questions/12435671/quaternion-lookat-function
    //https://gamedev.stackexchange.com/questions/15070/orienting-a-model-to-face-a-target
    quat quat::angleVec3(const vec3& a, const vec3& b, const vec3& up)
    {
        float dot = vec3::dot(a, b);
        // test for dot -1
        if(fabsf(dot + 1.0f) < 0.000001f)
        {
            // vector a and b point exactly in the opposite direction,
            // so it is a 180 degrees turn around the up-axis
            return quat(180.0f * TO_RADS, up);
        }
        // test for dot 1
        else if(fabsf(dot - 1.0f) < 0.000001f)
        {
            // vector a and b point exactly in the same direction
            // so we return the identity quaternion
            return quat(0.0f, 0.0f, 0.0f, 1.0f);
        }

        float rotAngle = acos(dot);
        vec3 rotAxis = vec3::cross(a, b);
        rotAxis = vec3::normalized(rotAxis);
        return quat(rotAngle, rotAxis);
    }

    quat quat::conjugate(const quat& v)
    {
        quat res;
        res.i = -v.i;
        res.j = -v.j;
        res.k = -v.k;
        res.w = v.w;
        return res;
    }

    vec3 quat::euler(const quat& v)
    {
        float i2 = v.i * v.i;
        float j2 = v.j * v.j;
        float k2 = v.k * v.k;

        vec3 res;
        res.x = atan2(2 * (v.w * v.i + v.j * v.k), 1 - 2 * (i2 + j2));
        res.y = asin(2 * (v.w * v.j - v.k * v.i));
        res.z = atan2(2 * (v.w * v.k + v.i * v.j), 1 - 2 * (j2 + k2));

        return res;
    }

    quat quat::face(const vec3& dir, const vec3& up)
    {
        return quat::angleVec3(vec3(0.0f, 0.0f, 1.0f), dir, up);
    }

    float quat::length(const quat& v)
    {
        float res = sqrt(quat::lengthSqr(v));

        return res;
    }

    float quat::lengthSqr(const quat& v)
    {
        float res = v.i * v.i + v.j * v.j + v.k * v.k + v.w * v.w;

        return res;
    }

    vec3 quat::rotateVec3(const quat& a, const vec3& b)
    {
        quat P = quat(b.x, b.y, b.z, 0.0f);

        quat temp = a * P * quat::conjugate(a);

        return vec3(temp.i, temp.j, temp.k);
    }

    quat quat::normalized(const quat& v)
    {
        float length = quat::length(v);
        quat res;
        res.i = v.i / length;
        res.j = v.j / length;
        res.k = v.k / length;
        res.w = v.w / length;
        return res;
    }

    quat quat::slerp(const quat& a, const quat& b, const float& t)
    {
        quat res;

        float cosHalfTheta = a.w * b.w + a.i * b.i + a.j * b.j + a.k * b.k;
        // if qa=qb or qa=-qb then theta = 0 and we can return qa
        if (abs(cosHalfTheta) >= 1.0f)
        {
            return a;
        }
        // Calculate temporary values.
        float halfTheta = acos(cosHalfTheta);
        float sinHalfTheta = sqrt(1.0f - cosHalfTheta*cosHalfTheta);
        // if theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to qa or qb
        if (fabs(sinHalfTheta) < 0.001f)
        {
            return a * 0.5f + b * 0.5f;
        }

        //calculate quaternion.
        res = a * sin((1 - t) * halfTheta);
        res = res + b * sin(t * halfTheta);
        res = res / sin(halfTheta);

        return res;
    }

    quat quat::add(const quat& a, const quat& b)
    {
        quat res;
        res.i = a.i + b.i;
        res.j = a.j + b.j;
        res.k = a.k + b.k;
        res.w = a.w + b.w;

        return res;
    }

    quat quat::sub(const quat& a, const quat& b)
    {
        quat res;
        res.i = a.i - b.i;
        res.j = a.j - b.j;
        res.k = a.k - b.k;
        res.w = a.w - b.w;

        return res;
    }

    quat quat::mul(const quat& a, const quat& b)
    {
        quat res;

        res.i = a.w * b.i + a.i * b.w + a.j * b.k - a.k * b.j;
        res.j = a.w * b.j - a.i * b.k + a.j * b.w + a.k * b.i;
        res.k = a.w * b.k + a.i * b.j - a.j * b.i + a.k * b.w;
        res.w = a.w * b.w - a.i * b.i - a.j * b.j - a.k * b.k;

        //res.i = a.i * b.w + a.w * b.i + a.j * b.k - a.k * b.j;
        //res.j = a.j * b.w + a.w * b.j + a.k * b.i - a.i * b.k;
        //res.k = a.k * b.w + a.w * b.k + a.i * b.j - a.j * b.i;
        //res.w = a.w * b.w - a.i * b.i - a.j * b.j - a.k * b.k;

        return res;
    }

    quat quat::mul(const quat& a, const float& b)
    {
        quat res;
        res.i = a.i * b;
        res.j = a.j * b;
        res.k = a.k * b;
        res.w = a.w * b;

        return res;
    }

    quat quat::div(const quat& a, const float& b)
    {
        quat res;
        res.i = a.i / b;
        res.j = a.j / b;
        res.k = a.k / b;
        res.w = a.w / b;

        return res;
    }

    bool quat::equals(const quat& a, const quat& b)
    {
        return a.i == b.i && a.j == b.j && a.k == b.k && a.w == b.w;
    }

    ///////////////////////////////////////
    //              METHODS              //
    ///////////////////////////////////////

    float quat::length() const
    {
        return quat::length(*this);
    }

    float quat::lengthSqr() const
    {
        return quat::lengthSqr(*this);
    }

    quat quat::normalized() const
    {
        return quat::normalized(*this);
    }

    quat quat::conjugate() const
    {
        return quat::conjugate(*this);
    }
}

bool operator==(const m3d::quat& a, const m3d::quat&b)
{
    return m3d::quat::equals(a, b);
}

bool operator!=(const m3d::quat& a, const m3d::quat&b)
{
    return !m3d::quat::equals(a, b);
}

m3d::quat operator-(const m3d::quat& a)
{
    return a.conjugate();
}

m3d::quat operator+(const m3d::quat& a, const m3d::quat& b)
{
    return m3d::quat::add(a, b);
}

m3d::quat operator-(const m3d::quat& a, const m3d::quat& b)
{
    return m3d::quat::sub(a, b);
}

m3d::quat operator*(const m3d::quat& a, const m3d::quat& b)
{
    return m3d::quat::mul(a, b);
}

m3d::quat operator*(const m3d::quat& a, const float& b)
{
    return m3d::quat::mul(a, b);
}

m3d::quat operator/(const m3d::quat& a, const float& b)
{
    return m3d::quat::div(a, b);
}

m3d::vec3 operator*(const m3d::quat& a, const m3d::vec3& b)
{
    return m3d::quat::rotateVec3(a, b);
}

m3d::vec3 operator*(const m3d::vec3& a, const m3d::quat& b)
{
    return m3d::quat::rotateVec3(b, a);
}

m3d::vec3 operator*=(m3d::vec3& a, const m3d::quat& b)
{
    a = m3d::quat::rotateVec3(b, a);
    return a;
}

m3d::quat& operator+=(m3d::quat& a, const m3d::quat& b)
{
    a = m3d::quat::add(a, b);
    return a;
}

m3d::quat& operator-=(m3d::quat& a, const m3d::quat& b)
{
    a = m3d::quat::sub(a, b);
    return a;
}

m3d::quat& operator*=(m3d::quat& a, const m3d::quat& b)
{
    a = m3d::quat::mul(a, b);
    return a;
}

m3d::quat& operator*=(m3d::quat& a, const float& b)
{
    a = m3d::quat::mul(a, b);
    return a;
}

m3d::quat& operator/=(m3d::quat& a, const float& b)
{
    a = m3d::quat::div(a, b);
    return a;
}
