#include "m3d/quat.h"
#include "m3d/vec3.h"
#include "m3d/math1D.h"
#include "m3d/mat4x4.h"
#include <cmath>
#include <algorithm>

#include <stdio.h>

namespace m3d
{
    quat::quat() : quat(0.0f, 0.0f, 0.0f, 1.0f) {};
    quat::quat(const float& i, const float& j, const float& k, const float& w) : i(i), j(j), k(k), w(w) {};
    quat::quat(const float& angle, const vec3& axis)
    {
        float s = std::sin(angle / 2.0f);
        i = axis.x * s;
        j = axis.y * s;
        k = axis.z * s;
        w = std::cos(angle / 2.0f);
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

    float quat::dot(const quat& a, const quat& b)
    {
        return a.i * b.i + a.j * b.j + a.k * b.k + a.w * b.w;
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

    quat quat::lookat(const vec3& source, const vec3& dest, const vec3& up)
    {
        return quat::fromMat4x4(mat4x4::lookat(source, dest, up));
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

    //https://en.wikipedia.org/wiki/Slerp#Source_code
    quat quat::slerp(const quat& a, const quat& b, const float& t)
    {
        // Only unit quaternions are valid rotations.
        // Normalize to avoid undefined behavior.
        quat v0 = a.normalized();
        quat v1 = b.normalized();

        // Compute the cosine of the angle between the two vectors.
        float dot = quat::dot(a, b);

        // If the dot product is negative, slerp won't take
        // the shorter path. Note that v1 and -v1 are equivalent when
        // the negation is applied to all four components. Fix by
        // reversing one quaternion.
        if (dot < 0.0f) {
            v1 = -v1;
            v1.w = -v1.w;
            dot = -dot;
        }

        const double DOT_THRESHOLD = 0.9995;
        if (dot > DOT_THRESHOLD) {
            // If the inputs are too close for comfort, linearly interpolate
            // and normalize the result.

            quat r = v1 - v0;
            quat result = v0 + r * t;
            return result.normalized();
        }

        // Since dot is in range [0, DOT_THRESHOLD], acos is safe
        float theta_0 = std::acos(dot);        // theta_0 = angle between input vectors
        float theta = theta_0 * t;          // theta = angle between v0 and result
        float sin_theta = std::sin(theta);     // compute this value only once
        float sin_theta_0 = std::sin(theta_0); // compute this value only once

        float s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
        float s1 = sin_theta / sin_theta_0;

        return ((v0 * s0) + (v1 * s1)).normalized();
    }

    //https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    quat quat::fromMat4x4(const mat4x4& mat)
    {
        quat res;

        float trace = mat.m[0][0] + mat.m[1][1] + mat.m[2][2];
        if( trace > 0 )
        {
            float s = 0.5f / sqrtf(trace+ 1.0f);
            res.w = 0.25f / s;
            res.i = ( mat.m[1][2] - mat.m[2][1] ) * s;
            res.j = ( mat.m[2][0] - mat.m[0][2] ) * s;
            res.k = ( mat.m[0][1] - mat.m[1][0] ) * s;
        }
        else
        {
            if( mat.m[0][0] > mat.m[1][1] && mat.m[0][0] > mat.m[2][2] )
            {
                float s = 2.0f * sqrtf( 1.0f + mat.m[0][0] - mat.m[1][1] - mat.m[2][2]);
                res.w = (mat.m[1][2] - mat.m[2][1] ) / s;
                res.i = 0.25f * s;
                res.j = (mat.m[1][0] + mat.m[0][1] ) / s;
                res.k = (mat.m[2][0] + mat.m[0][2] ) / s;
            }
            else if(mat.m[1][1] > mat.m[2][2])
            {
                float s = 2.0f * sqrtf( 1.0f + mat.m[1][1] - mat.m[0][0] - mat.m[2][2]);
                res.w = (mat.m[2][0] - mat.m[0][2] ) / s;
                res.i = (mat.m[1][0] + mat.m[0][1] ) / s;
                res.j = 0.25f * s;
                res.k = (mat.m[2][1] + mat.m[1][2] ) / s;
            }
            else
            {
                float s = 2.0f * sqrtf( 1.0f + mat.m[2][2] - mat.m[0][0] - mat.m[1][1] );
                res.w = (mat.m[0][1] - mat.m[1][0] ) / s;
                res.i = (mat.m[2][0] + mat.m[0][2] ) / s;
                res.j = (mat.m[2][1] + mat.m[1][2] ) / s;
                res.k = 0.25f * s;
            }
        }

        return res.normalized();
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

    vec3 quat::getRight()
    {
        return rotateVec3(*this, vec3(1.0f, 0.0f, 0.0f));
    }

    vec3 quat::getUp()
    {
        return rotateVec3(*this, vec3(0.0f, 1.0f, 0.0f));
    }

    vec3 quat::getForward()
    {
        return rotateVec3(*this, vec3(0.0f, 0.0f, -1.0f));
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
