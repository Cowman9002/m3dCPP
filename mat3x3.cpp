#include "m3d/mat3x3.h"
#include "m3d/mat4x4.h"
#include "m3d/vec2.h"
#include "m3d/vec3.h"
#include "m3d/quat.h"

#include <math.h>

namespace m3d
{
    mat3x3::mat3x3() {};

    mat3x3::mat3x3(const float& diagonal)
    {
        m[0][0] = diagonal;
        m[1][1] = diagonal;
        m[2][2] = diagonal;
    }

    mat3x3 mat3x3::initOrtho(const float& r, const float& l, const float& t, const float& b)
    {
        mat3x3 res;

        float rml = 1.0f / (r - l);
        float tmb = 1.0f / (t - b);

        res.m[0][0] = 2.0f * rml;
        res.m[1][1] = 2.0f * tmb;
        res.m[2][2] = 1.0f;
        res.m[0][2] = -(r + l) * rml;
        res.m[1][2] = -(t + b) * tmb;

        return res;
    }

    mat3x3 mat3x3::initOrthoCentered(const float& w, const float& h)
    {
        mat3x3 res;

        res.m[0][0] = 2.0f / w;
        res.m[1][1] = 2.0f / h;
        res.m[2][2] = 1.0f;

        return res;
    }

    mat3x3 mat3x3::initRotationFromQuat(const quat& quat)
    {
        mat3x3 res;

        // precalc most parts
        float i2 = quat.i * quat.i * 2.0f;
        float j2 = quat.j * quat.j * 2.0f;
        float k2 = quat.k * quat.k * 2.0f;

        float ij = quat.i * quat.j * 2.0f;
        float jk = quat.j * quat.k * 2.0f;
        float ik = quat.i * quat.k * 2.0f;

        float iw = quat.i * quat.w * 2.0f;
        float jw = quat.j * quat.w * 2.0f;
        float kw = quat.k * quat.w * 2.0f;

        res.m[0][0] = 1.0f - j2 - k2;   res.m[0][1] = ij - kw;          res.m[0][2] = ik + jw;
        res.m[1][0] = ij + kw;          res.m[1][1] = 1.0f - i2 - k2;   res.m[1][2] = jk - iw;
        res.m[2][0] = ik - jw;          res.m[2][1] = jk + iw;          res.m[2][2] = 1.0f - i2 - j2;

        return res;
    }

    mat3x3 mat3x3::mul(const mat3x3& a, const mat3x3& b)
    {
        mat3x3 res;

        for (unsigned i = 0 ; i < 3 ; i++ )
        {
            for (unsigned j = 0 ; j < 3 ; j++ )
            {
                float sum = 0;
                for (unsigned k = 0 ; k < 3 ; k++ )
                {
                    sum += a.m[i][k] * b.m[k][j];
                }

                res.m[i][j] = sum;
            }
        }

        return res;
    }

    vec3 mat3x3::mul(const mat3x3& a, const vec3& b)
    {
        vec3 res;

        res.x = a.m[0][0] * b.x + a.m[0][1] * b.y + a.m[0][2] * b.z;
        res.y = a.m[1][0] * b.x + a.m[1][1] * b.y + a.m[1][2] * b.z;
        res.z = a.m[2][0] * b.x + a.m[2][1] * b.y + a.m[2][2] * b.z;

        return res;
    }

    mat3x3 mat3x3::fromMat4x4(const mat4x4& m)
    {
        mat3x3 res;

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                res.m[i][j] = m.m[i][j];
            }
        }

        return res;
    }

    mat3x3& mat3x3::rotate(const float& radians)
    {
        float cosTheta = cosf(radians);
        float sinTheta = sinf(radians);

        m[0][0] = cosTheta;
        m[0][1] = -sinTheta;
        m[1][0] = sinTheta;
        m[1][1] = cosTheta;

        return *this;
    }

    mat3x3& mat3x3::scale(const vec2& scale)
    {
        m[0][0] = scale.x;
        m[1][1] = scale.y;

        return *this;
    }

    mat3x3& mat3x3::translate(const vec2& translation)
    {
        m[0][2] = translation.x;
        m[1][2] = translation.y;

        return *this;
    }

    mat4x4 mat3x3::toMat4x4()
    {
        return mat4x4::fromMat3x3(*this);
    }
}

m3d::mat3x3 operator*(const m3d::mat3x3& a, const m3d::mat3x3& b)
{
    return m3d::mat3x3::mul(a, b);
}

m3d::vec3 operator*(const m3d::mat3x3& a, const m3d::vec3& b)
{
    return m3d::mat3x3::mul(a, b);
}

m3d::vec3 operator*(const m3d::vec3& a, const m3d::mat3x3& b)
{
    return m3d::mat3x3::mul(b, a);
}

m3d::mat3x3& operator*=(m3d::mat3x3& a, const m3d::mat3x3& b)
{
    a = m3d::mat3x3::mul(a, b);
    return a;
}

m3d::vec3& operator*=(m3d::vec3& a, const m3d::mat3x3& b)
{
    a = m3d::mat3x3::mul(b, a);
    return a;
}
