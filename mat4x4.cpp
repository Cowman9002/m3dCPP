#include "m3d/mat4x4.h"
#include "m3d/mat3x3.h"
#include "m3d/vec3.h"
#include "m3d/vec4.h"
#include "m3d/quat.h"

#include <math.h>

namespace m3d
{
    mat4x4::mat4x4() {};

    mat4x4::mat4x4(const float& diagonal)
    {
        m[0][0] = diagonal;
        m[1][1] = diagonal;
        m[2][2] = diagonal;
        m[3][3] = diagonal;
    }

    mat4x4 mat4x4::initOrtho(const float& r, const float& l, const float& t, const float& b, const float& n, const float& f)
    {
        mat4x4 res;

        float rml = 1.0f / (r - l);
        float tmb = 1.0f / (t - b);
        float fmn = 1.0f / (f - n);

        res.m[0][0] = 2.0f * rml;
        res.m[1][1] = 2.0f * tmb;
        res.m[2][2] = 2.0f * fmn;
        res.m[3][3] = 1.0f;
        res.m[0][3] = -(r + l) * rml;
        res.m[1][3] = -(t + b) * tmb;
        res.m[2][3] = -(f + n) * fmn;

        return res;
    }

    mat4x4 mat4x4::initOrthoCentered(const float& w, const float& h, const float& n, const float& f)
    {
        mat4x4 res;

        float fmn = 1.0f / (f - n);

        res.m[0][0] = 2.0f / w;
        res.m[1][1] = 2.0f / h;
        res.m[2][2] = 2.0f * fmn;
        res.m[3][3] = 1.0f;
        res.m[2][3] = -(f + n) * fmn;

        return res;
    }

    mat4x4 mat4x4::initPerspective(const float& w, const float& h, const float& fov, const float& n, const float& f)
    {
        mat4x4 res;

        float cotFov = 1.0f / tanf(fov / 2.0f);
        float fmn = 1.0f / (f - n);
        float aspect = w / h;

        res.m[0][0] = cotFov / aspect;
        res.m[1][1] = cotFov;
        res.m[2][2] = -(f + n) * fmn;
        res.m[2][3] = -2.0f * (f * n) * fmn;
        res.m[3][2] = -1.0f;

        return res;
    }

    mat4x4 mat4x4::mul(const mat4x4& a, const mat4x4& b)
    {
        mat4x4 res;

        for (unsigned i = 0 ; i < 4 ; i++ )
        {
            for (unsigned j = 0 ; j < 4 ; j++ )
            {
                float sum = 0.0f;
                for (unsigned k = 0 ; k < 4 ; k++ )
                {
                    sum += a.m[i][k] * b.m[k][j];
                }

                res.m[i][j] = sum;
            }
        }

        return res;
    }


    vec4 mat4x4::mul(const mat4x4& a, const vec4& b)
    {
        vec4 res;

        res.x = a.m[0][0] * b.x + a.m[0][1] * b.y + a.m[0][2] * b.z + a.m[0][3] * b.w;
        res.y = a.m[1][0] * b.x + a.m[1][1] * b.y + a.m[1][2] * b.z + a.m[1][3] * b.w;
        res.z = a.m[2][0] * b.x + a.m[2][1] * b.y + a.m[2][2] * b.z + a.m[2][3] * b.w;
        res.w = a.m[3][0] * b.x + a.m[3][1] * b.y + a.m[3][2] * b.z + a.m[3][3] * b.w;

        return res;
    }

    mat4x4 mat4x4::fromMat3x3(const mat3x3& m)
    {
        mat4x4 res;

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                res.m[i][j] = m.m[i][j];
            }

            res.m[3][i] = 0.0f;
            res.m[i][3] = 0.0f;
        }

        res.m[3][3] = 1.0f;

        return res;
    }


    mat4x4 mat4x4::inverseHomogeneous(const mat4x4& mat)
    {
        return mat4x4();
    }

    mat4x4& mat4x4::rotateX(const float& r)
    {
        float sinTheta = sinf(r);
        float cosTheta = cosf(r);

        m[1][1] = cosTheta;
        m[1][2] = -sinTheta;
        m[2][1] = sinTheta;
        m[2][2] = cosTheta;

        return *this;
    }

    mat4x4& mat4x4::rotateY(const float& r)
    {
        float sinTheta = sinf(r);
        float cosTheta = cosf(r);

        m[0][0] = cosTheta;
        m[0][2] = sinTheta;
        m[2][0] = -sinTheta;
        m[2][2] = cosTheta;

        return *this;
    }

    mat4x4& mat4x4::rotateZ(const float& r)
    {
        float cosTheta = cosf(r);
        float sinTheta = sinf(r);

        m[0][0] = cosTheta;
        m[0][1] = -sinTheta;
        m[1][0] = sinTheta;
        m[1][1] = cosTheta;

        return *this;
    }

    mat4x4& mat4x4::rotate(const quat& r)
    {
        // precalc most parts
        float i2 = r.i * r.i * 2.0f;
        float j2 = r.j * r.j * 2.0f;
        float k2 = r.k * r.k * 2.0f;

        float ij = r.i * r.j * 2.0f;
        float jk = r.j * r.k * 2.0f;
        float ik = r.i * r.k * 2.0f;

        float iw = r.i * r.w * 2.0f;
        float jw = r.j * r.w * 2.0f;
        float kw = r.k * r.w * 2.0f;

        m[0][0] = 1.0f - j2 - k2;    m[0][1] = ij - kw;           m[0][2] = ik + jw;
        m[1][0] = ij + kw;           m[1][1] = 1.0f - i2 - k2;    m[1][2] = jk - iw;
        m[2][0] = ik - jw;           m[2][1] = jk + iw;           m[2][2] = 1.0f - i2 - j2;

        return *this;
    }

    mat4x4& mat4x4::scale(const vec3& scale)
    {
        m[0][0] = scale.x;
        m[1][1] = scale.y;
        m[2][2] = scale.z;

        return *this;
    }

    mat4x4& mat4x4::translate(const vec3& translation)
    {
        m[0][3] = translation.x;
        m[1][3] = translation.y;
        m[2][3] = translation.z;

        return *this;
    }

    mat3x3 mat4x4::toMat3x3()
    {
        return mat3x3::fromMat4x4(*this);
    }
}

m3d::mat4x4 operator*(const m3d::mat4x4& a, const m3d::mat4x4& b)
{
    return m3d::mat4x4::mul(a, b);
}

m3d::vec4 operator*(const m3d::mat4x4& a, const m3d::vec4& b)
{
    return m3d::mat4x4::mul(a, b);
}

m3d::vec4 operator*(const m3d::vec4& a, const m3d::mat4x4& b)
{
    return m3d::mat4x4::mul(b, a);
}

m3d::mat4x4& operator*=(m3d::mat4x4& a, const m3d::mat4x4& b)
{
    a = m3d::mat4x4::mul(a, b);
    return a;
}

m3d::vec4& operator*=(m3d::vec4& a, const m3d::mat4x4& b)
{
    a = m3d::mat4x4::mul(b, a);
    return a;
}
