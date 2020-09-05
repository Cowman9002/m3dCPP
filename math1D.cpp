#include "m3d/math1D.h"
#include <math.h>

namespace m3d
{
    float clamp(const float& v, const float& low, const float& high)
    {
        return fmin(fmax(low, v), high);
    }

    float lerp(const float& a, const float& b, const float& t)
    {
        return (1.0 - t) * a + b * t;
    }
}
