// Copyright (c) <2012> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

/*
#ifndef MINI3D_MATH_TENSOR_H
#define MINI3D_MATH_TENSOR_H

#include "vec3.hpp"
#include "ternion.hpp"

#include <cmath>
#include <cfloat>

namespace mini3d {
namespace math {

#define MIN(a,b) (a) < (b) ? a : b
#define MAX(a,b) (a) > (b) ? a : b

////////// TENSOR ////////////////////////////////////////////////////////////

class Tensor {
public:
    float R[9];
    
    Tensor(const Ternion r) {
        float p = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        float b = (1.0f - p) * 0.5f;
        float c = 2.0f / (1.0f + p);

        R[0] = c * (r[0] * r[0] + b);
        R[1] = c * (r[0] * r[1] + r[2]);
        R[2] = c * (r[0] * r[2] - r[1]);
        R[3] = c * (r[1] * r[0] - r[2]);
        R[4] = c * (r[1] * r[1] + b);
        R[5] = c * (r[1] * r[2] + r[0]);
        R[6] = c * (r[2] * r[0] + r[1]);
        R[7] = c * (r[2] * r[1] - r[0]);
        R[8] = c * (r[2] * r[2] + b);
    }
    
    Vec3 rotate(const Vec3 vector) {
        
    }

};

}
}

#endif
*/
