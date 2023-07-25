#pragma once
#include "xtensor/xarray.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xio.hpp"
#include <xtensor/xaxis_slice_iterator.hpp>
#include "xtensor-blas/xlinalg.hpp"
#include <SDL2/SDL.h>
#include <iostream>
#include <string>
#include <vector>
////////////////////////////
template<typename ...Args>
void 
LOG(Args && ...args)
{
    (std::cout << ... << args);
    std::cout<<std::endl;
}
////////////////////////////
struct Tform {
    xt::xarray<double> local = {
        {0,0,0,0},
        {0,0,0,0},
        {0,0,0,0},
        {0,0,0,1}
    };
    xt::xarray<double> world = {
        {0,0,0,0},
        {0,0,0,0},
        {0,0,0,0},
        {0,0,0,1}
    };
    std::vector<xt::xarray<double>*> obj; // Object in model space
    Tform() {}
    Tform(std::vector<xt::xarray<double>*> in_obj)
        :obj(in_obj) {}
};
///////////////////////////////////////
/// Quaternion base class
//////////////////////////////////////
struct Quat {
    xt::xarray<double> vec3 = {{0,0,0}};
    double w = 1;
    Quat() {}
    Quat(xt::xarray<double> invec, double w)
        : vec3(invec), w(w) {}

////////////////////////
/// Operator overrides
    Quat& operator=(const Quat& rhs)
    {
        this->vec3 = rhs.vec3;
        this->w = rhs.w;
        return *this;
    }
    Quat& operator+=(const Quat& rhs)
    {
        this->vec3 += rhs.vec3;
        this->w += rhs.w;
        return *this;
    }
    const Quat operator+(const Quat &rhs) const
    {
        Quat res = *this; // copy
        res += rhs;
        return res;
    }
    Quat& operator*=(const Quat& rhs)
    {

        // Given a[sa, av] b[sb, bv]
        // a*b=[sa*sb - a dot b, sa*bv + sb*av + a cross b]
        double wtemp = this->w*rhs.w - xt::linalg::dot(this->vec3, rhs.vec3)[0];
        this->vec3 = (this->w*rhs.vec3) + (rhs.w*this->vec3) + xt::linalg::cross(this->vec3, rhs.vec3);
        this->w = wtemp;

        return *this;
    }
    const Quat operator*(const Quat& rhs) const
    {
        Quat res = *this; // copy
        res *= rhs;
        return res;
    }
    const Quat operator/(double rhs) const
    {
        Quat res = *this; // copy
        res.vec3 /= rhs;
        res.w /= rhs;
        return res;
    }
////////////////////
/// Library functions
    double norm() const
    {
        return sqrt(w*w + xt::linalg::norm(this->vec3, 2));
    }

    void normalize()
    {
        double tnorm = this->norm();
        if (tnorm == 0)
            return;
        tnorm = 1/tnorm;
        this->w *= tnorm;
        this->vec3 *= tnorm;
    }
    const Quat conjugate() const 
    {
        return Quat(this->vec3 * -1, this-> w);
    }
    const Quat inverse() const 
    {
        // q_inv = q_conj / norm^2
        double tnorm = this->norm();
        tnorm *= tnorm;

        return this->conjugate() / tnorm;
    }
};

///////////////////////////
const int S_WIDTH = 800;
const int S_HEIGHT = 600;
// TODO: cam.h
/// Camera Parameters
/// Non-const so they can be changed at runtime in the future!
xt::xarray<double> fovDeg = 3.14/2;
float aspectRatio = S_WIDTH / S_HEIGHT;
float znear = -.01; // Lower bound for object zpos from camera
float zfar = 500; // Upper bound for object zpos
float fovx = (aspectRatio*xt::tan(fovDeg/2)[0]);
xt::xarray<double> projMat = {
    {1/fovx,0,0,0},
    {0,1/fovx,0,0},
    {0,0,1, -1},
    {0,0,1, 0}
};

Tform cam_pos;
const int cam_accel = 5;
const int cam_rev_accel = -1; // friction
int cam_vel = 1; // z velocity
int cam_vel_x = 1; // x velociy
/////////////////////////////
//Prism Object parameters
xt::xarray<double> tri_1 = {
    {-100, 0, 100, 1},
    {0, 100, 200, 1},
    {100, 0, 100, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_2 = {
    {-100, 0, 300, 1},
    {0, 100, 200, 1},
    {100, 0, 300, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_3 = {
    {100, 0, 300, 1},
    {0, 100, 200, 1},
    {100, 0, 100, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_4 = {
    {-100, 0, 300, 1},
    {0, 100, 200, 1},
    {-100, 0, 100, 1},
    {0,    0, 0, 1}
};
std::vector<xt::xarray<double>*> tobj = {
    &tri_1,
   // &tri_2,
   // &tri_3,
  //  &tri_4
};
Tform tri_prism_tf(tobj);

