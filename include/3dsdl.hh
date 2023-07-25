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

///////////////////////////////////////
template<typename ...Args>
void 
LOG(Args && ...args)
{
    (std::cout << ... << args);
    std::cout<<std::endl;
}
///////////////////////////////////////
/// Transform base class
//////////////////////////////////////
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
        : obj(in_obj) {}

    const xt::xarray<double> local_vec3() const 
    {
        return xt::xarray<double> {{local(0,0), local(1,1), local(2,2)}};
    }
    const xt::xarray<double> world_vec3() const 
    {
        return xt::xarray<double> {{local(0,0), local(1,1), local(2,2)}};
    }

};
///////////////////////////////////////
/// Quaternion base class
//////////////////////////////////////
struct Quat {
    xt::xarray<double> vec3 = {{0,0,0}};
    double w = 1;
    Quat() {}
    Quat(double w, const xt::xarray<double>& invec3)
        : vec3(invec3), w(w) {}

///////////////////////
/// Operator overrides
    friend std::ostream &operator<<(std::ostream &os, const Quat& qin)
    {
        return os << qin.w << " " << qin.vec3;

    }
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
        double wtemp = this->w*rhs.w - xt::linalg::dot(this->vec3,xt::transpose(rhs.vec3))[0];
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
//////////////////////
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
        return Quat(this->w, this->vec3 * -1);
    }
    const Quat inverse() const 
    {
        // q_inv = q_conj / norm^2
        double tnorm = this->norm();
        tnorm *= tnorm;

        return this->conjugate() / tnorm;
    }

/////////////////////
/// Static functions
////////////////////


    // usage: Quat::rotate_vec(xt::xarray<double>{{0,1,0}}, xt::xarray<double>{{1,0,0}}, pi/2); => ~[0, 0, 1]
    static const xt::xarray<double> rotate_vec(const xt::xarray<double>& invec, const xt::xarray<double>& axis, double theta)
    {   
        // create pure quaternion from tform
        Quat tform_l = Quat(0, invec);
        // axis of rotation
        xt::xarray<double> uaxis = axis / xt::linalg::norm(axis, 2);
        Quat rot_q = Quat(theta, uaxis);
        rot_q.w = xt::cos(xt::xarray<double>({theta*.5}))[0];
        rot_q.vec3 *= xt::sin(xt::xarray<double>({theta*.5}))[0];
        Quat rot_q_inv = rot_q.inverse();
        // perform rotation
        tform_l=(rot_q*tform_l)*rot_q_inv;
        // return transformed vector
        return tform_l.vec3;
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

