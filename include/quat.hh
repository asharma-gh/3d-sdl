#pragma once
#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xio.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include <iostream>

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
    static const Quat get_rotation_quat(const xt::xarray<double>& axis, double theta)
    {   
        // axis of rotation
        xt::xarray<double> uaxis = axis / xt::linalg::norm(axis, 2);
        Quat rot_q = Quat(theta, uaxis);
        rot_q.w = xt::cos(xt::xarray<double>({theta*.5}))[0];
        rot_q.vec3 *= xt::sin(xt::xarray<double>({theta*.5}))[0];
        // return quaternion rotation
        return rot_q;
    }
    static const xt::xarray<double> apply_rotation_quat(const Quat& rot_q,
                                                        const xt::xarray<double>& vec)
    {
        Quat quat = Quat(0, vec);
        return (rot_q*quat*rot_q.inverse()).vec3;
    }

};//end struct
