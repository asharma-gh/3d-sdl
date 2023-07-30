#pragma once
#include "quat.hh"

#include "xtensor/xarray.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xaxis_slice_iterator.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "SDL2/SDL.h"

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
    Quat local_rot_q;
    xt::xarray<double> scale = {{1,1,1}};
    xt::xarray<double> local = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
    };
    xt::xarray<double> world = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
    };
    std::vector<xt::xarray<double>*> obj; // Object in model space 
    std::vector<xt::xarray<double>> obj_w; // Object in world space
    Tform() {}
    Tform(std::vector<xt::xarray<double>*> in_obj)
        : obj(in_obj) 
    {
        for (int ii = 0; ii < obj.size(); ii++)
            this->obj_w.push_back(*(this->obj[ii]));
    }
   
    void rotate_along_axis_q(const xt::xarray<double>& axis, double theta)
    {
        Quat rot_q = Quat::get_rotation_quat(axis, theta);
        xt::xarray<double> centroid = this->get_centroid();
        for (int ii = 0; ii < obj_w.size(); ii++)
        {
            // translate each point about the centroid of obj_i to origin and perform rotation
            for (int jj = 0; jj < 3; jj++)
            {
                xt::xarray<double> row = xt::view(obj_w[ii], jj, xt::all());
                row -= centroid;
                xt::xarray<double> rv = Quat::apply_rotation_quat(rot_q, {{row[0], row[1], row[2]}});
                row = xt::xarray<double>{rv[0], rv[1], rv[2], 0};
                row += centroid;
                xt::view(obj_w[ii], jj, xt::all()) = row;
            }
        }

    }
    const xt::xarray<double> get_centroid() const
    {
        // compute centroid
        double xu=0.0;
        double yu=0.0;
        double zu=0.0;
        int sz = obj_w.size();
        for (int ii=0;ii<sz;ii++)
        {
            xu += obj_w[ii][0];
            yu += obj_w[ii][1];
            zu += obj_w[ii][2];
        }
        return xt::xarray<double> {xu/sz, yu/sz, zu/sz, 1};
    }
    void reset_transforms()
    {
        //TODO: local -> world. Currently local==world
        this->obj_w.clear();
        for (int ii = 0; ii < obj.size(); ii++)
            this->obj_w.push_back(*(this->obj[ii]));
    }
    void update_local_from_vec3(const xt::xarray<double>& invec)
    {
        for (int ii = 0;ii<3;ii++)
            local(ii,ii)=invec(ii);
    }
    void update_world_from_vec3(const xt::xarray<double>& invec)
    {
        for (int ii = 0;ii<3;ii++)
            world(ii,ii)=invec(ii);
    }
    const xt::xarray<double> local_vec3() const 
    {
        return xt::xarray<double> {{local(0,0), local(1,1), local(2,2)}};
    }
    const xt::xarray<double> world_vec3() const 
    {
        return xt::xarray<double> {{local(0,0), local(1,1), local(2,2)}};
    }
};//end struct
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
    {-100, 0, 0, 1},
    {0, 100, 50, 1},
    {100, 0, 0, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_2 = {
    {-100, 0, 100, 1},
    {0, 100, 50, 1},
    {100, 0, 100, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_3 = {
    {100, 0, 100, 1},
    {0, 100, 50, 1},
    {100, 0, 0, 1},
    {0,   0, 0, 1}
};
xt::xarray<double> tri_4 = {
    {-100, 0, 100, 1},
    {0, 100, 50, 1},
    {-100, 0, 0, 1},
    {0,    0, 0, 1}
};
std::vector<xt::xarray<double>*> tobj = {
    &tri_1,
    &tri_2,
    &tri_3,
    &tri_4
};
Tform tri_prism_tf(tobj);

