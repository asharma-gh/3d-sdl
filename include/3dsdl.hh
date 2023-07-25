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
    Tform(std::vector<xt::xarray<double>*> in_obj)
        :obj(in_obj) {}
};
struct Quat {
    xt::xarray<double> vec3;
    double w = 1;
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

xt::xarray<double> cam_pos_w = {{0,0,0,1}};
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
std::vector<xt::xarray<double>*> tri_prism = {
    &tri_1,
   // &tri_2,
   // &tri_3,
  //  &tri_4
};
Tform tprism(tri_prism);
