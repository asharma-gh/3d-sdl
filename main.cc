
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

const int S_WIDTH = 800;
const int S_HEIGHT = 600;
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

// Camera Parameters
xt::xarray<double> cam_pos_w = { // camera position in world coord
    {0, 0, 0, 0},
    {0, 50, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 1}
};
xt::xarray<double> cam_rot_w = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};
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
    &tri_2,
    &tri_3,
    &tri_4
};
////////////////////////////
template<typename ...Args>
void 
LOG(Args && ...args)
{
    (std::cout << ... << args);
    std::cout<<std::endl;
}
////////////////////////////
void quaternion_mat(const xt::xarray<double>& qa, const xt::xarray<double>& qb, xt::xarray<double>& out)
{
    double w,x,y,z;
    w = (qa(3,3)*qb(3,3)) - (qa(0,0)*qb(0,0)) - (qa(1,1)*qb(1,1)) - (qa(2,2)*qb(2,2));
    x = (qa(3,3)*qb(0,0)) + (qa(0,0)*qb(3,3)) + (qa(1,1)*qb(2,2)) - (qa(2,2)*qb(1,1));
    y = (qa(3,3)*qb(1,1)) - (qa(0,0)*qb(2,2)) + (qa(1,1)*qb(3,3)) + (qa(2,2)*qb(0,0));
    z = (qa(3,3)*qb(2,2)) + (qa(0,0)*qb(1,1)) - (qa(1,1)*qb(0,0)) + (qa(2,2)*qb(3,3));

    out(0,0) = x;
    out(1,1) = y;
    out(2,2) = z;
    out(3,3) = w;
}
void init_rot_quaternion(xt::xarray<double>& in_quat, xt::xarray<double> axis_cp, double angle)
{
    axis_cp = axis_cp / xt::linalg::norm(axis_cp, 2);
    xt::xarray<double> theta_2 = angle/2;
    for (int ii=0;ii<3;ii++)
    {
        in_quat(ii,ii) = axis_cp(ii,ii) * xt::sin(theta_2)[0];
    }
    in_quat(3,3) = xt::cos(theta_2)[0]; // w
}
void quaternion_rot(xt::xarray<double>& pts, const xt::xarray<double>& q_rot)
{
    double norm_2sq = xt::linalg::norm(q_rot, 2);
    norm_2sq *= norm_2sq;
    xt::xarray<double> conj_mat = {
        {-1, 0, 0, 0},
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0,  1}
    };
    xt::xarray<double> q_conj = xt::linalg::dot(q_rot, conj_mat);
    quaternion_mat(pts, q_rot, pts);
    quaternion_mat(pts, q_conj, pts);
    for(int ii=0;ii<3;ii++)
        pts(ii,ii) /= norm_2sq;
}
void world_to_cam(xt::xarray<double>& pts)
{
}
void 
persp_proj(xt::xarray<double>& pts)
{

    std::cout<<"BEFORE "<<pts<<std::endl;
    pts = xt::linalg::dot(pts, projMat);

    for (int ii=0;ii<3;ii++)
    {
        double s = pts(ii,3); // w
        if (s <= 0)
            continue;
        s=1/s;
        pts(ii,0) *= s*S_WIDTH/2;
        pts(ii,1) *= s*S_HEIGHT/2;
        std::cout<<"AFTER "<<pts<<std::endl;
        // r[1] = r[1]*s;
        // r[2] = r[2]*s;

    }
}

int 
main(int ac, char* av[])
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cout<<"SDL Init error! "<<SDL_GetError()<<std::endl;
        return -1;
    }
    SDL_Window* w = SDL_CreateWindow("3D Render",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,S_WIDTH,S_HEIGHT,SDL_WINDOW_SHOWN);
    // setup Renderer
    SDL_Renderer*  renderer = SDL_CreateRenderer(w,-1,SDL_RENDERER_ACCELERATED);
    // Event handling
    SDL_Event event; 
    bool quit = false;
    uint64_t cur_ticks = 0;
    bool is_frame = false;
    while(quit == false)
    { 
        uint64_t tick_start = SDL_GetTicks();
        if ((cur_ticks / 1000) >= 1/60)
        {
            // update physics once per frame
            is_frame = true; // update in case code below needs to update this frame
            if (cam_vel != 0)
            {
                cam_vel += (-1 * (cam_vel < 0) + (cam_vel > 0)) * cam_rev_accel;
            }
            if (cam_vel_x != 0)
            {
                cam_vel_x += (-1 * (cam_vel_x < 0) + (cam_vel_x > 0)) * cam_rev_accel;
            }
            cam_pos_w(2,2) += cam_vel; // z dir
            cam_pos_w(0,0) += cam_vel_x; // x dir
            // update camera physics
            cur_ticks = 0; // reset tick counter for next frame
        }
        // wait for event every 25 ms so frame time uniformly updates
        SDL_WaitEventTimeout(&event, 25);
        switch (event.type)
        {
            case SDL_QUIT:
                quit = true;
                break;
            case SDL_KEYDOWN:
                switch(event.key.keysym.sym)
                {
                    case SDLK_w:
                        LOG("W");
                        cam_vel += cam_accel;
                        break;
                    case SDLK_a:
                        cam_vel_x -= cam_accel;
                        break;
                    case SDLK_s:
                        cam_vel -= cam_accel;
                        break;
                    case SDLK_d:
                        cam_vel_x += cam_accel;
                        break;
                }
            case SDL_MOUSEMOTION:
                // TODO: Setup render thread
                int mX = event.motion.x - S_WIDTH/2;
                int mY = -1*event.motion.y + S_HEIGHT/2;
                // scalars are 1d arrays in xt
                double xd = 2*3.14/(S_WIDTH); // each pixel in range [0, pi]
                double yd = 2*3.14/(S_HEIGHT); 
                xt::xarray<double> xdeg;
                xdeg = xd*mY;
                xt::xarray<double> ydeg;
                ydeg = yd*mX;
                // compute rotation matrix around x and y axis
                xt::xarray<double> zdir { //rotating in the x direction is around the z axis
                    {xt::cos(xdeg)[0], -1*xt::sin(xdeg)[0], 0, 0},
                        {xt::sin(xdeg)[0], xt::cos(xdeg)[0], 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1}
                };

                xt::xarray<double> ydir {
                    {xt::cos(ydeg)[0], -1*xt::sin(ydeg)[0], 0, 0},
                        {0,                  -1, 0,                0},
                        {xt::sin(ydeg)[0], 0, xt::cos(ydeg)[0], 0},
                        {0,                   0, 0,                1}
                };
                /*
                   xt::xarray<double> xdir {
                   {1, 0, 0, 0},
                   {0, xt::cos(ydeg)[0], -1*xt::sin(ydeg)[0], 0},
                   {0, xt::sin(ydeg)[0], xt::cos(ydeg)[0], 0},
                   {0, 0, 0, 1}
                   };*/

                xt::xarray<double> tmat = xt::linalg::dot(zdir,ydir);

                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderClear(renderer);
                cam_pos_w = xt::linalg::dot(cam_pos_w, tmat);
                LOG(cam_pos_w);
                for (xt::xarray<double>* tri_ptr : tri_prism)
                {
                    xt::xarray<double> tri = *tri_ptr;
                    xt::xarray<double> res;
                    res = xt::linalg::dot(tri,tmat);
                    //persp_proj(res);
                    //std::cout<<xt::adapt(tri.shape())<<std::endl;
                    // translate tri around screen center
                    xt::xarray<double> tri_sc = {
                        {S_WIDTH/2, S_HEIGHT/2, 0, 0}
                    };
                    res = res+xt::view(tri_sc, 0);
                    //persp proj
                    // Calculate Camera Transform
                    //res = res - cam_pos;
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                    // v1 -> v2
                    SDL_RenderDrawLine(renderer, xt::view(res,0)[0], xt::view(res,0)[1], 
                            xt::view(res,1)[0], xt::view(res,1)[1]);
                    // v1 -> v3
                    SDL_RenderDrawLine(renderer, xt::view(res,0)[0], xt::view(res,0)[1], 
                            xt::view(res,2)[0], xt::view(res,2)[1]);
                    // v2 -> v3
                    SDL_RenderDrawLine(renderer, xt::view(res,1)[0], xt::view(res,1)[1], 
                            xt::view(res,2)[0], xt::view(res,2)[1]);

                }
                SDL_RenderPresent(renderer);
        }
        cur_ticks += (SDL_GetTicks() - tick_start);
    }
    SDL_DestroyWindow(w);
    SDL_Quit();
    return 0;
}

