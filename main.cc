
#include "xtensor/xarray.hpp"
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
    while(quit == false)
    { 
        SDL_WaitEvent(&event);
        switch (event.type)
        {
            case SDL_QUIT:
                quit = true;
                break;
            case SDL_MOUSEMOTION:
                // TODO: Setup render thread
                int mX = event.motion.x - S_WIDTH/2;
                int mY = -1*event.motion.y + S_HEIGHT/2;
                //std::cout<<mX<< " "<<mY<<std::endl;
                // scalars are 1d arrays in xt
                double xd = 6.28/(S_WIDTH); // each pixel in range [0, 2pi]
                double yd = 6.28/(S_HEIGHT); 
                xt::xarray<double> xdeg;
                xdeg = xd*mX;
                xt::xarray<double> ydeg;
                ydeg = yd*mY;
                // compute rotation matrix around x and y axis
                xt::xarray<double> xdir { //rotating in the x direction is around the y axis
                    {xt::cos(xdeg)[0], 0, -1*(xt::sin(xdeg)[0])},
                    {0               , 1,  0                   },
                    {xt::sin(xdeg)[0], 0,  xt::cos(xdeg)[0]    }
                };
                xt::xarray<double> ydir {
                    {1,   0,                   0               },
                    {0,   xt::cos(ydeg)[0],    xt::sin(ydeg)[0]},
                    {0,   -1*xt::sin(ydeg)[0], xt::cos(ydeg)[0]},
                };
                xt::xarray<double> tmat = xt::linalg::dot(xdir,ydir);
                xt::xarray<double> tri_1 = {
                    {-100, 0, 0},
                    {0, 100, 100},
                    {100, 0, 0}
                };
                xt::xarray<double> tri_2 = {
                    {-100, 0, 200},
                    {0, 100, 100},
                    {100, 0, 200}
                };
                xt::xarray<double> tri_3 = {
                    {100, 0, 200},
                    {0, 100, 100},
                    {100, 0, 0}
                };
                xt::xarray<double> tri_4 = {
                    {-100, 0, 200},
                    {0, 100, 100},
                    {-100, 0, 0}
                };
                std::vector<xt::xarray<double>*> tri_prism = {
                    &tri_1,
                    &tri_2,
                    &tri_3,
                    &tri_4
                };
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderClear(renderer);
                for (xt::xarray<double>* tri_ptr : tri_prism)
                {
                    xt::xarray<double> tri = *tri_ptr;
                    xt::xarray<double> res = xt::linalg::dot(tri,tmat);
                    // translate tri around screen center
                    xt::xarray<double> tri_sc = {
                        {S_WIDTH/2, S_HEIGHT/2, 0}
                    };
                    res = res + xt::view(tri_sc, 0);

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
    }
    SDL_DestroyWindow(w);
    SDL_Quit();
    return 0;
}

