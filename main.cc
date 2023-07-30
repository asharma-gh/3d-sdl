#include "3dsdl.hh"

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
    while (quit == false)
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
            cam_pos.world(2,2) += cam_vel; // z dir
            cam_pos.world(0,0) += cam_vel_x; // x dir
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
                // normalized screen coordinates such that Moving to the right increments X from the center
                // and moving the mouse up increments Y from the center
                double mX = double(event.motion.x - S_WIDTH/2)/S_WIDTH/2;
                double mY = double(-1*event.motion.y + S_HEIGHT/2)/S_HEIGHT/2;
                // scalars are 1d arrays in xt
                xt::xarray<double> xdeg;
                xdeg = 3.14*2*mY;
                xt::xarray<double> ydeg;
                ydeg = 3.14*2*mX;
                //LOG(mX, " ", mY);
                //LOG(xdeg, " ", ydeg);
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderClear(renderer);
                // Rotate the tri prism
                if (is_frame)
                {
                    tri_prism_tf.reset_transforms();
                    tri_prism_tf.rotate_along_axis_q(xt::xarray<double>{{1,0,0}}, xdeg[0]);
                    tri_prism_tf.rotate_along_axis_q(xt::xarray<double>{{0,1,0}}, ydeg[0]);
                }

                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                for (xt::xarray<double> tri : tri_prism_tf.obj_w)
                {
                    xt::xarray<double> res = tri;
                    xt::xarray<double> tri_sc = {
                        {S_WIDTH/2, S_HEIGHT/2, 0, 0}
                    };
                    res = res+xt::view(tri_sc, 0);
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

