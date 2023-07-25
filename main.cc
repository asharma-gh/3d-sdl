#include "3dsdl.hh"

void 
quaternion_mat(const xt::xarray<double>& qa, const xt::xarray<double>& qb, xt::xarray<double>& out)
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

void
init_rot_quaternion(xt::xarray<double>& in_quat, xt::xarray<double> axis_cp, double angle)
{
    axis_cp = axis_cp / xt::linalg::norm(axis_cp, 2);
    xt::xarray<double> theta_2 = angle/2;
    for (int ii=0;ii<3;ii++)
    {
        in_quat(ii,ii) = axis_cp(ii) * xt::sin(theta_2)[0];
    }
    in_quat(3,3) = xt::cos(theta_2)[0]; // w
}

void
quaternion_rot(xt::xarray<double>& pts, const xt::xarray<double>& q_rot)
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
void 
world_to_cam(xt::xarray<double>& pts)
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
                // normalized screen coordinates such that Moving to the right increments X from the center
                // and moving the mouse up increments Y from the center
                double mX = double(event.motion.x - S_WIDTH/2)/S_WIDTH/2;
                double mY = double(-1*event.motion.y + S_HEIGHT/2)/S_HEIGHT/2;
                // scalars are 1d arrays in xt
                xt::xarray<double> xdeg;
                xdeg = 3.14*mY;
                xt::xarray<double> ydeg;
                ydeg = 3.14*mX;
                LOG(mX, " ", mY);
                LOG(xdeg, " ", ydeg);
                xt::xarray<double>::shape_type shape = {4, 4};
                xt::xarray<double> q_rot = xt::xarray<double>::from_shape(shape);
                xt::xarray<double> axis = {
                    {0,1,0,0}
                };
                init_rot_quaternion(q_rot, axis, xdeg[0]); 
                //LOG(q_rot);
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderClear(renderer);
                for (xt::xarray<double>* tri_ptr : tri_prism)
                {
                    xt::xarray<double> tri = *tri_ptr;
                    xt::xarray<double> res = tri;
                    double norm = xt::linalg::norm(res, 2);
                    //res /= norm;
                    //quaternion_rot(res, q_rot);
                    //res *= 200;
                    LOG(res);
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

