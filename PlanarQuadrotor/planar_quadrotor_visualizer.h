#pragma once

#include <memory>

#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor *quadrotor_ptr;
public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr);
    void render(std::shared_ptr<SDL_Renderer> &gRenderer);
};