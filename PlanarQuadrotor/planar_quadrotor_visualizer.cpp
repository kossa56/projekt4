#include "planar_quadrotor_visualizer.h"
#include <algorithm>
#include <cmath>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0];
    float q_y = state[1];
    float q_theta = state[2];
    int szerokosc = 150;
    int wysokosc = 15;
    int smiglo_width = 25;
    int smiglo_height = 5;
    int przesuniecie_smigla = -20;


    float cos_theta = cos(q_theta);
    float sin_theta = sin(q_theta);

    // Obliczanie współrzędnych rogów korpusu
    SDL_Point body[4] = {
        {(int)(q_x + (-szerokosc / 2 * cos_theta - -wysokosc / 2 * sin_theta)), (int)(q_y + (-szerokosc / 2 * sin_theta + -wysokosc / 2 * cos_theta))},
        {(int)(q_x + (szerokosc / 2 * cos_theta - -wysokosc / 2 * sin_theta)), (int)(q_y + (szerokosc / 2 * sin_theta + -wysokosc / 2 * cos_theta))},
        {(int)(q_x + (szerokosc / 2 * cos_theta - wysokosc / 2 * sin_theta)), (int)(q_y + (szerokosc / 2 * sin_theta + wysokosc / 2 * cos_theta))},
        {(int)(q_x + (-szerokosc / 2 * cos_theta - wysokosc / 2 * sin_theta)), (int)(q_y + (-szerokosc / 2 * sin_theta + wysokosc / 2 * cos_theta))}
    };

    // Wypełnianie korpusu szarym kolorem
    Sint16 vx[4] = {body[0].x, body[1].x, body[2].x, body[3].x};
    Sint16 vy[4] = {body[0].y, body[1].y, body[2].y, body[3].y};
    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0x508080FF);

    // Rysowanie obrysu korpusu
    SDL_SetRenderDrawColor(gRenderer.get(), 0x80, 0x80, 0x80, 0xFF); // Szary kolor dla linii
    SDL_RenderDrawLines(gRenderer.get(), body, 4);
    SDL_RenderDrawLine(gRenderer.get(), body[3].x, body[3].y, body[0].x, body[0].y);

    // Wypełnione śmigła jako elipsy
    int lewo_smiglo_x = (int)(q_x + (-szerokosc / 2 * cos_theta - przesuniecie_smigla * sin_theta));
    int lewo_smiglo_y = (int)(q_y + (-szerokosc / 2 * sin_theta + przesuniecie_smigla * cos_theta));
    int prawo_smiglo_x = (int)(q_x + (szerokosc / 2 * cos_theta - przesuniecie_smigla * sin_theta));
    int prawo_smiglo_y = (int)(q_y + (szerokosc / 2 * sin_theta + przesuniecie_smigla * cos_theta));

    static float kat = 0.0f;
    kat += 0.05f;
    if (kat > 2 * M_PI) {
        kat = 0.0f;
    }

    // Rysowanie lewego śmigła w czerwonym kolorze
    filledEllipseColor(gRenderer.get(), lewo_smiglo_x - sin(kat) * 15, lewo_smiglo_y, smiglo_width, smiglo_height, 0xFFFF0000);
    filledEllipseColor(gRenderer.get(), lewo_smiglo_x + sin(kat) * 15, lewo_smiglo_y, smiglo_width, smiglo_height, 0xFF0000FF);

    // Rysowanie prawego śmigła w niebieskim kolorze
    filledEllipseColor(gRenderer.get(), prawo_smiglo_x - cos(kat) * 15, prawo_smiglo_y, smiglo_width, smiglo_height, 0xFFFF0000);
    filledEllipseColor(gRenderer.get(), prawo_smiglo_x + cos(kat) * 15, prawo_smiglo_y, smiglo_width, smiglo_height, 0xFF0000FF);

    // Rysowanie centralnych punktów śmigieł jako dłuższe brązowe prostokąty
    int smiglo_szerokosc = 3; // Zwiększono szerokość
    int smiglo_wysokosc = 50; // Zwiększono wysokość

    SDL_SetRenderDrawColor(gRenderer.get(), 0x8B, 0x45, 0x13, 0xFF); // Brązowy kolor

    // Lewy prostokąt
    SDL_Rect lewy_prostokat = {
        lewo_smiglo_x - smiglo_szerokosc / 2,
        lewo_smiglo_y - smiglo_wysokosc / 2,
        smiglo_szerokosc,
        smiglo_wysokosc
    };
    SDL_RenderFillRect(gRenderer.get(), &lewy_prostokat);

    // Prawy prostokąt
    SDL_Rect prawy_prostokat = {
        prawo_smiglo_x - smiglo_szerokosc / 2,
        prawo_smiglo_y - smiglo_wysokosc / 2,
        smiglo_szerokosc,
        smiglo_wysokosc
    };
    SDL_RenderFillRect(gRenderer.get(), &prawy_prostokat);
}
