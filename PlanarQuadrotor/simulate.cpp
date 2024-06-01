/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <thread>
#include <matplot/matplot.h>
#include <SDL/include/SDL.h>

//dźwięk silników
void generateEngineSound(Uint8* buffer, int length, double frequency, double left_gain) {
    const int amplitude = 1; // Amplituda dźwięku
    const double sampleRate = 44100.0; // Częstotliwość próbkowania

    for (int i = 0; i < length; ++i) {
        double time = i / sampleRate;
        double value = amplitude * sin(2.0 * M_PI * frequency * time);

        // Zmiana amplitudy w zależności od kierunku ruchu
        if (left_gain > 1.0)
            buffer[i] = (Uint8)(value * left_gain + 2); // Lewy głośnik głośniejszy
        else if (left_gain < 1.0)
            buffer[i] = (Uint8)(value + 2* left_gain); // Prawy głośnik głośniejszy
        else
            buffer[i] = (Uint8)(value + 2); // Równa amplituda dla obu głośników
    }
}


Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    //LQR
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
     // Inicjalizacja SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // Konfiguracja specyfikacji audio
    SDL_AudioSpec spec;
    spec.freq = 54100; // Częstotliwość próbkowania
    spec.format = AUDIO_U8; // Format dźwięku
    spec.channels = 2; // Liczba kanałów (stereo)
    spec.samples = 8096; // Rozmiar bufora próbek
    spec.callback = NULL; // Brak funkcji callback
    spec.userdata = NULL;

    // Otwarcie urządzenia audio
    SDL_AudioDeviceID deviceId = SDL_OpenAudioDevice(NULL, 0, &spec, NULL, 0);
    if (deviceId == 0) {
        std::cerr << "Failed to open audio device: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    const int start_x=640;
    const int start_y=360;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << start_x, start_y, 0, 0, 0, 0; 
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << -1, 7, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
              if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else 
                {
                    if (e.type == SDL_MOUSEMOTION)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    }
                    if(e.type == SDL_MOUSEBUTTONDOWN){
                        SDL_GetMouseState(&x, &y);
                        goal_state<<x,y,0,0,0,0;
                        std::cout<<"Mouse left-click "<<x<<" "<<y<<std::endl;
                        quadrotor.SetGoal(goal_state);
                    }
                }
                
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
        quadrotor.Update(dt);

        // Generowanie dźwięku silnika na podstawie ruchu quadrotora
        double state_theta = quadrotor.GetState()[2]; // Pobierz kąt theta
        double left_gain = 1 + std::abs(state_theta) / M_PI; // Oblicz współczynnik wzmocnienia dla lewego głośnika
        double frequency = 150; // Bazowa częstotliwość dźwięku
        Uint8* audioBuffer = new Uint8[spec.samples];
        generateEngineSound(audioBuffer, spec.samples, frequency, left_gain);
        SDL_QueueAudio(deviceId, audioBuffer, spec.samples);
        SDL_PauseAudioDevice(deviceId, 0);
        delete[] audioBuffer;
    }
    
    }
    // Zwalnianie zasobów SDL
    SDL_CloseAudioDevice(deviceId);
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
