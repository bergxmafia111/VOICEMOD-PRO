#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstdlib>      // for std::exit
#include <cmath>

#include "Eigen/Core"
#include "SDL.h"
#include "refract.h"

static const double DEFAULT_ETA = 1.457; // default refractive index

// Dimensions of the display window, initially 256 x 256.
static int windowWidth  = 256;
static int windowHeight = 256;


//------------------------------------------------------------------------------
// Renders the intersection points onto the SDL renderer at the current window
// size, adapting from [0..256] => [0..windowWidth] etc.
//------------------------------------------------------------------------------
static void
DrawIntersections(SDL_Renderer *renderer,
                  const std::vector<Eigen::Vector2d> &intersections)
{
    // Scale from the pre-coded 256x256 space into the current window size
    const float scaleX = static_cast<float>(windowWidth)  / 256.0f;
    const float scaleY = static_cast<float>(windowHeight) / 256.0f;

    // Clear the screen with black
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw intersection points in white
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    for (const auto &pt : intersections) {
        // DrawPointF is an SDL extension for float; might require SDL2.0.22+
        // If not available, you can cast & use SDL_RenderDrawPoint
        SDL_RenderDrawPointF(renderer, pt.x() * scaleX, pt.y() * scaleY);
    }

    // Show updated rendering
    SDL_RenderPresent(renderer);
}


//------------------------------------------------------------------------------
// Main Program
//------------------------------------------------------------------------------
int
main(int argc, char** argv)
{
    // Simple usage check
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <path/to/lens.obj> <distance-to-receiver-plane>\n"
                  << "Example:\n"
                  << "  " << argv[0] << " lens.obj 5.0\n";
        return 1;
    }

    // Attempt to parse arguments
    const std::string objFilePath = argv[1];
    const double receiverPlaneZ = std::stod(argv[2]);
    double planeZ = receiverPlaneZ; // store modifiable planeZ

    // Prepare data containers
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> refracteds;
    std::vector<Eigen::Vector2d> intersections;

    // 1) Parse OBJ
    try {
        ParseOBJ(objFilePath, &vertices, &normals);
    }
    catch (const std::exception &ex) {
        std::cerr << "Error parsing OBJ: " << ex.what() << std::endl;
        return 2;
    }

    if (vertices.empty() || normals.empty()) {
        std::cerr << "No vertices or normals found. Exiting.\n";
        return 2;
    }

    // 2) Refract
    try {
        Refract(normals, &refracteds, DEFAULT_ETA);
    }
    catch (const std::exception &ex) {
        std::cerr << "Error computing refractions: " << ex.what() << std::endl;
        return 3;
    }

    // 3) Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL Error: "
                  << SDL_GetError() << std::endl;
        return 4;
    }

    // Create a window & renderer
    SDL_Window *window = SDL_CreateWindow("Caustics Image",
                                          SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED,
                                          windowWidth,
                                          windowHeight,
                                          SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "Window could not be created! SDL Error: "
                  << SDL_GetError() << std::endl;
        SDL_Quit();
        return 5;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL Error: "
                  << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 6;
    }

    // 4) Compute intersections & Draw
    CalculateIntersections(vertices, refracteds, &intersections, planeZ);
    DrawIntersections(renderer, intersections);

    // Main loop
    bool quit = false;
    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_QUIT:
                    quit = true;
                    break;

                case SDL_WINDOWEVENT:
                    // Resized or something else
                    if (e.window.event == SDL_WINDOWEVENT_RESIZED) {
                        windowWidth  = e.window.data1;
                        windowHeight = e.window.data2;
                        // Re-draw with new scaling
                        DrawIntersections(renderer, intersections);
                    }
                    break;

                case SDL_KEYDOWN:
                    switch (e.key.keysym.sym) {
                        case SDLK_w:  // Increase planeZ
                            planeZ += 0.1;
                            CalculateIntersections(vertices, refracteds, &intersections, planeZ);
                            DrawIntersections(renderer, intersections);
                            std::cout << "PlaneZ changed to: " << planeZ << std::endl;
                            break;
                        case SDLK_s:  // Decrease planeZ
                            planeZ -= 0.1;
                            CalculateIntersections(vertices, refracteds, &intersections, planeZ);
                            DrawIntersections(renderer, intersections);
                            std::cout << "PlaneZ changed to: " << planeZ << std::endl;
                            break;
                        case SDLK_q:  // Print current planeZ
                            std::cout << "Current planeZ: " << planeZ << std::endl;
                            break;
                        case SDLK_ESCAPE:
                            quit = true;
                            break;
                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
        }
        // We don't necessarily need to do anything else here
        // but if you want a small delay or any logic, you can add it
        SDL_Delay(5);
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
