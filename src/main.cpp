#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstdlib>

#include "Eigen/Core"
#include "SDL.h"
#include "refract.h"

/**
 * The refractive index of the lens material that was used
 * to generate the lens geometry.
 */
static constexpr double ETA = 1.457;

/**
 * Default window dimensions. Adjusted upon user window resize.
 */
static int windowWidth = 256;
static int windowHeight = 256;

/**
 * @brief Renders the computed 2D intersections onto the provided SDL_Renderer.
 *
 * Each intersection is drawn as a single pixel. The coordinates stored
 * in `intersections` are assumed to be in a [0..256] range for both x and y,
 * and we scale them proportionally to match the current window size.
 *
 * @param renderer       Valid pointer to an SDL_Renderer.
 * @param intersections  The (x,y) positions to plot, each assumed [0..256] range.
 */
static void DrawIntersections(SDL_Renderer* renderer, const std::vector<Eigen::Vector2d>& intersections)
{
    // Convert from nominal 256x256 domain to the actual window size.
    const float scaleX = static_cast<float>(windowWidth) / 256.0f;
    const float scaleY = static_cast<float>(windowHeight) / 256.0f;

    // Clear the screen with black
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw each intersection point in white
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (const auto& point : intersections)
    {
        // Use floating coordinates version for drawing
        SDL_RenderDrawPointF(renderer, static_cast<float>(point.x()) * scaleX,
                                       static_cast<float>(point.y()) * scaleY);
    }

    // Present the updated content
    SDL_RenderPresent(renderer);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <lens.obj> <distance>\n\n"
                  << "  lens.obj   Path to an OBJ file containing lens geometry.\n"
                  << "  distance   Initial distance between the lens and the receiver plane.\n"
                  << "             (Wall is parallel to the x-y plane at z=<distance>.)\n" << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize vectors for lens geometry
    std::vector<Eigen::Vector3d> vertices;    // 3D positions where rays pass
    std::vector<Eigen::Vector3d> normals;     // Normal vectors at each vertex
    std::vector<Eigen::Vector3d> refracteds;  // Refracted directions (normalized)
    std::vector<Eigen::Vector2d> intersections;  // (x,y) intersection points on the receiver

    // Parse arguments
    const char* objPath = argv[1];
    double receiverPlane;
    try {
        receiverPlane = std::stod(argv[2]);
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing distance argument: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Attempt to parse the OBJ geometry
    ParseOBJ(objPath, &vertices, &normals);  // user-provided function from "refract.h"
    if (vertices.empty() || normals.empty()) {
        std::cerr << "Error: No geometry or normals found in " << objPath << std::endl;
        return EXIT_FAILURE;
    }

    // Refract the rays
    Refract(normals, &refracteds, ETA);

    // Initialize SDL
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    // Create a window for visualization
    SDL_Window* window = SDL_CreateWindow(
        "Caustics Visualizer",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        windowWidth,
        windowHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Create an accelerated renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED
    );
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Calculate the initial intersections
    CalculateIntersections(vertices, refracteds, &intersections, receiverPlane);
    DrawIntersections(renderer, intersections);

    bool quit = false;
    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_QUIT:
                    quit = true;
                    break;

                case SDL_WINDOWEVENT:
                    if (e.window.event == SDL_WINDOWEVENT_RESIZED) {
                        windowWidth = e.window.data1;
                        windowHeight = e.window.data2;
                        // Re-draw with new window dimension
                        DrawIntersections(renderer, intersections);
                    }
                    break;

                case SDL_KEYDOWN: {
                    const SDL_Keycode key = e.key.keysym.sym;
                    switch (key) {
                        case SDLK_w:
                            // Increase plane distance
                            receiverPlane += 0.1;
                            CalculateIntersections(vertices, refracteds, &intersections, receiverPlane);
                            DrawIntersections(renderer, intersections);
                            break;

                        case SDLK_s:
                            // Decrease plane distance
                            receiverPlane -= 0.1;
                            CalculateIntersections(vertices, refracteds, &intersections, receiverPlane);
                            DrawIntersections(renderer, intersections);
                            break;

                        case SDLK_q:
                            // Print the current distance
                            std::cout << "[INFO] Current lens-to-wall distance: " << receiverPlane << std::endl;
                            break;

                        case SDLK_ESCAPE:
                            quit = true;
                            break;

                        default:
                            break;
                    }
                } break;

                default:
                    break;
            }
        }
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
