#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h" // The magic bridge header

int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 800;
    
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Modern Raylib + ImGui Setup");
    SetTargetFPS(60);

    // Initialize rlImGui (pass true to manage and load default ImGui fonts)
    rlImGuiSetup(true);

    // Main game loop
    while (!WindowShouldClose()) {
        // 1. Update your game state here
        
        // 2. Drawing phase
        BeginDrawing();
        ClearBackground(DARKGRAY);

        // Draw native Raylib content
        DrawCircle(screenWidth / 2, screenHeight / 2, 50.0f, MAROON);

        // 3. ImGui Render Phase
        rlImGuiBegin();

        // Draw ImGui widgets
        ImGui::Begin("Developer Tools");
        ImGui::Text("Hello, user! This is a modern UI layout.");
        if (ImGui::Button("Click Me")) {
            // Handle button press
        }
        ImGui::End();

        rlImGuiEnd(); // Ends ImGui rendering layer

        EndDrawing();
    }

    // Cleanup
    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
