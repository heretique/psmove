#include "raylib.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_raylib.h"
#include "PSMoveConfigurator.h"


namespace ui = ImGui;


int main(int argc, char* argv[])
{
    int screenWidth = 1920;
    int screenHeight = 1080;

    InitWindow(screenWidth, screenHeight, "PS Move Tracker Configurator");

    ConfigureContext cc;
    cc.sm.addState<ConfigureMain>(States::Main, "Main");
    cc.sm.addState<ConfigureCamera>(States::ConfigureCamera, "Configure Camera");
    cc.sm.addState<ConfigureController>(States::ConfigureController, "Configure Controllers");
    cc.sm.addState<ConfigureTracker>(States::ConfigureTracker, "Configure Tracker");

    SetTargetFPS(60);

    ui::CreateContext();
    ui::StyleColorsDark();

    ImGui_ImplOpenGL3_Init();
    ImGui_ImplRaylib_Init();


    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);

    while (!WindowShouldClose() && !cc.shouldExit)
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplRaylib_NewFrame();
        ui::NewFrame();
        ImGui_ImplRaylib_ProcessEvent();

        BeginDrawing();

        {
            ClearBackground(RAYWHITE);

            cc.sm.tick();


            ui::Render();
            BeginTextureMode(target);       // Enable drawing to texture
            ClearBackground(BLANK);
            ImGui_ImplOpenGL3_RenderDrawData(ui::GetDrawData());
            EndTextureMode();


            // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
            DrawTextureRec(target.texture, { 0.f, 0.f, (float)target.texture.width, (float)-target.texture.height }, { 0.f, 0.f }, RAYWHITE);
        }

        DrawFPS(10, 10);
        EndDrawing();

    }

    //for (auto controller : controllers)
    //{
    //    psmove_disconnect(controller);
    //}

    ImGui_ImplRaylib_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();

    CloseWindow();

    return 0;
}