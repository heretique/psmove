#include "PSMoveConfigurator.h"
#include "opencv2/imgproc.hpp"
#include "imgui.h"
#include "RttrImguiInspector.h"

namespace ui = ImGui;

ConfigureContext::ConfigureContext()
    :sm(*this)
{

}

ConfigureContext::~ConfigureContext()
{

}

Configure::ResultType ConfigureMain::onStateEnter()
{
    return {};
}

Configure::ResultType ConfigureMain::onStateUpdate()
{
    Configure::ResultType ret{};
    ui::Begin("Main Settings");

    static bool showDemoWindow{ false };
    ui::Checkbox("Show Demo Window", &showDemoWindow);
    if (showDemoWindow)
    {
        ui::ShowDemoWindow();
    }

    const std::vector<std::string>& names = _context.sm.stateNames();
    for (const auto& name : names)
    {
        if (ui::Button(name.c_str()))
        {
            std::optional<States> state = _context.sm.stateFromName(name);
            if (state)
            {
                ret = *state;
                break;
            }
        }
    }
    if (ui::Button("Exit"))
    {
        _context.shouldExit = true;
    }
    ui::End();

    return ret;
}

Configure::ResultType ConfigureMain::onStateExit()
{
    return {};
}

Configure::ResultType ConfigureCamera::onStateEnter()
{
    capture = std::make_unique<cv::VideoCapture>(0, cv::CAP_DSHOW);
    if (!capture->isOpened())
    {
        return {};
    }
    capture->set(cv::CAP_PROP_FRAME_WIDTH, _context.screenWidth);
    capture->set(cv::CAP_PROP_FRAME_HEIGHT, _context.screenHeight);
    capture->set(cv::CAP_PROP_FPS, 60);
    *capture >> frame;
    Image videoImage;

    videoImage.width = frame.cols;
    videoImage.height = frame.rows;
    videoImage.format = UNCOMPRESSED_R8G8B8;
    videoImage.mipmaps = 1;
    videoImage.data = (void*)(frame.data);
    videoTex = LoadTextureFromImage(videoImage);

    return {};
}

Configure::ResultType ConfigureCamera::onStateUpdate()
{
    Configure::ResultType ret{};
    if (!capture->isOpened())
    {
        ui::Begin("Camera Initialize Error");
        if (ui::Button("Return"))
        {
            ret = States::Main;
        }
        ui::End();

        return ret;
    }
    *capture >> frame;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    UpdateTexture(videoTex, (void*)(frame.data));
    DrawTexture(videoTex, 0, 0, RAYWHITE);
    ui::Begin("Camera Settings");
    static bool autoExposure = capture->get(cv::CAP_PROP_AUTO_EXPOSURE);
    static int exposure = capture->get(cv::CAP_PROP_EXPOSURE);
    if (ui::Checkbox("Auto Exposure", &autoExposure))
    {
        capture->set(cv::CAP_PROP_AUTO_EXPOSURE, autoExposure);
    }
    if (!autoExposure)
    {
        if (ui::SliderInt("Exposure", &exposure, 1, -20))
        {
            capture->set(cv::CAP_PROP_EXPOSURE, exposure);
        }
    }

    if (ui::Button("Return"))
    {
        ret = States::Main;
    }

    ui::End();


    return ret;
}

Configure::ResultType ConfigureCamera::onStateExit()
{
    UnloadTexture(videoTex);
    frame.release();
    capture.reset();
    return {};
}


Configure::ResultType ConfigureController::onStateEnter()
{
    int controllersCount = psmove_count_connected();
    for (auto i = 0; i < controllersCount; ++i)
    {
        PSMove* controller = psmove_connect_by_id(i);
        if (controller)
        {
            controllers.push_back(controller);
        }
        else
        {
            printf("Couldn't connect controller %d to api!\n", i);
        }
    }
    return {};
}

Configure::ResultType ConfigureController::onStateUpdate()
{
    ResultType ret{};
    ui::Begin("Controller Settings");
    if (!controllers.size())
    {
        ui::Text("No controllers connected!");
    }
    else
    {

    }

    if (ui::Button("Return"))
    {
        ret = States::Main;
    }
    ui::End();
    return ret;
}

Configure::ResultType ConfigureController::onStateExit()
{
    for (auto controller : controllers)
    {
        psmove_disconnect(controller);
    }
    controllers.clear();
    return {};
}

Configure::ResultType ConfigureTracker::onStateEnter()
{
    //int controllersCount = psmove_count_connected();
    //for (auto i = 0; i < controllersCount; ++i)
    //{
    //    PSMove* controller = psmove_connect_by_id(i);
    //    if (controller)
    //    {
    //        controllers.push_back(controller);
    //    }
    //    else
    //    {
    //        printf("Couldn't connect controller %d to api!\n", i);
    //    }
    //}

    //if (!controllers.size())
    //{
    //    return {};
    //}

    settings.cameraFrameWidth = _context.screenWidth;
    settings.cameraFrameHeight = _context.screenHeight;
    settings.cameraFrameRate = 60;
    settings.colorMappingMaxAge = 0;
    settings.exposureMode = Exposure_LOW;
    settings.cameraMirror = PSMove_True;

    //tracker = std::make_unique<PSMoveTracker>();

    //if (!tracker->initialize(settings))
    //{
    //    printf("Could not init PSMoveTracker.\n");
    //    return States::Main;
    //}

    //int result;
    //for (auto i = 0; i < controllers.size(); ++i)
    //{
    //    for (;;) {
    //        printf("Calibrating controller %d...", i);
    //        fflush(stdout);
    //        result = tracker->enable(controllers[i]);

    //        if (result == Tracker_CALIBRATED) {
    //            bool auto_update_leds =
    //                tracker->getAutoUpdateLeds(controllers[i]);
    //            printf("OK, auto_update_leds is %s\n",
    //                    (auto_update_leds == PSMove_True) ? "enabled" : "disabled");
    //            break;
    //        }
    //        else {
    //            printf("ERROR - retrying\n");
    //        }
    //    }
    //}

    //frame = tracker->getImage();
    //Image videoImage;

    //videoImage.width = frame.cols;
    //videoImage.height = frame.rows;
    //videoImage.format = UNCOMPRESSED_R8G8B8;
    //videoImage.mipmaps = 1;
    //videoImage.data = (void*)(frame.data);
    //videoTex = LoadTextureFromImage(videoImage);

    return {};
}

Configure::ResultType ConfigureTracker::onStateUpdate()
{
    ResultType ret{};
    ui::Begin("Tracker Settings");

    //if (!controllers.size())
    //{
    //    ui::Text("There is no controller connected!");
    //}
    //else
    {
        rttr::variant var = settings;
        if (inspectVar(var))
        {
            settings = var.get_value<PSMoveTrackerSettings>();
        }

        //// tracker update
        //tracker->updateImage();
        //tracker->update(NULL);
        //tracker->annotate();
        //frame = tracker->getImage();
        //UpdateTexture(videoTex, (void*)(frame.data));
        //DrawTexture(videoTex, 0, 0, RAYWHITE);
    }

    if (ui::Button("Return"))
    {
        ret = States::Main;
    }
    ui::End();

    return ret;
}

Configure::ResultType ConfigureTracker::onStateExit()
{
    //UnloadTexture(videoTex);
    //frame.release();
    //tracker.reset();
    return {};
}
