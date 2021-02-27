#include "PSMoveConfigurator.h"
#include "opencv2/imgproc.hpp"
#include "imgui.h"
#include "RttrImguiInspector.h"
#include "raymath.h"

namespace ui = ImGui;

constexpr float kFloatTolerance = 2e-37f;

void MatrixDecompose(const Matrix& matrix, Quaternion& rotation, Vector3& scale, Vector3& translation)
{
    // Extract the translation.
    translation.x = matrix.m12;
    translation.y = matrix.m13;
    translation.z = matrix.m14;

    // Extract the scale.
    // This is simply the length of each axis (row/column) in the matrix.
    Vector3  xaxis{matrix.m0, matrix.m1, matrix.m2};
    float scaleX = Vector3Length(xaxis);

    Vector3  yaxis{matrix.m4, matrix.m5, matrix.m6};
    float scaleY = Vector3Length(yaxis);

    Vector3  zaxis{matrix.m8, matrix.m9, matrix.m10};
    float scaleZ = Vector3Length(zaxis);

    // Determine if we have a negative scale (true if determinant is less than
    // zero).
    // In this case, we simply negate a single axis of the scale.
    float det = MatrixDeterminant(matrix);
    if (det < 0)
        scaleZ = -scaleZ;

    scale.x = scaleX;
    scale.y = scaleY;
    scale.z = scaleZ;

    // Scale too close to zero, can't decompose rotation.
    if (scaleX < kFloatTolerance || scaleY < kFloatTolerance || abs(scaleZ) < kFloatTolerance)
    {
        printf("Scale too close to zero, can't decompose rotation");
        abort();
    }

    float rn;

    // Factor the scale out of the matrix axes.
    rn = 1.0f / scaleX;
    xaxis.x *= rn;
    xaxis.y *= rn;
    xaxis.z *= rn;

    rn = 1.0f / scaleY;
    yaxis.x *= rn;
    yaxis.y *= rn;
    yaxis.z *= rn;

    rn = 1.0f / scaleZ;
    zaxis.x *= rn;
    zaxis.y *= rn;
    zaxis.z *= rn;

    // Now calculate the rotation from the resulting matrix (axes).
    float trace = xaxis.x + yaxis.y + zaxis.z + 1.0f;

    if (trace > 1.0f)
    {
        float s = 0.5f / sqrt(trace);
        rotation.w = 0.25f / s;
        rotation.x = (yaxis.z - zaxis.y) * s;
        rotation.y = (zaxis.x - xaxis.z) * s;
        rotation.z = (xaxis.y - yaxis.x) * s;
    }
    else
    {
        // Note: since xaxis, yaxis, and zaxis are normalized,
        // we will never divide by zero in the code below.
        if (xaxis.x > yaxis.y && xaxis.x > zaxis.z)
        {
            float s = 0.5f / sqrt(1.0f + xaxis.x - yaxis.y - zaxis.z);
            rotation.w = (yaxis.z - zaxis.y) * s;
            rotation.x = 0.25f / s;
            rotation.y = (yaxis.x + xaxis.y) * s;
            rotation.z = (zaxis.x + xaxis.z) * s;
        }
        else if (yaxis.y > zaxis.z)
        {
            float s = 0.5f / sqrt(1.0f + yaxis.y - xaxis.x - zaxis.z);
            rotation.w = (zaxis.x - xaxis.z) * s;
            rotation.x = (yaxis.x + xaxis.y) * s;
            rotation.y = 0.25f / s;
            rotation.z = (zaxis.y + yaxis.z) * s;
        }
        else
        {
            float s = 0.5f / sqrt(1.0f + zaxis.z - xaxis.x - yaxis.y);
            rotation.w = (xaxis.y - yaxis.x) * s;
            rotation.x = (zaxis.x + xaxis.z) * s;
            rotation.y = (zaxis.y + yaxis.z) * s;
            rotation.z = 0.25f / s;
        }
    }
}

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
    capture = std::make_unique<cv::VideoCapture>(0, cv::CAP_MSMF);
    if (!capture->isOpened())
    {
        return {};
    }
    capture->set(cv::CAP_PROP_FRAME_WIDTH, _context.screenWidth);
    capture->set(cv::CAP_PROP_FRAME_HEIGHT, _context.screenHeight);
    capture->set(cv::CAP_PROP_FPS, 60);
    *capture >> frame;
    cv:cvtColor(frame, threshHSV, cv::COLOR_BGR2HSV);
    cv::inRange(threshHSV, minHSV, maxHSV, thresh);
    Image videoImage;

    videoImage.width = frame.cols;
    videoImage.height = frame.rows;
    videoImage.format = UNCOMPRESSED_R8G8B8;
    videoImage.mipmaps = 1;
    videoImage.data = (void*)(frame.data);
    videoTex = LoadTextureFromImage(videoImage);

    Image threshImage;
    threshImage.width = frame.cols;
    threshImage.height = frame.rows;
    threshImage.format = UNCOMPRESSED_GRAYSCALE;
    threshImage.mipmaps = 1;
    threshImage.data = (void*)(thresh.data);
    threshTex = LoadTextureFromImage(threshImage);

    int controllersCount = psmove_count_connected();
    for (auto i = 0; i < controllersCount; ++i)
    {
        PSMove* controller = psmove_connect_by_id(i);
        if (controller)
        {
            controllers.push_back(controller);
            psmove_set_rate_limiting(controller, PSMove_True);
        }
    }

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
    ui::Begin("Camera Settings");


    static bool autoExposure = capture->get(cv::CAP_PROP_AUTO_EXPOSURE);
    static int exposure = capture->get(cv::CAP_PROP_EXPOSURE);
    static bool showCamera = false;
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

    ui::Checkbox("Show camera view", & showCamera);

    ui::SliderInt("minH", &minHSV[0], 0, 179);
    ui::SliderInt("minS", &minHSV[1], 0, 255);
    ui::SliderInt("minV", &minHSV[2], 0, 255);
    ui::SliderInt("maxH", &maxHSV[0], 0, 179);
    ui::SliderInt("maxS", &maxHSV[1], 0, 255);
    ui::SliderInt("maxV", &maxHSV[2], 0, 255);

    if (showCamera)
    {
        UpdateTexture(videoTex, (void*)(frame.data));
        DrawTexture(videoTex, 0, 0, RAYWHITE);
    }
    else
    {
    cv:cvtColor(frame, threshHSV, cv::COLOR_BGR2HSV);
        cv::inRange(threshHSV, minHSV, maxHSV, thresh);
        cv::bitwise_and(frame, frame, frame, thresh);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        UpdateTexture(threshTex, (void*)(thresh.data));
        DrawTexture(threshTex, 0, 0, RAYWHITE);
    }

    if (!controllers.size())
    {
        ui::Text("No controllers connected!");
    }
    else
    {
        PSMove* move = controllers[currentController];

        ui::Text("Controller color:");
        static int R = 0;
        static int G = 0;
        static int B = 0;
        ui::SliderInt("R", &R, 0, 255);
        ui::SliderInt("G", &G, 0, 255);
        ui::SliderInt("B", &B, 0, 255);

        psmove_set_leds(move, R, G, B);
        psmove_update_leds(move);
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
        PSMove* move = psmove_connect_by_id(i);
        if (move)
        {
            Controller controller;
            controller.move = move;
            controllers.emplace_back(controller);
            if (psmove_has_calibration(move))
            {
                psmove_enable_orientation(move, PSMove_True);
                psmove_set_orientation_fusion_type(move, PSMoveOrientation_Fusion_Type::OrientationFusion_ComplementaryMARG);
            }
            psmove_set_rate_limiting(move, PSMove_True);
        }
        else
        {
            printf("Couldn't connect controller %d to api!\n", i);
        }
    }
    camera.position = { 0.f, 0.f, .5f }; // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 60.0f;                                // Camera field-of-view Y
    camera.type = CAMERA_PERSPECTIVE;
    SetCameraMode(camera, CAMERA_CUSTOM);
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
        int controllerIndex = 0;
        for (auto& controller : controllers)
        {
            PSMove* move = controller.move;
            const int xRotation = -90;
            controllerIndex++;
            psmove_poll(move);
            psmove_poll(move);
            ui::Text("Controller %d", controllerIndex);
            ui::Separator();

            PSMove_Battery_Level batteryLevel = psmove_get_battery(move);
            ui::Text("Battery level: %d", batteryLevel);

            if (psmove_has_orientation(move))
            {
                float x, y, z, w;
                psmove_get_orientation(move, &w, &x, &y, &z);
                Vector3 axis;
                float angle;
                Quaternion xRotQuat = QuaternionFromAxisAngle({ 1, 0, 0 }, xRotation * DEG2RAD);
                QuaternionToAxisAngle(QuaternionMultiply({ x, y, z, w }, xRotQuat), &axis, &angle);
                BeginMode3D(camera);

                DrawModelEx(_context.psMoveModel, { -.25f + (static_cast<float>(controllerIndex - 1)/ controllers.size()) * 1.f, 0, 0}, axis, angle * RAD2DEG, { 0.5f, .5f, .5f }, WHITE);

                EndMode3D();
            }
            else
            {

            }
            ui::PushID(controllerIndex);
            ui::SliderInt("R", &controller.R, 0, 255);
            ui::SliderInt("G", &controller.G, 0, 255);
            ui::SliderInt("B", &controller.B, 0, 255);
            ui::PopID();

            psmove_set_leds(move, controller.R, controller.G, controller.B);
            psmove_update_leds(move);
        }
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
        psmove_disconnect(controller.move);
    }
    controllers.clear();
    return {};
}

void ConfigureController::magnetormeterCalibrationUpdate()
{

}

bool ConfigureController::isMoveStableAndAlignedWithGravity()
{
    return false;
}

Configure::ResultType ConfigureTracker::onStateEnter()
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

    if (!controllers.size())
    {
        return {};
    }

    settings.cameraFrameWidth = _context.screenWidth;
    settings.cameraFrameHeight = _context.screenHeight;
    settings.cameraFrameRate = 60;
    settings.colorMappingMaxAge = 0;
    settings.exposureMode = Exposure_LOW;
    settings.cameraMirror = PSMove_True;

    tracker = std::make_unique<PSMoveTracker>();

    if (!tracker->initialize(0, cv::CAP_MSMF, settings))
    {
        printf("Could not init PSMoveTracker.\n");
        return States::Main;
    }

    int result;
    for (auto i = 0; i < controllers.size(); ++i)
    {
        for (;;) {
            printf("Calibrating controller %d...", i);
            fflush(stdout);
            result = tracker->enable(controllers[i]);

            if (result == Tracker_CALIBRATED) {
                bool auto_update_leds =
                    tracker->getAutoUpdateLeds(controllers[i]);
                printf("OK, auto_update_leds is %s\n",
                        (auto_update_leds == PSMove_True) ? "enabled" : "disabled");
                break;
            }
            else {
                printf("ERROR - retrying\n");
            }
        }
    }

    frame = tracker->getImage();
    Image videoImage;

    videoImage.width = frame.cols;
    videoImage.height = frame.rows;
    videoImage.format = UNCOMPRESSED_R8G8B8;
    videoImage.mipmaps = 1;
    videoImage.data = (void*)(frame.data);
    videoTex = LoadTextureFromImage(videoImage);

    return {};
}

Configure::ResultType ConfigureTracker::onStateUpdate()
{
    ResultType ret{};
    ui::Begin("Tracker Settings");

    if (!controllers.size())
    {
        ui::Text("There is no controller connected!");
    }
    else
    {
        rttr::variant var = settings;
        if (inspectVar(var))
        {
            settings = var.get_value<PSMoveTrackerSettings>();
        }

        // tracker update
        tracker->updateImage();
        tracker->update(NULL);
        tracker->annotate();
        frame = tracker->getImage();
        UpdateTexture(videoTex, (void*)(frame.data));
        DrawTexture(videoTex, 0, 0, RAYWHITE);
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
    UnloadTexture(videoTex);
    frame.release();
    tracker.reset();
    for (auto controller : controllers)
    {
        psmove_disconnect(controller);
    }
    controllers.clear();
    return {};
}

Configure::ResultType TestFusion::onStateEnter()
{
    int controllersCount = psmove_count_connected();
    for (auto i = 0; i < controllersCount; ++i)
    {
        PSMove* controller = psmove_connect_by_id(i);
        if (controller)
        {
            controllers.push_back(controller);
            std::deque<Vector3> trace;
            traces.emplace(std::make_pair((int)controller, std::move(trace)));
        }
        else
        {
            printf("Couldn't connect controller %d to api!\n", i);
        }
    }

    if (!controllers.size())
    {
        return {};
    }

    settings.cameraFrameWidth = _context.screenWidth;
    settings.cameraFrameHeight = _context.screenHeight;
    settings.cameraFrameRate = 60;
    settings.colorMappingMaxAge = 0;
    settings.exposureMode = Exposure_LOW;
    settings.cameraMirror = PSMove_True;

    tracker = std::make_unique<PSMoveTracker>();

    if (!tracker->initialize(0, cv::CAP_MSMF, settings))
    {
        printf("Could not init PSMoveTracker.\n");
        return States::Main;
    }

    int result;
    for (auto i = 0; i < controllers.size(); ++i)
    {
        for (;;) {
            printf("Calibrating controller %d...", i);
            fflush(stdout);
            result = tracker->enable(controllers[i]);

            if (result == Tracker_CALIBRATED) {
                bool auto_update_leds =
                    tracker->getAutoUpdateLeds(controllers[i]);
                printf("OK, auto_update_leds is %s\n",
                        (auto_update_leds == PSMove_True) ? "enabled" : "disabled");
                break;
            }
            else {
                printf("ERROR - retrying\n");
            }
        }
    }

    fusion = std::make_unique<PSMoveFusion>(tracker.get(), .037f, 1000.f);

    for (auto controller : controllers)
    {
        if (psmove_has_calibration(controller))
        {
            psmove_enable_orientation(controller, PSMove_True);
            psmove_set_orientation_fusion_type(controller, PSMoveOrientation_Fusion_Type::OrientationFusion_ComplementaryMARG);
        }
        psmove_set_rate_limiting(controller, PSMove_True);
    }

    camera.position = { 0.f, 6.f, 5.f }; // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 60.f;                                // Camera field-of-view Y
    camera.type = CAMERA_PERSPECTIVE;
    SetCameraMode(camera, CAMERA_ORBITAL);

    return {};
}

Configure::ResultType TestFusion::onStateUpdate()
{
    ResultType ret{};

    UpdateCamera(&camera);

    ui::Begin("Controller Settings");
    if (!controllers.size())
    {
        ui::Text("No controllers connected!");
    }
    else
    {
        tracker->updateImage();
        tracker->update(NULL);

        ui::Text("Camera Position (%.3f, %.3f, %.3f)", camera.position.x, camera.position.y, camera.position.z);
        ui::NewLine();
        int controllerIndex = 0;
        for (auto controller : controllers)
        {
            PSMove* move = controller;
            const int xRotation = -90;
            controllerIndex++;
            psmove_poll(move);
            psmove_poll(move);
            ui::Text("Controller %d", controllerIndex);
            ui::Separator();

            PSMove_Battery_Level batteryLevel = psmove_get_battery(move);
            ui::Text("Battery level: %d", batteryLevel);


            float x, y, z, w;
           
            fusion->getPosition(move, x, y, z);
            ui::Text("Position (%.3f, %.3f, %.3f)", x, y, z);
            Vector3 position{x, y, z};
            if (psmove_has_orientation(move))
            {
                psmove_get_orientation(move, &w, &x, &y, &z);
                Vector3 axis;
                float angle;

                Matrix cameraMatrix = MatrixInvert(GetCameraMatrix(camera));
                Quaternion cameraRotation;
                Quaternion cameraRotationUp;
                Vector3 cameraPosition;
                Vector3 cameraScale;

                MatrixDecompose(cameraMatrix, cameraRotation, cameraScale, cameraPosition);

                Quaternion xRotQuat = QuaternionFromAxisAngle({ 1, 0, 0 }, xRotation * DEG2RAD);
                BeginMode3D(camera);


                //Vector3 controllersPosition = Vector3Add(position, camera.position);
                QuaternionToAxisAngle(QuaternionMultiply(cameraRotation, QuaternionMultiply({ x, y, z, w }, xRotQuat)), &axis, &angle);
                Vector3 controllersPosition = Vector3Transform({position.x, position.y, -position.z}, cameraMatrix);
                ui::Text("Final Position (%.3f, %.3f, %.3f)", controllersPosition.x, controllersPosition.y, controllersPosition.z);
                DrawModelEx(_context.psMoveModel, controllersPosition, axis, angle * RAD2DEG, { 1.f, 1.f, 1.f }, WHITE);
                int buttons = psmove_get_buttons(move);
                if (buttons & Btn_MOVE)
                {
                    auto& trace = traces[(int)move];
                    trace.emplace_back(controllersPosition);
                    if (trace.size() > 1000)
                    {
                        trace.pop_front();
                    }
                }

                auto& trace = traces[(int)move];
                for (const auto& pos : trace)
                {
                    DrawSphere(pos, 0.01f, SKYBLUE);
                }

                DrawGrid(10, 1.f);
                EndMode3D();
            }
            else
            {

            }
        }
    }

    if (ui::Button("Return"))
    {
        ret = States::Main;
    }
    ui::End();
    return ret;
}

Configure::ResultType TestFusion::onStateExit()
{
    fusion.reset();
    tracker.reset();
    for (auto controller : controllers)
    {
        psmove_disconnect(controller);
    }
    controllers.clear();
    return {};
}
