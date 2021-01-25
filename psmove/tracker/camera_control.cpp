/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditti <benjamin.venditti@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#include "psmove_config.h"
#include "camera_control.h"
#include "psmove_tracker.h"

#include "../psmove_private.h"

#include <stdio.h>
#include <stdint.h>

#include "camera_control_private.h"

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "windows.h"

#define CL_DRIVER_REG_PATH "Software\\PS3EyeCamera\\Settings"

void
get_metrics(int *width, int *height)
{
    *width = psmove_util_get_env_int(PSMOVE_TRACKER_WIDTH_ENV);
    *height = psmove_util_get_env_int(PSMOVE_TRACKER_HEIGHT_ENV);

    if (*width == -1) {
        *width = PSMOVE_TRACKER_DEFAULT_WIDTH;
    }

    if (*height == -1) {
        *height = PSMOVE_TRACKER_DEFAULT_HEIGHT;
    }
}


CameraControl::CameraControl()
{
}


bool CameraControl::initialize(int cameraID)
{
    return initialize(cameraID, 0, 0, 0);
}


bool CameraControl::initialize(int cameraID, int width, int height, int framerate)
{
    this->cameraID = cameraID;

    if (framerate <= 0) {
        framerate = PSMOVE_TRACKER_DEFAULT_FPS;
    }

    char* video = psmove_util_get_env_string(PSMOVE_TRACKER_FILENAME_ENV);

    if (video) {
        psmove_DEBUG("Using '%s' as video input.\n", video);
        capture = std::make_unique<cv::VideoCapture>(video);
        psmove_free_mem(video);
    }
    else {
        capture = std::make_unique<cv::VideoCapture>(cameraID, cv::CAP_DSHOW);

        if (width <= 0 || height <= 0) {
            get_metrics(&width, &height);
        }

        capture->set(cv::CAP_PROP_FRAME_WIDTH, width);
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    this->width = width;
    this->height = height;

    return true;
}

int CameraControl::connectedCount()
{
    // Don't know how to get number of connected cameras through opencv...
    return -1;
}

cv::Mat CameraControl::queryFrame()
{
    cv::Mat result;

    *capture >> result;

    // undistort image
    if (!mapx.empty() && !mapy.empty()) {
        cv::remap(result, frame3chUndistort,
                mapx, mapy,
                cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS);
        result = frame3chUndistort;
    }


#if defined(CAMERA_CONTROL_DEBUG_CAPTURED_IMAGE)
    cv::imshow("camera input", result);
    cv::waitKey(1);
#endif

    return result;

}
void CameraControl::readCalibration(char* intrinsicsFile, char* distortionFile)
{
    cv::Mat intrinsic;
    cv::Mat distortion;

    {
        cv::FileStorage fs(intrinsicsFile, cv::FileStorage::READ);
        fs["intrinsic"] >> intrinsic;
    }
    {
        cv::FileStorage fs(distortionFile, cv::FileStorage::READ);
        fs["distortion"] >> distortion;
    }

    if (!intrinsic.empty() && !distortion.empty()) {
        if (frame3chUndistort.empty()) {
            frame3chUndistort = queryFrame().clone();
        }

        mapx = cv::Mat::zeros({ width, height }, CV_32FC1);
        mapy = cv::Mat::zeros({ width, height }, CV_32FC1);

        cv::initUndistortRectifyMap(intrinsic, distortion, cv::Mat::eye(3, 3, CV_8U), intrinsic, mapx.size(), mapx.type(), mapx, mapy);
        //cvInitUndistortMap(intrinsic, distortion, cc->mapx, cc->mapy);
    }
    else {
        fprintf(stderr, "Warning: No lens calibration files found.\n");
    }
}

void CameraControl::setParameters(int autoE, int autoG, int autoWB, int exposure, int gain, int wbRed, int wbGreen, int wbBlue, int contrast, int brightness, enum PSMove_Bool h_flip)
{
    int val;
    HKEY hKey;
    DWORD l = sizeof(DWORD);
    char* PATH = CL_DRIVER_REG_PATH;
    int err = RegOpenKeyEx(HKEY_CURRENT_USER, PATH, 0, KEY_ALL_ACCESS, &hKey);
    if (err != ERROR_SUCCESS) {
        printf("Error: %d Unable to open reg-key:  [HKCU] %s!", err, PATH);
        return;
    }
    val = autoE > 0;
    RegSetValueExA(hKey, "AutoAEC", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = autoG > 0;
    RegSetValueExA(hKey, "AutoAGC", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = autoWB > 0;
    RegSetValueExA(hKey, "AutoAWB", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = (int)((511 * exposure) / 0xFFFF);
    RegSetValueExA(hKey, "Exposure", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = (int)((79 * gain) / 0xFFFF);
    RegSetValueExA(hKey, "Gain", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = (int)((255 * wbRed) / 0xFFFF);
    RegSetValueExA(hKey, "WhiteBalanceR", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = (int)((255 * wbGreen) / 0xFFFF);
    RegSetValueExA(hKey, "WhiteBalanceG", 0, REG_DWORD, (CONST BYTE*) & val, l);
    val = (int)((255 * wbBlue) / 0xFFFF);
    RegSetValueExA(hKey, "WhiteBalanceB", 0, REG_DWORD, (CONST BYTE*) & val, l);

    int width, height;
    get_metrics(&width, &height);

    capture = std::make_unique<cv::VideoCapture>(cameraID);
    capture->set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture->set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

bool CameraControl::backupSystemSettings()
{
    return false;
}

bool CameraControl::restoreSystemSettings()
{
    return false;
}
