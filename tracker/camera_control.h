#pragma once
/**
 * PS Move API - An interface for the PS Move Motion Controller
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

#include "opencv2/core.hpp"
#include "psmove.h"
#include "../psmove_private.h"

#include <memory>

namespace cv {
    class VideoCapture;
}

class CameraControl
{
public:
    CameraControl();
    bool initialize(int cameraID);
    bool initialize(int cameraID, int cameraBackend, int width, int height, int framerate);
    int connectedCount();
    cv::Mat queryFrame();
    void readCalibration(char* intrinsicsFile, char* distortionFile);
    /**
 * Set the camera parameters used during capturing
 *
 * cc         - the camera control to modify
 * autoE      - value range [0-0xFFFF]
 * autoG      - value range [0-0xFFFF]
 * autoWB     - value range [0-0xFFFF]
 * exposure   - value range [0-0xFFFF]
 * gain       - value range [0-0xFFFF]
 * wbRed      - value range [0-0xFFFF]
 * wbGreen    - value range [0-0xFFFF]
 * wbBlue     - value range [0-0xFFFF]
 * contrast   - value range [0-0xFFFF]
 * brightness - value range [0-0xFFFF]
 **/
    void setParameters(int autoE, int autoG, int autoWB,
        int exposure, int gain,
        int wbRed, int wbGreen, int wbBlue,
        int contrast, int brightness, bool h_flip);

    bool backupSystemSettings();
    bool restoreSystemSettings();

private:
    int cameraID;
    cv::Mat frame3chUndistort;

    std::unique_ptr<cv::VideoCapture> capture;

    cv::Mat mapx;
    cv::Mat mapy;

    int width;
    int height;
};