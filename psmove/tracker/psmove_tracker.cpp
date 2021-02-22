/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditt <benjamin.venditti@gmail.com>
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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "psmove_tracker.h"
#include "../psmove_private.h"
#include "../psmove_port.h"

#include "camera_control.h"
#include "camera_control_private.h"
#include "tracker_helpers.h"
#include "rttr/registration"

#ifdef __linux
#  include "platform/psmove_linuxsupport.h"
#endif

 /**
  * Experimentally-determined parameters for a PS Eye camera
  * in wide angle mode with a PS Move, color = (255, 0, 255)
  **/
static PSMoveTracker_DistanceParameters
pseye_distance_parameters = {
    /* height = */ 517.281f,
    /* center = */ 1.297338f,
    /* hwhm = */ 3.752844f,
    /* shape = */ 0.4762335f,
};


static const float kMoveBallRadius = 46; // mm


void PSMoveTracker::setAutoUpdateLeds(PSMove* move,
        bool auto_update_leds)
{
    psmove_return_if_fail(move != NULL);
    TrackedController* tc = findController(move);
    psmove_return_if_fail(tc != NULL);
    tc->auto_update_leds = auto_update_leds;
}


bool PSMoveTracker::getAutoUpdateLeds(PSMove* move)
{
    psmove_return_val_if_fail(move != NULL, PSMove_False);

    TrackedController* tc = findController(move);
    psmove_return_val_if_fail(tc != NULL, PSMove_False);
    return tc->auto_update_leds;
}


void PSMoveTracker::setDimming(float dimming)
{
    settings.dimmingFactor = dimming;
}

float  PSMoveTracker::getDimming()
{
    return settings.dimmingFactor;
}

void PSMoveTracker::setExposure(enum PSMoveTrackerExposure exposure)
{
    settings.exposureMode = exposure;

    float target_luminance = 0;
    switch (settings.exposureMode) {
    case Exposure_LOW:
    target_luminance = 0;
    break;
    case Exposure_MEDIUM:
    target_luminance = 25;
    break;
    case Exposure_HIGH:
    target_luminance = 50;
    break;
    default:
    psmove_DEBUG("Invalid exposure mode: %d\n", exposure);
    break;
    }

    settings.cameraExposure = adaptToLight(target_luminance);

    cc->setParameters(0, 0, 0, settings.cameraExposure,
            0, 0xffff, 0xffff, 0xffff, -1, -1, settings.cameraMirror);
}

enum PSMoveTrackerExposure PSMoveTracker::getExposure()
{
    return settings.exposureMode;
}

void PSMoveTracker::setMirror(bool enabled)
{
    settings.cameraMirror = enabled;
    cc->setParameters(0, 0, 0, settings.cameraExposure,
        0, 0xffff, 0xffff, 0xffff, -1, -1, settings.cameraMirror);
}

bool PSMoveTracker::getMirror()
{
    return settings.cameraMirror;
}

int PSMoveTracker::countConnected()
{
    assert(cc);
    return cc->connectedCount();
}

enum PSMoveTracker_Status PSMoveTracker::enable(PSMove* move)
{
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    // Switch off the controller and all others while enabling another one
    for (TrackedController& tc : controllers) {
        psmove_set_leds(tc.move, 0, 0, 0);
        psmove_update_leds(tc.move);
    }
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);

    int i;

    /* Preset colors - use them in ascending order if not used yet */
    PSMoveRGBValue preset_colors[] = {
        {0xFF, 0x00, 0xFF}, /* magenta */
        {0x00, 0xFF, 0x00}, /* green */
        {0xFF, 0xFF, 0x00}, /* yellow */
        {0xFF, 0x00, 0x00}, /* red */
        {0x00, 0x00, 0xFF}, /* blue */
    };

    for (i = 0; i < ARRAY_LENGTH(preset_colors); i++) {
        if (!colorIsUsed(preset_colors[i])) {
            return enableWithColorInternal(move, preset_colors[i]);
        }
    }

    /* No colors are available anymore */
    return Tracker_CALIBRATION_ERROR;
}

int PSMoveTracker::oldColorIsTracked(PSMove* move, PSMoveRGBValue rgb)
{
    cv::Scalar color;

    if (!lookupColor(rgb, color)) {
        return 0;
    }

    TrackedController* tc = findController(NULL);

    if (!tc) {
        return 0;
    }

    tc->move = move;
    tc->color = rgb;
    tc->auto_update_leds = PSMove_True;

    tc->eColor = tc->eFColor = color;
    tc->eColorHSV = tc->eFColorHSV = th_brg2hsv(tc->eFColor);

    /* Try to track the controller, give up after 100 iterations */
    int i;
    for (i = 0; i < 100; i++) {
        psmove_set_leds(move,
            (unsigned char)(rgb.r * settings.dimmingFactor),
            (unsigned char)(rgb.g * settings.dimmingFactor),
            (unsigned char)(rgb.b * settings.dimmingFactor));
        psmove_update_leds(move);
        psmove_port_sleep_ms(10); // wait 10ms - ok, since we're not blinking
        updateImage();
        update(move);

        if (tc->is_tracked) {
            // TODO: Verify quality criteria to avoid bogus tracking
            return 1;
        }
    }

    disable(move);
    return 0;
}

int PSMoveTracker::lookupColor(PSMoveRGBValue rgb, cv::Scalar& color)
{
    unsigned char current = color_mapping.next_slot - 1;
    unsigned char dimming = (unsigned char)(255 * settings.dimmingFactor);

    while (current != color_mapping.next_slot) {
        if (memcmp(&rgb, &(color_mapping.map[current].from),
            sizeof(PSMoveRGBValue)) == 0 &&
                color_mapping.map[current].dimming == dimming) {
            PSMoveRGBValue to = color_mapping.map[current].to;
            color.val[0] = to.r;
            color.val[1] = to.g;
            color.val[2] = to.b;
            return 1;
        }

        current--;
    }

    return 0;
}

void PSMoveTracker::rememberColor(PSMoveRGBValue rgb, cv::Scalar color)
{
    unsigned char dimming = (unsigned char)(255 * settings.dimmingFactor);

    PSMoveRGBValue to;
    to.r = (unsigned char)color.val[0];
    to.g = (unsigned char)color.val[1];
    to.b = (unsigned char)color.val[2];

    unsigned char slot = color_mapping.next_slot++;
    color_mapping.map[slot].from = rgb;
    color_mapping.map[slot].dimming = dimming;
    color_mapping.map[slot].to = to;

    char* filename = psmove_util_get_file_path(COLOR_MAPPING_DAT);
    FILE* fp = fopen(filename, "wb");
    if (fp) {
        if (!fwrite(&(color_mapping),
            sizeof(struct ColorMappingRingBuffer),
            1, fp)) {
            psmove_WARNING("Cannot write data to: %s\n", filename);
        }
        else {
            printf("color mappings saved.\n");
        }

        fclose(fp);
    }
    psmove_free_mem(filename);
}

enum PSMoveTracker_Status PSMoveTracker::enableWithColor(PSMove* move,
    unsigned char r, unsigned char g, unsigned char b)
{
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    PSMoveRGBValue rgb = { r, g, b };
    return enableWithColorInternal(move, rgb);
}

bool PSMoveTracker::blinkingCalibration(PSMove* move,
    PSMoveRGBValue rgb, cv::Scalar& color, cv::Scalar& hsv_color)
{
    char* color_str = psmove_util_get_env_string(PSMOVE_TRACKER_COLOR_ENV);
    if (color_str != NULL) {
        int r, g, b;
        if (sscanf(color_str, "%02x%02x%02x", &r, &g, &b) == 3) {
            printf("r: %d, g: %d, b: %d\n", r, g, b);
            color = cv::Scalar(r, g, b, 0);
            hsv_color = th_brg2hsv(color);
            psmove_free_mem(color_str);
            return PSMove_True;
        }
        else {
            psmove_WARNING("Cannot parse color: '%s'\n", color_str);
        }
        psmove_free_mem(color_str);
    }

    updateImage();

    // Switch off all other controllers for better measurements
    for (TrackedController& tc : controllers)
    {
        psmove_set_leds(tc.move, 0, 0, 0);
        psmove_update_leds(tc.move);
    }

    cv::Mat mask;
    cv::Mat images[BLINKS]; // array of images saved during calibration for estimation of sphere color
    cv::Mat diffs[BLINKS]; // array of masks saved during calibration for estimation of sphere color
    int i;
    for (i = 0; i < BLINKS; i++) {
        // allocate the images
        images[i] = cv::Mat(frame.size(), CV_MAKETYPE(frame.depth(), 3));
        diffs[i] = cv::Mat(frame.size(), CV_MAKETYPE(frame.depth(), 1));
    }
    double sizes[BLINKS]; // array of blob sizes saved during calibration for estimation of sphere color
    float sizeBest = 0;
    int contourBestIdx = -1;

    float dimming = 1.0;
    if (settings.dimmingFactor > 0) {
        dimming = settings.dimmingFactor;
    }
    for (;;) {
        for (i = 0; i < BLINKS; i++) {
            // create a diff image
            getDiff(move, rgb, images[i], diffs[i], settings.calibrationBlinkDelay, dimming);

            // threshold it to reduce image noise
            cv::threshold(diffs[i], diffs[i], settings.calibrationDiffT, 0xFF, cv::THRESH_BINARY);

            // use morphological operations to further remove noise
            cv::erode(diffs[i], diffs[i], kCalib);
            cv::dilate(diffs[i], diffs[i], kCalib);
        }

        // put the diff images together to get hopefully only one intersection region
        // the region at which the controllers sphere resides.
        mask = diffs[0];
        for (i = 1; i < BLINKS; i++) {
            cv::bitwise_and(mask, diffs[i], mask);
        }

        // find the biggest contour and repaint the blob where the sphere is expected
        biggestContour(diffs[0], storage, contourBestIdx, sizeBest);
        mask.setTo(TH_COLOR_BLACK);
        if (contourBestIdx >= 0) {
            cv::drawContours(mask, storage, contourBestIdx, TH_COLOR_WHITE, cv::FILLED);
            //cvDrawContours(mask, contourBest, TH_COLOR_WHITE, TH_COLOR_WHITE, -1, CV_FILLED, 8, cv::Point(0, 0));
        }
        storage.clear();

        // calculate the average color from the first image
        color = cv::mean(images[0], mask);
        hsv_color = th_brg2hsv(color);
        psmove_DEBUG("Dimming: %.2f, H: %.2f, S: %.2f, V: %.2f\n", dimming,
                hsv_color.val[0], hsv_color.val[1], hsv_color.val[2]);

        if (settings.dimmingFactor == 0.) {
            if (hsv_color.val[1] > 128) {
                settings.dimmingFactor = dimming;
                break;
            }
            else if (dimming < 0.01) {
                break;
            }
        }
        else {
            break;
        }

        dimming *= 0.3f;
    }

    int valid_countours = 0;

    // calculate upper & lower bounds for the color filter
    cv::Scalar min = th_scalar_sub(hsv_color, rHSV);
    cv::Scalar max = th_scalar_add(hsv_color, rHSV);

    cv::Point firstPosition;
    for (i = 0; i < BLINKS; i++) {
        // Convert to HSV, then apply the color range filter to the mask
        cv::cvtColor(images[i], images[i], cv::COLOR_BGR2HSV);
        cv::inRange(images[i], min, max, mask);

        // use morphological operations to further remove noise
        cv::erode(mask, mask, kCalib);
        cv::dilate(mask, mask, kCalib);

        // find the biggest contour in the image and save its location and size
        biggestContour(mask, storage, contourBestIdx, sizeBest);
        sizes[i] = 0;
        float dist = FLT_MAX;
        cv::Rect bBox;
        if (contourBestIdx >= 0) {
            bBox = cv::boundingRect(storage[contourBestIdx]);
            if (i == 0) {
                firstPosition = cv::Point(bBox.x, bBox.y);
            }
            dist = (float)sqrt(pow(firstPosition.x - bBox.x, 2) + pow(firstPosition.y - bBox.y, 2));
            sizes[i] = sizeBest;
        }

        // CHECK for errors (no contour, more than one contour, or contour too small)
        if (contourBestIdx < 0) {
            // No contour
        }
        else if (sizes[i] <= settings.calibrationMinSize) {
            // Too small
        }
        else if (dist >= settings.calibrationMaxDistance) {
            // Too far apart
        }
        else {
            // all checks passed, increase the number of valid contours
            valid_countours++;
        }
        storage.clear();

    }

    // CHECK if sphere was found in each BLINK image
    if (valid_countours < BLINKS) {
        return PSMove_False;
    }

    // CHECK if the size of the found contours are similar
    double sizeVariance, sizeAverage;
    th_stats(sizes, BLINKS, &sizeVariance, &sizeAverage);
    if (sqrt(sizeVariance) >= (sizeAverage / 100.0 * settings.calibrationSizeStd)) {
        return PSMove_False;
    }

    return PSMove_True;
}


enum PSMoveTracker_Status PSMoveTracker::enableWithColorInternal(PSMove* move, PSMoveRGBValue rgb)
{
    // check if the controller is already enabled!
    if (findController(move)) {
        return Tracker_CALIBRATED;
    }

    // cannot use the same color for two different controllers
    if (colorIsUsed(rgb)) {
        return Tracker_CALIBRATION_ERROR;
    }

    // try to track the controller with the old color, if it works we are done
    if (oldColorIsTracked(move, rgb)) {
        return Tracker_CALIBRATED;
    }

    cv::Scalar color;
    cv::Scalar hsv_color;
    if (blinkingCalibration(move, rgb, color, hsv_color)) {
        // Find the next free slot to use as TrackedController
        TrackedController* tc = findController(NULL);

        if (tc != NULL) {
            tc->move = move;
            tc->color = rgb;
            tc->auto_update_leds = PSMove_True;

            rememberColor(rgb, color);
            tc->eColor = tc->eFColor = color;
            tc->eColorHSV = tc->eFColorHSV = hsv_color;

            return Tracker_CALIBRATED;
        }
    }

    return Tracker_CALIBRATION_ERROR;
}

int PSMoveTracker::getColor(PSMove* move, Color& color)
{
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController* tc = findController(move);

    if (tc) {
        color.r = (unsigned char)(tc->color.r * settings.dimmingFactor);
        color.g = (unsigned char)(tc->color.g * settings.dimmingFactor);
        color.b = (unsigned char)(tc->color.b * settings.dimmingFactor);

        return 1;
    }

    return 0;
}

int PSMoveTracker::getCameraColor(PSMove* move, Color& color)
{
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController* tc = findController(move);

    if (tc) {
        color.r = (unsigned char)(tc->eColor.val[0]);
        color.g = (unsigned char)(tc->eColor.val[1]);
        color.b = (unsigned char)(tc->eColor.val[2]);

        return 1;
    }

    return 0;
}

int PSMoveTracker::setCameraColor(PSMove* move, Color color)
{
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController* tc = findController(move);

    if (tc) {
        /* Update the current color */
        tc->eColor.val[0] = color.r;
        tc->eColor.val[1] = color.g;
        tc->eColor.val[2] = color.b;
        tc->eColorHSV = th_brg2hsv(tc->eColor);

        /* Update the "first" color (to avoid re-adaption to old color) */
        tc->eFColor = tc->eColor;
        tc->eFColorHSV = tc->eColorHSV;

        return 1;
    }

    return 0;
}


void PSMoveTracker::disable(PSMove* move)
{
    psmove_return_if_fail(move != NULL);

    TrackedController* tc = findController(move);

    if (tc) {
        // Clear the tracked controller state - also sets move = NULL
        memset(tc, 0, sizeof(TrackedController));

        // XXX: If we "defrag" controllers to avoid holes with NULL
        // controllers, we can simplify psmove_tracker_find_controller() and
        // abort search at the first encounter of a NULL controller
    }
}

enum PSMoveTracker_Status PSMoveTracker::getStatus(PSMove* move)
{
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    TrackedController* tc = findController(move);

    if (tc) {
        if (tc->is_tracked) {
            return Tracker_TRACKING;
        }
        else {
            return Tracker_CALIBRATED;
        }
    }

    return Tracker_NOT_CALIBRATED;
}

cv::Mat PSMoveTracker::getFrame() {
    return frame;
}

cv::Mat PSMoveTracker::getImage()
{
    cv::Mat result;

    if (initialized) {
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
        result = frame_rgb;
    }

    return result;
}

void PSMoveTracker::updateImage() {

    frame = cc->queryFrame();

#if !defined(__linux)
    // PS3EyeDriver, CLEyeDriver, and v4l support flipping the camera image in
    // hardware (or in the driver). Manual flipping is only required if we are
    // using none of these ways to configure the camera and thus have no way
    // to enable flipping in hardware (or the driver).
    if (settings.cameraMirror) {
        // mirror image horizontally, i.e. flip left to right
        cv::flip(frame, frame, 1);
    }
#endif
}

int PSMoveTracker::updateController(TrackedController* tc)
{
    if (tc->move == NULL)
    {
        return 0;
    }

    float x, y;
    int i = 0;
    int sphere_found = 0;

    if (tc->auto_update_leds) {
        Color color;
        getColor(tc->move, color);
        psmove_set_leds(tc->move, color.r, color.g, color.b);
        psmove_update_leds(tc->move);
    }

    // calculate upper & lower bounds for the color filter
    cv::Scalar min = th_scalar_sub(tc->eColorHSV, rHSV);
    cv::Scalar max = th_scalar_add(tc->eColorHSV, rHSV);

    // this is the tracking algorithm
    for (;;) {
        // get pointers to data structures for the given ROI-Level
        cv::Mat roi_i = roiI[tc->roi_level];
        cv::Mat roi_m = roiM[tc->roi_level];

        // adjust the ROI, so that the blob is fully visible, but only if we have a reasonable FPS
        if (debug_fps > settings.roiAdjustFpsT) {
            // TODO: check for validity differently
            cv::Point nRoiCenter;
            if (centerRoiOnController(tc, nRoiCenter)) {
                setRoi(tc, nRoiCenter.x, nRoiCenter.y, roi_i.size().width, roi_i.size().height);
            }
        }

        // apply the ROI
        //cvSetImageROI(frame, cvRect(tc->roi_x, tc->roi_y, roi_i->width, roi_i->height));
        cv::cvtColor(frame(cv::Rect(tc->roi_x, tc->roi_y, roi_i.size().width, roi_i.size().height)), roi_i, cv::COLOR_BGR2HSV);

        // apply color filter
        cv::inRange(roi_i, min, max, roi_m);

        // find the biggest contour in the image
        float sizeBest = 0;
        int contourBestIdx = -1;
        biggestContour(roi_m, storage, contourBestIdx, sizeBest);

        if (contourBestIdx >= 0) {
            cv::Moments mu; // ImageMoments are use to calculate the center of mass of the blob
            cv::Rect br = cv::boundingRect(storage[contourBestIdx]);

            // restore the biggest contour
            roi_m.setTo(TH_COLOR_BLACK);
            cv::drawContours(roi_m, storage, contourBestIdx, TH_COLOR_WHITE, cv::FILLED);
            // calculate image-moments
            mu = cv::moments(roi_m);
            // calculate the mass center
            cv::Point p = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
            cv::Point oldMCenter = cv::Point((int)tc->mx, (int)tc->my);
            tc->mx = (float)p.x + tc->roi_x;
            tc->my = (float)p.y + tc->roi_y;
            cv::Point newMCenter = cv::Point((int)tc->mx, (int)tc->my);

            // remember the old radius and calculate the new x/y position and radius of the found contour
            float oldRadius = tc->r;
            // estimate x/y position and radius of the sphere
            estimateCircleFromContour(storage[contourBestIdx], x, y, tc->r);

            // apply radius-smoothing if enabled
            if (settings.trackerAdaptiveZ) {
                // calculate the difference between calculated radius and the smoothed radius of the past
                float rDiff = (float)fabs(tc->rs - tc->r);
                // calculate a adaptive smoothing factor
                // a big distance leads to no smoothing, a small one to strong smoothing
                float rf = MIN(rDiff / 4 + 0.15f, 1);

                // apply adaptive smoothing of the radius
                tc->rs = tc->rs * (1 - rf) + tc->r * rf;
                tc->r = tc->rs;
            }

            // apply x/y coordinate smoothing if enabled
            if (settings.trackerAdaptiveXY) {
                // a big distance between the old and new center of mass results in no smoothing
                // a little one to strong smoothing
                float diff = (float)sqrt(th_dist_squared(oldMCenter, newMCenter));
                float f = MIN(diff / 7 + 0.15f, 1);
                // apply adaptive smoothing
                tc->x = tc->x * (1 - f) + (x + tc->roi_x) * f;
                tc->y = tc->y * (1 - f) + (y + tc->roi_y) * f;
            }
            else {
                // do NOT apply adaptive smoothing
                tc->x = x + tc->roi_x;
                tc->y = y + tc->roi_y;
            }

            // calculate the quality of the tracking
            int pixelInBlob = cv::countNonZero(roi_m);
            float pixelInResult = (float)(tc->r * tc->r * M_PI);
            tc->q1 = 0;
            tc->q2 = FLT_MAX;
            tc->q3 = tc->r;

            // decrease TQ1 by half if below 20px (gives better results if controller is far away)
            if (pixelInBlob < 20) {
                tc->q1 /= 2;
            }

            // The quality checks are all performed on the radius of the blob
            // its old radius and size.
            tc->q1 = pixelInBlob / pixelInResult;

            // always check pixel-ratio and minimal size
            sphere_found = tc->q1 > settings.trackerQualityT1 && tc->q3 > settings.trackerQualityT3;

            // use the mass center if the quality is very good
            // TODO: make 0.85 as a CONST
            if (tc->q1 > 0.85) {
                tc->x = tc->mx;
                tc->y = tc->my;
            }
            // only perform check if we already found the sphere once
            if (oldRadius > 0 && tc->search_tile == 0) {
                tc->q2 = (float)fabs(oldRadius - tc->r) / (oldRadius + FLT_EPSILON);

                // additionally check for to big changes
                sphere_found = sphere_found && tc->q2 < settings.trackerQualityT2;
            }

            // only if the quality is okay update the future ROI
            if (sphere_found) {
                // use adaptive color detection
                // only if 	1) the sphere has been found
                // AND		2) the UPDATE_RATE has passed
                // AND		3) the tracking-quality is high;
                int do_color_adaption = 0;
                long now = psmove_util_get_ticks();
                if (settings.colorUpdateRate > 0 && (now - tc->last_color_update) > settings.colorUpdateRate * 1000)
                    do_color_adaption = 1;

                if (do_color_adaption &&
                    tc->q1 > settings.colorUpdateQualityT1 &&
                    tc->q2 < settings.colorUpdateQualityT2 &&
                    tc->q3 > settings.colorUpdateQualityT3)
                {
                    // calculate the new estimated color (adaptive color estimation)
                    cv::Scalar newColor = cv::mean(roi_i, roi_m);

                    tc->eColor = th_scalar_mul(th_scalar_add(tc->eColor, newColor), 0.5);

                    tc->eColorHSV = th_brg2hsv(tc->eColor);
                    tc->last_color_update = now;
                    // CHECK if the current estimate is too far away from its original estimation
                    if (hsvcolorDiff(tc) > settings.colorAdaptionQualityT) {
                        tc->eColor = tc->eFColor;
                        tc->eColorHSV = tc->eFColorHSV;
                        sphere_found = 0;
                    }
                }

                // update the future roi box
                br.width = MAX(br.width, br.height) * 3;
                br.height = br.width;
                // find a suitable ROI level
                for (i = 0; i < ROIS; i++) {
                    if (br.width > roiI[i].size().width && br.height > roiI[i].size().height)
                        break;

                    tc->roi_level = i;

                    // update easy accessors
                    roi_i = roiI[tc->roi_level];
                    roi_m = roiM[tc->roi_level];
                }

                // assure that the roi is within the target image
                setRoi(tc, (int)(tc->x - roi_i.size().width / 2), (int)(tc->y - roi_i.size().height / 2), roi_i.size().width, roi_i.size().height);
            }
        }
        storage.clear();
        //cvResetImageROI(frame);

        if (sphere_found) {
            //tc->search_tile = 0;
            // the sphere was found
            break;
        }
        else if (tc->roi_level > 0) {
            // the sphere was not found, increase the ROI and search again!
            tc->roi_x += roi_i.size().width / 2;
            tc->roi_y += roi_i.size().height / 2;

            tc->roi_level = tc->roi_level - 1;

            // update easy accessors
            roi_i = roiI[tc->roi_level];
            roi_m = roiM[tc->roi_level];

            // assure that the roi is within the target image
            setRoi(tc, tc->roi_x - roi_i.size().width / 2, tc->roi_y - roi_i.size().height / 2, roi_i.size().width, roi_i.size().height);
        }
        else {
            int rx;
            int ry;
            // the sphere could not be found til a reasonable roi-level

            rx = settings.searchTileWidth * (tc->search_tile %
                settings.searchTilesHorizontal);
            ry = settings.searchTileHeight * (int)(tc->search_tile /
                settings.searchTilesHorizontal);
            tc->search_tile = ((tc->search_tile + 2) %
                settings.searchTilesCount);

            tc->roi_level = 0;
            setRoi(tc, rx, ry, roiI[tc->roi_level].size().width, roiI[tc->roi_level].size().height);
            break;
        }
    }

    // remember if the sphere was found
    tc->is_tracked = sphere_found;
    return sphere_found;
}

int PSMoveTracker::update(PSMove* move)
{
    int spheres_found = 0;

    long started = psmove_util_get_ticks();

    for (TrackedController& tc : controllers) {
        if (move == NULL || tc.move == move) {
            spheres_found += updateController(&tc);
        }
    }

    duration = psmove_util_get_ticks() - started;

    return spheres_found;
}

int PSMoveTracker::getPosition(PSMove* move, Position& pos)
{
    TrackedController* tc = findController(move);

    if (tc) {
        pos.x = tc->x;
        pos.y = tc->y;
        pos.radius = tc->r;
        // TODO: return age of tracking values (if possible)
        return 1;
    }

    return 0;
}

PSMoveTracker::Size PSMoveTracker::getSize()
{
    return { frame.size().width, frame.size().height };
}

//void PSMoveTracker::r_free(PSMoveTracker *tracker)
//{
//    psmove_return_if_fail(tracker != NULL);
//
//    if (frame_rgb != NULL) {
//        cvReleaseImage(&frame_rgb);
//    }
//
//    camera_control_restore_system_settings(cc, cc_settings);
//
//    cvReleaseMemStorage(&storage);
//
//    int i;
//    for (i=0; i < ROIS; i++) {
//        cvReleaseImage(&roiM[i]);
//        cvReleaseImage(&roiI[i]);
//    }
//    cvReleaseStructuringElement(&kCalib);
//
//    camera_control_delete(cc);
//    free(tracker);
//}

// -------- Implementation: internal functions only
int PSMoveTracker::adaptToLight(float target_luminance)
{
    //float minimum_exposure = -20;
    //float maximum_exposure = -1;
    //float current_exposure = (maximum_exposure + minimum_exposure) / 2.0f;

    //if (target_luminance == 0) {
    //    return (int)minimum_exposure;
    //}

    //float step_size = (maximum_exposure - minimum_exposure) / 4.0f;

    //// Switch off the controllers' LEDs for proper environment measurements
    //for (TrackedController& tc : controllers) {
    //    psmove_set_leds(tc.move, 0, 0, 0);
    //    psmove_update_leds(tc.move);
    //}

    //int i;
    //for (i = 0; i < 7; i++) {
    //    cc->setParameters(0, 0, 0,
    //            (int)current_exposure, 0, 0xffff, 0xffff, 0xffff, -1, -1, settings.cameraMirror);

    //    cv::Mat frame;
    //    waitForFrame(&frame, 50);
    //    assert(!frame.empty());

    //    // calculate the average color and luminance (energy)
    //    float luminance = (float)th_color_avg(cv::mean(frame));

    //    psmove_DEBUG("Exposure: %.2f, Luminance: %.2f\n", current_exposure, luminance);
    //    if (fabsf(luminance - target_luminance) < 1) {
    //        break;
    //    }

    //    // Binary search for the best exposure setting
    //    if (luminance > target_luminance) {
    //        current_exposure -= step_size;
    //    }
    //    else {
    //        current_exposure += step_size;
    //    }

    //    step_size /= 2.;
    //}
    int current_exposure = settings.cameraExposure;
    return (int)current_exposure;
}


TrackedController* PSMoveTracker::findController(PSMove* move)
{
    for (TrackedController& controller : controllers) {
        if (controller.move == move) {
            return &controller;
        }

        // XXX: Assuming a "defragmented" list of controllers, we could stop our
        // search here if we arrive at a controller where move == NULL and admit
        // failure immediately. See the comment in psmove_tracker_disable() for
        // what we would have to do to always keep the list defragmented.
    }

    return NULL;
}

void PSMoveTracker::waitForFrame(cv::Mat* frame, int delay)
{
    int elapsed_time = 0;
    int step = 10;

    while (elapsed_time < delay) {
        psmove_port_sleep_ms(step);
        *frame = cc->queryFrame();
        elapsed_time += step;
    }
}

void PSMoveTracker::getDiff(PSMove* move,
        PSMoveRGBValue rgb, cv::Mat& on, cv::Mat& diff, int delay,
        float dimming_factor)
{
    // the time to wait for the controller to set the color up
    cv::Mat frame;
    // switch the LEDs ON and wait for the sphere to be fully lit
    rgb.r = (unsigned char)(rgb.r * dimming_factor);
    rgb.g = (unsigned char)(rgb.g * dimming_factor);
    rgb.b = (unsigned char)(rgb.b * dimming_factor);
    psmove_set_leds(move, rgb.r, rgb.g, rgb.b);
    psmove_update_leds(move);

    // take the first frame (sphere lit)
    waitForFrame(&frame, delay);
    on = frame.clone();
    //cvCopy(frame, on, NULL);

    // switch the LEDs OFF and wait for the sphere to be off
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);

    // take the second frame (sphere diff)
    waitForFrame(&frame, delay);

    // convert both to grayscale images
    cv::Mat grey1 = cv::Mat(diff.size(), CV_MAKETYPE(diff.depth(), diff.channels()));
    cv::Mat grey2 = cv::Mat(diff.size(), CV_MAKETYPE(diff.depth(), diff.channels()));
    cv::cvtColor(frame, grey1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(on, grey2, cv::COLOR_BGR2GRAY);

    // calculate the diff of to images and save it in "diff"
    cv::absdiff(grey1, grey2, diff);
}

void PSMoveTracker::setRoi(TrackedController* tc, int roi_x, int roi_y, int roi_width, int roi_height) {
    tc->roi_x = roi_x;
    tc->roi_y = roi_y;

    if (tc->roi_x < 0)
        tc->roi_x = 0;
    if (tc->roi_y < 0)
        tc->roi_y = 0;

    if (tc->roi_x + roi_width > frame.size().width)
        tc->roi_x = frame.size().width - roi_width;
    if (tc->roi_y + roi_height > frame.size().height)
        tc->roi_y = frame.size().height - roi_height;
}

void PSMoveTracker::annotate() {
    cv::Point p;

    //CvFont fontSmall = cvFont(0.8, 1);
    //CvFont fontNormal = cvFont(1, 1);

    char text[256];
    cv::Scalar c;
    cv::Scalar avgC;
    float avgLum = 0;
    int roi_w = 0;
    int roi_h = 0;

    // general statistics
    avgC = cv::mean(frame);
    avgLum = (float)th_color_avg(avgC);
    cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.size().width, 25), TH_COLOR_BLACK, cv::FILLED);
    sprintf(text, "fps:%.0f", debug_fps);
    cv::putText(frame, text, cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, TH_COLOR_WHITE);
    if (duration) {
        debug_fps = (0.85f * debug_fps + 0.15f *
                (1000.0f / (float)duration));
    }
    sprintf(text, "avg(lum):%.0f", avgLum);
    cv::putText(frame, text, cv::Point(255, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, TH_COLOR_WHITE);


    // draw all/one controller information to camera image
    for (TrackedController& tc : controllers) {
        if (tc.is_tracked) {
            // controller specific statistics
            p.x = (int)tc.x;
            p.y = (int)tc.y;
            roi_w = roiI[tc.roi_level].size().width;
            roi_h = roiI[tc.roi_level].size().height;
            c = tc.eColor;

            cv::rectangle(frame, cv::Point(tc.roi_x, tc.roi_y), cv::Point(tc.roi_x + roi_w, tc.roi_y + roi_h), TH_COLOR_WHITE, 3);
            cv::rectangle(frame, cv::Point(tc.roi_x, tc.roi_y), cv::Point(tc.roi_x + roi_w, tc.roi_y + roi_h), TH_COLOR_RED);
            cv::rectangle(frame, cv::Point(tc.roi_x, tc.roi_y - 45), cv::Point(tc.roi_x + roi_w, tc.roi_y - 5), TH_COLOR_BLACK, cv::FILLED);

            int vOff = 0;
            if (roi_h == frame.size().height)
                vOff = roi_h;
            sprintf(text, "RGB:%x,%x,%x", (int)c.val[2], (int)c.val[1], (int)c.val[0]);
            cv::putText(frame, text, cv::Point(tc.roi_x, tc.roi_y + vOff - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, c);

            sprintf(text, "ROI:%dx%d", roi_w, roi_h);
            cv::putText(frame, text, cv::Point(tc.roi_x, tc.roi_y + vOff - 15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, c);

            double distance = distanceFromRadius(tc.r);

            sprintf(text, "radius: %.2f", tc.r);
            cv::putText(frame, text, cv::Point(tc.roi_x, tc.roi_y + vOff - 35), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, c);
            sprintf(text, "dist: %.2f cm", distance);
            cv::putText(frame, text, cv::Point(tc.roi_x, tc.roi_y + vOff - 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, c);

            cv::circle(frame, p, (int)tc.r, TH_COLOR_WHITE);
        }
        else {
            roi_w = roiI[tc.roi_level].size().width;
            roi_h = roiI[tc.roi_level].size().height;
            cv::rectangle(frame, cv::Point(tc.roi_x, tc.roi_y), cv::Point(tc.roi_x + roi_w, tc.roi_y + roi_h), tc.eColor, 3);
        }
    }
}

float PSMoveTracker::hsvcolorDiff(TrackedController* tc) {
    float diff = 0;
    diff += (float)fabs(tc->eFColorHSV.val[0] - tc->eColorHSV.val[0]) * 1.0f; // diff of HUE is very important
    diff += (float)fabs(tc->eFColorHSV.val[1] - tc->eColorHSV.val[1]) * 0.5f; // saturation and value not so much
    diff += (float)fabs(tc->eFColorHSV.val[2] - tc->eColorHSV.val[2]) * 0.5f;
    return diff;
}

void PSMoveTracker::biggestContour(cv::Mat img, VectorStorage& contours, int& idx, float& size) {
    size = 0;
    idx = -1;
    cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //cvFindContours(img, stor, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int c = 0;
    for (const auto& contour : contours)
    {
        float a = cv::contourArea(contour);
        if (a > size)
        {
            size = a;
            idx = c;
        }
        c++;
    }

    //for (; contour; contour = contour->h_next) {
    //    float f = (float)cvContourArea(contour, CV_WHOLE_SEQ, 0);
    //    if (f > *resSize) {
    //        *resSize = f;
    //        *resContour = contour;
    //    }
    //}
}

void PSMoveTracker::estimateCircleFromContour(const std::vector<cv::Point>& cont, float& x, float& y, float& radius)
{
    int i, j;
    float d = 0;
    float cd = 0;
    cv::Point m1 = cv::Point(0, 0);
    cv::Point m2 = cv::Point(0, 0);
    int found = 0;

    int step = MAX(1, cont.size() / 20);

    // compare every two points of the contour (but not more than 20)
    // to find the most distant pair
    for (i = 0; i < cont.size(); i += step) {
        const cv::Point& p1 = cont[i];
        for (j = i + 1; j < cont.size(); j += step) {
            const cv::Point& p2 = cont[j];
            cd = (float)th_dist_squared(p1, p2);
            if (cd > d) {
                d = cd;
                m1 = p1;
                m2 = p2;
                found = 1;
            }
        }
    }
    // calculate center of that pair
    if (found) {
        x = 0.5f * (m1.x + m2.x);
        y = 0.5f * (m1.y + m2.y);
    }
    // calculate the radius
    radius = (float)sqrt(d) / 2;
}

int PSMoveTracker::centerRoiOnController(TrackedController* tc, cv::Point& center)
{
    psmove_return_val_if_fail(tc != NULL, 0);

    cv::Scalar min = th_scalar_sub(tc->eColorHSV, rHSV);
    cv::Scalar max = th_scalar_add(tc->eColorHSV, rHSV);

    cv::Mat roi_i = roiI[tc->roi_level];
    cv::Mat roi_m = roiM[tc->roi_level];

    // cut out the roi!
    //cvSetImageROI(frame, cvRect(tc->roi_x, tc->roi_y, roi_i->width, roi_i->height));
    cv::cvtColor(frame(cv::Rect(tc->roi_x, tc->roi_y, roi_i.size().width, roi_i.size().height)), roi_i, cv::COLOR_BGR2HSV);

    // apply color filter
    cv::inRange(roi_i, min, max, roi_m);

    float sizeBest = 0;
    int contourBestIdx = -1;
    biggestContour(roi_m, storage, contourBestIdx, sizeBest);
    if (contourBestIdx >= 0) {
        roi_m.setTo(TH_COLOR_BLACK);
        cv::drawContours(roi_m, storage, contourBestIdx, TH_COLOR_WHITE, cv::FILLED);
        // calculate image-moments to estimate the better ROI center
        cv::Moments mu = cv::moments(roi_m);

        center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
        center.x += tc->roi_x - roi_m.size().width / 2;
        center.y += tc->roi_y - roi_m.size().height / 2;
    }

    return (contourBestIdx >= 0);
}

float PSMoveTracker::distanceFromRadius(float radius)
{
    double height = distance_parameters.height;
    double center = distance_parameters.center;
    double hwhm = distance_parameters.hwhm;
    double shape = distance_parameters.shape;
    double x = (double)radius;

    /**
     * Pearson type VII distribution
     * http://fityk.nieto.pl/model.html
     **/
    double a = pow((x - center) / hwhm, 2.);
    double b = pow(2., 1. / shape) - 1.;
    double c = 1. + a * b;
    double distance = height / pow(c, shape);

    return (float)distance;
}

void  PSMoveTracker::setDistanceParameters(float height, float center, float hwhm, float shape)
{
    distance_parameters.height = height;
    distance_parameters.center = center;
    distance_parameters.hwhm = hwhm;
    distance_parameters.shape = shape;
}


int PSMoveTracker::colorIsUsed(PSMoveRGBValue color)
{
    for (const TrackedController& tc : controllers) {
        if (memcmp(&tc.color, &color, sizeof(PSMoveRGBValue)) == 0) {
            return 1;
        }
    }

    return 0;
}

PSMoveTrackerSettings::PSMoveTrackerSettings()
{
    cameraFrameWidth = 0;
    cameraFrameHeight = 0;
    cameraFrameRate = 0;
    cameraAutoGain = PSMove_False;
    cameraGain = 0;
    cameraAutoWhiteBalance = PSMove_False;
    cameraExposure = -10;
    cameraBrightness = 0;
    cameraMirror = PSMove_False;
    exposureMode = Exposure_LOW;
    calibrationBlinkDelay = 200;
    calibrationDiffT = 20;
    calibrationMinSize = 50;
    calibrationMaxDistance = 30;
    calibrationSizeStd = 10;
    colorMappingMaxAge = 2 * 60 * 60;
    dimmingFactor = 1.f;
    colorHueFilterRange = 20;
    colorSaturationFilterRange = 85;
    colorValueFilterRange = 85;
    trackerAdaptiveXY = 1;
    trackerAdaptiveZ = 1;
    colorAdaptionQualityT = 35.f;
    colorUpdateRate = 1.f;
    searchTileWidth = 0;
    searchTileHeight = 0;
    searchTilesHorizontal = 0;
    searchTilesCount = 0;
    roiAdjustFpsT = 160;
    trackerQualityT1 = 0.3f;
    trackerQualityT2 = 0.7f;
    trackerQualityT3 = 4.7f;
    colorUpdateQualityT1 = 0.8f;
    colorUpdateQualityT2 = 0.2f;
    colorUpdateQualityT3 = 6.f;
    intrinsicsFile = "intrinsics.xml";
    distortionFile = "distortion.xml";
}


PSMoveTracker::PSMoveTracker()
{

}


PSMoveTracker::~PSMoveTracker()
{

}

bool PSMoveTracker::initialize()
{
    return initialize(PSMoveTrackerSettings());
}


bool PSMoveTracker::initialize(const PSMoveTrackerSettings& settings)
{
    int camera = 0;

    int camera_env = psmove_util_get_env_int(PSMOVE_TRACKER_CAMERA_ENV);
    if (camera_env != -1) {
        camera = camera_env;
        psmove_DEBUG("Using camera %d (%s is set)\n", camera,
                PSMOVE_TRACKER_CAMERA_ENV);
    }

    return initialize(camera, settings);
}


bool PSMoveTracker::initialize(int camera)
{
    return initialize(camera, PSMoveTrackerSettings());
}


bool PSMoveTracker::initialize(int camera, const PSMoveTrackerSettings& iSettings)
{
    this->settings = iSettings;
    rHSV = cv::Scalar(
        settings.colorHueFilterRange,
        settings.colorSaturationFilterRange,
        settings.colorValueFilterRange, 0);
    //storage = cvCreateMemStorage(0);

    // start the video capture device for tracking

    // Returns NULL if no control found.
    // e.g. PS3EYE set during compile but not plugged in.
    cc = std::make_unique<CameraControl>();
    if (!cc->initialize(camera,
        settings.cameraFrameWidth, settings.cameraFrameHeight, settings.cameraFrameRate))
    {
        return false;
    }



    char* intrinsics_xml = psmove_util_get_file_path(settings.intrinsicsFile);
    char* distortion_xml = psmove_util_get_file_path(settings.distortionFile);
    cc->readCalibration(intrinsics_xml, distortion_xml);
    psmove_free_mem(intrinsics_xml);
    psmove_free_mem(distortion_xml);

    //cc_settings = camera_control_backup_system_settings(cc);

#if !defined(__APPLE__)
    // try to load color mapping data (not on Mac OS X for now, because the
    // automatic white balance means we get different colors every time)
    char* filename = psmove_util_get_file_path(COLOR_MAPPING_DAT);
    FILE* fp = NULL;
    time_t now = time(NULL);
    struct stat st;
    memset(&st, 0, sizeof(st));

    if (stat(filename, &st) == 0 && now != (time_t)-1) {
        if (st.st_mtime >= (now - settings.colorMappingMaxAge)) {
            fp = fopen(filename, "rb");
        }
        else {
            printf("%s is too old - not restoring colors.\n", filename);
        }
    }

    if (fp) {
        if (!fread(&(color_mapping),
            sizeof(struct ColorMappingRingBuffer),
            1, fp)) {
            psmove_WARNING("Cannot read data from: %s\n", filename);
        }
        else {
            printf("color mappings restored.\n");
        }

        fclose(fp);
    }
    psmove_free_mem(filename);
#endif

    // Default to the distance parameters for the PS Eye camera
    distance_parameters = pseye_distance_parameters;

    // set mirror
    setMirror(settings.cameraMirror);

    // use static exposure
    setExposure(settings.exposureMode);

    // just query a frame so that we know the camera works
    cv::Mat frame;
    while (frame.empty()) {
        frame = cc->queryFrame();
    }

    // prepare ROI data structures

    /* Define the size of the biggest ROI */
    int size = psmove_util_get_env_int(PSMOVE_TRACKER_ROI_SIZE_ENV);

    if (size == -1) {
        size = MIN(frame.size().width, frame.size().height) / 2;
    }
    else {
        psmove_DEBUG("Using ROI size: %d\n", size);
    }

    int w = size, h = size;

    // We need to grab an image from the camera to determine the frame size
    updateImage();

    settings.searchTileWidth = w;
    settings.searchTileHeight = h;

    settings.searchTilesHorizontal = (frame.size().width +
        settings.searchTileWidth - 1) / settings.searchTileWidth;
    int search_tiles_vertical = (frame.size().height +
        settings.searchTileHeight - 1) / settings.searchTileHeight;

    settings.searchTilesCount = settings.searchTilesHorizontal *
        search_tiles_vertical;

    if (settings.searchTilesCount % 2 == 0) {
        /**
         * search_tiles_count must be uneven, so that when picking every second
         * tile, we still "visit" every tile after two scans when we wrap:
         *
         *  ABA
         *  BAB
         *  ABA -> OK, first run = A, second run = B
         *
         *  ABAB
         *  ABAB -> NOT OK, first run = A, second run = A
         *
         * Incrementing the count will make the algorithm visit the lower right
         * item twice, but will then cause the second run to visit 'B's.
         *
         * We pick every second tile, so that we only need half the time to
         * sweep through the whole image (which usually means faster recovery).
         **/
        settings.searchTilesCount++;
    }


    int i;
    for (i = 0; i < ROIS; i++) {
        roiI[i] = cv::Mat(cv::Size(w, h), CV_MAKETYPE(frame.depth(), 3));
        roiM[i] = cv::Mat(cv::Size(w, h), CV_MAKETYPE(frame.depth(), 1));

        /* Smaller rois are always square, and 70% of the previous level */
        h = w = (int)(MIN(w, h) * 0.7f);
    }

    // prepare structure used for erode and dilate in calibration process
    int ks = 5; // Kernel Size
    int kc = (ks + 1) / 2; // Kernel Center
    kCalib = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ks, ks), cv::Point(kc, kc));


    for (int i = 0; i < PSMOVE_TRACKER_MAX_CONTROLLERS; i++)
    {
        controllers.emplace_back(TrackedController());
    }

    initialized = true;
    return true;
}

RTTR_REGISTRATION
{
    using namespace rttr;
registration::class_<PSMoveTrackerSettings>("PSMoveTrackerSettings")
.constructor<>()
.property_readonly("Camera Frame Width", &PSMoveTrackerSettings::cameraFrameWidth)
.property_readonly("Camera Frame Height", &PSMoveTrackerSettings::cameraFrameHeight)
.property_readonly("Camera Frame Rate", &PSMoveTrackerSettings::cameraFrameRate)
.property("Camera Auto Gain", &PSMoveTrackerSettings::cameraAutoGain)
.property("Camera Auto White Balance", &PSMoveTrackerSettings::cameraAutoWhiteBalance)
.property("Camera Exposure", &PSMoveTrackerSettings::cameraExposure)
.property("Camera Brightness", &PSMoveTrackerSettings::cameraBrightness)
.property("Camera Mirror", &PSMoveTrackerSettings::cameraMirror)
.property("Exposure Mode", &PSMoveTrackerSettings::exposureMode)
.enumeration<PSMoveTrackerExposure>("PSMoveTrackerExposure")
(
    value("ExposureLow", PSMoveTrackerExposure::Exposure_LOW),
    value("ExposureMedium", PSMoveTrackerExposure::Exposure_MEDIUM),
    value("ExposureHigh", PSMoveTrackerExposure::Exposure_HIGH),
    value("ExposureInvalid", PSMoveTrackerExposure::Exposure_INVALID)
)
.property("Calibration Blink Delay", &PSMoveTrackerSettings::calibrationBlinkDelay)
.property("Calibration Diff T", &PSMoveTrackerSettings::calibrationDiffT)
.property("Calibration Min Size", &PSMoveTrackerSettings::calibrationMinSize)
.property("Calibration Max Distance", &PSMoveTrackerSettings::calibrationMaxDistance)
.property("Calibration Size Std", &PSMoveTrackerSettings::calibrationSizeStd)
.property("Color Mapping Max Age", &PSMoveTrackerSettings::colorMappingMaxAge)
.property("Dimming Factor", &PSMoveTrackerSettings::dimmingFactor)
.property("Color Hue Filter Range", &PSMoveTrackerSettings::colorHueFilterRange)
.property("Color Saturation Filter Range", &PSMoveTrackerSettings::colorSaturationFilterRange)
.property("Color Value Filter Range", &PSMoveTrackerSettings::colorValueFilterRange)
.property("Tracker Adaptive XY", &PSMoveTrackerSettings::trackerAdaptiveXY)
.property("Tracker Adaptive Z", &PSMoveTrackerSettings::trackerAdaptiveZ)
.property("Color Adaptation Quality T", &PSMoveTrackerSettings::colorAdaptionQualityT)
.property("Color Update Rate", &PSMoveTrackerSettings::colorUpdateRate)
.property("Search Tile Width", &PSMoveTrackerSettings::searchTileWidth)
.property("Search Tile Height", &PSMoveTrackerSettings::searchTileHeight)
.property("Search Tile Horizontal", &PSMoveTrackerSettings::searchTilesHorizontal)
.property("Search Tiles Count", &PSMoveTrackerSettings::searchTilesCount)
.property("ROI Adjust FPS T", &PSMoveTrackerSettings::roiAdjustFpsT)
.property("Tracker Quality T1", &PSMoveTrackerSettings::trackerQualityT1)
.property("Tracker Quality T2", &PSMoveTrackerSettings::trackerQualityT2)
.property("Tracker Quality T3", &PSMoveTrackerSettings::trackerQualityT3)
.property("Color Update Quality T1", &PSMoveTrackerSettings::colorUpdateQualityT1)
.property("Color Update Quality T2", &PSMoveTrackerSettings::colorUpdateQualityT2)
.property("Color Update Quality T3", &PSMoveTrackerSettings::colorUpdateQualityT3);
}

