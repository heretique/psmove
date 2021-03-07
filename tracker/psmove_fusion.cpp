
/**
* PS Move API - An interface for the PS Move Motion Controller
* Copyright (c) 2012 Thomas Perl <m@thp.io>
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


#include "psmove_fusion.h"
#include "../psmove_private.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#define PSMOVE_FUSION_STEP_EPSILON (.0001)

static const float kMoveSphereSize = 46.f; // 46 mm
static const float kCameraFocalDistance = 3.7f; // 3.7 mm

PSMoveFusion::PSMoveFusion(PSMoveTracker* tracker, float z_near, float z_far)
{
    this->tracker = tracker;

    mNear = z_near;
    mFar = z_far;

    PSMoveTracker::Size size = tracker->getSize();

    width = (float)size.x;
    height = (float)size.y;

    projection = glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT,
            width, height, z_near, z_far);
    viewport = glm::vec4(0.f, 0.f, width, height);
}

const glm::mat4& PSMoveFusion::getProjectionMatrix() const
{
    return projection;
}

const glm::mat4& PSMoveFusion::getModelViewMatrix(PSMove* move)
{
    assert(move);

    float w, x, y, z;
    psmove_get_orientation(move, &w, &x, &y, &z);
    glm::quat quaternion(w, x, y, z);

    getPosition(move, x, y, z);

    modelview = glm::translate(glm::mat4(1.f),
            glm::vec3(x, y, z)) * glm::mat4_cast(quaternion);

    return modelview;
}

void PSMoveFusion::getPosition(PSMove* move, float& x, float& y, float& z)
{
    psmove_return_if_fail(move != NULL);

    PSMoveTracker::Position pos;
    tracker->getPosition(move, pos);

    float winX = (float)pos.x;
    float winY = height - (float)pos.y;
    float winZ = .5f; /* start value for binary search */

    float targetWidth = 2.0f * pos.radius;

    glm::vec3 obj;

    /* Binary search for the best distance based on the current projection */
    //float step = .025;
    //while (step > PSMOVE_FUSION_STEP_EPSILON) {
    //    /* Calculate center position of sphere */
    //    obj = glm::unProject(glm::vec3(winX, winY, winZ),
    //            glm::mat4(1.f), projection, viewport);

    //    /* Project left edge center of sphere */
    //    glm::vec3 left = glm::project(glm::vec3(obj.x - kMoveSphereSize, obj.y, obj.z),
    //            glm::mat4(1.f), projection, viewport);

    //    /* Project right edge center of sphere */
    //    glm::vec3 right = glm::project(glm::vec3(obj.x + kMoveSphereSize, obj.y, obj.z),
    //            glm::mat4(1.f), projection, viewport);

    //    float width = std::abs(right.x - left.x);
    //    if (width > targetWidth) {
    //        /* Too near */
    //        winZ += step;
    //    }
    //    else if (width < targetWidth) {
    //        /* Too far away */
    //        winZ -= step;
    //    }
    //    else {
    //        /* Perfect fit */
    //        break;
    //    }
    //    step *= .5;
    //}

    float dist = kMoveSphereSize * kCameraFocalDistance / (pos.radius * 0.00533124510f); // constant here was found measuring the size on screen of the ball at 50cm;
    dist *= 0.001f; // transform mm in m
    obj = glm::unProject(glm::vec3(winX, winY, winZ),
        glm::mat4(1.f), projection, viewport);


    x = obj.x;
    y = obj.y;
    z = dist;
}
