#pragma once

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



#include "psmove_config.h"
#include "psmove.h"
#include "psmove_tracker.h"

#include <glm/glm.hpp>


/* Field of view of the PS Eye */
#define PSEYE_FOV_BLUE_DOT 75
#define PSEYE_FOV_RED_DOT 56


class ADDAPI PSMoveFusion
{
public:
    /**
 * \brief Create a new PS Move Fusion object
 *
 * Creates and returns a new \ref PSMoveFusion object.
 *
 * \param tracker The \ref PSMoveTracker instance from which position
 *                information should be obtained
 * \param z_near The Z coordinate of the near clipping plane
 * \param z_far The Z coordinate of the far clipping plane
 *
 * \return A new \ref PSMoveFusion handle or \c NULL on error
 **/
    PSMoveFusion(PSMoveTracker* tracker, float z_near, float z_far);

    /**
 * \brief Get a pointer to the 4x4 projection matrix
 *
 * This function returns the OpenGL projection matrix for the camera
 * used. Usually the return value can be loaded directly into the
 * GL_PROJECTION matrix of the user application using glLoadMatrix().
 *
 * \param fusion A valid \ref PSMoveFusion handle
 *
 * \return A pointer to a 16-item (4x4) float array representing
 *         the current projection matrix. The return value is only
 *         valid as long as the \ref PSMoveFusion object exists, the
 *         caller MUST NOT free() the return value.
 **/
    const glm::mat4& getProjectionMatrix() const;

    /**
 * \brief Get a pointer to the 4x4 model-view matrix for a controller
 *
 * This function returns the OpenGL model-view matrix for one motion
 * controller. The coordinate system origin is at the center of the
 * sphere, aligned with the controller. The returned matrix therefore
 * describes both the position and orientation of the controller in 3D
 * space. Usually the return value can be loaded directly into the
 * GL_MODELVIEW matrix of the user application using glLoadMatrix().
 *
 * \param fusion A valid \ref PSMoveFusion handle
 * \param move A valid \ref PSMove handle
 *
 * \return A pointer to a 16-item (4x4) float array representing
 *         the modelview matrix for the controller. The return value
 *         is only valid as long as the \ref PSMoveFusion object
 *         exists, the caller MUST NOT free() the return value.
 **/
    const glm::mat4& getModelViewMatrix(PSMove* move);

    /**
 * \brief Get the 3D position of a controller
 *
 * This function returns the 3D position (relative to the camera)
 * of the motion controller, based on the current projection matrix.
 *
 * \param fusion A valid \ref PSMoveFusion handle
 * \param move A valid \ref PSMove handle
 * \param x A pointer to store the X part of the position vector
 * \param y A pointer to store the Y part of the position vector
 * \param z A pointer to store the Z part of the position vector
 **/
    void getPosition(PSMove* move, float& x, float& y, float& z);
private:
    PSMoveTracker* tracker;

    float width;
    float height;

    glm::mat4 projection;
    glm::mat4 modelview;
    glm::vec4 viewport;
};
