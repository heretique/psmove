#pragma once
/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Benjamin Venditti <benjamin.venditti@gmail.com>
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

#include "opencv2/core.hpp"

/* Defines the range of x/y values for the position getting, etc.. */
#define PSMOVE_TRACKER_DEFAULT_WIDTH 1920
#define PSMOVE_TRACKER_DEFAULT_HEIGHT 1080
#define PSMOVE_TRACKER_DEFAULT_FPS 60

#define ROIS 4                          // the number of levels of regions of interest (roi)
#define BLINKS 2                        // number of diff images to create during calibration

/* Maximum number of controllers that can be tracked at once */
#define PSMOVE_TRACKER_MAX_CONTROLLERS 5

/* Name of the environment variable used to pick a camera */
#define PSMOVE_TRACKER_CAMERA_ENV "PSMOVE_TRACKER_CAMERA"

/* Name of the environment variable used to choose a pre-recorded video */
#define PSMOVE_TRACKER_FILENAME_ENV "PSMOVE_TRACKER_FILENAME"

/* Name of the environment variable used to set the biggest ROI size */
#define PSMOVE_TRACKER_ROI_SIZE_ENV "PSMOVE_TRACKER_ROI_SIZE"

/* Name of the environment variable used to set a fixed tracker color */
#define PSMOVE_TRACKER_COLOR_ENV "PSMOVE_TRACKER_COLOR"

/* Name of the environment variables for the camera image size */
#define PSMOVE_TRACKER_WIDTH_ENV "PSMOVE_TRACKER_WIDTH"
#define PSMOVE_TRACKER_HEIGHT_ENV "PSMOVE_TRACKER_HEIGHT"

#define COLOR_MAPPING_RING_BUFFER_SIZE 256  /* Has to be 256, so that next_slot automatically wraps */
#define COLOR_MAPPING_DAT "colormapping.dat"

class CameraControl;

using VectorStorage = std::vector<std::vector<cv::Point>>;

/*! Status of the tracker */
enum PSMoveTracker_Status {
    Tracker_NOT_CALIBRATED, /*!< Controller not registered with tracker */
    Tracker_CALIBRATION_ERROR, /*!< Calibration failed (check lighting, visibility) */
    Tracker_CALIBRATED, /*!< Color calibration successful, not currently tracking */
    Tracker_TRACKING, /*!< Calibrated and successfully tracked in the camera */
};

/*! Exposure modes */
enum PSMoveTrackerExposure {
    Exposure_LOW, /*!< Very low exposure: Good tracking, no environment visible */
    Exposure_MEDIUM, /*!< Middle ground: Good tracking, environment visibile */
    Exposure_HIGH, /*!< High exposure: Fair tracking, but good environment */
    Exposure_INVALID, /*!< Invalid exposure value (for returning failures) */
};

/* A structure to retain the tracker settings. Typically these do not change after init & calib.*/
struct PSMoveTrackerSettings {
    PSMoveTrackerSettings();

    /* Camera Controls*/
    int cameraFrameWidth;                     /* [0=auto] */
    int cameraFrameHeight;                    /* [0=auto] */
    int cameraFrameRate;                      /* [0=auto] */
    bool cameraAutoGain;          /* [PSMove_False] */
    int cameraGain;                            /* [0] [0,0xFFFF] */
    bool cameraAutoWhiteBalance; /* [PSMove_False] */
    int cameraExposure;                        /* [(255 * 15) / 0xFFFF] [0,0xFFFF] */
    int cameraBrightness;                      /* [0] [0,0xFFFF] */
    bool cameraMirror;             /* [PSMove_True] mirror camera image horizontally */

    /* Settings for camera calibration process */
    enum PSMoveTrackerExposure exposureMode;  /* [Exposure_LOW] exposure mode for setting target luminance */
    int calibrationBlinkDelay;                /* [200] number of milliseconds to wait between a blink  */
    int calibrationDiffT;                     /* [20] during calibration, all grey values in the diff image below this value are set to black  */
    int calibrationMinSize;                   /* [50] minimum size of the estimated glowing sphere during calibration process (in pixel)  */
    int calibrationMaxDistance;               /* [30] maximum displacement of the separate found blobs  */
    int calibrationSizeStd;                   /* [10] maximum standard deviation (in %) of the glowing spheres found during calibration process  */
    int colorMappingMaxAge;                  /* [2*60*60] Only re-use color mappings "younger" than this time in seconds  */
    float dimmingFactor;                       /* [1.f] dimming factor used on LED RGB values  */
    
    /* Settings for OpenCV image processing for sphere detection */
    int colorHueFilterRange;                 /* [20] +- range of Hue window of the hsv-colorfilter  */
    int colorSaturationFilterRange;          /* [85] +- range of Sat window of the hsv-colorfilter  */
    int colorValueFilterRange;               /* [85] +- range of Value window of the hsv-colorfilter  */

    /* Settings for tracker algorithms */
    int trackerAdaptiveXY;                    /* [1] specifies to use a adaptive x/y smoothing  */
    int trackerAdaptiveZ;                     /* [1] specifies to use a adaptive z smoothing  */
    float colorAdaptionQualityT;             /* [35] maximal distance (calculated by 'psmove_tracker_hsvcolor_diff') between the first estimated color and the newly estimated  */
    float colorUpdateRate;                    /* [1] every x seconds adapt to the color, 0 means no adaption  */
    // size of "search" tiles when tracking is lost
    int searchTileWidth;                      /* [0=auto] width of a single tile */
    int searchTileHeight;                     /* height of a single tile */
    int searchTilesHorizontal;                /* number of search tiles per row */
    int searchTilesCount;                     /* number of search tiles */

    /* THP-specific tracker threshold checks */
    int roiAdjustFpsT;                       /* [160] the minimum fps to be reached, if a better roi-center adjusment is to be perfomred */
    // if tracker thresholds not met, sphere is deemed not to be found
    float trackerQualityT1;                   /* [0.3f] minimum ratio of number of pixels in blob vs pixel of estimated circle. */
    float trackerQualityT2;                   /* [0.7f] maximum allowed change of the radius in percent, compared to the last estimated radius */
    float trackerQualityT3;                   /* [4.7f] minimum radius  */
    // if color thresholds not met, color is not adapted
    float colorUpdateQualityT1;              /* [0.8] minimum ratio of number of pixels in blob vs pixel of estimated circle. */
    float colorUpdateQualityT2;              /* [0.2] maximum allowed change of the radius in percent, compared to the last estimated radius */
    float colorUpdateQualityT3;              /* [6.f] minimum radius */

	/* Camera calibration */
	const char* intrinsicsFile;					/* [intrinsics.xml] File to read intrinsics matrix from */
	const char* distortionFile;					/* [distortion.xml] File to read distortion coefficients from */
}; /*!< Structure for storing tracker settings */

/**
 * Parameters of the Pearson type VII distribution
 * Source: http://fityk.nieto.pl/model.html
 * Used for calculating the distance from the radius
 **/
struct PSMoveTracker_DistanceParameters {
    float height;
    float center;
    float hwhm;
    float shape;
};

struct ColorMapping {
    /* The RGB LED value */
    PSMoveRGBValue from;

    /* The dimming factor for which this mapping is valid */
    unsigned char dimming;

    /* The value of the controller in the camera */
    PSMoveRGBValue to;
};

/**
 * A ring buffer used for saving color mappings of previous sessions. There
 * is a pointer "next_slot" that will point to the next free slot. From there,
 * new values can be saved (forward) and old values can be searched (backward).
 **/
struct ColorMappingRingBuffer {
    ColorMapping map[COLOR_MAPPING_RING_BUFFER_SIZE];
    unsigned char next_slot;
};

struct TrackedController {
    /* Move controller, or NULL if free slot */
    PSMove* move;

    /* Assigned RGB color of the controller */
    PSMoveRGBValue color;

    cv::Scalar eFColor;			// first estimated color (BGR)
    cv::Scalar eFColorHSV;		// first estimated color (HSV)

    cv::Scalar eColor;			// estimated color (BGR)
    cv::Scalar eColorHSV; 		// estimated color (HSV)

    int roi_x, roi_y;			// x/y - Coordinates of the ROI
    int roi_level; 	 			// the current index for the level of ROI
    float mx, my;				// x/y - Coordinates of center of mass of the blob
    float x, y, r;				// x/y - Coordinates of the controllers sphere and its radius
    int search_tile; 			// current search quadrant when controller is not found (reset to 0 if found)
    float rs;					// a smoothed variant of the radius

    float q1, q2, q3; // Calculated quality criteria from the tracker

    int is_tracked;				// 1 if tracked 0 otherwise
    long last_color_update;	// the timestamp when the last color adaption has been performed
    bool auto_update_leds;
};

class PSMoveTracker
{
public:
    struct Color
    {
        unsigned char r{ 0 }, g{ 0 }, b{ 0 };
    };

    struct Position
    {
        float x{ 0 }, y{ 0 }, radius{ 0 };
    };

    struct Size
    {
        int x{ 0 }, y{ 0 };
    };

public:
    PSMoveTracker();
    ~PSMoveTracker();
    bool initialize();
    bool initialize(const PSMoveTrackerSettings& settings);
    bool initialize(int camera);
    bool initialize(int camera, const PSMoveTrackerSettings& settings);

    int countConnected();
    /**
 * \brief Configure if the LEDs of a controller should be auto-updated
 *
 * If auto-update is enabled (the default), the tracker will set and
 * update the LEDs of the controller automatically. If not, the user
 * must set the LEDs of the controller and update them regularly. In
 * that case, the user can use psmove_tracker_get_color() to determine
 * the color that the controller's LEDs have to be set to.
 *
 * \param tracker A valid \ref PSMoveTracker handle
 * \param move A valid \ref PSMove handle
 * \param auto_update_leds \ref PSMove_True to auto-update LEDs from
 *                         the tracker, \ref PSMove_False if the user
 *                         will take care of updating the LEDs
 **/
    void setAutoUpdateLeds(PSMove* move,
                bool auto_update_leds);

    /**
 * \brief Check if the LEDs of a controller are updated automatically
 *
 * This is the getter function for psmove_tracker_set_auto_update_leds().
 * See there for details on what auto-updating LEDs means.
 *
 * \param tracker A valid \ref PSMoveTracker handle
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_True if the controller's LEDs are set to be
 *         updated automatically, \ref PSMove_False otherwise
 **/
    bool
        getAutoUpdateLeds(PSMove* move);


    /**
 * \brief Set the LED dimming value for all controller
 *
 * Usually it's not necessary to call this function, as the dimming
 * is automatically determined when the first controller is enabled.
 *
 * \param tracker A valid \ref PSMoveTracker handle
 * \param dimming A value in the range from 0 (LEDs switched off) to
 *                1 (full LED intensity)
 **/
    void setDimming(float dimming);

    /**
     * \brief Get the LED dimming value for all controllers
     *
     * See psmove_tracker_set_dimming() for details.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     *
     * \return The dimming value for the LEDs
     **/
    float getDimming();

    /**
     * \brief Set the desired camera exposure mode
     *
     * This function sets the desired exposure mode. This should be
     * called before controllers are added to the tracker, so that the
     * dimming for the controllers can be determined for the specific
     * exposure setting.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param exposure One of the \ref PSMoveTracker_Exposure values
     **/
    void setExposure(enum PSMoveTrackerExposure exposure);

    /**
     * \brief Get the desired camera exposure mode
     *
     * See psmove_tracker_set_exposure() for details.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     *
     * \return One of the \ref PSMoveTracker_Exposure values
     **/
    enum PSMoveTrackerExposure getExposure();

    /**
     * \brief Enable or disable horizontal camera image mirroring
     *
     * Enables or disables horizontal mirroring of the camera image. The
     * mirroring setting will affect the X coordinates of the controller
     * positions tracked, as well as the output image. In addition, the
     * sensor fusion module will mirror the orientation information if
     * mirroring is set here. By default, mirroring is disabled.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param enabled \ref PSMove_True to mirror the image horizontally,
     *                \ref PSMove_False to leave the image as-is (default)
     **/
    void setMirror(bool enabled);

    /**
     * \brief Query the current camera image mirroring state
     *
     * See psmove_tracker_set_mirror() for details.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     *
     * \return \ref PSMove_True if mirroring is enabled,
     *         \ref PSMove_False if mirroring is disabled
     **/
    bool getMirror();

    /**
     * \brief Enable tracking of a motion controller
     *
     * Calling this function will register the controller with the
     * tracker, and start blinking calibration. The user should hold
     * the motion controller in front of the camera and wait for the
     * calibration to finish.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     *
     * \return \ref Tracker_CALIBRATED if calibration succeeded
     * \return \ref Tracker_CALIBRATION_ERROR if calibration failed
     **/
    enum PSMoveTracker_Status enable(PSMove* move);

    /**
     * \brief Enable tracking with a custom sphere color
     *
     * This function does basically the same as psmove_tracker_enable(),
     * but forces the sphere color to a pre-determined value.
     *
     * Using this function might give worse tracking results, because
     * the color might not be optimal for a given lighting condition.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     * \param r The red intensity of the desired color (0..255)
     * \param g The green intensity of the desired color (0..255)
     * \param b The blue intensity of the desired color (0..255)
     *
     * \return \ref Tracker_CALIBRATED if calibration succeeded
     * \return \ref Tracker_CALIBRATION_ERROR if calibration failed
     **/
    enum PSMoveTracker_Status enableWithColor(PSMove* move,
                unsigned char r, unsigned char g, unsigned char b);

    bool blinkingCalibration(PSMove* move, PSMoveRGBValue rgb, cv::Scalar& color, cv::Scalar& hsv_color);

    /**
     * \brief Disable tracking of a motion controller
     *
     * If the \ref PSMove instance was never enabled, this function
     * does nothing. Otherwise it removes the instance from the
     * tracker and stops tracking the controller.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     **/
    void disable(PSMove* move);

    /**
     * \brief Get the desired sphere color of a motion controller
     *
     * Get the sphere color of the controller as it is set using
     * psmove_update_leds(). This is not the color as the sphere
     * appears in the camera - for that, see
     * psmove_tracker_get_camera_color().
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A Valid \ref PSmove handle
     * \param r Pointer to store the red component of the color
     * \param g Pointer to store the green component of the color
     * \param g Pointer to store the blue component of the color
     *
     * \return Nonzero if the color was successfully returned, zero if
     *         if the controller is not enabled of calibration has not
     *         completed yet.
     **/
    int getColor(PSMove* move, Color& color);

    /**
     * \brief Get the sphere color of a controller in the camera image
     *
     * Get the sphere color of the controller as it currently
     * appears in the camera image. This is not the color that is
     * set using psmove_update_leds() - for that, see
     * psmove_tracker_get_color().
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A Valid \ref PSmove handle
     * \param r Pointer to store the red component of the color
     * \param g Pointer to store the green component of the color
     * \param g Pointer to store the blue component of the color
     *
     * \return Nonzero if the color was successfully returned, zero if
     *         if the controller is not enabled of calibration has not
     *         completed yet.
     **/
    int getCameraColor(PSMove* move, Color& color);

    /**
     * \brief Set the sphere color of a controller in the camera image
     *
     * This function should only be used in special situations - it is
     * usually not required to manually set the sphere color as it appears
     * in the camera image, as this color is determined at runtime during
     * blinking calibration. For some use cases, it might be useful to
     * set the color manually (e.g. when the user should be able to select
     * the color in the camera image after lighting changes).
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     * \param r The red component of the color (0..255)
     * \param g The green component of the color (0..255)
     * \param b The blue component of the color (0..255)
     *
     * \return Nonzero if the color was successfully set, zero if
     *         if the controller is not enabled of calibration has not
     *         completed yet.
     **/
    int setCameraColor(PSMove* move, Color color);

    /**
     * \brief Query the tracking status of a motion controller
     *
     * This function returns the current tracking status (or calibration
     * status if the controller is not calibrated yet) of a controller.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     *
     * \return One of the \ref PSMoveTracker_Status values
     **/
    enum PSMoveTracker_Status getStatus(PSMove* move);

    /**
     * \brief Retrieve the next image from the camera
     *
     * This function should be called once per main loop iteration (even
     * if multiple controllers are tracked), and will grab the next camera
     * image from the camera input device.
     *
     * This function must be called before psmove_tracker_update().
     *
     * \param tracker A valid \ref PSMoveTracker handle
     **/
    void updateImage();

    /**
     * \brief Process incoming data and update tracking information
     *
     * This function tracks one or all motion controllers in the camera
     * image, and updates tracking information such as position, radius
     * and camera color.
     *
     * This function must be called after psmove_tracker_update_image().
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle (to update a single controller)
     *             or \c NULL to update all enabled controllers at once
     *
     * \return Nonzero if tracking was successful, zero otherwise
     **/
    int update(PSMove* move);

    /**
     * \brief Draw debugging information onto the current camera image
     *
     * This function has to be called after psmove_tracker_update(), and
     * will annotate the camera image with sphere positions and other
     * information. The camera image will be modified in place, so no
     * call to psmove_tracker_update() should be carried out before the
     * next call to psmove_tracker_update_image().
     *
     * This function is used for demonstration and debugging purposes, in
     * production environments you usually do not want to use it.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     */
    void annotate();

    /**
     * \brief Get the current camera image as backend-specific pointer
     *
     * This function returns a pointer to the backend-specific camera
     * image. Right now, the only backend supported is OpenCV, so the
     * return value will always be a pointer to an IplImage structure.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     *
     * \return A pointer to the camera image (currently always an IplImage)
     *         - the caller MUST NOT modify or free the returned object.
     **/
    cv::Mat getFrame();

    /**
     * \brief Get the current camera image as 24-bit RGB data blob
     *
     * This function converts the internal camera image to a tightly-packed
     * 24-bit RGB image. The \ref PSMoveTrackerRGBImage structure is used
     * to return the image data pointer as well as the width and height of
     * the camera imaged. The size of the image data is 3 * width * height.
     *
     * The returned pixel data pointer points to tracker-internal data, and must
     * not be freed. The returned RGB data will only be valid as long as the
     * tracker exists.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     *
     * \return A \ref PSMoveTrackerRGBImage describing the RGB data and size.
     *         The RGB data is owned by the tracker, and must not be freed by
     *         the caller. The return value is valid only for the lifetime of
     *         the tracker object.
     **/
    cv::Mat getImage();

    /**
     * \brief Get the current position and radius of a tracked controller
     *
     * This function obtains the position and radius of a controller in the
     * camera image.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param move A valid \ref PSMove handle
     * \param x A pointer to store the X part of the position, or \c NULL
     * \param y A pointer to store the Y part of the position, or \c NULL
     * \param radius A pointer to store the controller radius, or \C NULL
     *
     * \return The age of the sensor reading in milliseconds, or -1 on error
     **/
    int getPosition(PSMove* move, Position& pos);

    /**
     * \brief Get the camera image size for the tracker
     *
     * This function can be used to obtain the real camera image size used
     * by the tracker. This is useful to convert the absolute position and
     * radius values to values relative to the image size if a camera is
     * used for which the size is now known. By default, the PS Eye camera
     * is used with an image size of 640x480.
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param width A pointer to store the width of the camera image
     * \param height A pointer to store the height of the camera image
     **/
    Size getSize();

    /**
     * \brief Calculate the physical distance (in cm) of the controller
     *
     * Given the radius of the controller in the image (in pixels), this function
     * calculates the physical distance of the controller from the camera (in cm).
     *
     * By default, this function's parameters are set up for the PS Eye camera in
     * wide angle view. You can set different parameters using the function
     * psmove_tracker_set_distance_parameters().
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param radius The radius for which the distance should be calculated, the
     *               radius is returned by psmove_tracker_get_position()
     **/
    float distanceFromRadius(float radius);

    /**
     * \brief Set the parameters for the distance mapping function
     *
     * This function sets the parameters for the Pearson VII distribution
     * function that's used to map radius values to distance values in
     * psmove_tracker_distance_from_radius(). By default, the parameters are
     * set up so that they work well for a PS Eye camera in wide angle mode.
     *
     * The function is defined as in: http://fityk.nieto.pl/model.html
     *
     * distance = height / ((1+((radius-center)/hwhm)^2 * (2^(1/shape)-1)) ^ shape)
     *
     * \param tracker A valid \ref PSMoveTracker handle
     * \param height The height parameter of the Pearson VII distribution
     * \param center The center parameter of the Pearson VII distribution
     * \param hwhm The hwhm parameter of the Pearson VII distribution
     * \param shape The shape parameter of the Pearson VII distribution
     **/
    void setDistanceParameters(float height, float center, float hwhm, float shape);

private:
    int colorIsUsed(PSMoveRGBValue color);
    /**
 * Adapts the cameras exposure to the current lighting conditions
 *
 * This function will find the most suitable exposure.
 *
 * tracker - A valid PSMoveTracker * instance
 * target_luminance - The target luminance value (higher = brighter)
 *
 * Returns: the most suitable exposure
 **/
    int adaptToLight(float target_luminance);

    /**
     * Find the TrackedController * for a given PSMove * instance
     *
     * if move == NULL, the next free slot will be returned
     *
     * Returns the TrackedController * instance, or NULL if not found
     **/
    TrackedController* findController(PSMove* move);


    /**
     * Wait for a given time for a frame from the tracker
     *
     * tracker - A valid PSMoveTracker * instance
     * frame - A pointer to an IplImage * to store the frame
     * delay - The delay to wait for the frame
     **/
    void waitForFrame(cv::Mat* frame, int delay);

    /**
     * This function switches the sphere of the given PSMove on to the given color and takes
     * a picture via the given capture. Then it switches it of and takes a picture again. A difference image
     * is calculated from these two images. It stores the image of the lit sphere and
     * of the diff-image in the passed parameter "on" and "diff". Before taking
     * a picture it waits for the specified delay (in microseconds).
     *
     * tracker - the tracker that contains the camera control
     * move    - the PSMove controller to use
     * rgb     - the RGB color to use to lit the sphere
     * on	   - the pre-allocated image to store the captured image when the sphere is lit
     * diff    - the pre-allocated image to store the calculated diff-image
     * delay   - the time to wait before taking a picture (in microseconds)
     **/
    void getDiff(PSMove* move,
                PSMoveRGBValue rgb, cv::Mat& on, cv::Mat& diff, int delay,
                float dimming_factor);

    /**
     * This function sets the rectangle of the ROI and assures that it is always within the bounds
     * of the camera image.
     *
     * tracker          - A valid PSMoveTracker * instance
     * tc         - The TrackableController containing the roi to check & fix
     * roi_x	  - the x-part of the coordinate of the roi
     * roi_y	  - the y-part of the coordinate of the roi
     * roi_width  - the width of the roi
     * roi_height - the height of the roi
     * cam_width  - the width of the camera image
     * cam_height - the height of the camera image
     **/
    void setRoi(TrackedController* tc, int roi_x, int roi_y, int roi_width, int roi_height);

    /**
     * This function is just the internal implementation of "psmove_tracker_update"
     */
    int updateController(TrackedController* tc);

    /*
     *  This finds the biggest contour within the given image.
     *
     *  img  		- (in) 	the binary image to search for contours
     *  stor 		- (out) a storage that can be used to save the result of this function
     *  resContour 	- (out) points to the biggest contour found within the image
     *  resSize 	- (out)	the size of that contour in px²
     */
    void biggestContour(cv::Mat img, VectorStorage& contours, int& idx, float& size);

    /*
     * This returns a subjective distance between the first estimated (during calibration process) color and the currently estimated color.
     * Subjective, because it takes the different color components not equally into account.
     *    Result calculates like: abs(c1.h-c2.h) + abs(c1.s-c2.s)*0.5 + abs(c1.v-c2.v)*0.5
     *
     * tc - The controller whose first/current color estimation distance should be calculated.
     *
     * Returns: a subjective distance
     */
    float hsvcolorDiff(TrackedController* tc);

    /*
     * This will estimate the position and the radius of the orb.
     * It will calculate the radius by finding the two most distant points
     * in the contour. And its by choosing the mid point of those two.
     *
     * cont 	- (in) 	The contour representing the orb.
     * x            - (out) The X coordinate of the center.
     * y            - (out) The Y coordinate of the center.
     * radius	- (out) The radius of the contour that is calculated here.
     */
    void estimateCircleFromContour(const std::vector<cv::Point>& cont, float& oX, float& oY, float& oRadius);

    /*
     * This function return a optimal ROI center point for a given Tracked controller.
     * On very fast movements, it may happen that the orb is visible in the ROI, but resides
     * at its border. This function will simply look for the biggest blob in the ROI and return a
     * point so that that blob would be in the center of the ROI.
     *
     * tc - (in) The controller whose ROI center point should be adjusted.
     * tracker  - (in) The PSMoveTracker to use.
     * center - (out) The better center point for the current ROI
     *
     * Returns: nonzero if a new point was found, zero otherwise
     */
    int centerRoiOnController(TrackedController* tc, cv::Point& center);

    enum PSMoveTracker_Status enableWithColorInternal(PSMove* move, PSMoveRGBValue color);

    /*
     * This function reads old calibration color values and tries to track the controller with that color.
     * if it works, the function returns 1, 0 otherwise.
     * Can help to speed up calibration process on application startup.
     *
     * tracker     - (in) A valid PSMoveTracker
     * move  - (in) A valid PSMove controller
     * rgb - (in) The color the PSMove controller's sphere will be lit.
     */

    int oldColorIsTracked(PSMove* move, PSMoveRGBValue rgb);

    /**
     * Lookup a camera-visible color value
     **/
    int lookupColor(PSMoveRGBValue rgb, cv::Scalar& color);

    /**
     * Remember a color value after calibration
     **/
    void rememberColor(PSMoveRGBValue rgb, cv::Scalar color);


private:
    bool initialized{ false };
    std::unique_ptr<CameraControl> cc;
    //CameraControlSystemSettings* cc_settings;

    PSMoveTrackerSettings settings;  // Camera and tracker algorithm settings. Generally do not change after startup & calibration.

    cv::Mat frame; // the current frame of the camera
    cv::Mat frame_rgb; // the frame as tightly packed RGB data
    cv::Mat roiI[ROIS]; // array of images for each level of roi (colored)
    cv::Mat roiM[ROIS]; // array of images for each level of roi (greyscale)
    cv::Mat kCalib; // kernel used for morphological operations during calibration
    cv::Scalar rHSV; // the range of the color filter

    // Parameters for psmove_tracker_distance_from_radius()
    PSMoveTracker_DistanceParameters distance_parameters;

    std::vector<TrackedController> controllers; // controller data

    ColorMappingRingBuffer color_mapping; // remembered color mappings
    VectorStorage storage; // use to store the result of cvFindContour and cvHughCircles
    long duration; // duration of tracking operation, in ms

    // internal variables (debug)
    float debug_fps; // the current FPS achieved by "psmove_tracker_update"
};