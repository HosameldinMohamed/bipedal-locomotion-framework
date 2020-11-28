/**
 * @file ArucoDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H
#define BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>

#include <functional>
#include <memory>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Perception
{

/**
 * Aruco marker identifiers
 */    
struct ArucoMarkerData
{
    /**
     * Marker ID
     */
    int id;
    
    /**
     * Marker corners in camera coordinates
     * in the order
     * - top left
     * - top right
     * - bottom right
     * - bottom left
     */
    std::vector<cv::Point2f> corners;
    
    /**
     * Pose of the marker in camera frame
     * cam_H_marker
     */
    Eigen::Matrix4d pose;
};


/**
 * Aruco detector output
 */
struct ArucoDetectorOutput
{
    std::unordered_map<int, ArucoMarkerData> markers;
    double time_now{-1.0};
};

class ArucoDetector : public System::Advanceable<ArucoDetectorOutput>
{
public:
    ArucoDetector();
    ~ArucoDetector();
    
    /**
     * Initialize the detector
     * @note The following parameter are required by the filter:
     * - "marker_dictionary" OpenCV predefined aruco marker dictionary
     * - "marker_dictionary" marker length in m
     * - "camera_matrix" 9d vector representing the camera calbration matrix in row major order
     * - "distortion_coefficients" 5d vector containing camera distortion coefficients
     * @param[in] handlerWeak weak pointer to a ParametersHandler::IParametersHandler interface
     * @tparameter Derived particular implementation of the IParameterHandler
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handlerWeak);
        
    /**
     * Set image for which markers need to be detecte
     * @param[in] inputImg image as OpenCV mat
     * @param[in] time_now current time in chosen time units
     * @return True in case of success, false otherwise
     */
    bool setImage(const cv::Mat& inputImg, double time_now);

    /**
     * Compute one step of the detector
     * @return True in case of success, false otherwise
     */
    bool advance() final;

    /**
     * Get the detected markers' data from the current step
     * @return A struct containing a map container of detected markers.
     */
    const ArucoDetectorOutput& get() const final;
    
    /**
     * Get the detected marker data
     * @param[in] id marker id
     * @param[in] markerData detected marker identifiers data
     * @return True in case of success, false if marker was not detected
     */
    bool getArucoMarkerData(const int& id, ArucoMarkerData& markerData);
    
    /**
     * Get the image with drawn detected markers
     * @param[in] outputImg image with detected markers drawn on it
     * @param[in] drawFrames draw also estimated marker poses, set to false by default
     * @return True in case of success, false if no marker was not detected
     */
    bool getImgWithDetectedMarkers(cv::Mat& outputImg, bool drawFrames = false);

    /**
     * Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;
    
private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl{nullptr};
};

} // namespace Perception
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H

