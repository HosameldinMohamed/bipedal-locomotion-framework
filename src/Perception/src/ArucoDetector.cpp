/**
 * @file ArucoDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Perception;


class ArucoDetector::Impl
{
public:
    // parameters
    cv::Ptr<cv::aruco::Dictionary> dict; /**< container with detected markers data */
    double markerLength;  /**< marker length*/
    cv::Mat cameraMatrix; /**< camera calibration matrix*/
    cv::Mat distCoeff;    /**< distortion coefficients*/
    
    ArucoDetectorOutput out; /**< container with detected markers data */
    cv::Mat currentImg; /**< currently set image */
    double currentTime{-1.0}; /** time at which currentImg was set */
}; 

ArucoDetector::ArucoDetector() : 
               m_pimpl(std::make_unique<ArucoDetector::Impl>())
{
}

ArucoDetector::~ArucoDetector() = default;

bool ArucoDetector::initialize(std::weak_ptr<IParametersHandler> handlerWeak)
{
    
    return true;
}

bool ArucoDetector::advance()
{

    return true;
}

const ArucoDetectorOutput& ArucoDetector::get() const
{
    return m_pimpl->out;
}

bool ArucoDetector::isValid() const
{
    return true;
}


