/**
 * @file ArucoDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/Conversions/CommonConversions.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::Conversions;

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

class ArucoDetector::Impl
{
public:
    /**
     * clear all internal buffers
     */
    void resetBuffers();
        
    // parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary; /**< container with detected markers data */
    double markerLength;  /**< marker length*/
    cv::Mat cameraMatrix; /**< camera calibration matrix*/
    cv::Mat distCoeff;    /**< camera distortion coefficients*/
    
    ArucoDetectorOutput out; /**< container with detected markers data */
    cv::Mat currentImg; /**< currently set image */
    double currentTime{-1.0}; /** time at which currentImg was set */
    std::vector<int> currentDetectedMarkerIds; /**< currently detected marker ids */
    std::vector<std::vector<cv::Point2f> > currentDetectedMarkerCorners; /**< currently detected marker corners */
    std::vector<cv::Vec3d> currentDetectedMarkersRotVecs, currentDetectedMarkersTransVecs; /**< currently detected marker rotation and translation vectors */
    
    bool initialized{false}; /**< flag to check if detector was initialized properly */
    
    cv::Mat R; /**< placeholder rotation matrix as cv Mat object*/
    Eigen::Matrix3d Reig; /**< placeholder rotation matrix as Eigen object*/
    Eigen::Vector3d teig; /**< placeholder translation vector as Eigen object*/
    Eigen::Matrix4d poseEig; /**< placeholder pose matrix as Eigen object*/
    
    /**
     * Utility map for choosing Aruco marker dictionary depending on user parameter
     */
    std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> availableDict{{"4X4_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50}, 
                                                                                         {"4X4_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100}, 
                                                                                         {"4X4_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_250},
                                                                                         {"4X4_1000",cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000},
                                                                                         {"5X5_50",  cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50},
                                                                                         {"5X5_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_100},
                                                                                         {"5X5_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_250},
                                                                                         {"5X5_1000",cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000},
                                                                                         {"6X6_50",  cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50},
                                                                                         {"6X6_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_100},
                                                                                         {"6X6_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250},
                                                                                         {"6X6_1000",cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000},
                                                                                         {"7X7_50",  cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50},
                                                                                         {"7X7_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_100},
                                                                                         {"7X7_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_250},
                                                                                         {"7X7_1000",cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000},
                                                                                         {"ARUCO_ORIGINAL",cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL} };
}; 

ArucoDetector::ArucoDetector() : 
               m_pimpl(std::make_unique<Impl>())
{
}

ArucoDetector::~ArucoDetector() = default;

bool ArucoDetector::initialize(std::weak_ptr<IParametersHandler> handler)
{
    std::string_view printPrefix = "[ArucoDetector::initialize] ";
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr <<  printPrefix << "The parameter handler has expired. Please check its scope." << std::endl;
        return false;
    }
    
    std::string dictName;
    if (!handle->getParameter("marker_dictionary", dictName))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \" marker_dictionary \" in the configuration file."
        << std::endl;
        return false;
    }
    
    if (m_pimpl->availableDict.find(dictName) == m_pimpl->availableDict.end())
    {
        std::cerr << printPrefix 
        << "Undefined value set to \" marker_dictionary \" in the configuration file. Please choose one among \n" 
        << "available options: \"4X4_50\", \"4X4_100\", \"4X4_250\", \"4X4_1000\", \n"
        << "\"5X5_50\", \"5X5_100\", \"5X5_250\", \"5X5_1000\", \n"
        << "\"6X6_50\", \"6X6_100\", \"6X6_250\", \"6X6_1000\", \n"
        << "\"7X7_50\", \"7X7_100\", \"7X7_250\", \"7X7_1000\", \n"
        << " \"ARUCO_ORIGINAL\", \n"
        << "options coherent with v3.4.0 in https://docs.opencv.org/3.4.1/d9/d6a/group__aruco.html"
        << std::endl;
        return false;
    }
    
    m_pimpl->dictionary = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
    
    if (!handle->getParameter("marker_length", m_pimpl->markerLength))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \" marker_length \" in the configuration file."
        << std::endl;
        return false;
    }
    
    std::vector<double> calibVec;
    if (!handle->getParameter("camera_matrix", 
                               GenericContainer::make_vector(calibVec, GenericContainer::VectorResizeMode::Resizable)))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \" camera_matrix \" in the configuration file."
        << std::endl;
        return false;
    }
    
    m_pimpl->cameraMatrix = cv::Mat(3, 3, CV_64F);
    std::memcpy(m_pimpl->cameraMatrix.data, calibVec.data(), calibVec.size()*sizeof(double));
    
    std::vector<double> distCoeffVec;
    if (!handle->getParameter("distortion_coefficients", 
                               GenericContainer::make_vector(distCoeffVec, GenericContainer::VectorResizeMode::Resizable)))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \" distortion_coefficients \" in the configuration file."
        << std::endl;
        return false;
    }
    
    m_pimpl->distCoeff = cv::Mat(5, 1, CV_64F);
    std::memcpy(m_pimpl->distCoeff.data, distCoeffVec.data(), distCoeffVec.size()*sizeof(double));
    
    m_pimpl->initialized = true;
    return true;
}

bool ArucoDetector::setImage(const cv::Mat& inputImg, double timeNow)
{
    std::string_view printPrefix = "[ArucoDetector::setImage] ";
    if (!m_pimpl->initialized)
    {
        std::cerr << printPrefix <<
        "Unable to set image. Please initialize the ArucoDetector before setting the image."
        << std::endl;
        return false;        
    }
            
    inputImg.copyTo(m_pimpl->currentImg);
    m_pimpl->currentTime = timeNow;
    
    return true;
}

bool ArucoDetector::advance()
{
    std::string_view printPrefix = "[ArucoDetector::advance] ";
    if (!m_pimpl->initialized)
    {
        std::cerr << printPrefix <<
        "Unable to run advance(). Please initialize the ArucoDetector first."
        << std::endl;
        return false;        
    }
    
    if (m_pimpl->currentImg.empty())
    {
        std::cerr << printPrefix <<
        "Unable to run advance(). Please set an image first."
        << std::endl;
        return false; 
    }
    
    m_pimpl->resetBuffers();
    std::vector<std::vector<cv::Point2f> > detectedmarkerCorners;
    cv::aruco::detectMarkers(m_pimpl->currentImg, 
                             m_pimpl->dictionary,
                             m_pimpl->currentDetectedMarkerCorners,
                             m_pimpl->currentDetectedMarkerIds);
    
    if (m_pimpl->currentDetectedMarkerIds.size() > 0)
    {        
        cv::aruco::estimatePoseSingleMarkers(m_pimpl->currentDetectedMarkerCorners,
                                             m_pimpl->markerLength,
                                             m_pimpl->cameraMatrix,
                                             m_pimpl->distCoeff,
                                             m_pimpl->currentDetectedMarkersRotVecs, 
                                             m_pimpl->currentDetectedMarkersTransVecs);
        
        for (std::size_t idx = 0; idx < m_pimpl->currentDetectedMarkerIds.size(); idx++)
        {                        
            cv::Rodrigues(m_pimpl->currentDetectedMarkersRotVecs[idx], m_pimpl->R);
            cv::cv2eigen(m_pimpl->R, m_pimpl->Reig);
            m_pimpl->teig <<  m_pimpl->currentDetectedMarkersTransVecs[idx](0),
                              m_pimpl->currentDetectedMarkersTransVecs[idx](1),
                              m_pimpl->currentDetectedMarkersTransVecs[idx](2);
                              
            m_pimpl->poseEig = toEigenPose(m_pimpl->Reig, m_pimpl->teig);
            ArucoMarkerData markerData{m_pimpl->currentDetectedMarkerIds[idx],
                                       m_pimpl->currentDetectedMarkerCorners[idx],
                                       m_pimpl->poseEig};
            m_pimpl->out.markers[m_pimpl->currentDetectedMarkerIds[idx]] = markerData;                                      
        }
        m_pimpl->out.timeNow = m_pimpl->currentTime;
    }
    

    return true;
}

const ArucoDetectorOutput& ArucoDetector::get() const
{
    return m_pimpl->out;
}

bool ArucoDetector::isValid() const
{
    if (!m_pimpl->initialized)
    {
        return false;
    }
    
    return true;
}

bool ArucoDetector::getDetectedMarkerData(const int& id, ArucoMarkerData& markerData)
{
    if (m_pimpl->out.markers.find(id) == m_pimpl->out.markers.end())
    {
        return false;        
    }
    
    markerData = m_pimpl->out.markers.at(id);
    return true;
}


bool ArucoDetector::getImgWithDetectedMarkers(cv::Mat& outputImg, 
                                              const bool& drawFrames, 
                                              const double& axisLengthForDrawing)
{
    std::size_t nrDetectedMarkers = m_pimpl->currentDetectedMarkerIds.size();
    if (m_pimpl->currentImg.empty() ||  nrDetectedMarkers <= 0)
    {
        return false;
    }

    m_pimpl->currentImg.copyTo(outputImg);
    cv::aruco::drawDetectedMarkers(outputImg, 
                                   m_pimpl->currentDetectedMarkerCorners, 
                                   m_pimpl->currentDetectedMarkerIds);
    
    if (drawFrames)
    {
        for (std::size_t idx = 0; idx < nrDetectedMarkers; idx++)
        {
            cv::aruco::drawAxis(outputImg,
                                m_pimpl->cameraMatrix,
                                m_pimpl->distCoeff,
                                m_pimpl->currentDetectedMarkersRotVecs[idx],
                                m_pimpl->currentDetectedMarkersTransVecs[idx],
                                axisLengthForDrawing);
        }
        
    }
    
    return true;    
}


void ArucoDetector::Impl::resetBuffers()
{     
    currentDetectedMarkerCorners.clear();
    currentDetectedMarkerIds.clear();
    currentDetectedMarkersRotVecs.clear();
    currentDetectedMarkersTransVecs.clear();
    out.markers.clear();
    out.timeNow = -1.0;
}


