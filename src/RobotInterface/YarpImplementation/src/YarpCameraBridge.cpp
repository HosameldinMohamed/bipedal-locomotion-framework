/**
 * @file YarpCameraBridge.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>

// YARP os
#include <yarp/os/Time.h>

// YARP sig
#include <yarp/sig/Image.h>

// YARP Camera Interfaces
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/cv/Cv.h>
#include <yarp/os/LogStream.h>
// std
#include <cmath>
#include <set>
#include <algorithm>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::ParametersHandler;


struct YarpCameraBridge::Impl
{
    template <typename PixelCode>
    using StampedYARPImage = std::pair< yarp::sig::ImageOf<PixelCode>, double>;
    
    using StampedYARPFlexImage = std::pair<yarp::sig::FlexImage, double>;
    
    std::unordered_map<std::string, yarp::dev::IFrameGrabberImage*> wholeBodyFrameGrabberInterface; /** < map of cameras attached through frame grabber interfaces */
    std::unordered_map<std::string, yarp::dev::IRGBDSensor*> wholeBodyRGBDInterface; /** < map of cameras attached through RGBD interfaces */

    
    CameraBridgeMetaData metaData; /**< struct holding meta data **/
    bool bridgeInitialized{false}; /**< flag set to true if the bridge is successfully initialized */
    bool driversAttached{false}; /**< flag set to true if the bridge is successfully attached to required device drivers */
    
    StampedYARPImage<yarp::sig::PixelFloat> depthImage;
    StampedYARPFlexImage flexImage;
    StampedYARPImage<yarp::sig::PixelRgb> rgbImage;
    
    /**
     * Check if sensor is available in the relevant sensor map
     */
    template <typename SensorType>
    bool checkSensor(const std::unordered_map<std::string, SensorType* >& sensorMap,
                     const std::string& sensorName)
    {
        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            return false;
        }
        return true;
    }

    /**
     * Check if sensor is available in the relevant sensor measurement map
     */
    template <typename YARPDataType>
    bool checkValidSensorMeasure(std::string_view logPrefix,
                                 const std::unordered_map<std::string, YARPDataType >& sensorMap,
                                 const std::string& sensorName)
    {
        if (!checkValid(logPrefix))
        {
            return false;
        }

        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            std::cerr << logPrefix << sensorName << " sensor unavailable in the measurement map" << std::endl;
            return false;
        }

        return true;
    }

    /**
     * Check if the bridge is successfully initialized and attached to required device drivers
     */
    bool checkValid(const std::string_view methodName)
    {
        if (!(bridgeInitialized && driversAttached))
        {
            std::cerr << methodName << " CameraBridge is not ready. Please initialize and set drivers list.";
            return false;
        }
        return true;
    }


    using SubConfigLoader = bool (YarpCameraBridge::Impl::*)(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>,
                                                             CameraBridgeMetaData&);
    /**
     * Checks is a stream is enabled in configuration and
     * loads the relevant stream group from configuration
     */
    bool subConfigLoader(const std::string& enableStreamString,
                         const std::string& streamGroupString,
                         const SubConfigLoader loader,
                         std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                         CameraBridgeMetaData& metaData,
                         bool& enableStreamFlag)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::subConfigLoader] ";

        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            std::cerr << logPrefix
                      << "The handler is not pointing to an already initialized memory."
                      << std ::endl;
            return false;
        }

        enableStreamFlag = false;
        if (ptr->getParameter(enableStreamString, enableStreamFlag) && enableStreamFlag)
        {
            auto groupHandler = ptr->getGroup(streamGroupString);
            if (!(this->*loader)(groupHandler, metaData))
            {
                std::cerr << logPrefix << streamGroupString
                          << " group could not be initialized from the configuration file."
                          << std::endl;

                return false;
            }
        }

        return true;
    }
    
    /**
     * Configure cameras meta data
     */
    bool configureCameras(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                          CameraBridgeMetaData& metaData)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::configureCameras] ";
        auto ptr = handler.lock();
        if (ptr == nullptr) { return false; }

        if (ptr->getParameter("rgb_cameras_list", metaData.sensorsList.rgbCamerasList))
        {
            metaData.bridgeOptions.isRGBCameraEnabled = true;

            std::vector<int> rgbWidth, rgbHeight;
            if (!ptr->getParameter("rgb_image_width", rgbWidth))
            {
                std::cerr << logPrefix << " Required parameter \"rgb_image_width\" not available in the configuration"
                      << std::endl;
                return false;
            }

            if (!ptr->getParameter("rgb_image_height", rgbHeight))
            {
                std::cerr << logPrefix << " Required parameter \"rgb_image_height\" not available in the configuration"
                      << std::endl;
                return false;
            }

            if ( (rgbWidth.size() != metaData.sensorsList.rgbCamerasList.size()) ||
                (rgbHeight.size() != metaData.sensorsList.rgbCamerasList.size()) )
            {
                std::cerr << logPrefix << " Parameters list size mismatch" << std::endl;
                return false;
            }

            for (int idx = 0; idx < rgbHeight.size(); idx++)
            {
                std::pair<int, int> imgDimensions(rgbWidth[idx], rgbHeight[idx]);
                auto cameraName{metaData.sensorsList.rgbCamerasList[idx]};
                metaData.bridgeOptions.rgbImgDimensions[cameraName] = imgDimensions;
            }
        }

        if (ptr->getParameter("rgbd_cameras_list", metaData.sensorsList.rgbdCamerasList))
        {
            metaData.bridgeOptions.isRGBDCameraEnabled = true;

            std::vector<int> rgbdCamWidth, rgbdCamHeight;
            if (!ptr->getParameter("rgbd_image_width", rgbdCamWidth))
            {
                std::cerr << logPrefix << " Required parameter \"rgbd_image_width\" not available in the configuration"
                          << std::endl;
                return false;
            }

            if (!ptr->getParameter("rgbd_image_height", rgbdCamHeight))
            {
                std::cerr << logPrefix << " Required parameter \"rgbd_image_height\" not available in the configuration"
                          << std::endl;
                return false;
            }

            if ( (rgbdCamWidth.size() != metaData.sensorsList.rgbdCamerasList.size()) ||
                (rgbdCamHeight.size() != metaData.sensorsList.rgbdCamerasList.size()) )
            {
                std::cerr << logPrefix << " Parameters list size mismatch" << std::endl;
                return false;
            }

            for (int idx = 0; idx < rgbdCamHeight.size(); idx++)
            {
                std::pair<int, int> imgDimensions(rgbdCamWidth[idx], rgbdCamHeight[idx]);
                auto cameraName{metaData.sensorsList.rgbdCamerasList[idx]};
                metaData.bridgeOptions.rgbdImgDimensions[cameraName] = imgDimensions;
            }
        }
        
        if (!metaData.bridgeOptions.isRGBCameraEnabled  && !metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            std::cerr << logPrefix << " None of the camera types configured. Cannot use Camera bridge."
                        << std::endl;
            return false;
        }
        
        return true;
    }
    
    
    /**
     * Attach cameras
     */
    template <typename CameraType>
    bool attachCamera(const yarp::dev::PolyDriverList& devList,
                      const std::string sensorName,
                      std::unordered_map<std::string, CameraType* >& sensorMap)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::attachCamera] ";
        for (int devIdx = 0; devIdx < devList.size(); devIdx++)
        {
            if (sensorName != devList[devIdx]->key)
            {
                continue;
            }

            CameraType* cameraInterface{nullptr};
            if (devList[devIdx]->poly->view(cameraInterface))
            {
                if (cameraInterface == nullptr)
                {
                    std::cerr << logPrefix << " Could not view interface." << std::endl;
                    return false;
                }
                sensorMap[devList[devIdx]->key] = cameraInterface;
            }
        }
        return true;
    }
    
    /**
     * Attach all cameras
     */
    bool attachAllCameras(const yarp::dev::PolyDriverList& devList)
    {
        std::string_view printPrefix{"[YarpCameraBridge::attachAllCameras] "};
        if (!metaData.bridgeOptions.isRGBCameraEnabled && 
            !metaData.bridgeOptions.isRGBDCameraEnabled)
        {
             // do nothing
            std::cerr << printPrefix << "No camera types enaable. Not attaching any cameras." << std::endl;
            return false;
        }
        
        if (metaData.bridgeOptions.isRGBCameraEnabled)
        {
            std::string_view interfaceType{"RGB Cameras"};
            if (!attachAllCamerasOfSpecificType(devList,
                                                metaData.sensorsList.rgbCamerasList,
                                                interfaceType,
                                                wholeBodyFrameGrabberInterface))
            {
                return false;
            }
        }

        if (metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            std::string_view interfaceTypeDepth{"RGBD Cameras"};
            if (!attachAllCamerasOfSpecificType(devList,
                                                metaData.sensorsList.rgbdCamerasList,
                                                interfaceTypeDepth,
                                                wholeBodyRGBDInterface))
            {
                return false;
            }
        }

        return true;
    }

    /**
     * Attach all cameras of specific type and resize image buffers
     */
    template <typename CameraType>
    bool attachAllCamerasOfSpecificType(const yarp::dev::PolyDriverList& devList,
                                        const std::vector<std::string>& camList,
                                        std::string_view interfaceType,
                                        std::unordered_map<std::string, CameraType* >& sensorMap)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::attachAllCamerasOfSpecificType] ";
        for (auto cam : camList)
        {
            if (!attachCamera(devList, cam, sensorMap))
            {
                return false;
            }
        }

        if (sensorMap.size() != camList.size())
        {
            std::cout << logPrefix << " could not attach all desired cameras of type " << interfaceType  << "." << std::endl;
            return false;
        }

        return true;
    }

    /**
     * Resize image buffers ImageOf<PixelType>
     */    
    template <typename PixelType>
    bool resizeImageBuffer(const std::string& cam,
                           const std::unordered_map<std::string, std::pair<std::size_t, std::size_t> >& imgDimensionsMap,
                           StampedYARPImage<PixelType>& img)
    {
        auto iter = imgDimensionsMap.find(cam);
        if (iter == imgDimensionsMap.end())
        {
            return false;
        }
        auto imgDim = iter->second;
        if (img.first.width() != imgDim.first || img.first.height() != imgDim.second)
        {
            img.first.resize(imgDim.first, imgDim.second);        
        }
        return true;        
    }
    
    /**
     * Resize image buffers - Flex Image
     */   
    bool resizeImageBuffer(const std::string& cam,
                           const std::unordered_map<std::string, std::pair<std::size_t, std::size_t> >& imgDimensionsMap,
                           StampedYARPFlexImage& img)
    {
        auto iter = imgDimensionsMap.find(cam);
        if (iter == imgDimensionsMap.end())
        {
            return false;
        }
        auto imgDim = iter->second;
        img.first.setPixelCode(VOCAB_PIXEL_RGB);
        if (img.first.width() != imgDim.first || img.first.height() != imgDim.second)
        {
            img.first.resize(imgDim.first, imgDim.second);        
        }
        return true;        
    }

    
    template<typename CameraType, typename PixelType>
    bool readCameraImage(const std::string& cameraName,
                         std::unordered_map<std::string, CameraType*>& interfaceMap,
                         StampedYARPImage<PixelType>& image)
    {
        bool ok{true};
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::readCameraImage] ";

        if (!checkSensor(interfaceMap, cameraName))
        {
            return false;
        }

        auto iter = interfaceMap.find(cameraName);
        auto interface = iter->second;

        // StampedYARPImage is defined as a pair of yarp::sig::image and double
        yarp::os::Stamp* txTimestamp{nullptr};
        if constexpr (std::is_same_v<CameraType, yarp::dev::IFrameGrabberImage>)
        {
            if constexpr (!std::is_same_v<PixelType, yarp::sig::PixelRgb>) // (imageType != "RGB")
            {
                std::cerr << logPrefix << " Frame Grabber " << cameraName << "handles only RGB image" << std::endl;
                return false;
            }
            ok = interface->getImage(image.first);
        }
        else if constexpr (std::is_same_v<CameraType, yarp::dev::IRGBDSensor>)
        {
            if constexpr (std::is_same_v<PixelType, yarp::sig::PixelFloat>)// (imageType == "DEPTH")
            {
                ok = interface->getDepthImage(image.first, txTimestamp);
            }
        }

        if (!ok)
        {
            std::cerr << logPrefix << " Unable to read from " << cameraName << ", use previous image" << std::endl;
            return false;
        }

        image.second = yarp::os::Time::now();
        return true;
    }       
    
    template<typename CameraType>
    bool readCameraImage(const std::string& cameraName,
                         std::unordered_map<std::string, CameraType*>& interfaceMap,
                         StampedYARPFlexImage& img)
    {
        bool ok{true};
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::readCameraImage] ";

        if (!checkSensor(interfaceMap, cameraName))
        {
            return false;
        }

        auto iter = interfaceMap.find(cameraName);
        auto interface = iter->second;
        
        yarp::os::Stamp* txTimestamp{nullptr};
        if constexpr (std::is_same_v<CameraType, yarp::dev::IRGBDSensor>)
        {
            ok = interface->getRgbImage(img.first, txTimestamp);                     
        }

        if (!ok)
        {
            std::cerr << logPrefix << " Unable to read from " << cameraName << ", use previous image" << std::endl;
            return false;
        }

        img.second = yarp::os::Time::now();
        return true;
    }
    
}; // end YarpCameraBridge::Impl

YarpCameraBridge::YarpCameraBridge() : m_pimpl(std::make_unique<Impl>())
{
}

YarpCameraBridge::~YarpCameraBridge() = default;

bool YarpCameraBridge::initialize(std::weak_ptr<IParametersHandler> handler)
{
    constexpr std::string_view logPrefix = "[YarpCameraBridge::initialize] ";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << logPrefix << "The handler is not pointing to an already initialized memory."
                  << std ::endl;
        return false;
    }

    bool ret{true};    
    bool useCameras{false};
    ret = m_pimpl->subConfigLoader("stream_cameras", "Cameras",
                                    &YarpCameraBridge::Impl::configureCameras,
                                    handler,
                                    m_pimpl->metaData,
                                    useCameras);
    if (!ret)
    {
        std::cout << logPrefix << " Skipping the configuration of Cameras. YarpCameraBridge will not stream relevant measures." << std::endl;
    }


    m_pimpl->bridgeInitialized = true;
    return true;
}

bool YarpCameraBridge::setDriversList(const yarp::dev::PolyDriverList& deviceDriversList)
{
    constexpr std::string_view logPrefix = "[YarpCameraBridge::setDriversList] ";

    if (!m_pimpl->bridgeInitialized)
    {
        std::cerr << logPrefix << "Please initialize YarpCameraBridge before calling setDriversList(...)."
                  << std ::endl;
        return false;
    }

    bool ret{true};
    ret = ret && m_pimpl->attachAllCameras(deviceDriversList);

    if (!ret)
    {
        std::cerr << logPrefix << "Failed to attach to one or more device drivers."
                  << std ::endl;
        return false;
    }
    m_pimpl->driversAttached = true;
    return true;
}

bool YarpCameraBridge::isValid() const
{
    return m_pimpl->checkValid("[YarpCameraBridge::isValid]");
}

const CameraBridgeMetaData& YarpCameraBridge::get() const { return m_pimpl->metaData; }


bool YarpCameraBridge::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    if (!m_pimpl->checkValid("[YarpCameraBridge::getRGBCamerasList]"))
    {
        return false;
    }
    rgbCamerasList = m_pimpl->metaData.sensorsList.rgbCamerasList;
    return true;
}

bool YarpCameraBridge::getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList)
{
    if (!m_pimpl->checkValid("[YarpCameraBridge::getRGBDCamerasList]"))
    {
        return false;
    }
    rgbdCamerasList = m_pimpl->metaData.sensorsList.rgbdCamerasList;
    return true;
}


bool YarpCameraBridge::getColorImage(const std::string& camName,
                                     cv::Mat& colorImg,
                                     double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValid("YarpCameraBridge::getColorImage "))
    {
        return false;
    }
    
    m_pimpl->rgbImage.first.zero();
    if (m_pimpl->resizeImageBuffer(camName, m_pimpl->metaData.bridgeOptions.rgbImgDimensions, m_pimpl->rgbImage))
    {
        if (!m_pimpl->readCameraImage(camName, m_pimpl->wholeBodyFrameGrabberInterface, m_pimpl->rgbImage))
        {
            std::cerr << "YarpCameraBridge::getColorImage " << camName << " could not read image." << std::endl;
            return false;
        }
        
        colorImg = yarp::cv::toCvMat(m_pimpl->rgbImage.first);
        receiveTimeInSeconds = &m_pimpl->rgbImage.second;
    }
    else 
    {
        m_pimpl->flexImage.first.zero();
        if (!m_pimpl->resizeImageBuffer(camName, m_pimpl->metaData.bridgeOptions.rgbdImgDimensions, m_pimpl->flexImage))
        {
            std::cerr << "YarpCameraBridge::getColorImage " << camName << " could not resize image buffers." << std::endl;
            return false;
        }
        
        if (!m_pimpl->readCameraImage(camName, m_pimpl->wholeBodyRGBDInterface, m_pimpl->flexImage))
        {
            std::cerr << "YarpCameraBridge::getColorImage " << camName << " could not read image." << std::endl;
            return false;
        }
        
        auto& yarpImage = m_pimpl->flexImage.first;
        colorImg = cv::Mat(yarpImage.height(), yarpImage.width(), yarp::cv::type_code<yarp::sig::PixelRgb>::value,
                           yarpImage.getRawImage(), yarpImage.getRowSize());
        receiveTimeInSeconds = &m_pimpl->flexImage.second;
    }
    
    if (colorImg.rows <= 0 || colorImg.cols <= 0)
    {
        std::cerr << "YarpCameraBridge::getDepthImage " << camName << " image with invalid size." << std::endl;
        return false;
    }
    
    return true;
}

bool YarpCameraBridge::getDepthImage(const std::string& camName,
                                     cv::Mat& depthImg,
                                     double* receiveTimeInSeconds)
{   
    if (!m_pimpl->checkValid("YarpCameraBridge::getDepthImage "))
    {
        return false;
    }
    
    m_pimpl->depthImage.first.zero();
    if (!m_pimpl->resizeImageBuffer(camName, m_pimpl->metaData.bridgeOptions.rgbdImgDimensions, m_pimpl->depthImage))
    {
        std::cerr << "YarpCameraBridge::getDepthImage " << camName << " could not resize image buffers." << std::endl;
        return false;
    }
    
    if (!m_pimpl->readCameraImage(camName, m_pimpl->wholeBodyRGBDInterface, m_pimpl->depthImage))
    {
        std::cerr << "YarpCameraBridge::getDepthImage " << camName << " could not read image." << std::endl;
        return false;
    }
          
    depthImg = yarp::cv::toCvMat(m_pimpl->depthImage.first);
    receiveTimeInSeconds = &m_pimpl->depthImage.second;
    
    if (depthImg.rows <= 0 || depthImg.cols <= 0)
    {
        std::cerr << "YarpCameraBridge::getDepthImage " << camName << " image with invalid size." << std::endl;
        return false;
    }
        
    return true;
}
