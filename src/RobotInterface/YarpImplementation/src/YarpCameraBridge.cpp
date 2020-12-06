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
    using StampedYARPImage = std::pair<yarp::sig::ImageOf<PixelCode>, double>;
    
    std::unordered_map<std::string, yarp::dev::IFrameGrabberImage*> wholeBodyFrameGrabberInterface; /** < map of cameras attached through frame grabber interfaces */
    std::unordered_map<std::string, yarp::dev::IRGBDSensor*> wholeBodyRGBDInterface; /** < map of cameras attached through RGBD interfaces */
    std::unordered_map<std::string, StampedYARPImage<yarp::sig::PixelRgb> > wholeBodyCameraRGBImages; /** < map holding images **/
    std::unordered_map<std::string, StampedYARPImage<yarp::sig::PixelFloat> > wholeBodyCameraDepthImages; /** < map holding images **/
    
    std::vector<std::string> failedSensorReads;
    CameraBridgeMetaData metaData; /**< struct holding meta data **/
    bool bridgeInitialized{false}; /**< flag set to true if the bridge is successfully initialized */
    bool driversAttached{false}; /**< flag set to true if the bridge is successfully attached to required device drivers */
    bool checkForNAN{false}; /**< flag to enable binary search for NANs in the incoming measurement buffers */

    
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
                                                metaData.bridgeOptions.rgbImgDimensions,
                                                interfaceType,
                                                wholeBodyFrameGrabberInterface))
            {
                return false;
            }
            std::string imgType{"RGB"};
            // allocate rgb images of rgb cameras
            if (!resizeImageBuffers(imgType, 
                                    metaData.sensorsList.rgbCamerasList,
                                    metaData.bridgeOptions.rgbImgDimensions,
                                    wholeBodyCameraRGBImages))
            {
                std::cout << printPrefix << " Failed to allocate Depth images of type RGB camera type." << std::endl;
                return false;
            }
        }

        if (metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            std::string_view interfaceTypeDepth{"RGBD Cameras"};
            if (!attachAllCamerasOfSpecificType(devList,
                                                metaData.sensorsList.rgbdCamerasList,
                                                metaData.bridgeOptions.rgbdImgDimensions,
                                                interfaceTypeDepth,
                                                wholeBodyRGBDInterface))
            {
                return false;
            }
            
            // allocate depth images of rgbd cameras
            std::string imgType{"DEPTH"};
            if (!resizeImageBuffers(imgType,
                                    metaData.sensorsList.rgbdCamerasList,
                                    metaData.bridgeOptions.rgbdImgDimensions,
                                    wholeBodyCameraDepthImages))
            {
                std::cout << printPrefix << " Failed to allocate Depth images of type RGBD camera type." << std::endl;
                return false;
            }
            
            // resize also rgb images of RGBD cameras
            imgType = "RGB";
            if (!resizeImageBuffers(imgType,
                                    metaData.sensorsList.rgbdCamerasList,
                                    metaData.bridgeOptions.rgbdImgDimensions,
                                    wholeBodyCameraRGBImages))
            {
                std::cout << printPrefix << " Failed to allocate RGB images of type RGBD camera type." << std::endl;
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
                                        const std::unordered_map<std::string, std::pair<std::size_t, std::size_t> >& imgDimensionsMap,
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
     * Resize image buffers
     */
    template <typename PixelType>
    bool resizeImageBuffers(const std::string imgType,
                            const std::vector<std::string>& camList,
                            const std::unordered_map<std::string, std::pair<std::size_t, std::size_t> >& imgDimensionsMap,
                            std::unordered_map<std::string, StampedYARPImage<PixelType> >& imgBuffersMap)
    {
        for (const auto& cam : camList)
        {
            auto iter = imgDimensionsMap.find(cam);
            if (iter == imgDimensionsMap.end())
            {
                return false;
            }
            auto imgDim = iter->second;

            imgBuffersMap[cam].first.resize(imgDim.first, imgDim.second);
        }
        return true;        
    }
                        

    template<typename CameraType, typename PixelType>
    bool readCameraImage(const std::string& cameraName,
                         const std::string& imageType,
                         std::unordered_map<std::string, CameraType*>& interfaceMap,
                         std::unordered_map<std::string, StampedYARPImage<PixelType> >& imageMap)
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
            ok = interface->getImage(imageMap.at(cameraName).first);
        }
        else if constexpr (std::is_same_v<CameraType, yarp::dev::IRGBDSensor>)
        {
            if constexpr (std::is_same_v<PixelType, yarp::sig::PixelRgb>)//(imageType == "RGB")
            {
                // TO CHECK IMPLEMENTATION
                yarp::sig::FlexImage img;  
                img.setPixelCode(VOCAB_PIXEL_RGB);
//                 ok = interface->getRgbImage(img, txTimestamp);
            }
            else if constexpr (std::is_same_v<PixelType, yarp::sig::PixelFloat>)// (imageType == "DEPTH")
            {
                ok = interface->getDepthImage(imageMap.at(cameraName).first, txTimestamp);
            }
        }

        if (!ok)
        {
            std::cerr << logPrefix << " Unable to read from " << cameraName << ", use previous image" << std::endl;
            return false;
        }

        imageMap.at(cameraName).second = yarp::os::Time::now();
        return true;
    }
    
    bool readAllFrameGrabberCameras(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isRGBCameraEnabled)
        {
            // do nothing
            return true;
        }

        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::readAllFrameGrabberCameras] ";
        bool allRGBCamerasReadCorrectly{true};
        failedSensorReads.clear();
        for( auto const& camera : wholeBodyFrameGrabberInterface )
        {
            std::string imageType{"RGB"};
            const auto& cameraName = camera.first;
            bool ok = readCameraImage(cameraName,
                                      imageType,
                                      wholeBodyFrameGrabberInterface,
                                      wholeBodyCameraRGBImages);
            if (!ok)
            {
                std::cerr << logPrefix << " Read RGB image failed for " << cameraName << std::endl;
                failedSensorReads.emplace_back(cameraName);
            }
            allRGBCamerasReadCorrectly = ok && allRGBCamerasReadCorrectly;
        }

        return allRGBCamerasReadCorrectly;
    }

    bool readAllRGBDCameras(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            // do nothing
            return true;
        }

        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::readAllRGBDCameras] ";
        bool allRGBDCamerasReadCorrectly{true};
        failedSensorReads.clear();
        for( auto const& camera : wholeBodyRGBDInterface )
        {
            std::string imageType{"RGB"};
            bool ok{true};
            const auto& cameraName = camera.first;
            ok = readCameraImage(cameraName,
                                 imageType,
                                 wholeBodyRGBDInterface,
                                 wholeBodyCameraRGBImages) && ok;

            imageType = "DEPTH";
            ok = readCameraImage(cameraName,
                                 imageType,
                                 wholeBodyRGBDInterface,
                                 wholeBodyCameraDepthImages) && ok;

            if (!ok)
            {
                std::cerr << logPrefix << " Read RGB/Depth image failed for " << cameraName << std::endl;
                failedSensorReads.emplace_back(cameraName);
            }

            allRGBDCamerasReadCorrectly = ok && allRGBDCamerasReadCorrectly;
        }

        return allRGBDCamerasReadCorrectly;
    }
    
    bool readAllSensors(std::vector<std::string>& failedReadAllSensors)
    {
        failedReadAllSensors.clear();
        std::vector<std::string> failedReads;

        if (!readAllRGBDCameras(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(), failedReads.begin(), failedReads.end());
        }

        if (!readAllFrameGrabberCameras(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(), failedReads.begin(), failedReads.end());
        }
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


    if(!ptr->getParameter("check_for_nan", m_pimpl->checkForNAN))
    {
        std::cerr << logPrefix << "Unable to get check_for_nan."
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

bool YarpCameraBridge::advance()
{
    constexpr std::string_view logPrefix = "[YarpCameraBridge::advance] ";
    if (!m_pimpl->checkValid("[YarpCameraBridge::advance]"))
    {
        std::cerr << logPrefix << "Please initialize and set drivers list before running advance()."
                  << std ::endl;
        return false;
    }

    m_pimpl->readAllSensors(m_pimpl->failedSensorReads);

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
    if (!m_pimpl->checkValidSensorMeasure("YarpCameraBridge::getColorImage ",
                                           m_pimpl->wholeBodyCameraRGBImages, camName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyCameraRGBImages.find(camName);
    colorImg = yarp::cv::toCvMat(iter->second.first);
    if (colorImg.rows <= 0 || colorImg.cols <= 0)
    {
        std::cerr << "YarpCameraBridge::getColorImage " << camName << " image with invalid size." << std::endl;
        return false;
    }
    receiveTimeInSeconds = &iter->second.second;

    return true;
}

bool YarpCameraBridge::getDepthImage(const std::string& camName,
                                     cv::Mat& depthImg,
                                     double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpCameraBridge::getDepthImage ",
                                           m_pimpl->wholeBodyCameraDepthImages, camName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyCameraDepthImages.find(camName);
    depthImg = yarp::cv::toCvMat(iter->second.first);
    
    if (depthImg.rows <= 0 || depthImg.cols <= 0)
    {
        std::cerr << "YarpCameraBridge::getDepthImage " << camName << " image with invalid size." << std::endl;
        return false;
    }
    
    receiveTimeInSeconds = &iter->second.second;
    return true;
}
