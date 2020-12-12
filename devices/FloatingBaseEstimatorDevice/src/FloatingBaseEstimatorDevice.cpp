/**
 * @file FloatingBaseEstimatorDevice.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimatorDevice.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/DLGEKFBaseEstimator.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/os/LogStream.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice(double period,
                                                         yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock)
{
}

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

FloatingBaseEstimatorDevice::~FloatingBaseEstimatorDevice()
{
}

bool FloatingBaseEstimatorDevice::open(yarp::os::Searchable& config)
{
    YarpUtilities::getElementFromSearchable(config, "robot", m_robot);
    YarpUtilities::getElementFromSearchable(config, "port_prefix", m_portPrefix);
    YarpUtilities::getElementFromSearchable(config, "base_link_imu", m_baseLinkImuName);
    YarpUtilities::getElementFromSearchable(config, "left_foot_wrench", m_leftFootWrenchName);
    YarpUtilities::getElementFromSearchable(config, "right_foot_wrench", m_rightFootWrenchName);
    double devicePeriod{0.01};

    if (YarpUtilities::getElementFromSearchable(config, "sampling_period_in_s", devicePeriod))
    {
        setPeriod(devicePeriod);
    }

    if (!setupRobotModel(config))
    {
        return false;
    }

    if (!setupRobotSensorBridge(config))
    {
        return false;
    }

    if (!setupFeetContactStateMachines(config))
    {
        return false;
    }

    if (!setupBaseEstimator(config))
    {
        return false;
    }

    if (!YarpUtilities::getElementFromSearchable(config, "publish_rostf", m_publishROSTF))
     {
         m_publishROSTF = false;
     }

     if (m_publishROSTF)
     {
         if (!loadTransformBroadcaster())
         {
             return false;
         }
     }
     
     if (m_robot == iRonCubMK1)
     {
         YarpUtilities::getElementFromSearchable(config, "ironcub_aruco_port", m_iRonCubMKArucoPortName);   
         YarpUtilities::getElementFromSearchable(config, "ironcub_aruco_origin_frame", m_iRonCubArucoOriginFrame);          
     }
          
    return true;
}

bool FloatingBaseEstimatorDevice::setupRobotModel(yarp::os::Searchable& config)
{

    std::string modelFileName;
    std::vector<std::string> jointsList;
    if (!YarpUtilities::getElementFromSearchable(config, "model_file", modelFileName))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotModel] Missing required parameter \"model_file\"";
        return false;
    }

    if (!YarpUtilities::getVectorFromSearchable(config, "joint_list", jointsList))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotModel] Missing required parameter \"joint_list\"";
        return false;
    }

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelFilePath{rf.findFileByName(modelFileName)};
    yInfo() << "[FloatingBaseEstimatorDevice][setupRobotModel] Loading model from " << modelFilePath;

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadReducedModelFromFile(modelFilePath, jointsList))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotModel] Could not load robot model";
        return false;
    }

    m_model = modelLoader.model();
    return true;
}

bool FloatingBaseEstimatorDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotSensorBridge] Missing required group \"RobotSensorBridge\"";
        return false;
    }

    std::shared_ptr<YarpImplementation> bridgeHandler = std::make_shared<YarpImplementation>();
    bridgeHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(bridgeHandler))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotSensorBridge] Could not configure RobotSensorBridge";
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::setupFeetContactStateMachines(yarp::os::Searchable& config)
{
    auto csmConfig = config.findGroup("ContactSchmittTrigger");
    if (csmConfig.isNull())
    {
        yError() << "[FloatingBaseEstimatorDevice][setupFeetContactStateMachines] Missing required group \"ContactSchmittTrigger\"";
        return false;
    }

    iDynTree::SchmittParams lParams, rParams;
    auto lCSMConfig = csmConfig.findGroup("left_foot");
    if (lCSMConfig.isNull())
    {
        yError() << "[FloatingBaseEstimatorDevice][setupFeetContactStateMachines] Could not load left foot contact Schmitt trigger configuration group.";
        return false;
    }
    if (!parseFootSchmittParams(lCSMConfig, lParams))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupFeetContactStateMachines] Could not load left foot contact Schmitt trigger parameters";
        return false;
    }

    auto rCSMConfig = csmConfig.findGroup("right_foot");
    if (rCSMConfig.isNull())
    {
        yError() << "[FloatingBaseEstimatorDevice][setupFeetContactStateMachines] Could not load right foot contact Schmitt trigger configuration group.";
        return false;
    }
    if (!parseFootSchmittParams(rCSMConfig, rParams))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupFeetContactStateMachines] Could not load right foot contact Schmitt trigger parameters";
        return false;
    }

    m_lFootCSM = std::make_unique<iDynTree::ContactStateMachine>(lParams);
    m_rFootCSM = std::make_unique<iDynTree::ContactStateMachine>(rParams);

    return true;
}

bool FloatingBaseEstimatorDevice::parseFootSchmittParams(yarp::os::Searchable& config,
                                                         iDynTree::SchmittParams& params)
{
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_stable_contact_make_time", params.stableTimeContactMake)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_stable_contact_break_time", params.stableTimeContactBreak)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_contact_make_force_threshold", params.contactMakeForceThreshold)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_contact_break_force_threshold", params.contactBreakForceThreshold)) { return false; }

    return true;
}

bool FloatingBaseEstimatorDevice::setupBaseEstimator(yarp::os::Searchable& config)
{
    if (!YarpUtilities::getElementFromSearchable(config, "estimator_type", m_estimatorType))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotModel] Missing required parameter \"estimator_type\"";
        return false;
    }

    if (m_estimatorType == "InvEKF")
    {
        m_estimator = std::make_unique<InvariantEKFBaseEstimator>();
    }
    else if (m_estimatorType == "DLGEKF")
    {
        m_estimator = std::make_unique<DLGEKFBaseEstimator>();
    }

    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(config);
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    if (!m_estimator->initialize(parameterHandler, m_model))
    {
        yError() << "[FloatingBaseEstimatorDevice][setupRobotModel] Could not configure estimator";
        return false;
    }
    return true;
}

bool FloatingBaseEstimatorDevice::attachAll(const yarp::dev::PolyDriverList & poly)
{
    if (!m_robotSensorBridge->setDriversList(poly))
    {
        yError() << "[FloatingBaseEstimatorDevice][attachAll] Failed to attach to devices through RobotSensorBridge.";
        return false;
    }

    if (!openCommunications())
    {
        yError() << "[FloatingBaseEstimatorDevice][attachAll] Could not open ports for publishing outputs.";
        return false;
    }

        std::vector<std::string> cartesianWrenchesList;

    start();
    return true;
}

bool FloatingBaseEstimatorDevice::openCommunications()
{
    if (!openBufferedSigPort(m_comms.floatingBaseStatePort, m_portPrefix, "/floating_base/state:o"))
    {
        return false;
    }

    openBufferedSigPort(m_comms.internalStateAndStdDevPort, m_portPrefix, "/internal_state/stateAndStdDev:o");
    openBufferedSigPort(m_comms.contactStatePort, m_portPrefix, "/contacts/stateAndNormalForce:o");
    
    if (m_robot == iRonCubMK1)
    {
        bool ok = openBufferedSigPort(m_comms.ironCubMKArucoPort, m_portPrefix, "/aruco_board/state:i");
        
        if (!ok)
        {
            yError() << "[FloatingBaseEstimatorDevice][openCommunications] Could not connect to port ";
            return false;
        }
        
        if(!yarp::os::Network::connect(m_iRonCubMKArucoPortName,  m_portPrefix + "/aruco_board/state:i"))
        {
            yError() << "Unable to make a port connection between " << 
            m_iRonCubMKArucoPortName << " and " << m_portPrefix + "/aruco_board/state:i";
            return false;
        }
    }
    
    return true;
}

bool FloatingBaseEstimatorDevice::openBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port,
                                                      const std::string& portPrefix,
                                                      const std::string& address)
{
    bool ok{false};
    ok = port.open(portPrefix + address);
    if (!ok)
    {
        yError() << "[FloatingBaseEstimatorDevice][openBufferedSigPort] error opening port " << portPrefix + address;
        return false;
    }
    return true;
}

void FloatingBaseEstimatorDevice::run()
{
    // advance sensor bridge
    if (!m_robotSensorBridge->advance())
    {
        yWarning() << "Advance Sensor bridge failed.";
        return;
    }

    // update estimator measurements
    if (!updateMeasurements())
    {
        yWarning() << "Measurement updates failed.";
        return;
    }

    // advance estimator
    if (!m_estimator->advance())
    {
        yWarning()  << "Advance estimator failed.";
        return;
    }

    publish();
}

bool FloatingBaseEstimatorDevice::updateMeasurements()
{
    bool ok{true};

    ok = ok && updateContactStates();
    ok = ok && updateInertialBuffers();
    ok = ok && updateKinematics();

    return ok;
}

bool FloatingBaseEstimatorDevice::updateInertialBuffers()
{
    Eigen::Matrix<double, 12, 1> imuMeasure;
    if (!m_robotSensorBridge->getIMUMeasurement(m_baseLinkImuName, imuMeasure))
    {
        return false;
    }

    const int accOffset{3};
    const int gyroOffset{6};
    if (!m_estimator->setIMUMeasurement(imuMeasure.segment<3>(accOffset), imuMeasure.segment<3>(gyroOffset)))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::updateContactStates()
{
    Eigen::Matrix<double, 6, 1> lfWrench, rfWrench;
    double lfTimeStamp, rfTimeStamp;
    bool ok{true};
    ok = ok && m_robotSensorBridge->getCartesianWrench(m_leftFootWrenchName, lfWrench, &lfTimeStamp);
    if (ok)
    {
        m_currentlContactNormal = lfWrench(2);
        m_lFootCSM->contactMeasurementUpdate(lfTimeStamp, lfWrench(2)); // fz
    }

    ok = ok && m_robotSensorBridge->getCartesianWrench(m_rightFootWrenchName, rfWrench, &rfTimeStamp);
    if (ok)
    {
        m_currentrContactNormal = rfWrench(2);
        m_rFootCSM->contactMeasurementUpdate(rfTimeStamp, rfWrench(2)); // fz
    }

    if (!ok)
    {
        return false;
    }

    m_currentlFootState = m_lFootCSM->contactState();
    m_currentrFootState = m_rFootCSM->contactState();
    if (!m_estimator->setContacts(m_currentlFootState, m_currentrFootState))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::updateKinematics()
{
    Eigen::VectorXd encoders(m_model.getNrOfDOFs()), encoderSpeeds(m_model.getNrOfDOFs());
    double timeStamp;
    if (!m_robotSensorBridge->getJointPositions(encoders))
    {
        return false;
    }

    if (!m_robotSensorBridge->getJointVelocities(encoderSpeeds))
    {
        return false;
    }

    if (!m_estimator->setKinematics(encoders, encoderSpeeds))
    {
        return false;
    }
    return true;
}

bool FloatingBaseEstimatorDevice::updateiRonCubArucoBoardPose()
{
    if (m_robot == iRonCubMK1)
    {
        yarp::sig::Vector* arucoBoardPose{nullptr};
        yarp::sig::Vector poseVec;
        arucoBoardPose = m_comms.ironCubMKArucoPort.read(false);
        if(arucoBoardPose != nullptr)
        {   
            poseVec = *arucoBoardPose;
            Eigen::Vector3d pos;
            pos << poseVec(0),
                   poseVec(1),
                   poseVec(2);
            Eigen::Quaterniond quat = Eigen::Quaterniond(poseVec(3),
                                                      poseVec(4),
                                                      poseVec(5),
                                                      poseVec(6));
            
            if (!m_estimator->setGlobalPose(m_iRonCubArucoOriginFrame, quat, pos))
            {
                return false;
            }                             
        }
    }
    return true;
}

void FloatingBaseEstimatorDevice::publish()
{
    if (m_estimator->isValid())
    {
        auto estimatorOut = m_estimator->get();
        publishBaseLinkState(estimatorOut);
        publishInternalStateAndStdDev(estimatorOut);
    }
    publishFootContactStatesAndNormalForces();
}

void FloatingBaseEstimatorDevice::publishBaseLinkState(const FloatingBaseEstimators::Output& estimatorOut)
{
    size_t stateVecSize{12};
    size_t rpyOffset{3};
    size_t linVelOffset{6};
    size_t angVelOffset{9};

    auto basePos = estimatorOut.basePose.getPosition();
    Eigen::Vector3d baseRPY = iDynTree::toEigen(estimatorOut.basePose.getRotation().asRPY());
    auto baseLinearVel = estimatorOut.baseTwist.getLinearVec3();
    auto baseAngularVel = estimatorOut.baseTwist.getAngularVec3();

    yarp::sig::Vector& stateVec = m_comms.floatingBaseStatePort.prepare();
    stateVec.clear();
    stateVec.resize(stateVecSize);

    for (size_t idx = 0; idx < rpyOffset; idx++) { stateVec(idx) = basePos(idx); }
    for (size_t idx = rpyOffset; idx < linVelOffset; idx++) { stateVec(idx) = baseRPY(idx - rpyOffset); }
    for (size_t idx = linVelOffset; idx < angVelOffset; idx++) { stateVec(idx) = baseLinearVel(idx - linVelOffset); }
    for (size_t idx = angVelOffset; idx < stateVecSize; idx++) { stateVec(idx) = baseAngularVel(idx - angVelOffset); }

    yarp::sig::Matrix basePoseYARP;
    iDynTree::toYarp(estimatorOut.basePose, basePoseYARP);
    if (m_publishROSTF && m_transformInterface != nullptr)
    {
        if (!m_transformInterface->setTransform("/world", "/base_link", basePoseYARP))
        {
            yError() << "[FloatingBaseEstimatorDevice] Could not publish measured base pose transform from  primary IMU";
        }
    }

    m_comms.floatingBaseStatePort.write();
}

void FloatingBaseEstimatorDevice::publishInternalStateAndStdDev(const FloatingBaseEstimators::Output& estimatorOut)
{
    size_t vecSize{27};

    const auto& s = estimatorOut.state;
    const auto& d = estimatorOut.stateStdDev;
    Eigen::Vector3d imuRPY = s.imuOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    Eigen::Vector3d rfRPY = s.rContactFrameOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    Eigen::Vector3d lfRPY = s.lContactFrameOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    Eigen::VectorXd internalstateAndStdDev;
    internalstateAndStdDev.resize(vecSize + vecSize);
    internalstateAndStdDev << s.imuPosition, imuRPY, s.imuLinearVelocity,
                              s.rContactFramePosition, rfRPY,
                              s.lContactFramePosition, lfRPY,
                              s.accelerometerBias, s.gyroscopeBias,
                              d.imuPosition, d.imuOrientation, d.imuLinearVelocity,
                              d.rContactFramePosition, d.rContactFrameOrientation,
                              d.lContactFramePosition, d.lContactFrameOrientation,
                              d.accelerometerBias, d.gyroscopeBias;

    yarp::sig::Vector& stateAndStdDev = m_comms.internalStateAndStdDevPort.prepare();
    stateAndStdDev.clear();
    stateAndStdDev.resize(vecSize + vecSize);

    for (size_t idx = 0; idx < 2*vecSize; idx++)
    {
        stateAndStdDev(idx) = internalstateAndStdDev[idx];
    }

    m_comms.internalStateAndStdDevPort.write();
}

void FloatingBaseEstimatorDevice::publishFootContactStatesAndNormalForces()
{
    yarp::sig::Vector& contactVec = m_comms.contactStatePort.prepare();
    contactVec.clear();

    contactVec.resize(4);
    const int scaling_const_for_visualization{300};
    contactVec(0) = scaling_const_for_visualization*static_cast<int>(m_currentlFootState);
    contactVec(1) = scaling_const_for_visualization*static_cast<int>(m_currentrFootState);
    contactVec(2) = m_currentlContactNormal;
    contactVec(3) = m_currentrContactNormal;
    m_comms.contactStatePort.write();
}

bool FloatingBaseEstimatorDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}

void FloatingBaseEstimatorDevice::closeBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port)
{
    if (!port.isClosed())
    {
        port.close();
    }

    if (m_publishROSTF)
    {
        m_transformInterface = nullptr;
    }
}

void FloatingBaseEstimatorDevice::closeCommunications()
{
    closeBufferedSigPort(m_comms.floatingBaseStatePort);
    closeBufferedSigPort(m_comms.internalStateAndStdDevPort);
    closeBufferedSigPort(m_comms.contactStatePort);
}

bool FloatingBaseEstimatorDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    closeCommunications();
    return true;
}

bool FloatingBaseEstimatorDevice::loadTransformBroadcaster()
{
    yarp::os::Property tfBroadcasterSettings{{"device", yarp::os::Value("transformClient")},
                                             {"remote", yarp::os::Value("/transformServer")},
                                             {"local", yarp::os::Value(m_portPrefix + "/transformClient")}};

    if (!m_transformBroadcaster.open(tfBroadcasterSettings))
    {
        yError() << "[FloatingBaseEstimatorDevice][loadTransformBroadcaster] could not open transform broadcaster.";
        return false;
    }

    if (!m_transformBroadcaster.view(m_transformInterface))
    {
        yError() << "[FloatingBaseEstimatorDevice][loadTransformBroadcaster] could not access transform interface";
        return false;
    }

    return true;
}
