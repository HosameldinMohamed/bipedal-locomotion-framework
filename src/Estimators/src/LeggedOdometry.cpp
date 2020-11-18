/**
 * @file LeggedOdometry.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <manif/manif.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Conversions;

class LeggedOdometry::Impl
{
public:

    bool changeFixedFrame(const iDynTree::FrameIndex& newIdx,
                          iDynTree::KinDynComputations& kinDyn);
    void updateInternalState();


    // parameters
    std::string m_initialFixedFrame; /**<  Fixed frame at initialization assumed to be in rigid contact with the environment*/
    std::string m_initialRefFrameForWorld; /**< Reference frame for the world at initialization */
    iDynTree::FrameIndex m_initialFixedFrameIdx{iDynTree::FRAME_INVALID_INDEX};  /**< Index of fixed frame at initialization */
    iDynTree::FrameIndex m_initialRefFrameForWorldIdx{iDynTree::FRAME_INVALID_INDEX};  /**< Index of world reference frame at initialization */
    manif::SE3d m_refFrame_H_world; /**< pose of world wrt reference frame at initialization */

    bool m_odometryInitialized{false}; /**< flag to check if odometry was initialized */
//     bool m_flatFloor{false}; /**< flag to enable flat floor constraint */

    iDynTree::FrameIndex m_prevFixedFrameIdx;
    iDynTree::FrameIndex m_currentFixedFrameIdx;

    manif::SE3d m_world_H_fixedFrame; /**< Pose of fixed frame wrt world */
    friend class LeggedOdometry;
};

LeggedOdometry::LeggedOdometry() : m_pimpl(std::make_unique<Impl>())
{
    m_state.imuOrientation.setIdentity();
    m_state.imuPosition.setZero();
    m_state.imuLinearVelocity.setZero();
    m_state.rContactFrameOrientation.setIdentity();
    m_state.rContactFramePosition.setZero();
    m_state.lContactFrameOrientation.setIdentity();
    m_state.lContactFramePosition.setZero();
    m_state.accelerometerBias.setZero();
    m_state.gyroscopeBias.setZero();

    m_statePrev = m_state;
    m_estimatorOut.state = m_state;

    m_meas.acc.setZero();
    m_meas.gyro.setZero();
    m_meas.lfInContact = false;
    m_meas.rfInContact = false;

    m_measPrev = m_meas;
}

LeggedOdometry::~LeggedOdometry() = default;

bool LeggedOdometry::customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    const std::string_view printPrefix = "[LeggedOdometry::customInitialization] ";
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << printPrefix << "The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("initial_fixed_frame", m_pimpl->m_initialFixedFrame))
    {
        std::cerr <<  printPrefix <<
        "The parameter handler could not find \" initial_fixed_frame \" in the configuration file."
        << std::endl;
        return false;
    }
    else
    {
        m_pimpl->m_initialFixedFrameIdx = m_modelComp.kinDyn().model().getFrameIndex(m_pimpl->m_initialFixedFrame);
        if (m_pimpl->m_initialFixedFrameIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << printPrefix << "Specified fixed frame not available in the loaded URDF Model."
            << std::endl;
            return false;
        }
    }

    std::vector<double> initialWorldOrientationInRefFrame{1., 0., 0., 0.};
    std::vector<double> initialWorldPositionInRefFrame{0., 0., 0.};
    if (!handle->getParameter("initial_ref_frame_for_world", m_dt))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \"initial_ref_frame_for_world \" in the configuration file. Setting \"initial_fixed_frame\" as reference frame for world"
        << std::endl;
        m_pimpl->m_initialRefFrameForWorld = m_pimpl->m_initialFixedFrame;
    }
    else
    {
        m_pimpl->m_initialRefFrameForWorldIdx = m_modelComp.kinDyn().model().getFrameIndex(m_pimpl->m_initialRefFrameForWorld);
        if (m_pimpl->m_initialRefFrameForWorldIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << printPrefix << "Specified reference frame for world not available in the loaded URDF Model."
            << std::endl;
            return false;
        }

        // setup initial states
        if (!handle->getParameter("initial_world_orientation_in_ref_frame", GenericContainer::make_vector(initialWorldOrientationInRefFrame, GenericContainer::VectorResizeMode::Fixed)))
        {
            std::cerr <<  printPrefix << "The parameter handler could not find \"initial_world_orientation_in_ref_frame\" in the configuration file. Setting to identity" << std::endl;
        }

        if (!handle->getParameter("initial_world_position_in_ref_frame", GenericContainer::make_vector(initialWorldPositionInRefFrame, GenericContainer::VectorResizeMode::Fixed)))
        {
            std::cerr << printPrefix << "The parameter handler could not find \"initial_world_position_in_ref_frame\" in the configuration file. Setting to zero." << std::endl;
        }
    }

    Eigen::Quaterniond refFrame_quat_world = Eigen::Quaterniond(initialWorldOrientationInRefFrame[0], initialWorldOrientationInRefFrame[1],
                                                                initialWorldOrientationInRefFrame[2], initialWorldOrientationInRefFrame[3]); // here loaded as w x y z
    refFrame_quat_world.normalize();
    Eigen::Vector3d refFrame_p_world;
    refFrame_p_world << initialWorldPositionInRefFrame[0], initialWorldPositionInRefFrame[1], initialWorldPositionInRefFrame[2];
    m_pimpl->m_refFrame_H_world = manif::SE3d(refFrame_p_world, refFrame_quat_world);

//     if (!handle->getParameter("flat_floor", m_pimpl->m_flatFloor))
//     {
//         std::cerr << printPrefix <<
//         "The parameter handler could not find \" flat_floor \" in the configuration file. Setting to false."
//         << std::endl;
//         m_pimpl->m_flatFloor = false;
//     }

    return true;
}



bool LeggedOdometry::resetEstimator()
{
    return true;
}

void LeggedOdometry::Impl::updateInternalState()
{
//            manif::SE3d fixedFrame_H_base = toManifPose(modelComputations().kinDyn().
//                                                     getRelativeTransform(m_pimpl->initialFixedFrameIdx,
//                                                                          modelComputations().baseLinkIdx()));
}

bool LeggedOdometry::resetEstimator(const FloatingBaseEstimators::InternalState& newState)
{
    return true;
}


bool LeggedOdometry::updateKinematics(const FloatingBaseEstimators::Measurements& meas,
                                      const double& dt)
{
    const std::string_view printPrefix = "[LeggedOdometry::updateKinematics] ";
    if ( (meas.encoders.size() != modelComputations().nrJoints()) ||
        (meas.encodersSpeed.size() != modelComputations().nrJoints()))
    {
        std::cerr << printPrefix << "Kinematic measurements size mismatch. Please call setKinematics() before calling advance()."
        << std::endl;
        return false;
    }

    if (!modelComputations().kinDyn().setJointPos(iDynTree::make_span(meas.encoders.data(), meas.encoders.size())))
    {
        std::cerr << printPrefix << "Unable to set joint positions."
        << std::endl;
        return false;
    }

    if (!m_pimpl->m_odometryInitialized)
    {
        m_pimpl->m_currentFixedFrameIdx = m_pimpl->m_initialFixedFrameIdx;

        manif::SE3d refFrame_H_fixedFrame  = toManifPose(modelComputations().kinDyn().
                                                         getRelativeTransform(m_pimpl->m_initialRefFrameForWorldIdx,
                                                                              m_pimpl->m_initialFixedFrameIdx));

        m_pimpl->m_world_H_fixedFrame = m_pimpl->m_refFrame_H_world.inverse()*refFrame_H_fixedFrame;

        m_pimpl->m_prevFixedFrameIdx = m_pimpl->m_currentFixedFrameIdx;
        m_pimpl->m_odometryInitialized = true;
        return true;
    }

    // change fixed frame depending on switch times
    // remove contacts if outdated
    // update states

    return true;
}


bool LeggedOdometry::Impl::changeFixedFrame(const iDynTree::FrameIndex& newIdx,
                                            iDynTree::KinDynComputations& kinDyn)
{
    const std::string_view printPrefix = "[LeggedOdometry::changeFixedFrame] ";
    if (newIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << printPrefix << "Specified frame index not available in the loaded URDF Model."
            << std::endl;
            return false;
    }

    manif::SE3d oldFixed_H_newFixed  = toManifPose(kinDyn.getRelativeTransform(m_currentFixedFrameIdx, newIdx));
    m_world_H_fixedFrame = m_world_H_fixedFrame*oldFixed_H_newFixed;
    m_prevFixedFrameIdx = m_currentFixedFrameIdx;
    m_currentFixedFrameIdx = newIdx;

    return true;
}


