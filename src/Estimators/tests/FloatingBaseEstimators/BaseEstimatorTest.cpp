/**
 * @file InvariantEKFBaseEstimatorTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/DLGEKFBaseEstimator.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <ResourceFolderPath.h>

#include <Eigen/Dense>
#include <cmath>
#include <manif/manif.h>

#include <random>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;

double deg2rad(const double& ang)
{
    return ang*(M_PI/180);
}

bool populateConfig(std::weak_ptr<IParametersHandler> handler, int nr_joints)
{
    auto handle = handler.lock();
    if (handle == nullptr) {return false;}
    handle->setParameter("sampling_period_in_s", 0.01);

    auto modelInfoGroup = std::make_shared<StdImplementation>();
    handle->setGroup("ModelInfo", modelInfoGroup);
    modelInfoGroup->setParameter("base_link", "root_link");
    modelInfoGroup->setParameter("base_link_imu", "root_link_imu_acc");
    modelInfoGroup->setParameter("left_foot_contact_frame", "l_sole");
    modelInfoGroup->setParameter("right_foot_contact_frame", "r_sole");

    auto optionsGroup = std::make_shared<StdImplementation>();
    handle->setGroup("Options", optionsGroup);
    optionsGroup->setParameter("enable_imu_bias_estimation", false);
    optionsGroup->setParameter("enable_ekf_update", true);

    auto sensorsdevGroup = std::make_shared<StdImplementation>();
    handle->setGroup("SensorsStdDev", sensorsdevGroup);
    IParametersHandler::shared_ptr sensdev_handler = sensorsdevGroup;
    sensdev_handler->setParameter("accelerometer_measurement_noise_std_dev", std::vector<double>{0.004, 0.004, 0.004});
    sensdev_handler->setParameter("gyroscope_measurement_noise_std_dev", std::vector<double>{0.005, 0.005, 0.005});
    sensdev_handler->setParameter("contact_foot_linear_velocity_noise_std_dev", std::vector<double>{9e-3, 9.5e-3, 7e-3});
    sensdev_handler->setParameter("contact_foot_angular_velocity_noise_std_dev", std::vector<double>{0.007, 0.0075, 0.004});
    sensdev_handler->setParameter("swing_foot_linear_velocity_noise_std_dev", std::vector<double>{0.05, 0.05, 0.05});
    sensdev_handler->setParameter("swing_foot_angular_velocity_noise_std_dev", std::vector<double>{0.015, 0.015, 0.015});
    sensdev_handler->setParameter("forward_kinematic_measurement_noise_std_dev", std::vector<double>{1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6});
    std::vector<double> encoder_noise(nr_joints);
    for (auto& v : encoder_noise)
    {
        v = deg2rad(0.5);
    }
    sensdev_handler->setParameter("encoders_measurement_noise_std_dev", encoder_noise);

    auto initStateGroup = std::make_shared<StdImplementation>();
    handle->setGroup("InitialStates", initStateGroup);
    IParametersHandler::shared_ptr initstate_handler = initStateGroup;
    initstate_handler->setParameter("imu_orientation_quaternion_wxyz", std::vector<double>{0.3218, -0.6304, -0.6292, 0.3212});
    initstate_handler->setParameter("l_contact_frame_orientation_quaternion_wxyz", std::vector<double>{1.0000, -0.0059, -0.0001, -0.0015});
    initstate_handler->setParameter("r_contact_frame_orientation_quaternion_wxyz", std::vector<double>{1.0000, 0.0059, -0.0002, -0.0004});

    initstate_handler->setParameter("imu_position_xyz", std::vector<double>{0.0296,-0.1439, 0.4915});
    initstate_handler->setParameter("imu_linear_velocity_xyz", std::vector<double>{0, 0, 0});
    initstate_handler->setParameter("l_contact_frame_position_xyz", std::vector<double>{0.0791, -0.0817, 0.0109});
    initstate_handler->setParameter("r_contact_frame_position_xyz", std::vector<double>{0.0788, -0.2282, 0.0109});

    auto priordevGroup = std::make_shared<StdImplementation>();
    handle->setGroup("PriorsStdDev", priordevGroup);
    IParametersHandler::shared_ptr prior_handler = priordevGroup;
    prior_handler->setParameter("imu_orientation", std::vector<double>{deg2rad(10), deg2rad(10), deg2rad(1)});
    prior_handler->setParameter("imu_position", std::vector<double>{1e-3, 1e-3, 1e-3});
    prior_handler->setParameter("imu_linear_velocity", std::vector<double>{0.075, 0.05, 0.05});
    prior_handler->setParameter("l_contact_frame_orientation", std::vector<double>{deg2rad(10), deg2rad(10), deg2rad(10)});
    prior_handler->setParameter("l_contact_frame_position", std::vector<double>{1e-3, 1e-3, 1e-3});
    prior_handler->setParameter("r_contact_frame_orientation", std::vector<double>{deg2rad(10), deg2rad(10), deg2rad(10)});
    prior_handler->setParameter("r_contact_frame_position", std::vector<double>{1e-3, 1e-3, 1e-3});

    return true;
}

bool testEstimator(const IParametersHandler::shared_ptr& parameterHandler,
                   const iDynTree::Model& model,
                   const std::vector<std::string>& joints_list,
                   FloatingBaseEstimator& estimator)
{
    REQUIRE(estimator.initialize(parameterHandler, model));
    REQUIRE(estimator.modelComputations().isModelSet());
    REQUIRE(estimator.modelComputations().nrJoints() == joints_list.size());
    REQUIRE(estimator.modelComputations().baseLink() == "root_link");
    REQUIRE(estimator.modelComputations().baseLinkIMU() == "root_link_imu_acc");
    REQUIRE(estimator.modelComputations().leftFootContactFrame() == "l_sole");
    REQUIRE(estimator.modelComputations().rightFootContactFrame() == "r_sole");

    auto b_H_imu = model.getFrameTransform(model.getFrameIndex("root_link_imu_acc"));

    ASSERT_EQUAL_TRANSFORM(b_H_imu, estimator.modelComputations().base_H_IMU());

    // ground truth
    Eigen::Vector3d simIMUPos;
    Eigen::Quaterniond simImuQuat = Eigen::Quaterniond(0.3218, -0.6304, -0.6292, 0.3212);
    simImuQuat.normalize(); // normalize the user defined quaternion to respect internal tolerances for unit norm constraint
    simIMUPos << 0.0296, -0.1439,  0.4915;

    std::mt19937 gen(42);

    /** The noise is modelled as Gaussian vector with zero mean and diagonal covariance matrix */
    std::normal_distribution<> accNoise{0, 0.004};
    std::normal_distribution<> gyroNoise{0, 0.005};
    std::normal_distribution<> encoderNoise{0, deg2rad(0.5)};

    // IMU measures
    Eigen::Vector3d acc, gyro;

    // contact states
    bool lf_contact{true}, rf_contact{true};

    // kinematic measures
    Eigen::VectorXd encoders(joints_list.size()), encoder_speeds(joints_list.size());

    encoder_speeds.setZero();
    FloatingBaseEstimators::Output out;
    manif::SO3Tangentd rotError;
    // set measurements and advance the estimator
    for (int i = 0; i < 100; i++)
    {
        //static pose measurements
        acc << 0.0,   -7.9431,   -5.7513;
        gyro << 0.0, 0.0, 0.0;
        encoders << -0.0001, 0.0000, 0.0000,
                     0.1570, 0.0003, -0.0000,
                    -0.0609, 0.4350, 0.1833, 0.5375,
                    -0.0609, 0.4349, 0.1834, 0.5375,
                     0.0895, 0.0090, -0.0027,
                    -0.5694, -0.3771, -0.0211,
                     0.0896, 0.0090, -0.0027,
                    -0.5695, -0.3771, -0.0211;

        // add noise to IMU measurements
        for (int idx = 0; idx < 3; idx++)
        {
            acc(idx) += accNoise(gen);
            gyro(idx) += gyroNoise(gen);
        }
        // add noise to encoder measurements
        for (int idx = 0; idx < encoders.size(); idx++)
        {
            encoders(idx) += encoderNoise(gen);
        }

        REQUIRE(estimator.setIMUMeasurement(acc, gyro));
        REQUIRE(estimator.setContacts(lf_contact, rf_contact));
        REQUIRE(estimator.setKinematics(encoders, encoder_speeds));
        REQUIRE(estimator.advance());
        out = estimator.get();

        std::cout << "---------------------------Iteration: " << i <<"-------------------------------" << std::endl;
        std::cout << "Simulated IMU Orientation quaternion xyz w: " << simImuQuat.coeffs().transpose() << std::endl;
        std::cout << "Estimated IMU Orientation quaternion xyz w: " << out.state.imuOrientation.coeffs().transpose() << std::endl << std::endl;
        std::cout << "Simulated IMU Position: " << simIMUPos.transpose() << std::endl;
        std::cout << "Estimated IMU Position: " << out.state.imuPosition.transpose() << std::endl;
        std::cout << "-------------------------------------------------------------" << std::endl;

        manif::SO3d estR(out.state.imuOrientation);
        manif::SO3d simR(simImuQuat);
        rotError = estR - simR; // performs logvee(R1.T R2)
    }

    REQUIRE(rotError.weightedNorm() < 0.02);
    REQUIRE((simIMUPos - out.state.imuPosition).norm() < 2e-2);

    // test reset methods
    out = estimator.get();
    FloatingBaseEstimators::InternalState resetState;
    resetState = out.state;
    // changing imu position should be reflected also on feet contact positions
    // to maintian consistent resetting of the internal state
    resetState.imuPosition(0) += 10;
    resetState.rContactFramePosition(0) += 10;
    resetState.lContactFramePosition(0) += 10;
    REQUIRE(estimator.resetEstimator(resetState));
    REQUIRE(estimator.advance());
    auto newOut = estimator.get();
    REQUIRE( std::abs(newOut.state.imuPosition(0) - resetState.imuPosition(0)) < 1e-2);


    // reset base pose
    auto basePose = out.basePose;
    auto basePosition = basePose.getPosition();
    auto imuPosition = out.state.imuPosition;
    basePosition(1) -= 10.0;
    imuPosition(1) -= 10.0;
    // convert to iDynTree Rotation to angle axis and then to quaternion - otherwise, produces inverse rotations
    auto angleaxis = Eigen::AngleAxisd(iDynTree::toEigen(basePose.getRotation()));
    auto baseOrientation = Eigen::Quaterniond(angleaxis);
    estimator.resetEstimator(baseOrientation, iDynTree::toEigen(basePosition));
    REQUIRE(estimator.advance());
    newOut = estimator.get();
    // roughly computing from base position 2 cm tolerance
    REQUIRE( std::abs(newOut.state.imuPosition(1) - (imuPosition(1))) < 2e-2);

    return true;
}

TEST_CASE("DILIGENT Base Estimator")
{
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    std::vector<std::string> joints_list = {"neck_pitch", "neck_roll", "neck_yaw",
        "torso_pitch", "torso_roll", "torso_yaw",
        "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
        "l_hip_pitch", "l_hip_roll", "l_hip_yaw",
        "l_knee", "l_ankle_pitch", "l_ankle_roll",
        "r_hip_pitch", "r_hip_roll", "r_hip_yaw",
        "r_knee", "r_ankle_pitch", "r_ankle_roll"};
    int nr_joints = joints_list.size();

    // Populate the input configuration to be passed to the estimator
    REQUIRE(populateConfig(parameterHandler,nr_joints));

    // Load the reduced iDynTree model to be passed to the estimator
    std::string model_path{getFBEURDFModelPath()};
    std::cout << model_path << std::endl;


    iDynTree::ModelLoader mdl_ldr;
    REQUIRE(mdl_ldr.loadReducedModelFromFile(model_path, joints_list));

    auto model = mdl_ldr.model().copy();

    DLGEKFBaseEstimator dlgekf;
    REQUIRE(testEstimator(parameterHandler, model, joints_list, dlgekf));

}
