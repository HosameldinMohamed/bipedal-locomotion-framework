/**
 * @file FloatingBaseEstimatorIO.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

#include <Eigen/Dense>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

#include <manif/manif.h>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace FloatingBaseEstimators
{

/**
* @brief Struct holding the elements of the state representation
*
*/
struct InternalState
{
    Eigen::Quaterniond imuOrientation; /**< Orientation of the base link IMU in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d imuPosition;   /**< Position of the base link IMU in the inertial frame */
    Eigen::Vector3d imuLinearVelocity; /**< linear part of the mixed-velocity representation of the IMU with respect to the inertial frame */
    Eigen::Quaterniond lContactFrameOrientation; /**< Orientation of the left foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d lContactFramePosition; /**< Position of the left foot contact frame in the inertial frame */
    Eigen::Quaterniond rContactFrameOrientation; /**< Orientation of the right foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d rContactFramePosition; /**< Position of the right foot contact frame in the inertial frame*/
    Eigen::Vector3d accelerometerBias; /**< Bias of the accelerometer expressed in the IMU frame */
    Eigen::Vector3d gyroscopeBias; /**< Bias of the gyroscope expressed in the IMU frame */

    void print() const
    {
        std::cout <<  "=== Base Estimator State ==="<< std::endl;
        std::cout << "IMU Quaternion xyz w: " << imuOrientation.coeffs().transpose() << std::endl;
        std::cout << "IMU Position xyz: " << imuPosition.transpose() << std::endl;
        std::cout << "IMU Linear Velocity xyz: " << imuLinearVelocity.transpose() << std::endl;

        std::cout << "Left Foot Contact Quaternion xyz w: " << lContactFrameOrientation.coeffs().transpose() << std::endl;
        std::cout << "Left Foot Contact Position xyz: " << lContactFramePosition.transpose() << std::endl;

        std::cout << "Right Foot Contact Quaternion xyz w: " << rContactFrameOrientation.coeffs().transpose() << std::endl;
        std::cout << "Right Foot Contact Position xyz: " << rContactFramePosition.transpose() << std::endl;

        std::cout << "Accelerometer Bias xyz: " << accelerometerBias.transpose() << std::endl;
        std::cout << "Gyroscope Bias xyz: " << gyroscopeBias.transpose() << std::endl;
        std::cout <<  "==========================="<< std::endl;
    }
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Output
{
    InternalState state; /**< Current state estimate of the estimator */
    StateStdDev stateStdDev; /**< Current state covariance matrix */

    iDynTree::Transform basePose; /**< Estimated base link pose */
    iDynTree::Twist baseTwist; /**< Estimate base link velocity in mixed-velocity representation */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Measurements
{
    Eigen::Vector3d acc, gyro; /**< accelerometer and gyroscope measurements expressed in IMU frame */
    Eigen::VectorXd encoders, encodersSpeed; /**< Joint position and joint velocity measurements */
    bool lfInContact{false}; /**< left foot contact state */
    bool rfInContact{false}; /**< right foot contact state */
    
    /** stamped global poses, 
     * the usage of this map must be in a way 
     * that every time an element is used, 
     * it must be erased from the map 
     */
    std::unordered_map<int, manif::SE3d > globalPoses; 
};

} // namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H
