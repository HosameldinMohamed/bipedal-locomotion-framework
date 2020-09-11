/**
 * @file DLGEKFBaseEstimator.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IMU_BIPED_MATRIX_LIEGROUP_H
#define BIPEDAL_LOCOMOTION_IMU_BIPED_MATRIX_LIEGROUP_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <manif/manif.h>
#include <Eigen/Dense>
#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{
    using Rotation = manif::SO3d;
    using Position = Eigen::Vector3d;
    using LinearVelocity = Eigen::Vector3d;
    using Pose = manif::SE3d;
    using ExtendedPose = manif::SE_2_3d;
    using AccelerometerBias = Eigen::Vector3d;
    using GyroscopeBias = Eigen::Vector3d;

    using ExtendedMotionVector = manif::SE_2_3Tangentd;
    using LinearAcceleration = Eigen::Vector3d;
    using AngularVelocity = manif::SO3Tangentd;
    using Twist = manif::SE3Tangentd;

    class IMUBipedMatrixLieGroupTangent;

    class IMUBipedMatrixLieGroup
    {
    public:
        /**
         * Destructor necessary for PIMPL idiom
         */
        ~IMUBipedMatrixLieGroup();

        /**
         * Default constructor
         * IMU Biases are unused by default
         * This will store an Identity matrix
         *  X = blkdiag(Xbase, Xlf, Xrf)
         *  Xbase  is a 5x5 identity matrix,
         *  Xlf, Xrf are a 4x4 Identity matrix
         */
        IMUBipedMatrixLieGroup();

        /**
         * Identity constructor with IMU bias states enabled
         * @param[in] estimateBias flag to enable handling IMU bias states
         *
         *  X = blkdiag(Xbase, Xlf, Xrf, I6)
         *  Xbase  is a 5x5 identity matrix,
         *  Xlf, Xrf are a 4x4 Identity matrix
         * I6 is a 6x6 Identity matrix
         */
        IMUBipedMatrixLieGroup(const bool& estimateBias);

        /**
         * SE23 x SE3 x SE3 constructor (IMU bias disabled)
         * @param[in] baseExtPose Extended pose of base link as manif::SE_2_3d object
         * @param[in] lfPose Pose of left foot contact frame as manif::SE3d object
         * @param[in] rfPose Pose of right foot contact frame as manif::SE3d object
         */
        IMUBipedMatrixLieGroup(const ExtendedPose& baseExtPose,
                               const Pose& lfPose,
                               const Pose& rfPose);

        /**
         * SE23 x SE3 x SE3 x R6 constructor (IMU bias enabled)
         * @param[in] baseExtPose Extended pose of base link as manif::SE_2_3d object
         * @param[in] lfPose Pose of left foot contact frame as manif::SE3d object
         * @param[in] rfPose Pose of right foot contact frame as manif::SE3d object
         * @param[in] accBias Accelerometer bias in IMU frame as Eigen::Vector3d object
         * @param[in] gyroBias Gyroscope bias in IMU frame as Eigen::Vector3d object
         */
        IMUBipedMatrixLieGroup(const ExtendedPose& baseExtPose,
                               const Pose& lfPose,
                               const Pose& rfPose,
                               const AccelerometerBias& accBias,
                               const GyroscopeBias& gyroBias);

        /**
         * SE23 x SE3 x SE3 constructor (IMU bias disabled)
         * @param[in] baseRotation Orientation of base link as manif::SO3d object
         * @param[in] basePosition Position of base link as Eigen::Vector3d object
         * @param[in] baseLinearVelocity Linear velocity of base link as Eigen::Vector3d object
         * @param[in] lfRotation Orientation of left foot contact frame as manif::SO3d object
         * @param[in] lfPosition Position of left foot contact frame as Eigen::Vector3d object
         * @param[in] rfRotation Orientation of right foot contact frame as manif::SO3d object
         * @param[in] rfPosition Position of right foot contact frame as Eigen::Vector3d object
         */
        IMUBipedMatrixLieGroup(const Rotation& baseRotation,
                               const Position& basePosition,
                               const LinearVelocity& baseLinearVelocity,
                               const Rotation& lfRotation,
                               const Position& lfPosition,
                               const Rotation& rfRotation,
                               const Position& rfPosition);

        /**
         * SE23 x SE3 x SE3 x R6 constructor (IMU bias enabled)
         * @param[in] baseRotation Orientation of base link as manif::SO3d object
         * @param[in] basePosition Position of base link as Eigen::Vector3d object
         * @param[in] baseLinearVelocity Linear velocity of base link as Eigen::Vector3d object
         * @param[in] lfRotation Orientation of left foot contact frame as manif::SO3d object
         * @param[in] lfPosition Position of left foot contact frame as Eigen::Vector3d object
         * @param[in] rfRotation Orientation of right foot contact frame as manif::SO3d object
         * @param[in] rfPosition Position of right foot contact frame as Eigen::Vector3d object
         * @param[in] accBias Accelerometer bias in IMU frame as Eigen::Vector3d object
         * @param[in] gyroBias Gyroscope bias in IMU frame as Eigen::Vector3d object
         */
        IMUBipedMatrixLieGroup(const Rotation& baseRotation,
                               const Position& basePosition,
                               const LinearVelocity& baseLinearVelocity,
                               const Rotation& lfRotation,
                               const Position& lfPosition,
                               const Rotation& rfRotation,
                               const Position& rfPosition,
                               const AccelerometerBias& accBias,
                               const GyroscopeBias& gyroBias);

        /**
         * Copy constructor
         */
        IMUBipedMatrixLieGroup(const IMUBipedMatrixLieGroup& other);

        /**
         * Copy assignment operator
         */
        IMUBipedMatrixLieGroup operator=(const IMUBipedMatrixLieGroup& other);

        /**
         * Constructor from Eigen Matrix
         */
        IMUBipedMatrixLieGroup(const Eigen::Ref<Eigen::MatrixXd> X);

        /**
         * Update from a Eigen Matrix object
         */
        void fromMatrixLieGroup(const Eigen::Ref<Eigen::MatrixXd> X);

        /**
         * Get the underlying Matrix Lie group
         * if bias enabled \f[ X \in  \mathbb{R}^{20 \times 20} \f]
         * \f[ X = \begin{bmatrix} R_b & p_b & v_b &     &     &      &      &     & \\
         *                             &   1 &     &     &     &      &      &     &  \\
         *                             &     &   1 &     &     &      &      &     & \\
         *                             &     &     & Rlf & plf &      &      &     & \\
         *                             &     &     &     &   1 &      &      &     &  \\
         *                             &     &     &     &     &  Rrf &  prf &     &  \\
         *                             &     &     &     &     &      &    1 &     &  \\
         *                             &     &     &     &     &      &      & I_6 & \begin{matrix} b_a \\ b_g \end{matrix} \\
         *                             &     &     &     &     &      &      &     & 1
         * \end{bmatrix} \f]
         *  If bias is disabled, last 7 rows and columns are omitted.
         *  \f[ X \in  \mathbb{R}^{13 \times 13} \f]
         */
        Eigen::MatrixXd asMatrixLieGroup();

        /**
         * Left composition of IMU Matrix Lie group
         * If this = X1 and other = X2,
         * lcompose gives X1 o X2
         *
         * @param[in] other IMUBipedMatrixLieGroup
         */
        IMUBipedMatrixLieGroup lcompose(const IMUBipedMatrixLieGroup& other);

        /**
         * Right composition of IMU Matrix Lie group
         * If this = X1 and other = X2,
         * lcompose gives  X2 o X1
         *
         * @param[in] other IMUBipedMatrixLieGroup
         */
        IMUBipedMatrixLieGroup rcompose(const IMUBipedMatrixLieGroup& other);

        /**
         * Left composition of IMU Matrix Lie group
         * If this = X1 and other = X2,
         * lcompose gives X1 o X2
         *
         * @param[in] other IMUBipedMatrixLieGroup
         */
        IMUBipedMatrixLieGroup operator*(const IMUBipedMatrixLieGroup& other);

        /**
         * Inverse of IMU Biped Matrix Lie group
         */
        IMUBipedMatrixLieGroup inverse();

        /**
         * set to identity element of IMU Biped matrix Lie group
         */
        void setIdentity();

        /**
         * Return identity element of the IMU Biped matrix Lie group
         * @param[in] enableBiasStates consider IMU bias states in the Matrix Lie group
         */
        static IMUBipedMatrixLieGroup Identity(const bool& enableBiasStates);

        /**
         * Logarithm mapping of IMU Biped matrix Lie group
         */
        IMUBipedMatrixLieGroupTangent log();

        /**
         * Adjoint matrix of IMU biped matrix Lie group
         */
        Eigen::MatrixXd adj();

        ExtendedPose baseExtenedPose() const; /**< get extended pose of the base link*/
        Pose basePose() const; /**< get pose of the base link*/
        Rotation baseRotation() const; /**< get rotation of the base link*/
        Position basePosition() const; /**< get position of the base link*/
        LinearVelocity baseLinearVelocity() const; /**< get linear velocity of base link */

        Pose leftFootContactPose() const; /**< get pose of the left foot contact frame*/
        Rotation leftFootContactRotation() const; /**< get rotation of the left foot contact frame*/
        Position leftFootContactPosition() const; /**< get position of the left foot contact frame*/

        Pose rightFootContactPose() const; /**< get pose of the right foot contact frame*/
        Rotation rightFootContactRotation() const; /**< get rotation of the right foot contact frame*/
        Position rightFootContactPosition() const; /**< get position of the right foot contact frame*/

        AccelerometerBias accelerometerBias() const; /**< get accelerometer bias in the IMU frame*/
        GyroscopeBias gyroscopeBias() const; /**< get gyroscope bias in the IMU frame */
        bool areBiasStatesActive() const; /**< check if bias states are active **/

        bool setBaseExtendedPose(const ExtendedPose& baseExtenedPose); /**< set extended pose of the base link*/
        bool setBasePose(const Pose& basePose); /**< set pose of the base link*/
        bool setBaseRotation(const Rotation& baseRotation); /**< set rotation of the base link*/
        bool setBasePosition(const Position& basePosition); /**< set position of the base link*/
        bool setBaseLinearVelocity(const LinearVelocity& baseLinearVelocity); /**< set linear velocity of base link */

        bool setLeftFootContactPose(const Pose& lfPose); /**< set pose of the left foot contact frame*/
        bool setLeftFootContactRotation(const Rotation& lfRotation); /**< set rotation of the left foot contact frame*/
        bool setLeftFootContactPosition(const Position& lfPosition); /**< set position of the left foot contact frame*/

        bool setRightFootContactPose(const Pose& rfPose); /**< set pose of right foot contact frame*/
        bool setRightFootContactRotation(const Rotation& rfRotation); /**< set rotation of right foot contact frame*/
        bool setRightFootContactPosition(const Position& rfPosition); /**< set position of right foot contact frame*/

        bool setAccelerometerBias(const AccelerometerBias& accelerometerBias); /**< set accelerometer bias in the IMU frame*/
        bool setGyroscopeBias(const AccelerometerBias& gyroscopeBias);  /**< set gyroscope bias in the IMU frame */

    private:
        class Impl;
        std::unique_ptr<Impl> m_pimpl;
    };


    class IMUBipedMatrixLieGroupTangent
    {
    public:
        ~IMUBipedMatrixLieGroupTangent();

        IMUBipedMatrixLieGroupTangent();

        IMUBipedMatrixLieGroupTangent(const bool& estimateBias);

        IMUBipedMatrixLieGroupTangent(const ExtendedMotionVector& vBase,
                                      const Twist& vLF,
                                      const Twist& vRF);

        IMUBipedMatrixLieGroupTangent(const ExtendedMotionVector& vBase,
                                      const Twist& vLF,
                                      const Twist& vRF,
                                      const AccelerometerBias& accBias,
                                      const GyroscopeBias& gyroBias);

        IMUBipedMatrixLieGroupTangent(const LinearVelocity& baseLinearVelocity,
                                      const AngularVelocity& baseAngularVelocity,
                                      const LinearAcceleration& baseLinearAcceleration,
                                      const LinearVelocity& lfLinearVelocity,
                                      const AngularVelocity& lfAngularVelocity,
                                      const LinearVelocity& rfLinearVelocity,
                                      const AngularVelocity& rfAngularVelocity);

        IMUBipedMatrixLieGroupTangent(const LinearVelocity& baseLinearVelocity,
                                      const AngularVelocity& baseAngularVelocity,
                                      const LinearAcceleration& baseLinearAcceleration,
                                      const LinearVelocity& lfLinearVelocity,
                                      const AngularVelocity& lfAngularVelocity,
                                      const LinearVelocity& rfLinearVelocity,
                                      const AngularVelocity& rfAngularVelocity,
                                      const AccelerometerBias& accBias,
                                      const GyroscopeBias& gyroBias);

        IMUBipedMatrixLieGroupTangent(const IMUBipedMatrixLieGroupTangent& other);
        IMUBipedMatrixLieGroupTangent operator=(const IMUBipedMatrixLieGroupTangent& other);

        IMUBipedMatrixLieGroupTangent(const Eigen::VectorXd& v);

        void fromVector(const Eigen::VectorXd& v);
        Eigen::VectorXd toVector();

        IMUBipedMatrixLieGroupTangent operator+(const IMUBipedMatrixLieGroupTangent& other);
        IMUBipedMatrixLieGroupTangent operator-(const IMUBipedMatrixLieGroupTangent& other);
        IMUBipedMatrixLieGroupTangent operator-();
        void setZero();

        Eigen::MatrixXd hat();
        IMUBipedMatrixLieGroup exp();
        Eigen::MatrixXd rjac();
        Eigen::MatrixXd rjacinv();
        Eigen::MatrixXd ljac();
        Eigen::MatrixXd ljacinv();
        Eigen::MatrixXd smallAdj();

        ExtendedMotionVector baseExtenedMotionVector() const; /**< get extended motion vector of the base link*/
        Twist baseTwist() const; /**< get twist of the base link*/
        LinearVelocity baseLinearVelocity() const; /**< get linear velocity of the base link*/
        AngularVelocity baseAngularVelocity() const; /**< get angular velocity of the base link*/
        LinearAcceleration baseLinearAcceleration() const; /**< get linear acceleration of base link */

        Twist leftFootContactTwist() const; /**< get twist of the left foot contact frame*/
        LinearVelocity leftFootContactLinearVelocity() const; /**< get linear velocity of the left foot contact frame*/
        AngularVelocity leftFootContactAngularVelocity() const; /**< get angular velocity of the left foot contact frame*/

        Twist rightFootContactTwist() const; /**< get twistof the right foot contact frame*/
        LinearVelocity rightFootContactLinearVelocity() const; /**< get linear velocity of the right foot contact frame*/
        AngularVelocity rightFootContactAngularVelocity() const; /**< get angular velocity of the right foot contact frame*/

        AccelerometerBias accelerometerBias() const; /**< get accelerometer bias in the IMU frame*/
        GyroscopeBias gyroscopeBias() const; /**< get gyroscope bias in the IMU frame */
        bool areBiasComponentsActive() const; /**< check if bias components are active **/

        bool setBaseExtendedMotionVector(const ExtendedMotionVector& baseExtenedMotionVector);  /**< set extended motion vector of the base link*/
        bool setBaseTwist(const Twist& baseTwist);  /**< set twist of the base link*/
        bool setBaseLinearVelocity(const LinearVelocity& baseLinearVelocity); /**< set linear velocity of the base link*/
        bool setBaseAngularVelocity(const AngularVelocity& baseAngularVelocity); /**< set angular velocity of the base link*/
        bool setBaseLinearAcceleration(const LinearAcceleration& baseLinearAcceleration); /**< set linear acceleration of base link */

        bool setLeftFootContactTwist(const Twist& leftFootContactTwist); /**< set twist of the left foot contact frame*/
        bool setLeftFootContactLinearVelocity(const LinearVelocity& leftFootContactLinearVelocity); /**< set linear velocity of the left foot contact frame*/
        bool setLeftFootContactAngularVelocity(const AngularVelocity& leftFootContactAngularVelocity); /**< set angular velocity of the left foot contact frame*/

        bool setRightFootContactTwist(const Twist& rightFootContactTwist); /**< set twist of the right foot contact frame*/
        bool setRightFootContactLinearVelocity(const LinearVelocity& rightFootContactLinearVelocity); /**< set linear velocity of the right foot contact frame*/
        bool setRightFootContactAngularVelocity(const AngularVelocity& rightFootContactAngularVelocity); /**< set angular velocity of the right foot contact frame*/

        bool setAccelerometerBias(const AccelerometerBias& accelerometerBias); /**< set accelerometer bias*/
        bool setGyroscopeBias(const GyroscopeBias& gyroscopeBias);  /**< set gyroscope bias*/

        static size_t vecSizeWithoutBias() { return static_cast<size_t>(21); };
        static size_t vecSizeWithBias() { return static_cast<size_t>(27); };

        static size_t basePositionOffset() { return static_cast<size_t>(0); };
        static size_t baseOrientationOffset() { return static_cast<size_t>(3); };
        static size_t baseLinearVelocityOffset() { return static_cast<size_t>(6); };
        static size_t leftFootContactPositionOffset() { return static_cast<size_t>(9); };
        static size_t leftFootContactOrientationOffset() { return static_cast<size_t>(12); };
        static size_t rightFootContactPositionOffset() { return static_cast<size_t>(15); };
        static size_t rightFootContactOrientationOffset() { return static_cast<size_t>(18); };
        static size_t accelerometerBiasOffset() { return static_cast<size_t>(21); };
        static size_t gyroscopeBiasOffset() { return static_cast<size_t>(24); };

        private:
        class Impl;
        std::unique_ptr<Impl> m_pimpl;
    };


} // namespace Estimators
} // namespace BipedalLocomotion
#endif // BIPEDAL_LOCOMOTION_IMU_BIPED_MATRIX_LIEGROUP_H
