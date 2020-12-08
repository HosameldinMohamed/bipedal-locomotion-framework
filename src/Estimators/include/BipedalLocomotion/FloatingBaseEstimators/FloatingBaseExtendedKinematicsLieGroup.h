/**
 * @file FloatingBaseExtendedKinematicsLieGroup.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FB_EXT_KINEMATICS_MATRIX_LIEGROUP_H
#define BIPEDAL_LOCOMOTION_FB_EXT_KINEMATICS_MATRIX_LIEGROUP_H

#include <manif/manif.h>
#include <Eigen/Dense>
#include <memory>
#include <map>

namespace BipedalLocomotion
{
namespace Estimators
{
    class FloatingBaseExtendedKinematicsLieGroupTangent;

    class FloatingBaseExtendedKinematicsLieGroup
    {
    public:
        /**
         * Destructor necessary for PIMPL idiom
         */
        ~FloatingBaseExtendedKinematicsLieGroup();

        /**
         * Default constructor
         * Only extended pose of the base will be constructed
         */
        FloatingBaseExtendedKinematicsLieGroup();
        
        /**
         * Construct group representation only with base extended state 
         * Only extended pose of the base will be constructed
         */
        FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose);

        /**
         * Identity constructor for base pose with augmented vector states enabled
         * @param[in] augVecDimensions dimension of augmented vector states
         * 
         *  X = blkdiag(Xbase, I_{n+1})
         *  Xbase  is a 5x5 identity matrix,
         *  I_{n+1} is a (n+1) x(n+1) Identity matrix with the augmented vector as zero vector
         */
        FloatingBaseExtendedKinematicsLieGroup(const int& augVecDimensions);

        /**
         * SE23 x (SE3)^k constructor 
         * 
         * @param[in] baseExtPose extended pose of base link as manif::SE_2_3d object
         * @param[in] supportFramesPose ordered map of frame index and support frame poses as manif::SE3d object
         * @note the underlying matrix Lie group will be constructed maintaining an ascending order of support frame indices
         */
        FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose,
                                               const std::map<int, manif::SE3d>& supportFramesPose);
 
        /**
         * SE23 x (SE3)^k x Rn constructor 
         * @param[in] baseExtPose extended pose of base link as manif::SE_2_3d object
         * @param[in] supportFramesPose ordered map of frame index and support frame poses as manif::SE3d object
         * @param[in] augmentedVector augmented vector manif::Rnd<n> object
         * @note the underlying matrix Lie group will be constructed maintaining an ascending order of support frame indices
         */
        FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose,
                                               const std::map<int, manif::SE3d>& supportFramesPose,
                                               Eigen::Ref<const Eigen::VectorXd> augmentedVector); 
        
        /**
         * Copy constructor
         */
        FloatingBaseExtendedKinematicsLieGroup(const FloatingBaseExtendedKinematicsLieGroup& other);        
        
        /**
         * Copy assignment operator
         */
        FloatingBaseExtendedKinematicsLieGroup operator=(const FloatingBaseExtendedKinematicsLieGroup& other);

        /**
         * Get the underlying Matrix Lie group         
         */
        Eigen::MatrixXd asMatrixLieGroup() const;

        /**
         * Left composition of FloatingBaseExtendedKinematicsLieGroup
         * If this = X1 and other = X2,
         * lcompose gives X1 o X2
         *
         * @param[in] other FloatingBaseExtendedKinematicsLieGroup
         */
        FloatingBaseExtendedKinematicsLieGroup lcompose(const FloatingBaseExtendedKinematicsLieGroup& other);

        /**
         * Right composition of FloatingBaseExtendedKinematicsLieGroup
         * If this = X1 and other = X2,
         * lcompose gives  X2 o X1
         *
         * @param[in] other FloatingBaseExtendedKinematicsLieGroup
         */
        FloatingBaseExtendedKinematicsLieGroup rcompose(const FloatingBaseExtendedKinematicsLieGroup& other);

        /**
         * Left composition of FloatingBaseExtendedKinematicsLieGroup
         * If this = X1 and other = X2,
         * lcompose gives X1 o X2
         * @warning if the support frames are not the same, they are simply ignored
         * @param[in] other FloatingBaseExtendedKinematicsLieGroup
         */
        FloatingBaseExtendedKinematicsLieGroup operator*(const FloatingBaseExtendedKinematicsLieGroup& other);

        /**
         * Inverse of FloatingBaseExtendedKinematicsLieGroup
         */
        FloatingBaseExtendedKinematicsLieGroup inverse();

        /**
         * set to identity element of IMU Biped matrix Lie group
         */
        void setIdentity();

        /**
         * Logarithm mapping of FloatingBaseExtendedKinematicsLieGroup
         */
        FloatingBaseExtendedKinematicsLieGroupTangent log();

        /**
         * Adjoint matrix of FloatingBaseExtendedKinematicsLieGroup
         */
        Eigen::MatrixXd adj();

        manif::SE_2_3d baseExtendedPose() const; /**< get extended pose of the base link*/
        manif::SE3d basePose() const; /**< get pose of the base link*/
        manif::SO3d baseRotation() const; /**< get rotation of the base link*/
        Eigen::Vector3d basePosition() const; /**< get position of the base link*/
        Eigen::Vector3d baseLinearVelocity() const; /**< get linear velocity of base link */

        int nrOfSupportFrames() const;
        std::vector<int> supportFrameIndices() const;
        bool supportFramePose(const int& idx, manif::SE3d& pose) const; /**< get pose of the support frame of index idx*/
        std::map<int, manif::SE3d> supportFramesPose() const; /**< get pose of the support frame of index idx*/
        bool supportFrameRotation(const int& idx, manif::SO3d& rotation) const; /**< get rotation of the support frame of index idx*/
        bool supportFramePosition(const int& idx, Eigen::Ref<Eigen::Vector3d> position) const; /**< get position of the support frame of index idx*/

        bool augmentedVector(Eigen::VectorXd& augVec) const; /**< get augmented vector from the  group*/
        void disableAugmentedVector(); /**< removes the augmented vector from group representation, if active */
        bool isAugmentedVectorUsed() const; /**< check if augmented vector is used **/

        void setBaseExtendedPose(const manif::SE_2_3d& baseExtendedPose); /**< set extended pose of the base link*/
        void setBasePose(const manif::SE3d& basePose); /**< set pose of the base link*/
        void setBaseRotation(const manif::SO3d& baseRotation); /**< set rotation of the base link*/
        void setBasePosition(Eigen::Ref<const Eigen::Vector3d> basePosition); /**< set position of the base link*/
        void setBaseLinearVelocity(Eigen::Ref<const Eigen::Vector3d> baseLinearVelocity); /**< set linear velocity of base link */

        bool setSupportFramePose(const int& idx, const manif::SE3d& pose); /**< set pose of existing support frame*/
        bool setSupportFrameRotation(const int& idx, const manif::SO3d& rotation); /**< set rotation of existing support frame*/
        bool setSupportFramePosition(const int& idx, Eigen::Ref<const Eigen::Vector3d> position); /**< set position of existing support frame*/
        
        bool addSupportFramePose(const int& idx, const manif::SE3d& pose); /**< add pose of a new support frame, returns false without setting if frame already exists*/
        bool removeSupportFrame(const int& idx); /**< remove pose of an existing support frame, returns false otherwise*/
 
        void setAugmentedVector(Eigen::Ref<const Eigen::VectorXd> augVec); /**< set augmented vector from the  group*/

    private:
        class Impl;
        std::unique_ptr<Impl> m_pimpl;
    };


    class FloatingBaseExtendedKinematicsLieGroupTangent
    {
    public:
        ~FloatingBaseExtendedKinematicsLieGroupTangent();

        FloatingBaseExtendedKinematicsLieGroupTangent();
        
        /**
         * Construct group velocity only with base extended state 
         * Only extended motion vector of the base will be constructed
         */
        FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec);

        /**
         * Group velocity constructor for base link with augmented vector states enabled
         * @param[in] augVecDimensions dimension of augmented vector states
         * 
         */
        FloatingBaseExtendedKinematicsLieGroupTangent(const int& augVecDimensions);

        /**
         * R9 x (R6)^k constructor 
         * 
         * @param[in] baseExtMotionVec extended motion vector of base link as manif::SE_2_3Tangentd object
         * @param[in] supportFramesTwists ordered map of frame index and support frame twist as manif::SE3Tangentd object
         * @note the underlying velocity vector will be constructed maintaining an ascending order of support frame indices
         */
        FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec,
                                                      const std::map<int, manif::SE3Tangentd>& supportFramesTwist);
 
        /**
         * R9 x (R6)^k x Rn constructor 
         * @param[in] baseExtMotionVec extended motion vector of base link as manif::SE_2_3Tangentd object
         * @param[in] supportFramesTwist ordered map of frame index and support frame twist as manif::SE3Tangentd object
         * @param[in] augmentedVector augmented vector manif::Rnd<n> object
         * @note the underlying velocity vector will be constructed maintaining an ascending order of support frame indices
         */
        FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec,
                                                      const std::map<int, manif::SE3Tangentd>& supportFramesTwist,
                                                      Eigen::Ref<const Eigen::VectorXd> augmentedVector);
        
        /**
         * Copy constructor
         */
        FloatingBaseExtendedKinematicsLieGroupTangent(const FloatingBaseExtendedKinematicsLieGroupTangent& other);        
        
        /**
         * Copy assignment operator
         */
        FloatingBaseExtendedKinematicsLieGroupTangent operator=(const FloatingBaseExtendedKinematicsLieGroupTangent& other);


        Eigen::VectorXd toVector();
        FloatingBaseExtendedKinematicsLieGroupTangent operator+(const FloatingBaseExtendedKinematicsLieGroupTangent& other);
        FloatingBaseExtendedKinematicsLieGroupTangent operator-(const FloatingBaseExtendedKinematicsLieGroupTangent& other);
        FloatingBaseExtendedKinematicsLieGroupTangent operator-();
        void setZero();

        Eigen::MatrixXd hat();
        FloatingBaseExtendedKinematicsLieGroup exp();
        Eigen::MatrixXd rjac();
        Eigen::MatrixXd rjacinv();
        Eigen::MatrixXd ljac();
        Eigen::MatrixXd ljacinv();
        Eigen::MatrixXd smallAdj();

        manif::SE_2_3Tangentd baseExtenedMotionVector() const; /**< get extended motion vector of the base link*/
        manif::SE3Tangentd baseTwist() const; /**< get twist of the base link*/
        Eigen::Vector3d baseLinearVelocity() const; /**< get linear velocity of the base link*/
        Eigen::Vector3d baseAngularVelocity() const; /**< get angular velocity of the base link*/
        Eigen::Vector3d baseLinearAcceleration() const; /**< get linear acceleration of base link */
        
        std::size_t nrOfSupportFrames() const;
        std::vector<int> supportFrameIndices() const;
        bool supportFrameTwist(const int& idx, manif::SE3Tangentd& twist) const; /**< get twist of the support frame of index idx*/
        std::map<int, manif::SE3Tangentd> supportFramesTwist() const; /**< get pose of the support frame of index idx*/
        bool supportFrameLinearVelocity(const int& idx, Eigen::Ref<Eigen::Vector3d> linearVelocity) const; /**< get linear velocity of the support frame of index idx*/
        bool supportFrameAngularVelocity(const int& idx, Eigen::Ref<Eigen::Vector3d> angularVelocity) const; /**< get angular velocity of the support frame of index idx*/
        
        bool augmentedVector(Eigen::VectorXd& augVec) const; /**< get augmented vector from the  group*/
        void disableAugmentedVector(); /**< removes the augmented vector from group representation, if active */
        bool isAugmentedVectorUsed() const; /**< check if augmented vector is used **/
 
        void setBaseExtendedMotionVector(const manif::SE_2_3Tangentd& baseExtenedMotionVector);  /**< set extended motion vector of the base link*/
        void setBaseTwist(const manif::SE3Tangentd& baseTwist);  /**< set twist of the base link*/
        void setBaseLinearVelocity(Eigen::Ref<const Eigen::Vector3d> baseLinearVelocity); /**< set linear velocity of the base link*/
        void setBaseAngularVelocity(Eigen::Ref<const Eigen::Vector3d> baseAngularVelocity); /**< set angular velocity of the base link*/
        void setBaseLinearAcceleration(Eigen::Ref<const Eigen::Vector3d> baseLinearAcceleration); /**< set linear acceleration of base link */

        bool setSupportFrameTwist(const int& idx, const manif::SE3Tangentd& twist); /**< set twist of existing support frame*/
        bool setSupportFrameLinearVelocity(const int& idx, Eigen::Ref<const Eigen::Vector3d> linearVelocity); /**< set linear velocity of existing support frame*/
        bool setSupportFrameAngularVelocity(const int& idx, Eigen::Ref<const Eigen::Vector3d> angularVelocity); /**< set angular velocity of existing support frame*/
        
        bool addSupportFrameTwist(const int& idx, const manif::SE3Tangentd& twist); /**< add twist of a new support frame, returns false without setting if frame already exists*/
        bool removeSupportFrameTwist(const int& idx); /**< remove twist of an existing support frame, returns false otherwise*/
 
        void setAugmentedVector(Eigen::Ref<const Eigen::VectorXd> augVec); /**< set augmented vector from the group velocity*/
        
        std::size_t size() const;
        
        private:
        class Impl;
        std::unique_ptr<Impl> m_pimpl;
    };


} // namespace Estimators
} // namespace BipedalLocomotion
#endif // BIPEDAL_LOCOMOTION_FB_EXT_KINEMATICS_MATRIX_LIEGROUP_H
