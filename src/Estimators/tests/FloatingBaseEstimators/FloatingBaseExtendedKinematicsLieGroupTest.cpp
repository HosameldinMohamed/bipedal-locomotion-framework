/**
 * @file FloatingBaseExtendedKinematicsLieGroupTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseExtendedKinematicsLieGroup.h>


#include <Eigen/Dense>

using namespace BipedalLocomotion::Estimators;

bool isPoseIdentity(manif::SE3d& c2)
{
    manif::SE3d c1;
    c1.Identity();
    return c1.coeffs().isApprox(c2.coeffs());
}

bool checkZeroVector(const Eigen::VectorXd& v)
{
    Eigen::VectorXd zero(v.size());
    zero.setZero();
    return zero.isApprox(v);
}

manif::SE3d getIdentityPose()
{
    manif::SE3d pose;
    pose.Identity();
    return pose;
}

manif::SE3d getRandomPose()
{
    auto pose = manif::SE3d::Random();
    return pose;
}

manif::SO3d getRandomRotation()
{
    auto rot = manif::SO3d::Random();
    return rot;
}

Eigen::VectorXd getRandomXd(const int& n)
{
    Eigen::VectorXd x = Eigen::VectorXd::Random(n);
    return x;
}

bool isRotationEqual(const manif::SO3d& a, const manif::SO3d& b)
{
   return a.coeffs().isApprox(b.coeffs());
}


bool isVecEqual(Eigen::Ref<const Eigen::VectorXd> a, Eigen::Ref<const Eigen::VectorXd> b)
{
    return a.isApprox(b);
}

bool isMatrixEqual(Eigen::Ref<const Eigen::MatrixXd> a, Eigen::Ref<const Eigen::MatrixXd> b)
{
    return a.isApprox(b);
}


TEST_CASE("Group default constructor")
{
    FloatingBaseExtendedKinematicsLieGroup X;
    REQUIRE(X.nrOfSupportFrames() == 0);
    auto basePose = X.basePose();
    auto baseVel = X.baseLinearVelocity();
    REQUIRE(isPoseIdentity(basePose));
    REQUIRE(checkZeroVector(baseVel));
    REQUIRE(!X.isAugmentedVectorUsed());
}

TEST_CASE("Group augmented vector constructor")
{
    int augVecDim = 6;
    FloatingBaseExtendedKinematicsLieGroup X(augVecDim);
    REQUIRE(X.nrOfSupportFrames() == 0);

    REQUIRE(X.isAugmentedVectorUsed());
    Eigen::VectorXd augVec;
    REQUIRE(X.augmentedVector(augVec));
    REQUIRE(augVec.size() == augVecDim);
}

TEST_CASE("Group constructor with support frame")
{
    auto pose1 = getRandomPose();
    auto pose2 = getRandomPose();

    Eigen::Vector3d pos1 = pose1.translation();
    Eigen::Vector3d pos2 = pose2.translation();    
    
    // add poses for index 1 and 10
    std::map<int, manif::SE3d> poseMap;
    poseMap[1] = pose1;
    poseMap[10] = pose2;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap);
    
    // nr of support frames should be 2
    REQUIRE(X.nrOfSupportFrames() == 2);
    
    Eigen::Vector3d out1;
    // index 2 should not be available
    REQUIRE(!X.supportFramePosition(2, out1));
    
    REQUIRE(X.supportFramePosition(1, out1));
    REQUIRE(isVecEqual(out1, pos1));
    REQUIRE(X.supportFramePosition(10, out1));
    REQUIRE(isVecEqual(out1, pos2));
    
    manif::SO3d outRot1;
    REQUIRE(X.supportFrameRotation(1, outRot1));
    REQUIRE(isRotationEqual(outRot1, pose1.asSO3()));
}

TEST_CASE("Group constructor with support frame empty map")
{
    // add poses for index 1 and 10
    std::map<int, manif::SE3d> poseMap;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap);
    
    // nr of support frames should be 0
    REQUIRE(X.nrOfSupportFrames() == 0);
}

TEST_CASE("Group constructor with support frame and augmented vector")
{
    auto pose1 = getRandomPose();
    std::map<int, manif::SE3d> poseMap;
    poseMap[1] = pose1;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    Eigen::VectorXd augVec = getRandomXd(6);
    
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap, augVec);
    REQUIRE(X.nrOfSupportFrames() == 1);
    REQUIRE(X.isAugmentedVectorUsed());
    
    Eigen::VectorXd augVecOut;
    REQUIRE(X.augmentedVector(augVecOut));
    REQUIRE(isVecEqual(augVecOut, augVec));
}

TEST_CASE("Group copy constructor and copy assignment Operator")
{
    auto pose1 = getRandomPose();
    std::map<int, manif::SE3d> poseMap;
    poseMap[1] = pose1;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    Eigen::VectorXd augVec = getRandomXd(6);
    
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap, augVec);
    
    Eigen::VectorXd xAug, x2Aug, x3Aug;
    REQUIRE(X.augmentedVector(xAug));
    auto X2(X);
    REQUIRE(X2.augmentedVector(x2Aug));
    REQUIRE(isVecEqual(xAug, x2Aug));
    
    auto X3 = X;
    REQUIRE(X2.augmentedVector(x3Aug));
    REQUIRE(isVecEqual(xAug, x3Aug));
    
    auto pose = getRandomPose();
    REQUIRE(X.addSupportFramePose(2, pose));
    auto X4 = X;
    
    REQUIRE(X.nrOfSupportFrames() == 2);
    REQUIRE(X4.nrOfSupportFrames() == 2);
}

TEST_CASE("Group representation as matrix Lie group")
{
    auto pose1 = getRandomPose();
    auto pose2 = getRandomPose();
    
    // add poses for index 1 and 10
    std::map<int, manif::SE3d> poseMap;
    poseMap[1] = pose1;
    poseMap[10] = pose2;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    Eigen::VectorXd augVec = getRandomXd(6);
    
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap, augVec);
    Eigen::MatrixXd G =  X.asMatrixLieGroup();
    REQUIRE(G.rows() == 20);
    REQUIRE(G.cols() == 20);
}

TEST_CASE("Group inverse")
{
    auto rot = getRandomRotation();
    
    Eigen::Vector3d t;
    t << 0,0,0;
    manif::SE_2_3d baseExtPose(t, rot, t);
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose);
    auto Xinv = X.inverse();
    
    REQUIRE(isRotationEqual(rot.inverse(), Xinv.baseRotation()));    
    
    auto pose1 = getRandomPose();
    X.addSupportFramePose(10, pose1);
    auto X2inv = X.inverse();
    Eigen::Vector3d pos1out;
    REQUIRE(X2inv.supportFramePosition(10, pos1out));
    REQUIRE(isVecEqual(pose1.inverse().translation(), pos1out));
}

TEST_CASE("Exp and Log operators")
{
    auto pose1 = getRandomPose();
    std::map<int, manif::SE3d> poseMap;
    poseMap[1] = pose1;
    
    manif::SE_2_3d baseExtPose;
    baseExtPose.Identity();
    
    Eigen::VectorXd augVec = getRandomXd(6);
    FloatingBaseExtendedKinematicsLieGroup X(baseExtPose, poseMap, augVec);    
    auto v = X.log();
    Eigen::MatrixXd vhat = v.hat();
    // vector dimensions 
    REQUIRE(v.size() == 9+6+6);
    // matrix Lie algebra shape
    REQUIRE(vhat.rows() == 5+4+7);
    REQUIRE(vhat.cols() == 5+4+7);

    FloatingBaseExtendedKinematicsLieGroupTangent vel(baseExtPose.log());
    vel.setAugmentedVector(augVec);            
    
    // Please make not of the following failure case
    // even if you add a different frame index related twist,
    // the underlying mathematical object (matrix formation)
    // will be the same as v
    // in this case desired matrix Lie algebra comparison would be false
    // but since the frame semantics are not considered
    // the comparison turns out to be true
    REQUIRE(vel.addSupportFrameTwist(/*idx=*/ 2, pose1.log()));
    REQUIRE(isMatrixEqual(vhat, vel.hat()));    
    
    REQUIRE(vel.addSupportFrameTwist(/*idx=*/ 1, pose1.log()));
    REQUIRE(!isMatrixEqual(vhat, vel.hat()));
    REQUIRE(vel.removeSupportFrameTwist(/*idx=*/ 2));
    REQUIRE(!vel.addSupportFrameTwist(/*idx=*/ 1, pose1.log()));
    REQUIRE(isMatrixEqual(vhat, vel.hat()));    
    
    auto vExp = vel.exp();
    REQUIRE(isMatrixEqual(X.asMatrixLieGroup(), vExp.asMatrixLieGroup()));
}

TEST_CASE("Vector operations")
{
    Eigen::VectorXd v1B = getRandomXd(9);
    Eigen::VectorXd v2B = getRandomXd(9);
    auto v1Base = manif::SE_2_3Tangentd(v1B);
    auto v2Base = manif::SE_2_3Tangentd(v2B);
    
    Eigen::VectorXd v1F = getRandomXd(6);
    Eigen::VectorXd v2F = getRandomXd(6);
    std::map<int, manif::SE3Tangentd> v1FMap;
    auto v1Foot1 = manif::SE3Tangentd(v1F);
    v1FMap[1] = v1Foot1;
    
    std::map<int, manif::SE3Tangentd> v2FMap;    
    auto v2Foot1 = manif::SE3Tangentd(v2F);
    v2FMap[1] = v2Foot1;
    
    // in case we add another support frame twist to v2FMap 
    // and use the bilinear operations of + and -
    // due to unavailability of that frame in v1Map
    // the operation for the corresponding frame will
    // simply be ignored and will be carried out 
    // to only frames that are available in both the maps
    
    FloatingBaseExtendedKinematicsLieGroupTangent v1(v1Base, v1FMap);
    FloatingBaseExtendedKinematicsLieGroupTangent v2(v2Base, v2FMap);
    
    REQUIRE(isVecEqual(-v1.baseExtenedMotionVector().coeffs(), -v1B));
    
    auto v3 = v1 + v2;
    REQUIRE(isVecEqual(v3.baseExtenedMotionVector().coeffs(), v1B + v2B));
    
    auto v4 = v1 - v2;
    manif::SE3Tangentd v4Foot1;
    REQUIRE(v4.supportFrameTwist(1, v4Foot1));
    REQUIRE(isVecEqual(v4Foot1.coeffs(), v1F - v2F));
    
    REQUIRE(v1.ljac().rows() == 9+6);
}
