/**
 * @file FloatingBaseExtendedKinematicsLieGroup.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseExtendedKinematicsLieGroup.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Conversions;

class FloatingBaseExtendedKinematicsLieGroup::Impl
{
public:
    manif::SE_2_3d baseExtPose; /**< Internal state base extended pose*/
    
    std::map<int, manif::SE3d> supportFramePose; /**< Internal state support frame pose*/
    std::vector<int> supportFrameIndices; /**< Internal suport frame indices */
    
    Eigen::VectorXd augmentedVector; /**< Internal state augmented vector*/
    bool isAugmentedVectorUsed{false}; /**< Internal flag to check activation of augmented vector states*/
    
    Eigen::MatrixXd X;    /**< Matrix Lie group*/
    Eigen::MatrixXd AdjX; /**< Adjoint matrix representation of the group*/
    
    const std::size_t extPoseSize{5};
    const std::size_t extPoseDim{9};
    
    const std::size_t poseSize{4};
    const std::size_t poseDim{6};
    
    bool checkValidSupportFrame(const int& idx)
    {
        if (supportFramePose.find(idx) == supportFramePose.end())
        {            
            return false;
        }        
        return true;
    }
    
    void clearSupportFrames()
    {
        supportFramePose.clear();
        supportFrameIndices.clear();
    }
    
    FloatingBaseExtendedKinematicsLieGroup compose(const FloatingBaseExtendedKinematicsLieGroup& other, std::string& leftOrRight)
    {
        auto indices = other.supportFrameIndices();

        std::map<int, manif::SE3d> supportFrameComposition;
        for (auto idx : indices)
        {
            if (checkValidSupportFrame(idx))
            {
                manif::SE3d otherPose;
                other.supportFramePose(idx, otherPose);
                if (leftOrRight == "left")
                {
                    supportFrameComposition[idx] = supportFramePose.at(idx)*otherPose;            
                }
                else if (leftOrRight == "right")
                {
                    supportFrameComposition[idx] = otherPose*supportFramePose.at(idx);
                }
            }        
        }
        
        manif::SE_2_3d baseExtPosCompose;
        if (leftOrRight == "left")
        {
            baseExtPosCompose = baseExtPose*other.baseExtendedPose();            
        }
        else if (leftOrRight == "right")
        {
            baseExtPosCompose = other.baseExtendedPose()*baseExtPose;
        }
        
        if (other.isAugmentedVectorUsed() && isAugmentedVectorUsed)
        {
            Eigen::VectorXd otherAug;
            other.augmentedVector(otherAug);
            return FloatingBaseExtendedKinematicsLieGroup(baseExtPosCompose,
                                                          supportFrameComposition,
                                                          augmentedVector + otherAug);
        }
    
        return FloatingBaseExtendedKinematicsLieGroup(baseExtPosCompose,
                                                      supportFrameComposition);        
    }        
};

FloatingBaseExtendedKinematicsLieGroup::~FloatingBaseExtendedKinematicsLieGroup()
{
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose.setIdentity();
    m_pimpl->clearSupportFrames();
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = baseExtPose;
    m_pimpl->clearSupportFrames();
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup(const int& augVecDimensions)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose.setIdentity();
    m_pimpl->clearSupportFrames();

    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->augmentedVector.resize(augVecDimensions);
    m_pimpl->augmentedVector.setZero();
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose,
                                                                               const std::map<int, manif::SE3d>& supportFramesPose)  
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = baseExtPose;
    
    m_pimpl->clearSupportFrames();
    m_pimpl->supportFramePose = supportFramesPose;    
    for (auto& supportFrame : m_pimpl->supportFramePose)
    {
        m_pimpl->supportFrameIndices.emplace_back(supportFrame.first);
    }
    
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup(const manif::SE_2_3d& baseExtPose,
                                                                               const std::map<int, manif::SE3d>& supportFramesPose,
                                                                               Eigen::Ref<const Eigen::VectorXd> augmentedVector)  
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = baseExtPose;
    
    m_pimpl->clearSupportFrames();
    m_pimpl->supportFramePose = supportFramesPose;    
    for (auto& supportFrame : m_pimpl->supportFramePose)
    {
        m_pimpl->supportFrameIndices.emplace_back(supportFrame.first);
    }
    
    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->augmentedVector = augmentedVector;
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroup::asMatrixLieGroup() const
{       
    std::size_t size; 
    size = m_pimpl->extPoseSize + 
        (m_pimpl->poseSize * nrOfSupportFrames());
        
    if (m_pimpl->isAugmentedVectorUsed)
    {
        size += (m_pimpl->augmentedVector.size() + 1);
    }
    
    m_pimpl->X.resize(size, size);
    m_pimpl->X.setIdentity();
    m_pimpl->X.topLeftCorner<5, 5>() = m_pimpl->baseExtPose.isometry();
    
    int idx = 0;
    for (auto& iter : m_pimpl->supportFramePose)
    {
        int q = m_pimpl->extPoseSize + (m_pimpl->poseSize*idx);
        m_pimpl->X.block<4, 4>(q, q)= iter.second.transform();  
        idx++;
    }

    if (m_pimpl->isAugmentedVectorUsed)
    {
        m_pimpl->X.bottomRightCorner(m_pimpl->augmentedVector.size() + 1, 1) << m_pimpl->augmentedVector, 1;
    }

    return m_pimpl->X;
}

FloatingBaseExtendedKinematicsLieGroup::FloatingBaseExtendedKinematicsLieGroup(const FloatingBaseExtendedKinematicsLieGroup& other)
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = other.baseExtendedPose();
      
    m_pimpl->isAugmentedVectorUsed = other.isAugmentedVectorUsed();
    if (m_pimpl->isAugmentedVectorUsed)
    {
        other.augmentedVector(m_pimpl->augmentedVector);
    }
    
    m_pimpl->supportFramePose = other.supportFramesPose();
    m_pimpl->supportFrameIndices = other.supportFrameIndices();

}

FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroup::operator=(const FloatingBaseExtendedKinematicsLieGroup& other)
{
    return FloatingBaseExtendedKinematicsLieGroup(other);    
}

void FloatingBaseExtendedKinematicsLieGroup::setBaseExtendedPose(const manif::SE_2_3d& baseExtenedPose)
{
    m_pimpl->baseExtPose = baseExtenedPose;
}

void FloatingBaseExtendedKinematicsLieGroup::setBasePose(const manif::SE3d& basePose)
{
    m_pimpl->baseExtPose = manif::SE_2_3d(basePose.translation(),
                                          basePose.quat(),
                                          m_pimpl->baseExtPose.linearVelocity());
}

void FloatingBaseExtendedKinematicsLieGroup::setBaseRotation(const manif::SO3d& baseRotation)
{
    m_pimpl->baseExtPose = manif::SE_2_3d(m_pimpl->baseExtPose.translation(),
                                          baseRotation,
                                          m_pimpl->baseExtPose.linearVelocity());
}

void FloatingBaseExtendedKinematicsLieGroup::setBasePosition(Eigen::Ref<const Eigen::Vector3d> basePosition)
{
    m_pimpl->baseExtPose = manif::SE_2_3d(basePosition,
                                        m_pimpl->baseExtPose.quat(),
                                        m_pimpl->baseExtPose.linearVelocity());
}

void FloatingBaseExtendedKinematicsLieGroup::setBaseLinearVelocity(Eigen::Ref<const Eigen::Vector3d> baseLinearVelocity)
{
    m_pimpl->baseExtPose = manif::SE_2_3d(m_pimpl->baseExtPose.translation(),
                                          m_pimpl->baseExtPose.quat(),
                                          baseLinearVelocity);
}

void FloatingBaseExtendedKinematicsLieGroup::disableAugmentedVector()
{
    m_pimpl->isAugmentedVectorUsed = false;    
}

void FloatingBaseExtendedKinematicsLieGroup::setAugmentedVector(Eigen::Ref<const Eigen::VectorXd> augVec)
{
    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->augmentedVector = augVec;
}

bool FloatingBaseExtendedKinematicsLieGroup::setSupportFramePose(const int& idx, const manif::SE3d& pose)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::setSupportFramePose] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    
    m_pimpl->supportFramePose.at(idx) = pose;
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroup::setSupportFramePosition(const int& idx, Eigen::Ref<const Eigen::Vector3d> position)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::setSupportFramePosition] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    auto& pose = m_pimpl->supportFramePose.at(idx);
    pose.translation(position);
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroup::setSupportFrameRotation(const int& idx, const manif::SO3d& rotation)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::setSupportFrameRotation] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    auto& pose = m_pimpl->supportFramePose.at(idx);
    pose.quat(rotation.quat());
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroup::addSupportFramePose(const int& idx, const manif::SE3d& pose)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::addSupportFramePose] "};
    
    if (m_pimpl->supportFramePose.find(idx) == m_pimpl->supportFramePose.end())
    {
        m_pimpl->supportFramePose[idx] = pose;
        m_pimpl->supportFrameIndices.emplace_back(idx);
        std::sort(m_pimpl->supportFrameIndices.begin(), m_pimpl->supportFrameIndices.end());
        return true;
    }
    
    std::cerr << printPrefix << "Frame already exists. use set method to change the frame pose." << std::endl;
    return false;
}

bool FloatingBaseExtendedKinematicsLieGroup::removeSupportFrame(const int& idx)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::removeSupportFrame] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    
    m_pimpl->supportFramePose.erase(idx);
    m_pimpl->supportFrameIndices.erase(std::remove(m_pimpl->supportFrameIndices.begin(), 
                                                   m_pimpl->supportFrameIndices.end(), 
                                                   idx), 
                                       m_pimpl->supportFrameIndices.end());
    return true;
}

manif::SE_2_3d FloatingBaseExtendedKinematicsLieGroup::baseExtendedPose() const
{
    return m_pimpl->baseExtPose;
}

manif::SE3d FloatingBaseExtendedKinematicsLieGroup::basePose() const
{
    return manif::SE3d(m_pimpl->baseExtPose.translation(), m_pimpl->baseExtPose.quat());
}

manif::SO3d FloatingBaseExtendedKinematicsLieGroup::baseRotation() const
{
    return manif::SO3d(m_pimpl->baseExtPose.quat());
}

Eigen::Vector3d FloatingBaseExtendedKinematicsLieGroup::basePosition() const
{
    return m_pimpl->baseExtPose.translation();
}

Eigen::Vector3d FloatingBaseExtendedKinematicsLieGroup::baseLinearVelocity() const
{
    return m_pimpl->baseExtPose.linearVelocity();
}

bool FloatingBaseExtendedKinematicsLieGroup::isAugmentedVectorUsed() const
{
    return m_pimpl->isAugmentedVectorUsed;
}

bool FloatingBaseExtendedKinematicsLieGroup::augmentedVector(Eigen::VectorXd& augVec) const
{
    std::string_view printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::augmentedVector] "};
    if (!m_pimpl->isAugmentedVectorUsed)
    {
        std::cerr << printPrefix << "Augmented vector not available in the group representation" << std::endl;
        return false;
    }
    
    augVec = m_pimpl->augmentedVector;
    return true;
}

int FloatingBaseExtendedKinematicsLieGroup::nrOfSupportFrames() const
{
    return m_pimpl->supportFramePose.size();
}

std::vector<int> FloatingBaseExtendedKinematicsLieGroup::supportFrameIndices() const
{
    return m_pimpl->supportFrameIndices;
}

bool FloatingBaseExtendedKinematicsLieGroup::supportFramePose(const int& idx, manif::SE3d& pose) const
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::supportFrameRotation] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    
    pose = m_pimpl->supportFramePose.at(idx);
        
    return true;
}

std::map<int, manif::SE3d> FloatingBaseExtendedKinematicsLieGroup::supportFramesPose() const
{
    return m_pimpl->supportFramePose;
}


bool FloatingBaseExtendedKinematicsLieGroup::supportFramePosition(const int& idx, Eigen::Ref<Eigen::Vector3d> position) const 
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::supportFrameRotation] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    
    position = m_pimpl->supportFramePose.at(idx).translation();
    return true;    
}

bool FloatingBaseExtendedKinematicsLieGroup::supportFrameRotation(const int& idx, manif::SO3d& rotation) const 
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::supportFrameRotation] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {       
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the group representation" << std::endl;
        return false;
    }
    
    rotation = m_pimpl->supportFramePose.at(idx).asSO3();
    return true;    
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroup::adj()
{    
    int size = m_pimpl->extPoseDim + (m_pimpl->poseDim*nrOfSupportFrames());
    if (m_pimpl->isAugmentedVectorUsed)
    {
        size += m_pimpl->augmentedVector.size();
    }
    m_pimpl->AdjX.resize(size, size);
    m_pimpl->AdjX.setIdentity();
    
    m_pimpl->AdjX.topLeftCorner<9, 9>() = m_pimpl->baseExtPose.adj();
    
    int idx = 0;
    for (auto& iter : m_pimpl->supportFramePose)
    {
        int q = m_pimpl->extPoseDim + (m_pimpl->poseDim*idx);
        m_pimpl->X.block<6, 6>(q, q) = iter.second.adj();  
        idx++;
    }
    
    // bias related Adjoint is identity

    return m_pimpl->AdjX;
}

void FloatingBaseExtendedKinematicsLieGroup::setIdentity()
{
    m_pimpl->baseExtPose.setIdentity();
    for (auto& iter : m_pimpl->supportFramePose)
    {
        iter.second.setIdentity();
    }

    if (m_pimpl->isAugmentedVectorUsed && m_pimpl->augmentedVector.size() > 0)
    {
        m_pimpl->augmentedVector.setZero();
    }
}

FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroup::inverse()
{        
    std::map<int, manif::SE3d> invSupportPose;
    for (auto& iter : m_pimpl->supportFramePose)
    {
        invSupportPose[iter.first] = iter.second.inverse();
    }
    
    if (m_pimpl->isAugmentedVectorUsed)
    {
        return FloatingBaseExtendedKinematicsLieGroup(m_pimpl->baseExtPose.inverse(),
                                                      invSupportPose,
                                                     -m_pimpl->augmentedVector);
    }
    
    return FloatingBaseExtendedKinematicsLieGroup(m_pimpl->baseExtPose.inverse(),
                                                  invSupportPose);
}



FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroup::operator*(const FloatingBaseExtendedKinematicsLieGroup& other)
{
    return lcompose(other);
}

FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroup::lcompose(const FloatingBaseExtendedKinematicsLieGroup& other)
{
    std::string left{"left"};
    return m_pimpl->compose(other, left);
}

FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroup::rcompose(const FloatingBaseExtendedKinematicsLieGroup& other)
{
    std::string right{"right"};
    return m_pimpl->compose(other, right);
}

FloatingBaseExtendedKinematicsLieGroupTangent FloatingBaseExtendedKinematicsLieGroup::log()
{
    auto vB = m_pimpl->baseExtPose.log();
    std::map<int, manif::SE3Tangentd> vF;
    for (auto& [idx, pose] : m_pimpl->supportFramePose)
    {
        vF[idx] = pose.log();
    }

    if (m_pimpl->isAugmentedVectorUsed)
    {
        return FloatingBaseExtendedKinematicsLieGroupTangent(vB, vF, m_pimpl->augmentedVector);
    }

    return FloatingBaseExtendedKinematicsLieGroupTangent(vB, vF);
}


class FloatingBaseExtendedKinematicsLieGroupTangent::Impl
{
public:
    manif::SE_2_3Tangentd vBase; /**< Internal base extended motion vector*/
    
    std::map<int, manif::SE3Tangentd> vF; /**< Internal state support frame twists*/
    std::vector<int> fIndices; /**< Internal suport frame indices */

    Eigen::VectorXd vAugmented; /**< Internal augmented vector */
    bool isAugmentedVectorUsed{false}; /**< Internal flag to check activation of augmented vector*/
    
    Eigen::MatrixXd vHat;      /**< Matrix Lie algebra */
    Eigen::MatrixXd Jl;        /**< Left Jacobian of group*/
    Eigen::MatrixXd Jlinv;     /**< Left Jacobian inverse of group */
    Eigen::MatrixXd Jr;        /**< Right Jacobian of group */
    Eigen::MatrixXd Jrinv;     /**< Right Jacobian inverse of group */
    Eigen::MatrixXd smallAdj;  /**< small adjoint matrix of group */
    const std::size_t extPoseSize{5};
    const std::size_t poseSize{4};
    const std::size_t extMotionDim{9};
    const std::size_t twistDim{6};  
    
    bool checkValidSupportFrame(const int& idx)
    {
        if (vF.find(idx) == vF.end())
        {            
            return false;
        }        
        return true;
    }
    
    void clearSupportFrames()
    {
        vF.clear();
        fIndices.clear();
    }
    
    FloatingBaseExtendedKinematicsLieGroupTangent operation(const FloatingBaseExtendedKinematicsLieGroupTangent& other, std::string& plusOrMinus)
    {
        manif::SE_2_3Tangentd vB;
        auto indices = other.supportFrameIndices();
        if (plusOrMinus == "plus")
        {
            vB = vBase + other.baseExtenedMotionVector();
        }
        else if (plusOrMinus == "minus")
        {
            vB = vBase - other.baseExtenedMotionVector();
        }
        
        std::map<int, manif::SE3Tangentd> outTwist;
        for (auto idx : indices)
        {
            if (checkValidSupportFrame(idx))
            {
                manif::SE3Tangentd otherTwist;
                other.supportFrameTwist(idx, otherTwist);
                if (plusOrMinus == "plus")
                {
                    outTwist[idx] = vF.at(idx) + otherTwist;
                }
                else if (plusOrMinus == "minus")
                {
                    outTwist[idx] = vF.at(idx) - otherTwist;
                }                
            }
        }

        if (isAugmentedVectorUsed)
        {
            Eigen::VectorXd otherAug, outAug;
            other.augmentedVector(otherAug);
            if (plusOrMinus == "plus")
            {
                outAug = vAugmented + otherAug;
            }
            else if (plusOrMinus == "minus")
            {
                outAug = vAugmented - otherAug;
            }
            return FloatingBaseExtendedKinematicsLieGroupTangent(vB,
                                                                 outTwist,
                                                                 outAug);
        }
        return FloatingBaseExtendedKinematicsLieGroupTangent(vB,
                                                             outTwist);
    }        
};

FloatingBaseExtendedKinematicsLieGroupTangent::~FloatingBaseExtendedKinematicsLieGroupTangent()
{

}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase.setZero();
    m_pimpl->clearSupportFrames();
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = baseExtMotionVec;
    m_pimpl->clearSupportFrames();
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent(const int& augVecDimensions)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase.setZero();
    m_pimpl->clearSupportFrames();

    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->vAugmented.resize(augVecDimensions);
    m_pimpl->vAugmented.setZero();
}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec,
                                                                                             const std::map<int, manif::SE3Tangentd>& supportFramesTwist)  
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = baseExtMotionVec;
    
    m_pimpl->clearSupportFrames();
    m_pimpl->vF = supportFramesTwist;    
    for (auto& supportFrame : m_pimpl->vF)
    {
        m_pimpl->fIndices.emplace_back(supportFrame.first);
    }
    
    m_pimpl->isAugmentedVectorUsed = false;
}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent(const manif::SE_2_3Tangentd& baseExtMotionVec,
                                                                                             const std::map<int, manif::SE3Tangentd>& supportFramesTwist,
                                                                                             Eigen::Ref<const Eigen::VectorXd> augmentedVector)  
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = baseExtMotionVec;
    
    m_pimpl->clearSupportFrames();
    m_pimpl->vF = supportFramesTwist;    
    for (auto& supportFrame : m_pimpl->vF)
    {
        m_pimpl->fIndices.emplace_back(supportFrame.first);
    }
    
    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->vAugmented = augmentedVector;
}

FloatingBaseExtendedKinematicsLieGroupTangent::FloatingBaseExtendedKinematicsLieGroupTangent(const FloatingBaseExtendedKinematicsLieGroupTangent& other)
: m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = other.baseExtenedMotionVector();
     
    m_pimpl->isAugmentedVectorUsed = other.isAugmentedVectorUsed();
    if (m_pimpl->isAugmentedVectorUsed)
    {
        other.augmentedVector(m_pimpl->vAugmented);
    }
    
    m_pimpl->vF = other.supportFramesTwist();
    m_pimpl->fIndices = other.supportFrameIndices();
}

FloatingBaseExtendedKinematicsLieGroupTangent FloatingBaseExtendedKinematicsLieGroupTangent::operator=(const FloatingBaseExtendedKinematicsLieGroupTangent& other)
{
    return FloatingBaseExtendedKinematicsLieGroupTangent(other);    
}

std::size_t FloatingBaseExtendedKinematicsLieGroupTangent::size() const
{
    std::size_t _size{0}; 
    _size = m_pimpl->extMotionDim + 
        (m_pimpl->twistDim * nrOfSupportFrames());
        
    if (m_pimpl->isAugmentedVectorUsed)
    {
        _size += m_pimpl->vAugmented.size();
    }
    return _size;
}

Eigen::VectorXd FloatingBaseExtendedKinematicsLieGroupTangent::toVector()
{
    Eigen::VectorXd v;
    v.resize(size());
    
    v.head<9>() = m_pimpl->vBase.coeffs();
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        v.segment<6>(q)= iter.second.coeffs();  
        idx++;
    } 
        
    if (m_pimpl->isAugmentedVectorUsed)
    {        
        v.tail(m_pimpl->vAugmented.size()) = m_pimpl->vAugmented;
    }

    return v;
}

FloatingBaseExtendedKinematicsLieGroupTangent FloatingBaseExtendedKinematicsLieGroupTangent::operator-()
{
    std::map<int, manif::SE3Tangentd> invTwist;
    for (auto& [idx, v] : m_pimpl->vF)
    {
        invTwist[idx] = -v;
    }
    
    if (m_pimpl->isAugmentedVectorUsed)
    {
        return FloatingBaseExtendedKinematicsLieGroupTangent(-m_pimpl->vBase,
                                                             invTwist,
                                                             -m_pimpl->vAugmented);
    }

    return FloatingBaseExtendedKinematicsLieGroupTangent(-m_pimpl->vBase, invTwist);
}

FloatingBaseExtendedKinematicsLieGroupTangent FloatingBaseExtendedKinematicsLieGroupTangent::operator+(const FloatingBaseExtendedKinematicsLieGroupTangent& other)
{
    std::string plusOrMinus{"plus"};
    return m_pimpl->operation(other, plusOrMinus);
}

FloatingBaseExtendedKinematicsLieGroupTangent FloatingBaseExtendedKinematicsLieGroupTangent::operator-(const FloatingBaseExtendedKinematicsLieGroupTangent& other)
{
    std::string plusOrMinus{"minus"};
    return m_pimpl->operation(other, plusOrMinus);
}


void FloatingBaseExtendedKinematicsLieGroupTangent::setZero()
{
    m_pimpl->vBase.setZero();
    for (auto& iter : m_pimpl->vF)
    {
        iter.second.setZero();
    }

    if (m_pimpl->isAugmentedVectorUsed && m_pimpl->vAugmented.size() > 0)
    {
        m_pimpl->vAugmented.setZero();
    }
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::hat()
{
    auto matSize = m_pimpl->extPoseSize + 
                            (m_pimpl->poseSize * nrOfSupportFrames());
        
    if (m_pimpl->isAugmentedVectorUsed)
    {
        if (m_pimpl->vAugmented.size() > 0)
        {
            matSize += (m_pimpl->vAugmented.size() + 1);
        }
    }
    
    m_pimpl->vHat.resize(matSize, matSize);
    m_pimpl->vHat.setZero();
    m_pimpl->vHat.topLeftCorner<5, 5>() = m_pimpl->vBase.hat();
    
    int idx{0};
    for (auto& iter : m_pimpl->vF)
    {
        int q = static_cast<int>(m_pimpl->extPoseSize) + (static_cast<int>(m_pimpl->poseSize)*idx);
        m_pimpl->vHat.block<4, 4>(q, q)= iter.second.hat();  
        idx++;
    }

    if (m_pimpl->isAugmentedVectorUsed)
    {
        if (m_pimpl->vAugmented.size() > 0)
        {
            m_pimpl->vHat.bottomRightCorner(m_pimpl->vAugmented.size() + 1, 1) << m_pimpl->vAugmented, 0;
        }
    }

    return m_pimpl->vHat;
}

FloatingBaseExtendedKinematicsLieGroup FloatingBaseExtendedKinematicsLieGroupTangent::exp()
{
    auto XBase = m_pimpl->vBase.exp();
    std::map<int, manif::SE3d> XFeet;
    for (auto& [idx, v] : m_pimpl->vF)
    {
        XFeet[idx] = v.exp();
    }

    if (m_pimpl->isAugmentedVectorUsed)
    {
        return FloatingBaseExtendedKinematicsLieGroup(XBase, XFeet, m_pimpl->vAugmented);
    }

    return FloatingBaseExtendedKinematicsLieGroup(XBase, XFeet);
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::ljac()
{
    std::size_t dim = size();

    m_pimpl->Jl.resize(dim, dim);
    m_pimpl->Jl.setIdentity();
    
    m_pimpl->Jl.topLeftCorner<9, 9>() = m_pimpl->vBase.ljac();
    
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        m_pimpl->Jl.block<6, 6>(q, q) = iter.second.ljac();  
        idx++;
    }
    
    // bias related Jacobian is identity
    return m_pimpl->Jl;    
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::ljacinv()
{
    std::size_t dim = size();

    m_pimpl->Jlinv.resize(dim, dim);
    m_pimpl->Jlinv.setIdentity();
    
    m_pimpl->Jlinv.topLeftCorner<9, 9>() = m_pimpl->vBase.ljacinv();
    
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        m_pimpl->Jlinv.block<6, 6>(q, q) = iter.second.ljacinv();  
        idx++;
    }
    
    // bias related Jacobian inverse is identity
    return m_pimpl->Jlinv;   
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::rjac()
{
    std::size_t dim = size();

    m_pimpl->Jr.resize(dim, dim);
    m_pimpl->Jr.setIdentity();
    
    m_pimpl->Jr.topLeftCorner<9, 9>() = m_pimpl->vBase.rjac();
    
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        m_pimpl->Jr.block<6, 6>(q, q) = iter.second.rjac();  
        idx++;
    }
    
    // bias related Jacobian is identity
    return m_pimpl->Jr; 
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::rjacinv()
{
    std::size_t dim = size();

    m_pimpl->Jrinv.resize(dim, dim);
    m_pimpl->Jrinv.setIdentity();
    
    m_pimpl->Jrinv.topLeftCorner<9, 9>() = m_pimpl->vBase.rjacinv();
    
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        m_pimpl->Jrinv.block<6, 6>(q, q) = iter.second.rjacinv();  
        idx++;
    }
    
    // bias related Jacobian inverse is identity
    return m_pimpl->Jrinv; 
}

Eigen::MatrixXd FloatingBaseExtendedKinematicsLieGroupTangent::smallAdj()
{
    std::size_t dim = size();

    m_pimpl->smallAdj.resize(dim, dim);
    m_pimpl->smallAdj.setZero();
    
    m_pimpl->smallAdj.topLeftCorner<9, 9>() = m_pimpl->vBase.smallAdj();
    
    int idx = 0;
    for (auto& iter : m_pimpl->vF)
    {
        int q = m_pimpl->extMotionDim + (m_pimpl->twistDim*idx);
        m_pimpl->smallAdj.block<6, 6>(q, q) = iter.second.smallAdj();  
        idx++;
    }
    
    // bias small adjoint set to zero
    return m_pimpl->smallAdj; 
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setBaseExtendedMotionVector(const manif::SE_2_3Tangentd& baseExtenedMotionVector)
{
    m_pimpl->vBase = baseExtenedMotionVector;
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setBaseTwist(const manif::SE3Tangentd& baseTwist)
{
    Eigen::Matrix<double, 9, 1> v;
    v << baseTwist.v(), baseTwist.w(), m_pimpl->vBase.a();
    m_pimpl->vBase = manif::SE_2_3Tangentd(v);
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setBaseLinearVelocity(Eigen::Ref<const Eigen::Vector3d> baseLinearVelocity)
{
    Eigen::Matrix<double, 9, 1> v;
    v << baseLinearVelocity, m_pimpl->vBase.w(), m_pimpl->vBase.a();
    m_pimpl->vBase = manif::SE_2_3Tangentd(v);
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setBaseAngularVelocity(Eigen::Ref<const Eigen::Vector3d> baseAngularVelocity)
{
    Eigen::Matrix<double, 9, 1> v;
    v << m_pimpl->vBase.v(), baseAngularVelocity, m_pimpl->vBase.a();
    m_pimpl->vBase = manif::SE_2_3Tangentd(v);
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setBaseLinearAcceleration(Eigen::Ref<const Eigen::Vector3d> baseLinearAcceleration)
{
    Eigen::Matrix<double, 9, 1> v;
    v << m_pimpl->vBase.v(), m_pimpl->vBase.w(), baseLinearAcceleration;
    m_pimpl->vBase = manif::SE_2_3Tangentd(v);
}

void FloatingBaseExtendedKinematicsLieGroupTangent::disableAugmentedVector()
{
    m_pimpl->isAugmentedVectorUsed = false;    
}

void FloatingBaseExtendedKinematicsLieGroupTangent::setAugmentedVector(Eigen::Ref<const Eigen::VectorXd> augVec)
{
    m_pimpl->isAugmentedVectorUsed = true;
    m_pimpl->vAugmented = augVec;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameTwist(const int& idx, const manif::SE3Tangentd& twist)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameTwist] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
    
    m_pimpl->vF.at(idx) = twist;
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameLinearVelocity(const int& idx, Eigen::Ref<const Eigen::Vector3d> linearVelocity)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameLinearVelocity] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
        
    auto& twist = m_pimpl->vF.at(idx);
    Eigen::Matrix<double, 6, 1> v;
    v << linearVelocity, twist.w();
    twist = manif::SE3Tangentd(v);
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameAngularVelocity(const int& idx, Eigen::Ref<const Eigen::Vector3d> angularVelocity)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::setSupportFrameAngularVelocity] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
        
    auto& twist = m_pimpl->vF.at(idx);
    Eigen::Matrix<double, 6, 1> v;
    v <<  twist.v(), angularVelocity;
    twist = manif::SE3Tangentd(v);
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::addSupportFrameTwist(const int& idx, const manif::SE3Tangentd& twist)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::addSupportFrameTwist] "};
    
    if (m_pimpl->vF.find(idx) == m_pimpl->vF.end())
    {
        m_pimpl->vF[idx] = twist;
        m_pimpl->fIndices.emplace_back(idx);
        std::sort(m_pimpl->fIndices.begin(), m_pimpl->fIndices.end());
        return true;
    }
    
    std::cerr << printPrefix << "Frame already exists. use set method to change the frame twist." << std::endl;
    return false;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::removeSupportFrameTwist(const int& idx)
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroup::removeSupportFrameTwist] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
    
    m_pimpl->vF.erase(idx);
    m_pimpl->fIndices.erase(std::remove(m_pimpl->fIndices.begin(), 
                                        m_pimpl->fIndices.end(), 
                                        idx), 
                                        m_pimpl->fIndices.end());
    return true;
}

manif::SE_2_3Tangentd FloatingBaseExtendedKinematicsLieGroupTangent::baseExtenedMotionVector() const
{
    return m_pimpl->vBase;
}

manif::SE3Tangentd FloatingBaseExtendedKinematicsLieGroupTangent::baseTwist() const
{
    Eigen::Matrix<double, 6, 1> v;
    v << m_pimpl->vBase.v(), m_pimpl->vBase.w();
    return manif::SE3Tangentd(v);
}

Eigen::Vector3d FloatingBaseExtendedKinematicsLieGroupTangent::baseLinearVelocity() const
{
    return m_pimpl->vBase.v();
}

Eigen::Vector3d FloatingBaseExtendedKinematicsLieGroupTangent::baseAngularVelocity() const
{
    return m_pimpl->vBase.w();
}

Eigen::Vector3d FloatingBaseExtendedKinematicsLieGroupTangent::baseLinearAcceleration() const
{
    return m_pimpl->vBase.a();
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::isAugmentedVectorUsed() const
{
    return m_pimpl->isAugmentedVectorUsed;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::augmentedVector(Eigen::VectorXd& augVec) const
{
    std::string_view printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::augmentedVector] "};
    if (!m_pimpl->isAugmentedVectorUsed)
    {
        std::cerr << printPrefix << "Augmented vector not available in the velocity representation" << std::endl;
        return false;
    }
    
    augVec = m_pimpl->vAugmented;
    return true;
}

std::size_t FloatingBaseExtendedKinematicsLieGroupTangent::nrOfSupportFrames() const
{
    return m_pimpl->vF.size();
}

std::vector<int> FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameIndices() const
{
    return m_pimpl->fIndices;
}

std::map<int, manif::SE3Tangentd> FloatingBaseExtendedKinematicsLieGroupTangent::supportFramesTwist() const
{
    return m_pimpl->vF;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameTwist(const int& idx, manif::SE3Tangentd& twist) const
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameTwist] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
    
    twist = m_pimpl->vF.at(idx);
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameLinearVelocity(const int& idx, Eigen::Ref<Eigen::Vector3d> linearVelocity) const
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameLinearVelocity] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
    
    linearVelocity = m_pimpl->vF.at(idx).v();
    return true;
}

bool FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameAngularVelocity(const int& idx, Eigen::Ref<Eigen::Vector3d> angularVelocity) const
{
    std::string printPrefix{"[FloatingBaseExtendedKinematicsLieGroupTangent::supportFrameAngularVelocity] "};
    if (!m_pimpl->checkValidSupportFrame(idx))
    {
        std::cerr << printPrefix << "support frame of idx: " << idx <<" not available in the velocity representation" << std::endl;
        return false;
    }
    
    angularVelocity = m_pimpl->vF.at(idx).w();
    return true;
}

