/**
 * @file IMUBipedMatrixLieGroup.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/IMUBipedMatrixLieGroup.h>

using namespace BipedalLocomotion::Estimators;

class IMUBipedMatrixLieGroup::Impl
{
public:
    ExtendedPose baseExtPose; /**< Internal state base extended pose*/
    Pose lfPose; /**< Internal state left foot contact frame pose*/
    Pose rfPose; /**< Internal state base right foot contact frame pose*/
    AccelerometerBias accBias; /**< Internal state accelerometer bias*/
    GyroscopeBias gyroBias; /**< Internal state gyroscope bias*/
    bool biasStatesEnabled{false}; /**< Internal flag to check activation of bias states*/

    void deactivateBiasStatesAndSetToZero(); /**< deactivate bias states and set them to zero **/

    const int matrixLieGroupSizeWithoutBias{13}; /**< \f[ X \in \mathbb{R}^{13 \times 13}  \f] */
    const int matrixLieGroupSizeWithBias{20}; /**< \f[ X \in \mathbb{R}^{20 \times 20}  \f] */

    const int tangentSizeWithoutBias{21}; /**< Tangent vector considering bias elements*/
    const int tangentSizeWithBias{27}; /**< Tangent vector without bias elements*/
};

IMUBipedMatrixLieGroup::~IMUBipedMatrixLieGroup()
{
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose.setIdentity();
    m_pimpl->lfPose.setIdentity();
    m_pimpl->rfPose.setIdentity();

    m_pimpl->deactivateBiasStatesAndSetToZero();
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const bool& estimateBias)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose.setIdentity();
    m_pimpl->lfPose.setIdentity();
    m_pimpl->rfPose.setIdentity();

    m_pimpl->biasStatesEnabled = true;
    m_pimpl->accBias.setZero();
    m_pimpl->gyroBias.setZero();
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const ExtendedPose& baseExtPose,
                                               const Pose& lfPose,
                                               const Pose& rfPose) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = baseExtPose;
    m_pimpl->lfPose = lfPose;
    m_pimpl->rfPose = rfPose;

    m_pimpl->deactivateBiasStatesAndSetToZero();
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const ExtendedPose& baseExtPose,
                                               const Pose& lfPose,
                                               const Pose& rfPose,
                                               const AccelerometerBias& accBias,
                                               const GyroscopeBias& gyroBias)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = baseExtPose;
    m_pimpl->lfPose = lfPose;
    m_pimpl->rfPose = rfPose;

    m_pimpl->biasStatesEnabled = true;
    m_pimpl->accBias = accBias;
    m_pimpl->gyroBias = gyroBias;
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const Rotation& baseRotation,
                                               const Position& basePosition,
                                               const LinearVelocity& baseLinearVelocity,
                                               const Rotation& lfRotation,
                                               const Position& lfPosition,
                                               const Rotation& rfRotation,
                                               const Position& rfPosition) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = ExtendedPose(basePosition, baseRotation, baseLinearVelocity);
    m_pimpl->lfPose = Pose(lfPosition, lfRotation);
    m_pimpl->rfPose = Pose(rfPosition, rfRotation);

    m_pimpl->deactivateBiasStatesAndSetToZero();
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const Rotation& baseRotation,
                                               const Position& basePosition,
                                               const LinearVelocity& baseLinearVelocity,
                                               const Rotation& lfRotation,
                                               const Position& lfPosition,
                                               const Rotation& rfRotation,
                                               const Position& rfPosition,
                                               const AccelerometerBias& accBias,
                                               const GyroscopeBias& gyroBias)  : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = ExtendedPose(basePosition, baseRotation, baseLinearVelocity);
    m_pimpl->lfPose = Pose(lfPosition, lfRotation);
    m_pimpl->rfPose = Pose(rfPosition, rfRotation);
    m_pimpl->biasStatesEnabled = true;
    m_pimpl->accBias = accBias;
    m_pimpl->gyroBias = gyroBias;
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const IMUBipedMatrixLieGroup& other) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->baseExtPose = other.baseExtenedPose();
    m_pimpl->lfPose = other.leftFootContactPose();
    m_pimpl->rfPose = other.rightFootContactPose();

    if (other.areBiasStatesActive())
    {
        m_pimpl->accBias = other.accelerometerBias();
        m_pimpl->gyroBias = other.gyroscopeBias();
    }
    else
    {
        m_pimpl->deactivateBiasStatesAndSetToZero();
    }
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::operator=(const IMUBipedMatrixLieGroup& other)
{
    return IMUBipedMatrixLieGroup(other);
}

IMUBipedMatrixLieGroup::IMUBipedMatrixLieGroup(const Eigen::Ref<Eigen::MatrixXd> X) : m_pimpl(std::make_unique<Impl>())
{
    fromMatrixLieGroup(X);
}

void IMUBipedMatrixLieGroup::fromMatrixLieGroup(const Eigen::Ref<Eigen::MatrixXd> X)
{
    Eigen::Quaterniond qB(X.block<3, 3>(0, 0));
    Rotation rotB(qB);
    Position pB = X.block<3, 1>(0, 3);
    LinearVelocity vB = X.block<3, 1>(0, 4);
    m_pimpl->baseExtPose = ExtendedPose(pB, rotB, vB);

    Eigen::Quaterniond qLF(X.block<3, 3>(5, 5));
    Rotation rotLF(qLF);
    Position pLF = X.block<3, 1>(5, 8);
    m_pimpl->lfPose = Pose(pLF, rotLF);

    Eigen::Quaterniond qRF(X.block<3, 3>(9, 9));
    Rotation rotRF(qLF);
    Position pRF = X.block<3, 1>(9, 12);
    m_pimpl->rfPose = Pose(pRF, rotRF);

    if (X.rows() == m_pimpl->matrixLieGroupSizeWithBias &&
        X.cols() == m_pimpl->matrixLieGroupSizeWithBias)
    {
        m_pimpl->biasStatesEnabled = true;
        m_pimpl->accBias = X.block<3, 1>(13, 19);
        m_pimpl->gyroBias = X.block<3, 1>(16, 19);
    }
}

Eigen::MatrixXd BipedalLocomotion::Estimators::IMUBipedMatrixLieGroup::asMatrixLieGroup()
{
    Eigen::MatrixXd X;
    if (m_pimpl->biasStatesEnabled)
    {
        X.resize(m_pimpl->matrixLieGroupSizeWithBias, m_pimpl->matrixLieGroupSizeWithBias);
    }
    else
    {
        X.resize(m_pimpl->matrixLieGroupSizeWithoutBias, m_pimpl->matrixLieGroupSizeWithoutBias);
    }

    X.setIdentity();
    X.topLeftCorner<5, 5>() = m_pimpl->baseExtPose.isometry();
    X.block<4, 4>(5, 5) = m_pimpl->lfPose.transform();
    X.block<4, 4>(9, 9) = m_pimpl->rfPose.transform();

    if (m_pimpl->biasStatesEnabled)
    {
        X.block<6, 1>(13, 19) << m_pimpl->accBias, m_pimpl->gyroBias;
    }

    return X;
}

void IMUBipedMatrixLieGroup::Impl::deactivateBiasStatesAndSetToZero()
{
    biasStatesEnabled = false;
    accBias.setZero();
    gyroBias.setZero();
}

bool IMUBipedMatrixLieGroup::areBiasStatesActive() const
{
    return m_pimpl->biasStatesEnabled;
}

bool IMUBipedMatrixLieGroup::setBaseExtendedPose(const ExtendedPose& baseExtenedPose)
{
    m_pimpl->baseExtPose = baseExtenedPose;
    return true;
}

bool IMUBipedMatrixLieGroup::setBasePose(const Pose& basePose)
{
    m_pimpl->baseExtPose = ExtendedPose(basePose.translation(),
                                        basePose.quat(),
                                        m_pimpl->baseExtPose.linearVelocity());
    return true;
}

bool IMUBipedMatrixLieGroup::setBaseRotation(const Rotation& baseRotation)
{
    m_pimpl->baseExtPose = ExtendedPose(m_pimpl->baseExtPose.translation(),
                                        baseRotation,
                                        m_pimpl->baseExtPose.linearVelocity());
    return true;
}

bool IMUBipedMatrixLieGroup::setBasePosition(const Position& basePosition)
{
    m_pimpl->baseExtPose = ExtendedPose(basePosition,
                                        m_pimpl->baseExtPose.quat(),
                                        m_pimpl->baseExtPose.linearVelocity());
    return true;
}

bool IMUBipedMatrixLieGroup::setBaseLinearVelocity(const LinearVelocity& baseLinearVelocity)
{
    m_pimpl->baseExtPose = ExtendedPose(m_pimpl->baseExtPose.translation(),
                                        m_pimpl->baseExtPose.quat(),
                                        baseLinearVelocity);
    return true;
}

bool IMUBipedMatrixLieGroup::setRightFootContactPose(const Pose& rfPose)
{
    m_pimpl->rfPose = rfPose;
    return true;
}

bool IMUBipedMatrixLieGroup::setRightFootContactRotation(const Rotation& rfRotation)
{
    m_pimpl->rfPose = Pose(m_pimpl->rfPose.translation(),
                           rfRotation);
    return true;
}

bool IMUBipedMatrixLieGroup::setRightFootContactPosition(const Position& rfPosition)
{
    m_pimpl->rfPose = Pose(rfPosition,
                           m_pimpl->rfPose.quat());
    return true;
}

bool IMUBipedMatrixLieGroup::setLeftFootContactPose(const Pose& lfPose)
{
    m_pimpl->lfPose = lfPose;
    return true;
}

bool IMUBipedMatrixLieGroup::setLeftFootContactRotation(const Rotation& lfRotation)
{
    m_pimpl->lfPose = Pose(m_pimpl->lfPose.translation(),
                           lfRotation);
    return true;
}

bool IMUBipedMatrixLieGroup::setLeftFootContactPosition(const Position& lfPosition)
{
    m_pimpl->lfPose = Pose(lfPosition,
                           m_pimpl->lfPose.quat());
    return true;
}

bool IMUBipedMatrixLieGroup::setAccelerometerBias(const AccelerometerBias& accelerometerBias)
{
    if (!m_pimpl->biasStatesEnabled)
    {
        std::cerr << "[IMUBipedMatrixLieGroup::setAccelerometerBias] Bias states inactive, cannot set accelerometer bias" << std::endl;
        return false;
    }

    m_pimpl->accBias = accelerometerBias;
    return true;
}

bool IMUBipedMatrixLieGroup::setGyroscopeBias(const GyroscopeBias& gyroscopeBias)
{
    if (!m_pimpl->biasStatesEnabled)
    {
        std::cerr << "[IMUBipedMatrixLieGroup::setGyroscopeBias] Bias states inactive, cannot set gyroscope bias" << std::endl;
        return false;
    }

    m_pimpl->gyroBias = gyroscopeBias;
    return true;
}

ExtendedPose IMUBipedMatrixLieGroup::baseExtenedPose() const
{
    return m_pimpl->baseExtPose;
}

Pose IMUBipedMatrixLieGroup::basePose() const
{
    return Pose(m_pimpl->baseExtPose.translation(), m_pimpl->baseExtPose.quat());
}

Rotation IMUBipedMatrixLieGroup::baseRotation() const
{
    return Rotation(m_pimpl->baseExtPose.quat());
}

Position IMUBipedMatrixLieGroup::basePosition() const
{
    return m_pimpl->baseExtPose.translation();
}

LinearVelocity IMUBipedMatrixLieGroup::baseLinearVelocity() const
{
    return m_pimpl->baseExtPose.linearVelocity();
}

Pose IMUBipedMatrixLieGroup::rightFootContactPose() const
{
    return Pose(m_pimpl->rfPose.translation(), m_pimpl->rfPose.quat());
}

Rotation IMUBipedMatrixLieGroup::rightFootContactRotation() const
{
    return Rotation(m_pimpl->rfPose.quat());
}

Position IMUBipedMatrixLieGroup::rightFootContactPosition() const
{
    return m_pimpl->rfPose.translation();
}

Pose IMUBipedMatrixLieGroup::leftFootContactPose() const
{
    return Pose(m_pimpl->lfPose.translation(), m_pimpl->lfPose.quat());
}

Rotation IMUBipedMatrixLieGroup::leftFootContactRotation() const
{
    return Rotation(m_pimpl->lfPose.quat());
}

Position IMUBipedMatrixLieGroup::leftFootContactPosition() const
{
    return m_pimpl->lfPose.translation();
}

AccelerometerBias IMUBipedMatrixLieGroup::accelerometerBias() const
{
    return m_pimpl->accBias;
}

GyroscopeBias IMUBipedMatrixLieGroup::gyroscopeBias() const
{
    return m_pimpl->gyroBias;
}

Eigen::MatrixXd IMUBipedMatrixLieGroup::adj()
{
    Eigen::MatrixXd AdjX;
    if (m_pimpl->biasStatesEnabled)
    {
        AdjX.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        AdjX.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    AdjX.setIdentity();
    AdjX.topLeftCorner<9, 9>() = m_pimpl->baseExtPose.adj();
    AdjX.block<6, 6>(9, 9) = m_pimpl->lfPose.adj();
    AdjX.block<6, 6>(15, 15) = m_pimpl->rfPose.adj();

    // bias related Adjoint is identity

    return AdjX;
}

void IMUBipedMatrixLieGroup::setIdentity()
{
    m_pimpl->baseExtPose.setIdentity();
    m_pimpl->lfPose.setIdentity();
    m_pimpl->rfPose.setIdentity();

    if (m_pimpl->biasStatesEnabled)
    {
        m_pimpl->accBias.setZero();
        m_pimpl->gyroBias.setZero();
    }
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::Identity(const bool& enableBiasStates)
{
    return IMUBipedMatrixLieGroup(enableBiasStates);
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::inverse()
{
    if (m_pimpl->biasStatesEnabled)
    {
        return IMUBipedMatrixLieGroup(m_pimpl->baseExtPose.inverse(),
                                      m_pimpl->lfPose.inverse(),
                                      m_pimpl->rfPose.inverse(),
                                     -m_pimpl->accBias,
                                     -m_pimpl->gyroBias);
    }

    return IMUBipedMatrixLieGroup(m_pimpl->baseExtPose.inverse(),
                                  m_pimpl->lfPose.inverse(),
                                  m_pimpl->rfPose.inverse());
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::operator*(const IMUBipedMatrixLieGroup& other)
{
    return lcompose(other);
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::lcompose(const IMUBipedMatrixLieGroup& other)
{
    if (other.areBiasStatesActive())
    {
        return IMUBipedMatrixLieGroup(m_pimpl->baseExtPose*other.baseExtenedPose(),
                                      m_pimpl->lfPose*other.leftFootContactPose(),
                                      m_pimpl->rfPose*other.rightFootContactPose(),
                                      m_pimpl->accBias + other.accelerometerBias(),
                                      m_pimpl->gyroBias + other.gyroscopeBias());
    }

    return IMUBipedMatrixLieGroup(m_pimpl->baseExtPose*other.baseExtenedPose(),
                                  m_pimpl->lfPose*other.leftFootContactPose(),
                                  m_pimpl->rfPose*other.rightFootContactPose());
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroup::rcompose(const IMUBipedMatrixLieGroup& other)
{
    if (other.areBiasStatesActive())
    {
        return IMUBipedMatrixLieGroup(other.baseExtenedPose()*m_pimpl->baseExtPose,
                                      other.leftFootContactPose()*m_pimpl->lfPose,
                                      other.rightFootContactPose()*m_pimpl->rfPose,
                                      m_pimpl->accBias + other.accelerometerBias(),
                                      m_pimpl->gyroBias + other.gyroscopeBias());
    }

    return IMUBipedMatrixLieGroup(other.baseExtenedPose()*m_pimpl->baseExtPose,
                                  other.leftFootContactPose()*m_pimpl->lfPose,
                                  other.rightFootContactPose()*m_pimpl->rfPose);
}

IMUBipedMatrixLieGroupTangent IMUBipedMatrixLieGroup::log()
{
    auto baseExtPoseTangent = m_pimpl->baseExtPose.log();
    auto so3base = AngularVelocity(baseExtPoseTangent.w());

    auto lfPoseTangent = m_pimpl->lfPose.log();
    auto so3lf = AngularVelocity(lfPoseTangent.w());

    auto rfPoseTangent = m_pimpl->rfPose.log();
    auto so3rf = AngularVelocity(rfPoseTangent.w());

    if (m_pimpl->biasStatesEnabled)
    {
        return IMUBipedMatrixLieGroupTangent(baseExtPoseTangent.v(),
                                             so3base,
                                             baseExtPoseTangent.a(),
                                             lfPoseTangent.v(),
                                             so3lf,
                                             rfPoseTangent.v(),
                                             so3rf,
                                             m_pimpl->accBias,
                                             m_pimpl->gyroBias);
    }

    return IMUBipedMatrixLieGroupTangent(baseExtPoseTangent.v(),
                                         so3base,
                                         baseExtPoseTangent.a(),
                                         lfPoseTangent.v(),
                                         so3lf,
                                         rfPoseTangent.v(),
                                         so3rf);
}


class IMUBipedMatrixLieGroupTangent::Impl
{
public:
    ExtendedMotionVector vBase; /**< Internal base extended motion vector*/
    Twist vLF; /**< Internal left foot contact frame  twist vector*/
    Twist vRF; /**< Internal right foot contact frame twist vector*/
    AccelerometerBias accBias; /**< Internal state accelerometer bias*/
    GyroscopeBias gyroBias; /**< Internal state gyroscope bias*/
    bool biasComponentsEnabled{false}; /**< Internal flag to check activation of bias states*/

    void deactivateBiasComponentsAndSetToZero(); /**< deactivate bias states and set them to zero **/


    const int matrixLieGroupSizeWithoutBias{13}; /**< \f[ X \in \mathbb{R}^{13 \times 13}  \f] */
    const int matrixLieGroupSizeWithBias{20}; /**< \f[ X \in \mathbb{R}^{20 \times 20}  \f] */

    const int tangentSizeWithoutBias{21}; /**< Tangent vector considering bias elements*/
    const int tangentSizeWithBias{27}; /**< Tangent vector without bias elements*/
};

IMUBipedMatrixLieGroupTangent::~IMUBipedMatrixLieGroupTangent()
{

}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase.setZero();
    m_pimpl->vLF.setZero();
    m_pimpl->vRF.setZero();
    m_pimpl->deactivateBiasComponentsAndSetToZero();
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const ExtendedMotionVector& vBase,
                                                             const Twist& vLF,
                                                             const Twist& vRF) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = vBase;
    m_pimpl->vLF = vLF;
    m_pimpl->vRF = vRF;
    m_pimpl->deactivateBiasComponentsAndSetToZero();
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const ExtendedMotionVector& vBase,
                                                             const Twist& vLF,
                                                             const Twist& vRF,
                                                             const AccelerometerBias& accBias,
                                                             const GyroscopeBias& gyroBias) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = vBase;
    m_pimpl->vLF = vLF;
    m_pimpl->vRF = vRF;
    m_pimpl->accBias = accBias;
    m_pimpl->gyroBias = gyroBias;
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const bool& estimateBias) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase.setZero();
    m_pimpl->vLF.setZero();
    m_pimpl->vRF.setZero();
    m_pimpl->accBias.setZero();
    m_pimpl->gyroBias.setZero();
    m_pimpl->biasComponentsEnabled = true;
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const LinearVelocity& baseLinearVelocity,
                                                             const AngularVelocity& baseAngularVelocity,
                                                             const LinearAcceleration& baseLinearAcceleration,
                                                             const LinearVelocity& lfLinearVelocity,
                                                             const AngularVelocity& lfAngularVelocity,
                                                             const LinearVelocity& rfLinearVelocity,
                                                             const AngularVelocity& rfAngularVelocity) : m_pimpl(std::make_unique<Impl>())
{
    Eigen::Matrix<double, 9, 1> vBase;
    vBase << baseLinearVelocity, baseAngularVelocity.coeffs(), baseLinearAcceleration;
    m_pimpl->vBase = vBase;

    Eigen::Matrix<double, 6, 1> vLF, vRF;
    vLF << lfLinearVelocity, lfAngularVelocity.coeffs();
    m_pimpl->vLF = vLF;
    vRF << rfLinearVelocity, rfAngularVelocity.coeffs();
    m_pimpl->vRF = vRF;
    m_pimpl->accBias.setZero();
    m_pimpl->gyroBias.setZero();
    m_pimpl->biasComponentsEnabled = false;
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const LinearVelocity& baseLinearVelocity,
                                                             const AngularVelocity& baseAngularVelocity,
                                                             const LinearAcceleration& baseLinearAcceleration,
                                                             const LinearVelocity& lfLinearVelocity,
                                                             const AngularVelocity& lfAngularVelocity,
                                                             const LinearVelocity& rfLinearVelocity,
                                                             const AngularVelocity& rfAngularVelocity,
                                                             const AccelerometerBias& accBias,
                                                             const GyroscopeBias& gyroBias) : m_pimpl(std::make_unique<Impl>())
{
    Eigen::Matrix<double, 9, 1> vBase;
    vBase << baseLinearVelocity, baseAngularVelocity.coeffs(), baseLinearAcceleration;
    m_pimpl->vBase = vBase;

    Eigen::Matrix<double, 6, 1> vLF, vRF;
    vLF << lfLinearVelocity, lfAngularVelocity.coeffs();
    m_pimpl->vLF = vLF;
    vRF << rfLinearVelocity, rfAngularVelocity.coeffs();
    m_pimpl->vRF = vRF;
    m_pimpl->accBias = accBias;
    m_pimpl->gyroBias = gyroBias;
    m_pimpl->biasComponentsEnabled = true;
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const IMUBipedMatrixLieGroupTangent& other) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->vBase = other.baseExtenedMotionVector();
    m_pimpl->vLF = other.leftFootContactTwist();
    m_pimpl->vRF = other.rightFootContactTwist();

    m_pimpl->biasComponentsEnabled = other.areBiasComponentsActive();
    m_pimpl->accBias = other.accelerometerBias();
    m_pimpl->gyroBias = other.gyroscopeBias();
}

IMUBipedMatrixLieGroupTangent IMUBipedMatrixLieGroupTangent::operator=(const IMUBipedMatrixLieGroupTangent& other)
{
    return IMUBipedMatrixLieGroupTangent(other);
}

IMUBipedMatrixLieGroupTangent::IMUBipedMatrixLieGroupTangent(const Eigen::VectorXd& v) : m_pimpl(std::make_unique<Impl>())
{
    fromVector(v);
}

void IMUBipedMatrixLieGroupTangent::fromVector(const Eigen::VectorXd& v)
{
    auto vB = v.segment<9>(0);
    m_pimpl->vBase = ExtendedMotionVector(vB);

    auto vLF = v.segment<6>(9);
    m_pimpl->vLF = Twist(vLF);

    auto vRF = v.segment<6>(15);
    m_pimpl->vRF = Twist(vRF);

    if (v.size() == m_pimpl->tangentSizeWithBias)
    {
        m_pimpl->biasComponentsEnabled = true;
        m_pimpl->accBias = v.segment<3>(21);
        m_pimpl->gyroBias = v.segment<3>(24);
    }
}

IMUBipedMatrixLieGroupTangent IMUBipedMatrixLieGroupTangent::operator-()
{
    if (m_pimpl->biasComponentsEnabled)
    {
        return IMUBipedMatrixLieGroupTangent(-m_pimpl->vBase,
                                             -m_pimpl->vLF,
                                             -m_pimpl->vRF,
                                             -m_pimpl->accBias,
                                             -m_pimpl->gyroBias);
    }

    return IMUBipedMatrixLieGroupTangent(-m_pimpl->vBase,
                                         -m_pimpl->vLF,
                                         -m_pimpl->vRF);
}

IMUBipedMatrixLieGroupTangent IMUBipedMatrixLieGroupTangent::operator+(const IMUBipedMatrixLieGroupTangent& other)
{
    auto vBase = m_pimpl->vBase + other.baseExtenedMotionVector();
    auto vLF = m_pimpl->vLF + other.leftFootContactTwist();
    auto vRF = m_pimpl->vRF + other.rightFootContactTwist();

    if (m_pimpl->biasComponentsEnabled)
    {
        return IMUBipedMatrixLieGroupTangent(vBase,
                                             vLF,
                                             vRF,
                                             m_pimpl->accBias + other.accelerometerBias(),
                                             m_pimpl->gyroBias + other.gyroscopeBias());
    }
    return IMUBipedMatrixLieGroupTangent(vBase, vLF, vRF);
}

IMUBipedMatrixLieGroupTangent IMUBipedMatrixLieGroupTangent::operator-(const IMUBipedMatrixLieGroupTangent& other)
{
    auto vBase = m_pimpl->vBase - other.baseExtenedMotionVector();
    auto vLF = m_pimpl->vLF - other.leftFootContactTwist();
    auto vRF = m_pimpl->vRF - other.rightFootContactTwist();

    if (m_pimpl->biasComponentsEnabled)
    {
        return IMUBipedMatrixLieGroupTangent(vBase,
                                             vLF,
                                             vRF,
                                             m_pimpl->accBias - other.accelerometerBias(),
                                             m_pimpl->gyroBias - other.gyroscopeBias());
    }
    return IMUBipedMatrixLieGroupTangent(vBase, vLF, vRF);
}


void IMUBipedMatrixLieGroupTangent::setZero()
{
    m_pimpl->vBase.setZero();
    m_pimpl->vLF.setZero();
    m_pimpl->vRF.setZero();
    if (m_pimpl->biasComponentsEnabled)
    {
        m_pimpl->accBias.setZero();
        m_pimpl->gyroBias.setZero();
    }
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::hat()
{
    Eigen::MatrixXd x;
    if (m_pimpl->biasComponentsEnabled)
    {
        x.resize(m_pimpl->matrixLieGroupSizeWithBias, m_pimpl->matrixLieGroupSizeWithBias);
    }
    else
    {
        x.resize(m_pimpl->matrixLieGroupSizeWithoutBias, m_pimpl->matrixLieGroupSizeWithoutBias);
    }

    x.setZero();
    x.topLeftCorner<5, 5>() = m_pimpl->vBase.hat();
    x.block<4, 4>(5, 5) = m_pimpl->vLF.hat();
    x.block<4, 4>(9, 9) = m_pimpl->vRF.hat();

    if (m_pimpl->biasComponentsEnabled)
    {
        x.block<6, 1>(13, 19) << m_pimpl->accBias, m_pimpl->gyroBias;
    }

    return x;
}

IMUBipedMatrixLieGroup IMUBipedMatrixLieGroupTangent::exp()
{

    auto Xbase = m_pimpl->vBase.exp();
    auto Xlf = m_pimpl->vLF.exp();
    auto Xrf = m_pimpl->vRF.exp();

    if (m_pimpl->biasComponentsEnabled)
    {
        return IMUBipedMatrixLieGroup(Xbase, Xlf, Xrf, m_pimpl->accBias, m_pimpl->gyroBias);
    }

    return IMUBipedMatrixLieGroup(Xbase, Xlf, Xrf);
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::ljac()
{
    Eigen::MatrixXd Jl;
    if (m_pimpl->biasComponentsEnabled)
    {
        Jl.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        Jl.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    Jl.setIdentity();
    Jl.topLeftCorner<9, 9>() = m_pimpl->vBase.ljac();
    Jl.block<6, 6>(9, 9) = m_pimpl->vLF.ljac();
    Jl.block<6, 6>(15, 15) = m_pimpl->vRF.ljac();

    // biases handled by identity matrix
    return Jl;
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::ljacinv()
{
    Eigen::MatrixXd Jlinv;
    if (m_pimpl->biasComponentsEnabled)
    {
        Jlinv.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        Jlinv.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    Jlinv.setIdentity();
    Jlinv.topLeftCorner<9, 9>() = m_pimpl->vBase.ljacinv();
    Jlinv.block<6, 6>(9, 9) = m_pimpl->vLF.ljacinv();
    Jlinv.block<6, 6>(15, 15) = m_pimpl->vRF.ljacinv();

    // biases handled by identity matrix
    return Jlinv;
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::rjac()
{
    Eigen::MatrixXd Jr;
    if (m_pimpl->biasComponentsEnabled)
    {
        Jr.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        Jr.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    Jr.setIdentity();
    Jr.topLeftCorner<9, 9>() = m_pimpl->vBase.rjac();
    Jr.block<6, 6>(9, 9) = m_pimpl->vLF.rjac();
    Jr.block<6, 6>(15, 15) = m_pimpl->vRF.rjac();

    // biases handled by identity matrix
    return Jr;
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::rjacinv()
{
    Eigen::MatrixXd Jrinv;
    if (m_pimpl->biasComponentsEnabled)
    {
        Jrinv.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        Jrinv.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    Jrinv.setIdentity();
    Jrinv.topLeftCorner<9, 9>() = m_pimpl->vBase.rjacinv();
    Jrinv.block<6, 6>(9, 9) = m_pimpl->vLF.rjacinv();
    Jrinv.block<6, 6>(15, 15) = m_pimpl->vRF.rjacinv();

    // biases handled by identity
    return Jrinv;
}

Eigen::MatrixXd IMUBipedMatrixLieGroupTangent::smallAdj()
{
    Eigen::MatrixXd smallAdj;
    if (m_pimpl->biasComponentsEnabled)
    {
        smallAdj.resize(m_pimpl->tangentSizeWithBias, m_pimpl->tangentSizeWithBias);
    }
    else
    {
        smallAdj.resize(m_pimpl->tangentSizeWithoutBias, m_pimpl->tangentSizeWithoutBias);
    }

    smallAdj.setZero();
    smallAdj.topLeftCorner<9, 9>() = m_pimpl->vBase.smallAdj();
    smallAdj.block<6, 6>(9, 9) = m_pimpl->vLF.smallAdj();
    smallAdj.block<6, 6>(15, 15) = m_pimpl->vRF.smallAdj();

    // biases handled by zeros
    return smallAdj;
}


ExtendedMotionVector IMUBipedMatrixLieGroupTangent::baseExtenedMotionVector() const
{
    return m_pimpl->vBase;
}

Twist IMUBipedMatrixLieGroupTangent::baseTwist() const
{
    Eigen::Matrix<double, 6, 1> v;
    v << m_pimpl->vBase.v(), m_pimpl->vBase.w();
    return Twist(v);
}

LinearVelocity IMUBipedMatrixLieGroupTangent::baseLinearVelocity() const
{
    return m_pimpl->vBase.v();
}

AngularVelocity IMUBipedMatrixLieGroupTangent::baseAngularVelocity() const
{
    return m_pimpl->vBase.w();
}

LinearAcceleration IMUBipedMatrixLieGroupTangent::baseLinearAcceleration() const
{
    return m_pimpl->vBase.a();
}

Twist IMUBipedMatrixLieGroupTangent::leftFootContactTwist() const
{
    return m_pimpl->vLF;
}

LinearVelocity IMUBipedMatrixLieGroupTangent::leftFootContactLinearVelocity() const
{
    return m_pimpl->vLF.v();
}

AngularVelocity IMUBipedMatrixLieGroupTangent::leftFootContactAngularVelocity() const
{
    return m_pimpl->vLF.w();
}

Twist IMUBipedMatrixLieGroupTangent::rightFootContactTwist() const
{
    return m_pimpl->vRF;
}

LinearVelocity IMUBipedMatrixLieGroupTangent::rightFootContactLinearVelocity() const
{
    return m_pimpl->vRF.v();
}

AngularVelocity IMUBipedMatrixLieGroupTangent::rightFootContactAngularVelocity() const
{
    return m_pimpl->vRF.w();
}

AccelerometerBias IMUBipedMatrixLieGroupTangent::accelerometerBias() const
{
    return m_pimpl->accBias;
}

GyroscopeBias IMUBipedMatrixLieGroupTangent::gyroscopeBias() const
{
    return m_pimpl->gyroBias;
}

bool IMUBipedMatrixLieGroupTangent::areBiasComponentsActive() const
{
    return m_pimpl->biasComponentsEnabled;
}

bool IMUBipedMatrixLieGroupTangent::setBaseExtendedMotionVector(const ExtendedMotionVector& baseExtenedMotionVector)
{
    m_pimpl->vBase = baseExtenedMotionVector;
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setBaseTwist(const Twist& baseTwist)
{
    Eigen::Matrix<double, 9, 1> v;
    v << baseTwist.v(), baseTwist.w(), m_pimpl->vBase.a();
    m_pimpl->vBase = ExtendedMotionVector(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setBaseLinearVelocity(const LinearVelocity& baseLinearVelocity)
{
    Eigen::Matrix<double, 9, 1> v;
    v << baseLinearVelocity, m_pimpl->vBase.w(), m_pimpl->vBase.a();
    m_pimpl->vBase = ExtendedMotionVector(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setBaseAngularVelocity(const AngularVelocity& baseAngularVelocity)
{
    Eigen::Matrix<double, 9, 1> v;
    v << m_pimpl->vBase.v(), baseAngularVelocity.coeffs(), m_pimpl->vBase.a();
    m_pimpl->vBase = ExtendedMotionVector(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setBaseLinearAcceleration(const LinearAcceleration& baseLinearAcceleration)
{
    Eigen::Matrix<double, 9, 1> v;
    v << m_pimpl->vBase.v(), m_pimpl->vBase.w(), baseLinearAcceleration;
    m_pimpl->vBase = ExtendedMotionVector(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setRightFootContactTwist(const Twist& rightFootContactTwist)
{
    m_pimpl->vRF = rightFootContactTwist;
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setRightFootContactLinearVelocity(const LinearVelocity& rightFootContactLinearVelocity)
{
    Eigen::Matrix<double, 6, 1> v;
    v << rightFootContactLinearVelocity, m_pimpl->vRF.w();
    m_pimpl->vRF = Twist(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setRightFootContactAngularVelocity(const AngularVelocity& rightFootContactAngularVelocity)
{
    Eigen::Matrix<double, 6, 1> v;
    v << m_pimpl->vRF.v(), rightFootContactAngularVelocity.coeffs();
    m_pimpl->vRF = Twist(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setLeftFootContactTwist(const Twist& lefttFootContactTwist)
{
    m_pimpl->vLF = lefttFootContactTwist;
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setLeftFootContactLinearVelocity(const LinearVelocity& leftFootContactLinearVelocity)
{
    Eigen::Matrix<double, 6, 1> v;
    v << leftFootContactLinearVelocity, m_pimpl->vLF.w();
    m_pimpl->vLF = Twist(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setLeftFootContactAngularVelocity(const AngularVelocity& leftFootContactAngularVelocity)
{
    Eigen::Matrix<double, 6, 1> v;
    v << m_pimpl->vLF.v(), leftFootContactAngularVelocity.coeffs();
    m_pimpl->vLF = Twist(v);
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setAccelerometerBias(const AccelerometerBias& accelerometerBias)
{
    m_pimpl->accBias = accelerometerBias;
    return true;
}

bool IMUBipedMatrixLieGroupTangent::setGyroscopeBias(const GyroscopeBias& gyroscopeBias)
{
    m_pimpl->gyroBias = gyroscopeBias;
    return true;
}

void IMUBipedMatrixLieGroupTangent::Impl::deactivateBiasComponentsAndSetToZero()
{
    biasComponentsEnabled = false;
    accBias.setZero();
    gyroBias.setZero();
}
