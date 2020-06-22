/**
 * @file FloatingBaseSystemDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/System/DynamicalSystem.h>
#include <BipedalLocomotion/System/ContactWrench.h>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * FloatingBaseDynamicalSystem describes a floating base dynamical system.
 * The FloatingBaseDynamicalSystem inherits from a generic DynamicalSystem where:
 * - DynamicalSystem::StateType is described by an std::tuple containing:
 *   - iDynTree::Vector6: the base velocity expressed in mixed representation;
 *   - iDynTree::VectorDynsize: the joint velocities [in rad/s];
 *   - iDynTree::Position: position of the base w.r.t. the inertial frame
 *   - iDynTree::Rotation: rotation matrix \f${} ^ I R _ {b}\f$. Matrix that transform a vector
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - iDynTree::VectorDynsize: the joint positions [in rad].
 * - DynamicalSystem::StateDerivativeType is described by an std::tuple containing:
 *   - iDynTree::Vector6: the base acceleration expressed in mixed representation;
 *   - iDynTree::VectorDynsize: the joint accelerations [in rad/s^2];
 *   - iDynTree::Vector3: base velocity w.r.t. the inertial frame;
 *   - iDynTree::Matrix3x3: rate of change of the rotation matrix \f${} ^ I \dot{R} _ {b}\f$.
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - iDynTree::VectorDynsize: the joint velocities [in rad/s].
 * - DynamicalSystem::InputType is described by an std::tuple containing:
 *   - iDynTree::VectorDynsize: the joint torques [in Nm];
 *   - std::vector<ContactWrench>: List of contact wrenches.
 */
class FloatingBaseDynamicalSystem : public DynamicalSystem<std::tuple<iDynTree::Vector6,
                                                                      iDynTree::VectorDynSize,
                                                                      iDynTree::Position,
                                                                      iDynTree::Rotation,
                                                                      iDynTree::VectorDynSize>,
                                                           std::tuple<iDynTree::Vector6,
                                                                      iDynTree::VectorDynSize,
                                                                      iDynTree::Vector3,
                                                                      iDynTree::Matrix3x3,
                                                                      iDynTree::VectorDynSize>,
                                                           std::tuple<iDynTree::VectorDynSize,
                                                                      std::vector<ContactWrench>>>
{
    static constexpr size_t m_baseDoFs = 6; /**< Number of degree of freedom associated to the
                                               floating base */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to an existing instance of
                                                               kinDynComputations object */
    std::size_t m_actuatedDoFs{0}; /**< Number of actuated degree of freedom */

    iDynTree::Vector3 m_gravity; /**< Gravity vector */

    iDynTree::MatrixDynSize m_massMatrix; /**< Floating-base mass matrix  */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Coriolis and
                                                                         Gravitational term  */
    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    // quantities useful to avoid dynamic allocation in the dynamic allocation in the
    // FloatingBaseDynamicalSystem::dynamics method
    iDynTree::VectorDynSize m_generalizedRobotAcceleration;
    iDynTree::VectorDynSize m_knownCoefficent;

    bool m_useMassMatrixRegularizationTerm{false};
    iDynTree::MatrixDynSize m_massMatrixReglarizationTerm;

public:
    /**
     * Constructor.
     * @note The constructor set the gravity acceleration vector to
     * \f$\begin{bmatrix} 0 & 0 & -9.81\end{bmatrix}^\top\f$. Please call setGravityVector() if you
     * want define your custom gravity vector.
     */
    FloatingBaseDynamicalSystem();

    /**
     * Set the vector of gravity.
     * @param gravity a 3D vector describing the gravity acceleration.
     */
    void setGravityVector(const iDynTree::Vector3& gravity);

    /**
     * Set a kinDynComputations object.
     * @param kinDyn a pointer to the kinDynComputations object.
     * @return true in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Set the mass matrix regularization term. i.e. $\f\bar{M} = M + M _ {reg}\f$. Where  $\fM\f$
     * is the mass matrix and  $\fM_{reg}\f$ is the matrix regularization term.
     * @param matrix the regularization term for the mass matrix.
     * @notice Calling this function is not mandatory. Call it only if you want to add a
     * regularization term.
     * @return true in case of success, false otherwise.
     */
    bool setMassMatrixRegularization(const iDynTree::MatrixDynSize& matrix);

    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input has to be set separately with the method setControlInput.
     * @param state tuple containing a const reference to the state elements.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const StateType& state,
                  const double& time,
                  StateDerivativeType& stateDerivative) final;

    /**
     * Destructor.
     */
    ~FloatingBaseDynamicalSystem() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_H
