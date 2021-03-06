/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
#define BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/System/DynamicalSystem.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace System
{

/**
 * LinearTimeInvariantSystem describes a MIMO linear time invariant system of the form \f$\dot{x} =
 * Ax + Bu\f$ where \a x is the state and \a u the control input. The state, its derivative and the
 * control input are described by vectors
 * The LinearTimeInvariantSystem inherits from a generic DynamicalSystem where:
 * - DynamicalSystem::StateType is described by an std::tuple containing:
 *   - Eigen::VectorXd: a generic state.
 * - DynamicalSystem::StateDerivativeType is described by an std::tuple containing:
 *   - Eigen::VectorXd: a generic state derivative.
 * - DynamicalSystem::InputType is described by an std::tuple containing:
 *   - Eigen::VectorXd: a generic control input.
 */
class LinearTimeInvariantSystem : public DynamicalSystem<std::tuple<Eigen::VectorXd>,
                                                         std::tuple<Eigen::VectorXd>,
                                                         std::tuple<Eigen::VectorXd>>
{
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;

    bool m_isInitialized{false};

public:
    /**
     * Set the system matrices.
     * @param A the A matrix.
     * @param B the B matrix.
     * @return true in case of success, false otherwise.
     */
    bool setSystemMatrices(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           const Eigen::Ref<const Eigen::MatrixXd>& B);

    /**
     * Computes the linear system dynamics. It returns \f$\dot{x} = Ax + Bu\f$
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const double& time,
                  StateDerivativeType& stateDerivative) final;

    /**
     * Destructor
     */
    ~LinearTimeInvariantSystem() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
