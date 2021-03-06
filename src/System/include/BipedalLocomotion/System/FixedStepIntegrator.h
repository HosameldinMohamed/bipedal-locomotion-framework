/**
 * @file FixedStepIntegrator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_H

#include <memory>
#include <tuple>

#include <BipedalLocomotion/System/Integrator.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Fixed step integrator base class. Please inherit publicly from this class in order to specify a
 * custom integration method.
 * @tparam DynamicalSystemDerived a class derived from DynamicalSystem
 */
template <typename DynamicalSystemDerived>
class FixedStepIntegrator : public Integrator<DynamicalSystemDerived>
{

protected:
    double m_dT{0.0}; /**< Fixed step size */

    /**
     * Integrate one step.
     * @param t0 initial time.
     * @param dT sampling time.
     * @return true in case of success, false otherwise.
     */
    virtual bool oneStepIntegration(double t0, double dT) = 0;

public:

    /**
     * Constructor
     * @param dT the sampling time
     */
    FixedStepIntegrator(const double& dT)
        : m_dT{dT}
    {
    }

    /**
     * Integrate the dynamical system from initialTime to finalTime.
     * @note We assume a constant control input in the interval.
     * @param initialTime initial time of the integration.
     * @param finalTime final time of the integration.
     * @return true in case of success, false otherwise.
     */
    bool integrate(double initialTime, double finalTime) final;

    ~FixedStepIntegrator() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#include <BipedalLocomotion/System/FixedStepIntegrator.tpp>

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_H
