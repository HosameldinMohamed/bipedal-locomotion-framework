/**
 * @file DLGEKFBaseEstimator.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_DLGEKF_BASE_ESTIMATOR_H
#define BIPEDAL_LOCOMOTION_DLGEKF_BASE_ESTIMATOR_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>

namespace BipedalLocomotion
{
namespace Estimators
{


/**
 *
 *
 */
class DLGEKFBaseEstimator : public FloatingBaseEstimator
{
public:
    /**
     * Constructor
     */
    DLGEKFBaseEstimator();

    /**
     * Destructor (necessary for PIMPL idiom)
     */
    ~DLGEKFBaseEstimator();

    /**
     * To prevent function hiding due to overloading of virtual methods
     */
    using FloatingBaseEstimator::resetEstimator;

    /**
     * Reset the base pose estimate and consequently the internal state of the estimator
     * @param[in] newState internal state of the estimator
     * @param[in] newPriorDev internal state priors
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                        const FloatingBaseEstimators::StateStdDev& newPriorDev);

    /**
     * Reset the base pose estimate and consequently the internal state of the estimator
     * @param[in] newState internal state of the estimator
     * @param[in] newPriorDev internal state priors
     * @param[in] newSensorsDev sensor measurement noise
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                        const FloatingBaseEstimators::StateStdDev& newPriorDev,
                        const FloatingBaseEstimators::SensorsStdDev& newSensorsDev);

    /**
     * Toggle bias estimation
     * @param[in] flag flag for bias estimation
     */
    void toggleBiasEstimation(const bool& flag) { m_options.imuBiasEstimationEnabled = flag; };

    /**
     * Toggle EKF update
     * @param[in] flag flag for ekf update
     */
    void toggleEKFUpdate(const bool& flag) { m_options.ekfUpdateEnabled = flag; };
    
protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the estimator
    * @return bool
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;

    /**
    * Propagate the states through the prediction model, if there exists any (eg. a strap-down IMU model)
    * @param[in] meas measurements passed as exogeneous inputs to the prediction model
    * @param[in] dt sampling period in seconds
    * @param[out] state previous state estimate
    * @param[out] P previous state covariance matrix
    * @return True in case of success, false otherwise.
    */
    virtual bool predictState(const FloatingBaseEstimators::Measurements& meas,
                              const double& dt) override;

    /**
    * Update the predicted state estimates using kinematics and contact measurements
    * @param[in] meas measurements to update the predicted states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(const FloatingBaseEstimators::Measurements& meas,
                                  const double& dt) override;
                                  
    /**
    * Update the predicted state estimates using global pose measurements
    * @param[in] meas measurements to update the predicted states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateWithGlobalPose(FloatingBaseEstimators::Measurements& meas,
                                      const double& dt) override;

private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_INVEKF_BASE_ESTIMATOR_H

