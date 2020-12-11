/**
 * @file ContactDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Contacts;

bool ContactDetector::initialize(std::weak_ptr<IParametersHandler> handler)
{
    std::string_view printPrefix = "[ContactDetector::initialize] ";
    if (m_detectorState != State::NotInitialized)
    {
        std::cerr << printPrefix << "The contact detector already seems to be initialized."
        << std::endl;
        return false;
    }

    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << printPrefix << "The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }


    if (!customInitialization(handler))
    {
        std::cerr << printPrefix <<
        "Could not run custom initialization of the contact detector."
        << std::endl;
        return false;
    }

    std::cerr << printPrefix <<
        "Contact detector initialization is successful."
        << std::endl;

    m_detectorState = State::Initialized;
    return true;
}

bool ContactDetector::customInitialization(std::weak_ptr<IParametersHandler> handler)
{
    return true;
}

bool ContactDetector::advance()
{    
    std::string_view printPrefix = "[ContactDetector::advance] ";
    if (m_detectorState == State::NotInitialized)
    {
        std::cerr << printPrefix << "Please initialize the contact detector before running advance."
        << std::endl;
        return false;        
    }    
    else 
    {
        m_detectorState = State::Running;        
    }
    
    if (!updateContactStates())
    {
        return false;
    }
    return true;
}

bool ContactDetector::updateContactStates()
{
    return true;
}

bool ContactDetector::resetContacts()
{
    for (auto& [name, contact] : m_contactStates)
    {
        contact.switchTime = 0.0;
        contact.isActive = false;
    }
    return true;
}


const EstimatedContactList& ContactDetector::get() const
{
    return m_contactStates;
}

bool ContactDetector::get(const std::string& contactName, EstimatedContact& contact) const
{
    if ( m_contactStates.find(contactName) == m_contactStates.end() )
    {
        std::cerr << "[ContactDetector::get] Contact not found.";
        return false;
    }

    contact = m_contactStates.at(contactName);
    return true;
}


bool ContactDetector::isValid() const
{
    return (m_detectorState == State::Running);
}


