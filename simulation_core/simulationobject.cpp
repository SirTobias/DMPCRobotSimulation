#include "simulationobject.h"

/**
 * @brief base class constructor without any parameters
 */
SimulationObject::SimulationObject() :
    m_name(""),
    m_twait(0),
    m_tenter(0),
    m_tleave(0)
{
}

/**
 * @brief base class constructor with setting a name for the simulation object
 * @param name for SimulationObject
 */
SimulationObject::SimulationObject(const QString &name, const QString &productConfig) :
    m_name(name),
    m_twait(0),
    m_tenter(0),
    m_tleave(0),
    m_productConfig(productConfig)
{
}

/**
 * @brief Destructor
 */
SimulationObject::~SimulationObject() {

}

/**
 * @brief get/set for productconfig which is defined in SimulationResourceConfig
 * @return config as QString
 */
QString SimulationObject::getProductConfig() const {
    return m_productConfig;
}

/**
 * @brief get/set for productconfig which is defined in SimulationResourceConfig
 * @param config set ProductConfig
 */
void SimulationObject::setProductConfig(const QString &config) {
    m_productConfig = config;
}

/**
 * @brief the wait time is the time for being processed
 * @param t time to set
 */
void SimulationObject::setWaitTime(const double &t) {
    this->m_twait = t;
}

/**
 * @brief when Simulation Object enters a ressource
 * @param t time to set
 */
void SimulationObject::setEnterTime(const double &t) {
    this->m_tenter = t;
}

/**
 * @brief time when the object leaves an SimulationRessource
 * @param t time to set
 */
void SimulationObject::setLeaveTime(const double &t) {
    this->m_tleave = t;
}

/**
 * @brief the wait time is the time for being processed
 * @return time
 */
double SimulationObject::getWaitTime() const {
    return this->m_twait;
}

/**
 * @brief when Simulation Object enters a ressource
 * @return time
 */
double SimulationObject::getEnterTime() const {
    return this->m_tenter;
}

/**
 * @brief time when the object leaves an SimulationRessource
 * @return time
 */
double SimulationObject::getLeaveTime() const {
    return this->m_tleave;
}

/**
 * @brief return the name of the SimulationObject
 * @return name
 */
QString SimulationObject::getName() const {
    return m_name;
}

/**
 * @brief set the name of the SimulationObject - another possibility is the constructor
 * @param name to set
 */
void SimulationObject::setName(const QString &name) {
    m_name = name;
}
