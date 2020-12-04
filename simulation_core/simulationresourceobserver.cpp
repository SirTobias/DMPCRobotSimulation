#include "simulationresourceobserver.h"
#include <iostream>

/**
 * @brief SimulationResourceObserver::SimulationResourceObserver
 */
SimulationResourceObserver::SimulationResourceObserver(const QString &resName) :
    m_throughput(0),
    m_workedTime(0),
    m_idleTime(0),
    m_allTime(0),
    m_resourceName(resName)
{
}

/**
 * @brief returns throughput
 * @return throughput
 */
double SimulationResourceObserver::getThroughput() const {
    return m_throughput;
}

/**
 * @brief all simulation time
 * @return simulation time
 */
double SimulationResourceObserver::getAllTime() const {
    return m_allTime;
}

/**
 * @brief time when the resource is in work process
 * @return worked time
 */
double SimulationResourceObserver::getWorkedTime() const {
    return m_workedTime;
}

/**
 * @brief add throughput time
 * @param throughput time
 */
void SimulationResourceObserver::addThroughPut(const double &t) {
    m_throughput += t;
}

/**
 * @brief add all passed time
 * @param passed time
 */
void SimulationResourceObserver::addAllTime(const double &t) {
    m_allTime += t;
}

/**
 * @brief returns idle time
 * @return idle time
 */
double SimulationResourceObserver::getIdleTime() const {
    return m_idleTime;
}

/**
 * @brief returns current progress
 * @return current progress
 */
double SimulationResourceObserver::getCurrentProgress() const {
    return m_currentProgress;
}

/**
 * @brief add idle time
 * @param idle time
 */
void SimulationResourceObserver::addIdleTime(const double &t) {
    m_idleTime += t;
}

/**
 * @brief add the worked time
 * @param worked time
 */
void SimulationResourceObserver::addWorkedTime(const double &t) {
    m_workedTime += t;
}

/**
 * @brief add the current progress for the current object (these variable would be resetted after every completion)
 * @param add time for current completion
 */
void SimulationResourceObserver::addCurrentProgress(const double &t) {
    m_currentProgress += t;
}

/**
 * @brief reset the time for the current completion, should be called after finishing one object
 */
void SimulationResourceObserver::resetCurrentProgress() {
    m_currentProgress = 0;
}

/**
 * @brief print current state of the SimulationResource
 */
void SimulationResourceObserver::printEvaluation() {
    std::cout << "SimulationResource name: " << m_resourceName.toLocal8Bit().data() << std::endl;
    std::cout << "allTime: " << m_allTime << std::endl;
    std::cout << "idleTime: " << m_idleTime << std::endl;
    std::cout << "workedTime: " << m_workedTime << std::endl;
    std::cout << "workLoad: " << ((double)m_workedTime / (double)m_allTime) << std::endl;
}
