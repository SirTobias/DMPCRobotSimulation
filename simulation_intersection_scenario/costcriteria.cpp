#include "costcriteria.h"

#include "intersectionparameters.h"
#include <cmath>

/**
 * @brief CostCriteria::CostCriteria
 */
CostCriteria::CostCriteria(const std::shared_ptr<SystemFunction>& sysFunc) {
    m_globalLiveTime = sysFunc->getGlobalLiveTime();
    m_globalWaitTime = sysFunc->getGlobalWaitTime();
    m_reservationRequests = sysFunc->getReservationRequests();
    m_waitTimeNextCell = sysFunc->getWaitTimeNextCell();
    m_externalReservationRequests = sysFunc->getExternalReservationRequests();
}

CostCriteria::CostCriteria(const CarInformation& carInfo) {
    m_globalLiveTime = carInfo.getGlobalLiveTime();
    m_globalWaitTime = carInfo.getGlobalWaitTime();
    m_reservationRequests = carInfo.getReservationRequests();
    m_waitTimeNextCell = carInfo.getWaitTimeNextCell();
    m_externalReservationRequests = carInfo.getExternalReservationRequests();
}

/**
 * @brief CostCriteria::getGlobalLiveTime
 * @return
 */
int64_t CostCriteria::getGlobalLiveTime() const {
    return m_globalLiveTime;
}

/**
 * @brief CostCriteria::setGlobalLiveTime
 * @param t
 */
void CostCriteria::setGlobalLiveTime(const int64_t &t) {
    m_globalLiveTime = t;
}

/**
 * @brief CostCriteria::getGlobalWaitTime
 * @return
 */
int64_t CostCriteria::getGlobalWaitTime() const {
    return m_globalWaitTime;
}

/**
 * @brief CostCriteria::setGlobalWaitTime
 * @param t
 */
void CostCriteria::setGlobalWaitTime(const int64_t &t) {
    m_globalWaitTime = t;
}

/**
 * @brief CostCriteria::getReservationRequests
 * @return
 */
int64_t CostCriteria::getReservationRequests() const {
    return m_reservationRequests;
}

/**
 * @brief CostCriteria::setReservationRequests
 * @param r
 */
void CostCriteria::setReservationRequests(const int64_t &r) {
    m_reservationRequests = r;
}

/**
 * @brief CostCriteria::getWaitTimeNextCell
 * @return
 */
int64_t CostCriteria::getWaitTimeNextCell() const {
    return m_waitTimeNextCell;
}

/**
 * @brief CostCriteria::setWaitTimeNextCell
 * @param t
 */
void CostCriteria::setWaitTimeNextCell(const int64_t &t) {
    m_waitTimeNextCell = t;
}

/**
 * @brief CostCriteria::getExternalReservationRequests
 * @return
 */
int64_t CostCriteria::getExternalReservationRequests() const {
    return m_externalReservationRequests;
}

/**
 * @brief CostCriteria::setExternalReservationRequests
 * @param r
 */
void CostCriteria::setExternalReservationRequests(const int64_t &r) {
    m_externalReservationRequests = r;
}

/**
 * @brief CostCriteria::getCumulatedCosts sums up the weighted criteria and return the sum
 * @return sum of weighted criteria
 */
double CostCriteria::getCumulatedCosts() const {
    double costs = 0.0;
    if (InterSectionParameters::directComm == 1) {
        //global waitTime
        costs += InterSectionParameters::weightGlobalWaitTime * (double)getGlobalLiveTime();
        //global livetime
        costs += InterSectionParameters::weightGlobalLiveTime * (double)getGlobalWaitTime();
        //count reservation requests
        costs += (double)InterSectionParameters::weightReservRequests * (double)getReservationRequests();
        //wait time for certain next cell
        costs += (double)InterSectionParameters::weightTimeNextCell * (double)getWaitTimeNextCell();
        //amount of external reservation requests on current cell
        costs += (double)InterSectionParameters::weightExternalReservRequests * (double)getExternalReservationRequests();
    }
    return costs;
}

/**
 * @brief CostCriteria::getCumulatedQuadraticCosts squares the components, sums them up and returns the accumulated sum
 * @return sum_of(criteria)^2
 */
double CostCriteria::getCumulatedQuadraticCosts() const {
    double costs = 0.0;
    if (InterSectionParameters::directComm == 1) {
        //global waitTime
        costs += std::pow(InterSectionParameters::weightGlobalWaitTime * (double)getGlobalLiveTime(),2);
        //global livetime
        costs += std::pow(InterSectionParameters::weightGlobalLiveTime * (double)getGlobalWaitTime(),2);
        //count reservation requests
        costs += std::pow((double)InterSectionParameters::weightReservRequests * (double)getReservationRequests(), 2);
        //wait time for certain next cell
        costs += std::pow((double)InterSectionParameters::weightTimeNextCell * (double)getWaitTimeNextCell(), 2);
        //amount of external reservation requests on current cell
        costs += std::pow((double)InterSectionParameters::weightExternalReservRequests * (double)getExternalReservationRequests(), 2);
    }
    return costs;
}
