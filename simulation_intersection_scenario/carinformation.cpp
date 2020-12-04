#include "carinformation.h"

/**
 * @brief CarInformation::CarInformation constructs the structure with given ID
 * @param carId Car-ID
 */
CarInformation::CarInformation(const std::string &carId) :
    m_carId(carId),
    m_lastState(0,0,0),
    m_globalLiveTime(0),
    m_globalWaitTime(0),
    m_reservationRequests(0),
    m_waitTimeNextCell(0),
    m_externalReservationRequests(0)
{
}


/**
 * @brief CarInformation::getCarId get the id of the car
 * @return id as string
 */
std::string CarInformation::getCarId() const {
    return m_carId;
}

/**
 * @brief CarInformation::setGlobalLiveTime global live time for the car
 * @param t global live time as int
 */
void CarInformation::setGlobalLiveTime(const int64_t& t) {
    m_globalLiveTime = t;
}

/**
 * @brief CarInformation::getGlobalLiveTime return global live time for the car
 * @return global live time
 */
int64_t CarInformation::getGlobalLiveTime() const {
    return m_globalLiveTime;
}

/**
 * @brief CarInformation::setGlobalWaitTime global wait time for the car
 * @param t global live time as int
 */
void CarInformation::setGlobalWaitTime(const int64_t& t) {
    m_globalWaitTime = t;
}

/**
 * @brief CarInformation::getGlobalWaitTime return global wait time for the car
 * @return global live time
 */
int64_t CarInformation::getGlobalWaitTime() const {
    return m_globalWaitTime;
}

/**
 * @brief CarInformation::setReservationRequests amount of reservations requests for the next cell
 * @param r reservation requests as int
 */
void CarInformation::setReservationRequests(const int64_t& r) {
    m_reservationRequests = r;
}

/**
 * @brief CarInformation::getReservationRequests amount of reservations requests for the next cell
 * @return reservation requests as int
 */
int64_t CarInformation::getReservationRequests() const {
    return m_reservationRequests;
}

/**
 * @brief CarInformation::setWaitTimeNextCell wait time for a next certain cell
 * @param t wait time as int
 */
void CarInformation::setWaitTimeNextCell(const int64_t& t) {
    m_waitTimeNextCell = t;
}

/**
 * @brief CarInformation::getWaitTimeNextCell wait time for a next certain cell
 * @return wait time as int
 */
int64_t CarInformation::getWaitTimeNextCell() const {
    return m_waitTimeNextCell;
}

/**
 * @brief CarInformation::setExternalReservationRequests external reservation requests from other cars for current cell
 * @param r external reservation requests
 */
void CarInformation::setExternalReservationRequests(const int64_t& r) {
    m_externalReservationRequests = r;
}

/**
 * @brief CarInformation::getExternalReservationRequests external reservation requests from other cars for current cell
 * @return external reservation requests as int
 */
int64_t CarInformation::getExternalReservationRequests() const {
    return m_externalReservationRequests;
}

/**
 * @brief CarInformation::operator =
 * @param carInfo
 * @return true, if carinfo == this, else false
 */
bool CarInformation::operator=(const CarInformation& carInfo) const {
    return (m_carId == carInfo.getCarId()
            && m_lastState == carInfo.getLastState()
            && m_globalLiveTime == carInfo.getGlobalLiveTime()
            && m_globalWaitTime == carInfo.getGlobalWaitTime()
            && m_reservationRequests == carInfo.getReservationRequests()
            && m_waitTimeNextCell == carInfo.getWaitTimeNextCell()
            && m_externalReservationRequests == carInfo.getExternalReservationRequests());
}

/**
 * @brief CarInformation::operator !=
 * @param carInfo
 * @return true, if carinfo != this, else false
 */
bool CarInformation::operator!=(const CarInformation& carInfo) const {
    return !(operator=(carInfo));
}

/**
 * @brief CarInformation::getLastState
 * @return
 */
const PathItem& CarInformation::getLastState() const {
    return m_lastState;
}

/**
 * @brief CarInformation::setLastState
 * @param pathItem
 */
void CarInformation::setLastState(const PathItem& pathItem) {
    m_lastState = pathItem;
}

/**
 * @brief CarInformation::setTarget
 * @param pathItem
 */
void CarInformation::setTarget(const PathItem& pathItem) {
    m_target = pathItem;
}

/**
 * @brief getTarget
 * @return
 */
const PathItem& CarInformation::getTarget() const {
    return m_target;
}
