#ifndef CARINFORMATION_H
#define CARINFORMATION_H

#include <pathitem.h>
#include <string>

/**
 * @brief The CarInformation class implements a class which stores infromation about each CarInformation
 * from which messages are received.
 */
class CarInformation
{
public:
    CarInformation(const std::string& carId);
    unsigned int getTime() const;
    std::string getCarId() const;
    void setGlobalLiveTime(const int64_t& t);
    int64_t getGlobalLiveTime() const;
    void setGlobalWaitTime(const int64_t& t);
    int64_t getGlobalWaitTime() const;
    void setReservationRequests(const int64_t& r);
    int64_t getReservationRequests() const;
    void setWaitTimeNextCell(const int64_t& t);
    int64_t getWaitTimeNextCell() const;
    void setExternalReservationRequests(const int64_t& r);
    int64_t getExternalReservationRequests() const;
    void setLastState(const PathItem& pathItem);
    const PathItem& getLastState() const;
    void setTarget(const PathItem& pathItem);
    const PathItem& getTarget() const;
    bool operator=(const CarInformation& carInfo) const;
    bool operator!=(const CarInformation& carInfo) const;

private:
    ///Car-ID
    std::string m_carId;
    ///last state of the car
    PathItem m_lastState;
    ///target
    PathItem m_target;
    ///global live time for the car, when it is in the intersection
    int64_t m_globalLiveTime;
    /// global wait time for the car, where the car do not move
    int64_t m_globalWaitTime;
    /// amount of reservation requests for next cell
    int64_t m_reservationRequests;
    /// wait time for a next certain cell
    int64_t m_waitTimeNextCell;
    /// amount of external reservation requests
    int64_t m_externalReservationRequests;

};

#endif // CARINFORMATION_H
