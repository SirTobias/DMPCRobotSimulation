#ifndef COSTCRITERIA_H
#define COSTCRITERIA_H

#include "carinformation.h"
#include "systemfunction.h"

/**
 * @brief The CostCriteria class defines the different criteria to exchange the cumulated costs
 * between the cars for direct communication. As
 */
class CostCriteria
{
public:
    CostCriteria(const std::shared_ptr<SystemFunction>& sysFunc);
    CostCriteria(const CarInformation& carInfo);
    int64_t getGlobalLiveTime() const;
    void setGlobalLiveTime(const int64_t &t);
    int64_t getGlobalWaitTime() const;
    void setGlobalWaitTime(const int64_t &t);
    int64_t getReservationRequests() const;
    void setReservationRequests(const int64_t &r);
    int64_t getWaitTimeNextCell() const;
    void setWaitTimeNextCell(const int64_t &t);
    int64_t getExternalReservationRequests() const;
    void setExternalReservationRequests(const int64_t &r);
    double getCumulatedCosts() const;
    double getCumulatedQuadraticCosts() const;
private:
    ///criterias for coupled cost function
    /// global life time for the car
    int64_t m_globalLiveTime;
    /// global wait time for the car, when it does not move
    int64_t m_globalWaitTime;
    /// amount of reservation requests for next cell
    int64_t m_reservationRequests;
    /// wait time for a next certain cell
    int64_t m_waitTimeNextCell;
    /// amount of external reservation requests
    int64_t m_externalReservationRequests;
};

#endif // COSTCRITERIA_H
