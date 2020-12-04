#ifndef SYSTEMFUNCTION_H
#define SYSTEMFUNCTION_H

#include "path.h"
#include "carinformation.h"
#include "intersectionparameters.h"

#include <QtCore/QString>

#include <string>


/**
 * @brief The SystemFunction class
 * reserve the cell for the next time step
 */
class SystemFunction
{
public:
    SystemFunction(const QString &car, const Path& path);
    SystemFunction(const QString &car, const std::vector<double> startPos);
    PathItem prelimReserveCell(const PathItem& control);
    double getPossibleTimeForPrelimReservation(const PathItem& item) const;
    void clearPrelimPath();

    size_t countCarInfos() const;
    const CarInformation& getCarInfoEnd() const;
    void applyNextState(const PathItem &pathItem);
    void applyNextState(const std::vector<double> &umin, const double &t0, const double &T);
    std::vector<double> getHolonomicSystem(const std::vector<double> x0, const std::vector<double> &u, const double &t0, const double &tEnd) const;
    std::vector<std::vector <double> > getHolonomicSystemTrajectory(const std::vector<double> x0,
                                                                    const std::vector<std::vector<double> > &u, const double &t0, const double &T, const size_t &N) const;
    Path mapPredictionToCells(const std::vector<std::vector<double> > &x, const double &t0, const double& T, const double& radius = 0.0) const;
    SystemFunctionUsage getSystemFunctionType() const;
    bool reservePrelimSolution(const PathItem& pathItem);
    Path setPossiblePrelimTimeForPath(const Path& path) const;

    //get-set
    PathItem getStart() const;
    std::vector<double> getStartContinuous() const;
    PathItem getCurrentState() const;
    std::vector<double> getCurrentContinuousState() const;
    std::vector<double> getPos(const size_t& N) const;
    Path& getPath();
    Path getPath() const;
    Path& getPrelimPath();
    Path getPrelimPath() const;
    void setPrelimPath(const Path& path);
    double getLambda() const;

    int64_t getGlobalLiveTime() const;
    void setGlobalLiveTime(const double &t);
    int64_t getGlobalWaitTime() const;
    void setGlobalWaitTime(const int64_t &t);
    int64_t getReservationRequests() const;
    void setReservationRequests(const int64_t &r);
    int64_t getWaitTimeNextCell() const;
    void setWaitTimeNextCell(const int64_t &t);
    int64_t getExternalReservationRequests() const;
    void setExternalReservationRequests(const int64_t &r);
    void setCarInfo(const std::string& id, const CarInformation& carInfo);
    const CarInformation& getCarInfo(const size_t& pos) const;
    void setGlobalTime(const double &t);
    int64_t getGlobalTime() const;
    void setIntervalControlDynamic(const std::vector<double>& vec);
private:
    ///preliminary path (clear it for each optimization step of one car)
    Path m_prelimPath;
    ///Path of current and further stati
    Path m_path;
    ///ID of the car of the given system function
    QString m_car;
    ///criterias for coupled cost function
    /// global life time for the car
    double m_globalLiveTime;
    /// global wait time for the car, when it does not move
    int64_t m_globalWaitTime;
    /// amount of reservation requests for next cell
    int64_t m_reservationRequests;
    /// wait time for a next certain cell
    int64_t m_waitTimeNextCell;
    /// amount of external reservation requests
    int64_t m_externalReservationRequests;
    ///last preliminary reserved Cell in previous step (needed for reservationRequests)
    PathItem m_lastReserved;
    ///stores information on current neighbour cars, references are stored in m_neighbourCars (class Car)
    std::map<std::string, CarInformation> m_carInfo;
    ///stores, if the current solution can be applied and therefore new state can be reached
    bool m_reserved;
    ///distinction for use of discrete or continuous function
    SystemFunctionUsage m_systemFuncType;
    ///start position for the continuous system
    std::vector<double> m_startPos;
    std::vector<std::vector<double> > m_currentPos;
    static constexpr double m_intervalSplit = 100.0;
    ///global time of the simulation
    double m_globalTime;
    ///boundary for reserve next cell
    static constexpr double boundaryTol = 0.01;
    ///interval of control of dynamic
    std::vector<double> m_intervalControlDynamic;

};

#endif // SYSTEMFUNCTION_H
