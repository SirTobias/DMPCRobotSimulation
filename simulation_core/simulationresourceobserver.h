#ifndef SIMULATIONRESOURCEOBSERVER_H
#define SIMULATIONRESOURCEOBSERVER_H
#include "simsharedlib.h"

#include "simulationobserver.h"

/**
 * @brief The SimulationResourceObserver class implements the observer methods to register the incoming values for adding for example time values signaled by the SimulationResource.
 */
class SIM_CORE_EXPORT SimulationResourceObserver : public SimulationObserver<double>
{
    Q_OBJECT
public:
    SimulationResourceObserver(const QString &resName);
    double getCurrentProgress() const;
    double getThroughput() const;
    double getAllTime() const;
    double getWorkedTime() const;
    double getIdleTime() const;

public slots:
    void addCurrentProgress(const double &t);
    void resetCurrentProgress();
    void addAllTime(const double &t);
    void addIdleTime(const double &t);
    void addWorkedTime(const double &t);
    void addThroughPut(const double &t);
    void printEvaluation();

private:
    ///collects the progress on the current object
    double m_currentProgress;
    ///throughput here defined as quotient of m_summedTime to m_allTime
    double m_throughput;
    ///summed worked time
    double m_workedTime;
    ///idled time the resource did wait or not working
    double m_idleTime;
    ///all time - sum of worked and idle
    double m_allTime;
    ///SimulationResource object name
    QString m_resourceName;
};

#endif // SIMULATIONRESOURCEOBSERVER_H
