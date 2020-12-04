#ifndef SIMULATIONRESOURCEOBSERVERNOTIFIER_H
#define SIMULATIONRESOURCEOBSERVERNOTIFIER_H
#include <QObject>
#include "simulationresourceobserver.h"
#include <QPointer>

/**
 * @brief this class notifies any connected objects over signals if any change is happened to the connected SimulationResource
 */
class SimulationResourceObserverNotifier : public QObject
{
    Q_OBJECT
public:
    SimulationResourceObserverNotifier(const QString &resName);
    ~SimulationResourceObserverNotifier();
signals:
    void notifyAddAllTime(const double &t);
    void notifyAddIdleTime(const double &t);
    void notifyAddWorkedTime(const double &t);
    void notifyAddThroughPut(const double &t);
    void notifyAddCurrentProgress(const double &t);
    void notifyResetCurrentProgress();
    void notifyprintEvaluation();
private:
    ///Simulation observer reference
    QPointer<SimulationResourceObserver> simResObserver;
    ///observed resource name
    QString m_resourceName;
};

#endif // SIMULATIONRESOURCEOBSERVERNOTIFIER_H
