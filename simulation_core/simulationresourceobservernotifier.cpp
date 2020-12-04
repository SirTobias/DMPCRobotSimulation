#include "simulationresourceobservernotifier.h"
#include <QtCore/QStringBuilder>
#include <QtCore/QDebug>

/**
 * @brief SimulationResourceObserverNotifier::SimulationResourceObserverNotifier
 */
SimulationResourceObserverNotifier::SimulationResourceObserverNotifier(const QString &resName) :
    m_resourceName(resName)
{
    simResObserver = new SimulationResourceObserver(m_resourceName);
    if (simResObserver) {
        connect(this, SIGNAL(notifyAddWorkedTime(double)), simResObserver, SLOT(addWorkedTime(double)));
        connect(this, SIGNAL(notifyAddAllTime(double)), simResObserver, SLOT(addAllTime(double)));
        connect(this, SIGNAL(notifyAddIdleTime(double)), simResObserver, SLOT(addIdleTime(double)));
        connect(this, SIGNAL(notifyAddThroughPut(double)), simResObserver, SLOT(addThroughPut(double)));
        connect(this, SIGNAL(notifyprintEvaluation()), simResObserver, SLOT(printEvaluation()));
        connect(this, SIGNAL(notifyAddCurrentProgress(double)), simResObserver, SLOT(addCurrentProgress(double)));
        connect(this, SIGNAL(notifyResetCurrentProgress()), simResObserver, SLOT(resetCurrentProgress()));
        qDebug() << QString("SimulationResourceObserver for " % m_resourceName % "created\n");
    }
}

/**
 * @brief Destructor: deletes the belonging SimulationResourceObserver and itself
 */
SimulationResourceObserverNotifier::~SimulationResourceObserverNotifier() {
    if (simResObserver) {
        simResObserver->disconnect();
        delete simResObserver;
    }
    simResObserver = nullptr;
}
