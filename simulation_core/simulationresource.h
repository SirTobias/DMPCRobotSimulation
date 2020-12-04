 #ifndef SIMULATION_RESOURCE_H
#define SIMULATION_RESOURCE_H

#include "simsharedlib.h"

#include <QList>
#include "simdef.h"
#include "simulationactor.h"
#include "simulationobject.h"
#include "simulationresourceconfig.h"
#include "simulationresourceobserver.h"
#include "simulationresourceobservernotifier.h"
#include "QtCore/QPointer"
/**
 * @author Tobias Sprodowski
 * @brief This class defines a simulation resource which is needed by simulation actors.
 * Mostly this class would be modeled as a semaphore, e.g. a production unit in a logistic chain
 * or a street in traffic simulation.
 *
 *
 */
class SIM_CORE_EXPORT SimulationResource : public adevs::Atomic<IOLogistic>
{
public:
    SimulationResource(const QString &name = QString(), const SimulationResourceConfig &productConfig = SimulationResourceConfig());
    virtual ~SimulationResource();
    void addToInputQueue(QPointer<SimulationObject> object);
    void addToOutputQueue(QPointer<SimulationObject> object);
    ///internal transition function
    virtual void delta_int();
    ///external transition function
    virtual void delta_ext(double e, const adevs::Bag<IOLogistic> &bag);
    ///intermediate transition function
    virtual void delta_conf(const adevs::Bag<IOLogistic> &bag);
    virtual double ta();
    ///Output function
    virtual void output_func(adevs::Bag<IOLogistic> &op);
    ///garbage objects
    virtual void gc_output(adevs::Bag<IOLogistic> &gb);
    static const int arrive; /**< PortValues: Model Input Port */
    static const int depart; /**< PortValues: Model Output Port */

    ///get-set
    void setMaxProcessing(const unsigned long &max);
    void setFinishProcessingTime(const double &t);
    void setTime(const double &t);
    unsigned long getMaxProcessing() const;
    double getProcessingTime() const;
    double getTime() const;
    QString getName() const;
    void setName(const QString &name);
    SimulationResourceConfig getCurrentConfig() const;
    void setCurrentConfig(const SimulationResourceConfig &currentConfig);
    void printEvaluation();

protected:

    ///resource config for each product
    SimulationResourceConfig m_currentConfig;
    ///input queue with objects to be worked on
    QList<QPointer<SimulationObject> > m_inputQueue;
    /// output queue with objects that are finished
    QList<QPointer<SimulationObject> > m_outputQueue;
    ///Simulation observer notifier
    QPointer<SimulationResourceObserverNotifier> m_simResObserveNotifier;
    /// max. parallel eradication
    unsigned long m_maxProcessing;
    /// time to finish object
    double m_processingTime;
    ///current local time
    double m_time;
    /// time spent on object until now
    double m_spent;
    /// name of the resource which is observed
    QString m_name;
};

#endif // SIMULATION_RESOURCE_H
