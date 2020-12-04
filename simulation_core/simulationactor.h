#ifndef SIMULATIONACTOR_H
#define SIMULATIONACTOR_H

#include "simsharedlib.h"
#include "simdef.h"

#include <QObject>
#include <QList>
#include <QString>


/**
 * @brief initializes the SimulationActor with the source
 * either it uses a probability distribution or a file which intializes the simulation objects
 *
 */
enum class SimulationActorCreation : unsigned int __attribute__((visibility("default"))) {
    UNIFORM = 0,
    POISSON = 1,
    FROMFILE = 2
};

class SimulationActor;

/**
 * @brief the SimulationActor is the base class for actors like a logistic chain which handles simulation objects and processes them
 * it is also possible to communicate with the other actors
 *
 */
class SIM_CORE_EXPORT SimulationActor : public adevs::Atomic<IOLogistic>
{

public:
    SimulationActor(SimulationActorCreation creator, const QString &name = QString(), const QString &source = QString(), const QString &productConfig = QString());
    virtual ~SimulationActor();

    //virtual Simulation Methods
    virtual void delta_int();
    virtual void delta_ext(double e, const adevs::Bag<IOLogistic> &bag);
    virtual void delta_conf(const adevs::Bag<IOLogistic> &xb);
    virtual void output_func(adevs::Bag<IOLogistic> &yb);
    virtual double ta();
    virtual void gc_output(adevs::Bag<IOLogistic> &gb);

    //get-set
    QList<SimulationActor> getBackwardActors() const;
    QList<SimulationActor> getForwardActors() const;
    QList<SimulationActor*> getActorList() const;
    void appendBackwardActor(const SimulationActor &actor);
    void appendForwardActor(const SimulationActor &actor);
    bool removeBackwardActor(const SimulationActor &actor);
    bool removeForwardActor(const SimulationActor &actor);
    bool operator==(const SimulationActor &actor);
    QString getName() const;
    void setName(const QString &name);
    QString getSource() const;
    void setSource(const QString &source);
    QList<SimulationObject *> getObjectList() const;
    void setObjectList(const QList<SimulationObject *> &objectList);
    void appendObjectInList(SimulationObject *obj);
    static const int created; /**< Output port of the created simulation object */

private:
    ///predecessor actor list
    QList<SimulationActor> m_backwardActors;
    ///successor actor list
    QList<SimulationActor> m_forwardActors;
    ///holds an instance to the global actor list
    QList<SimulationActor*> m_actorList;
    ///current Objects which are in the queue
    QList<SimulationObject*> m_objectList;
    ///local time of the actor
    double m_time;
    ///name of the actor
    QString m_name;
    ///source of the file where the SimulationObjects are created
    QString m_source;
    ///product configuration
    QString m_productConfig;
};

#endif // SIMULATIONACTOR_H
