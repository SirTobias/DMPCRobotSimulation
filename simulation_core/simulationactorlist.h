#ifndef SIMULATIONACTORLIST_H
#define SIMULATIONACTORLIST_H
#include "simsharedlib.h"

#include <QList>
#include <QPointer>
#include "simulationactor.h"

/**
 * @brief @brief This class provides an ActorList with Pointers to another Actors
 * This is a base structure for the actors to organize their successor and predessor lists
 *
 */
class SIM_CORE_EXPORT SimulationActorList
{
public:
    SimulationActorList();
    ~SimulationActorList();
    ///register
    static void registerSimulationActor(SimulationActor* actor);
    static bool unregisterSimulationActor(SimulationActor* actor);
    ///get-set
    static QList<SimulationActor*> getActorList();
    ///actor which are here globally registered
    static QList<SimulationActor*> actors;
};

#endif // SIMULATIONACTORLIST_H
