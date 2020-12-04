#include "simulationactorlist.h"

QList<SimulationActor*> SimulationActorList::actors = QList<SimulationActor*>();

/**
 * @brief creates the current global simulation list
 */
SimulationActorList::SimulationActorList()
{

}

/**
 * @brief deletes each actor in the simulationactorlist
 */
SimulationActorList::~SimulationActorList() {
    for (int i = 0; i < SimulationActorList::actors.size(); i++) {
        delete actors[i];
    }
}

/**
 * @brief register an actor for the global SimulationActorList
 * @param actor which should be registered
 */
void SimulationActorList::registerSimulationActor(SimulationActor* actor) {
    Q_ASSERT_X(actor,__FUNCTION__ ,"pointer for unregister an actor should not be NULL");
    if (actor) {
        std::cout << "simulationactor " << actor->getName().toLocal8Bit().data() << " added" << std::endl;
        actors.append(actor);
    }
}

/**
 * @brief unregister SimulationActor normally which should be deleted
 * @param actor that should be u    nregistered
 * @return true if actor is found and can be removed
 */
bool SimulationActorList::unregisterSimulationActor(SimulationActor* actor) {
    Q_ASSERT_X(actor,__FUNCTION__ ,"pointer for unregister an actor should not be NULL");
    return actors.removeOne(actor);
}

/**
 * @brief returns the complete actorlist
 * @return QList with all current actors
 */
QList<SimulationActor*> SimulationActorList::getActorList() {
    return SimulationActorList::actors;
}
