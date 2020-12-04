#ifndef SIMULATIONOBSERVER_H
#define SIMULATIONOBSERVER_H
#include "simsharedlib.h"

#include <QObject>

/**
 * This class is the base class for the simulation observer.
 * The events are taken by signals called by the objects and executed in the observer classes
 */
template<class T> class SIM_CORE_EXPORT SimulationObserver : public QObject
{
public:
    SimulationObserver() {}
    void printEvaluation();
private:
};

#endif // SIMULATIONOBSERVER_H
