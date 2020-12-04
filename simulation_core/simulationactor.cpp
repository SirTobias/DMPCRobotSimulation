#include "simulationactor.h"
#include <cstdint>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <QStringBuilder>


const int SimulationActor::created = 1;

/**
 * @brief creates an actor for the simulation which can be seen as a process
 * @param creator which kind of creation the object should arrive (appending on a file or a probability distribution)
 * @param name name of the actor
 * @param source the path to a file if the objects are created from file
 */
SimulationActor::SimulationActor(SimulationActorCreation creator, const QString &name, const QString &source, const QString &productConfig) :
    m_name(name),
    m_productConfig(productConfig)
{
    if (creator == SimulationActorCreation::UNIFORM) {
    }
    else if (creator == SimulationActorCreation::POISSON) {
    }
    else if (creator == SimulationActorCreation::FROMFILE) {
        QFile file(source);
        if (file.open(QIODevice::ReadOnly| QIODevice::Text)) {
            QTextStream textLine(&file);
            double arrivalTime = 0;
            double lastArrivalTime = 0;
            QString line = textLine.readLine();
            while (!line.isNull()) {
                if (line.contains(",")) {
                    QStringList lineList = line.split(",");
                    for (int i = 0; i < lineList.size(); i++) {
                        SimulationObject* object = new SimulationObject();
                        //TODO: hier noch berücksichtigen, dass unterschiedliche Zeiten für jedes Objekt vorliegen können
                        //dies muss in der Ressourcen-Produktkonfigurationen berücksichtigt werden
                        m_objectList.append(object);
                    }
                }
                //else there are only arrival times
                else {
                    bool isConverted = false;
                    arrivalTime = line.toDouble(&isConverted);
                    if (isConverted) {
                        //TODO: objectname is here taken from the actor, later it should be possible to set names for every object over filelist
                        SimulationObject* object = new SimulationObject(QString("SimObject_from") % getName() % QString("creatTime_") % QString::number(arrivalTime), m_productConfig);
                        double diffTime = arrivalTime - lastArrivalTime;
                        object->setEnterTime(diffTime);
                        lastArrivalTime = arrivalTime;
                        std::cout << object->getName().toLocal8Bit().data() << " created with enter time: " << object->getEnterTime() << std::endl;
                        m_objectList.append(object);
                    }
                }
                line = textLine.readLine();
            } //readline
        }
        else {
            std::cout << "Error: could not open file: " << source.toLocal8Bit().data() << std::endl;
        }
    }
}

/**
 * @brief Destructor
 */
SimulationActor::~SimulationActor() {

}

/**
 * @brief is needed by the adevs-base-classes
 * @param actor
 * @return true if equal, else false
 */
bool SimulationActor::operator==(const SimulationActor &actor) {
    if (*this == actor)
        return true;
    else
        return false;
}

/**
 * @brief internal event: remove first object from object list
 */
void SimulationActor::delta_int() {
    m_objectList.removeFirst();
}

/**
 * @brief the simlation actors have no input so this method would be empty
 * @param e the event time
 * @param bag the container containing the simulation objects
 */
void SimulationActor::delta_ext(double e, const adevs::Bag<IOLogistic> &bag) {

}

/**
 * @brief is called if a external and internal event are arriving
 * so an external event is not occurring here and only the internal method is called
 * @param xb
 */
void SimulationActor::delta_conf(const adevs::Bag<IOLogistic> &xb) {
    delta_int();
}

/**
 * @brief takes the first generated simulation actor in list for pushing it to the next actor
 * @param yb
 */
void SimulationActor::output_func(adevs::Bag<IOLogistic> &yb) {
    IOLogistic outputForProduce(created, m_objectList.first());
    yb.insert(outputForProduce);
}

/**
 * @brief gets the difference between the current time e.g. an incoming event and the end time of the event on which the actor is just working on
 * @return return the difference, if there is no event in the queue, DBL_MAX is returned
 */
double SimulationActor::ta() {
    if (m_objectList.isEmpty()) {
        return DBL_MAX;
    }
    else {
        return m_objectList.first()->getEnterTime();
    }
}

/**
 * @brief this function is for garbaging the produced objects that the actor gets or have created
 * be careful with objects you have not created and you delete here because of pointers from other objects
 * showing on this
 * @param gb Container with objects to be garbaged
 */
void SimulationActor::gc_output(adevs::Bag<IOLogistic> &gb) {

}


/**
 * @brief return the backward actors which are seen as predecessors
 * @return QList with predecessors
 */
QList<SimulationActor> SimulationActor::getBackwardActors() const {
    return this->m_backwardActors;
}

/**
 * @brief return the forward actors which are seen as successors
 * @return QList with successors
 */
QList<SimulationActor> SimulationActor::getForwardActors() const {
    return this->m_forwardActors;
}

/**
 * @brief return this global actor list
 * @return QList with all actors
 */
QList<SimulationActor*> SimulationActor::getActorList() const {
    return this->m_actorList;
}

/**
 * @brief appends backward actor at the backward list (predecessor)
 * @param actor to be appended at the bacward list
 */
void SimulationActor::appendBackwardActor(const SimulationActor &actor) {
    this->m_backwardActors.append(actor);
}

/**
 * @brief appends backward actor at the forward list (successor)
 * @param actor to be appended at the forward list
 */
void SimulationActor::appendForwardActor(const SimulationActor &actor) {
    this->m_forwardActors.append(actor);
}

/**
 * @brief removes the actor from the backward list (predecessor)
 * @param actor to be removed
 * @return true if it can be removed, else returns false
 */
bool SimulationActor::removeBackwardActor(const SimulationActor &actor) {
    return this->m_backwardActors.removeOne(actor);
}

/**
 * @brief removes the actor from the forward list (successor)
 * @param actor to be removed
 * @return true if it can be removed, else returns false
 */
bool SimulationActor::removeForwardActor(const SimulationActor &actor) {
    return this->m_forwardActors.removeOne(actor);
}

/**
 * @brief returns the name of the SimulationActor which is normally given by the constructor
 * @return QString containing the name
 */
QString SimulationActor::getName() const {
    return this->m_name;
}

/**
 * @brief set the name for the SimulationActor
 * @param name which should be set
 */
void SimulationActor::setName(const QString &name) {
    this->m_name = name;
}

/**
 * @brief SimulationActor::getSource
 * @return
 */
QString SimulationActor::getSource() const {
    return m_source;
}

/**
 * @brief set the path to the source if you read the creation of objects out of a file
 * @param source to the source
 */
void SimulationActor::setSource(const QString &source) {
    this->m_source = source;
}

/**
 * @brief get the list with the current object the actor has
 * @return QList with current objects
 */
QList<SimulationObject *> SimulationActor::getObjectList() const
{
    return m_objectList;
}

/**
 * @brief set the list with the current SimulationObjects, should not be needed for normal simulations
 * normally you should use SimulationActor::appendObjectInList
 * @param objectList list with the SimulationObjects
 */
void SimulationActor::setObjectList(const QList<SimulationObject *> &objectList)
{
    m_objectList = objectList;
}

/**
 * @brief appends a SimulationObject in the current list of the actor
 * @param obj object that should be appended
 */
void SimulationActor::appendObjectInList(SimulationObject* obj) {
    m_objectList.append(obj);
}

