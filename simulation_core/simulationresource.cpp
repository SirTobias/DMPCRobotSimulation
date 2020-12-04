#include "simulationresource.h"


const int SimulationResource::arrive = 0;
const int SimulationResource::depart = 1;

/**
 * @brief Standard-Constructor with standard parameters
 */
SimulationResource::SimulationResource(const QString &name, const SimulationResourceConfig &productConfig) :
    Atomic<IOLogistic>(),
    m_currentConfig(productConfig),
    m_inputQueue(QList<QPointer<SimulationObject> >()),
    m_outputQueue(QList<QPointer<SimulationObject> >()),
    m_maxProcessing(0),
    m_processingTime(0),
    m_time(0),
    m_spent(0),
    m_name(name)
{
    m_simResObserveNotifier = new SimulationResourceObserverNotifier(m_name);

}

/**
 * @brief Destructor: deletes also all objects in the output and input queue
 */
SimulationResource::~SimulationResource() {
    for (int i = 0; i < m_outputQueue.size(); i++) {
        QPointer<SimulationObject> obj = m_outputQueue[i];
        if (obj) {
            delete obj;
        }
        obj = nullptr;
    }
    for (int i = 0; i < m_inputQueue.size(); i++) {
        QPointer<SimulationObject> obj = m_inputQueue[i];
        if (obj) {
            delete obj;
        }
        obj = nullptr;
    }
    if (m_simResObserveNotifier) {
        m_simResObserveNotifier->disconnect();
        delete m_simResObserveNotifier;
    }
    m_simResObserveNotifier = nullptr;
}

/**
 * @brief internal transition function
 * calls the next object in the queue
 */
void SimulationResource::delta_int() {
    std::cout << "internal function called at " << getTime() << std::endl;
    //update the internal clock
    double timeAdvance = ta();
    m_time += timeAdvance;
    //add only if there is any event coming, because otherwise the real time is set at the end to DBL_MAX
    if (timeAdvance < DBL_MAX) {
        m_spent += timeAdvance;
        m_simResObserveNotifier->notifyAddAllTime(timeAdvance);
    }
    //notify with the whole time spent on the current leaving object
    if (m_spent != 0) {
        m_simResObserveNotifier->notifyAddWorkedTime(timeAdvance);
        m_simResObserveNotifier->notifyAddCurrentProgress(timeAdvance);
    }
    //reset time spent on object - busy time or workload for an object
    m_spent = 0;
    //remove first object from input queue
    if (!m_inputQueue.isEmpty()) {
        //due to the bags of adevs all handling about the objects has to be within bags
        m_inputQueue.removeFirst();
        std::cout << m_name.toLocal8Bit().data() << ": input-queue size: " << m_inputQueue.size() << std::endl;
        std::cout << m_name.toLocal8Bit().data() << ": next object is put in output-queue at: " << getTime() + timeAdvance << std::endl;
        //work time - alert observer
        if (m_simResObserveNotifier) {
            //notify to reset the currentProcessTime, because work on new object begins
            m_simResObserveNotifier->notifyResetCurrentProgress();
        }
    }
}

/**
 * @brief external transition function
 * the simulation object arrives here
 * @param e event time the objects arrive
 * @param bag the container which holds the objects
 */
void SimulationResource::delta_ext(double e, const adevs::Bag<IOLogistic> &bag) {
    std::cout << m_name.toLocal8Bit().data() << ": event coming in for input-queue: " << getTime() + e << std::endl;
    //local clock time update
    m_time += e;
    //observer signal for adding the whole time
    if (m_simResObserveNotifier) {
        m_simResObserveNotifier->notifyAddAllTime(e);
    }

    //update processing time for current object that is being processed
    if (!m_inputQueue.isEmpty()) {
        m_spent += e;
        //notify for working time on current object
        if (m_simResObserveNotifier) {
            m_simResObserveNotifier->notifyAddCurrentProgress(e);
            m_simResObserveNotifier->notifyAddWorkedTime(e);
        }
    }
    else {
        //if resource not working yet, add time to idletime
        m_simResObserveNotifier->notifyAddIdleTime(e);
    }
    //add incoming objects at the end of the input queue
    for (adevs::Bag<IOLogistic>::const_iterator it = bag.begin(); it != bag.end(); it++) {
        //TODO: lifecycle-time for object - try to avoid new
        QPointer<SimulationObject> simObj = (*it).value;
        if (simObj) {
            m_inputQueue.append(simObj);
            simObj->setEnterTime(getTime());
            //wait time for the object at this resource
            simObj->setWaitTime(getCurrentConfig().getProductConfig(simObj->getProductConfig()));
            std::cout << m_name.toLocal8Bit().data() << ": simulationobject " << simObj->getName().toLocal8Bit().data() << " put in input-queue at time " << getTime() << std::endl;
        }
    }
    std::cout << "next simulationobject " << m_inputQueue.first()->getName().toLocal8Bit().data() << " is ready at " << getTime() + ta() << std::endl;
}

/**
 * @brief intermediate transition function
 * calls first the internal function to get the next object
 * then call the external events to process the incoming events
 * @param bag external events which have to be processed
 */
void SimulationResource::delta_conf(const adevs::Bag<IOLogistic> &bag) {
    delta_int();
    delta_ext(0, bag);
}

/**
 * @brief calculates the time difference between current Objects wait time and the already spent time
 * @return time
 */
double SimulationResource::ta() {
    if (m_inputQueue.isEmpty()) {
        return DBL_MAX;
    }
    else {
        return m_inputQueue.first()->getWaitTime() - m_spent;
    }
}

/**
 * @brief takes the object from the input, process it and pass it to the output container
 * @param op output container
 */
void SimulationResource::output_func(adevs::Bag<IOLogistic> &op) {
    if (!m_inputQueue.isEmpty()) {
        QPointer<SimulationObject> object = m_inputQueue.first();
        if (object) {
            object->setLeaveTime(getTime() + ta());
            IOLogistic output(depart, object);
            op.insert(output);
            std::cout << m_name.toLocal8Bit().data() << ": simulationobject " << object->getName().toLocal8Bit().data() << " released at " << m_time + ta() << "." << std::endl;
        }
    }
}

/**
 * @brief is called after the SimulationResource::output_func(adevs::Bag<IOLogistic> &op
 * push the processed objects to the ouput queue
 * @param gb incoming objects which should be deleted
 */
void SimulationResource::gc_output(adevs::Bag<IOLogistic> &gb) {
    adevs::Bag<IOLogistic>::iterator it;
    for (it = gb.begin(); it != gb.end(); it++) {
        m_outputQueue.append((*it).value);
    }
    std::cout << m_name.toLocal8Bit().data() << ": size of output-queue: " << m_outputQueue.size() << std::endl;
    while (!m_outputQueue.isEmpty()) {
        std::cout << "delete " << m_outputQueue.first()->getName().toLocal8Bit().data() << std::endl;
        delete m_outputQueue.takeFirst();
    }
}

/**
 * @brief set the maximum of parallel processing objects
 * @param max number of objects
 */
void SimulationResource::setMaxProcessing(const unsigned long &max) {
    m_maxProcessing = max;
}

/**
 * @brief set the processing time that will be needed for finishing the current object
 * @param t
 */
void SimulationResource::setFinishProcessingTime(const double &t) {
    m_processingTime = t;
}

/**
 * @brief local time for the SimulationResource
 * @param t time
 */
void SimulationResource::setTime(const double &t) {
    m_time = t;
}

/**
 * @brief maximum of parallel processing objects
 * @return number of max processing objects
 */
unsigned long SimulationResource::getMaxProcessing() const {
    return m_maxProcessing;
}

/**
 * @brief he processing time that will be needed for finishing the current object
 * @return time
 */
double SimulationResource::getProcessingTime() const {
    return m_processingTime;
}

/**
 * @brief local time
 * @return time
 */
double SimulationResource::getTime() const {
    return m_time;
}

/**
 * @brief get Resource name
 * @return name
 */
QString SimulationResource::getName() const
{
    return m_name;
}

/**
 * @brief SimulationResource::setName
 * @param name
 */
void SimulationResource::setName(const QString &name)
{
    m_name = name;
}

/**
 * @brief returns a simulation config
 * @return SimulationConfig
 */
SimulationResourceConfig SimulationResource::getCurrentConfig() const
{
    return m_currentConfig;
}

/**
 * @brief SimulationResource::setCurrentConfig
 * @param currentConfig
 */
void SimulationResource::setCurrentConfig(const SimulationResourceConfig &currentConfig)
{
    m_currentConfig = currentConfig;
}

/**
 * @brief SimulationResource::addToInputQueue
 * @param object
 */
void SimulationResource::addToInputQueue(QPointer<SimulationObject> object) {
    m_inputQueue.append(object);
}

/**
 * @brief SimulationResource::addToOutputQueue
 * @param object
 */
void SimulationResource::addToOutputQueue(QPointer<SimulationObject> object) {
    m_outputQueue.append(object);
}

/**
 * @brief SimulationResource::printEvaluation
 */
void SimulationResource::printEvaluation() {
    if (m_simResObserveNotifier) {
        m_simResObserveNotifier->notifyprintEvaluation();
    }
}
