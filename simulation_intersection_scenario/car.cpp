#include "car.h"
#include "costfunction.h"
#include "intersection.h"
#include "astarpathcalculation.h"
#include "dstarlite.h"
#include "floydwarshallpathcalculation.h"
#include "mpccontroller.h"
#include "globalcarlist.h"

#include <QtCore/QDebug>


/**
 * @brief Car::Car constructs the car
 * with cost function and target with discrete system function
 * @param name id of the car
 * @param start start cell
 * @param target target cell
 * @param lambda damping parameter for the control
 * @param pathAlgorithm chosen pathAlgorithm
 * @param T sampling interval
 */
Car::Car(const QString &name, const PathItem &start, const PathItem& target, const size_t &N, const double& lambda, const PathAlgorithm& pathAlgorithm, const double& T, const std::pair<double, double> &bounds) :
    m_start(start),
    m_target(target),
    m_name(name),
    m_currentSolutionReserved(false),
    m_countNeighbourCars(0),
    m_countCommunicatedConstraints(0),
    m_differentConstraints(0),
    m_controlBounds(bounds),
    m_delta(0),
    m_numberCellsReserved(0)
{
    if (pathAlgorithm == PathAlgorithm::ASTARCENTRALIZED || pathAlgorithm == PathAlgorithm::ASTARDECENTRALZED) {
        m_pathCalc = std::make_shared<AStarPathCalculation>(name, start, target, T, pathAlgorithm);
    }
    else if (pathAlgorithm == PathAlgorithm::DSTAR) {
        m_pathCalc = std::make_shared<DstarLite>(name, start, target, T);
    }
    else if (pathAlgorithm == PathAlgorithm::FLOYDWARSHALL) {
        m_pathCalc = std::make_shared<FloydWarshallPathCalculation>(name, start, target, T);
    }
    else if (pathAlgorithm == PathAlgorithm::MPCCOBYLA) {
        m_pathCalc = std::make_shared<MpcController>(name, start, target, N, lambda, bounds);
    }
}

/**
 * @brief Car::Car constructs the car with continuous system function
 * @param name id of the car
 * @param start continuous start position of the car
 * @param target continuous target point of the car
 * @param N horizon length to initialize controller
 * @param lambda fraction for control impact
 * @param pathAlgorithm chosen PathAlgorithm
 */
Car::Car(const QString& name, const std::vector<double>& start, const std::vector<double>& target, const size_t &N, const double &lambda, const PathAlgorithm &pathAlgorithm, const double& T, const std::pair<double, double> &bounds) :
    m_start(PathItem()),
    m_target(PathItem()),
    m_name(name),
    m_currentSolutionReserved(false),
    m_countNeighbourCars(0),
    m_countCommunicatedConstraints(0),
    m_differentConstraints(0),
    m_delta(0),
    m_numberCellsReserved(0)
{
    //m_mpcControl = std::make_shared<MpcController>(name, start, target, N, lambda);
    if (pathAlgorithm == PathAlgorithm::ASTARCENTRALIZED || pathAlgorithm == PathAlgorithm::ASTARDECENTRALZED) {
        m_pathCalc = std::make_shared<AStarPathCalculation>(name, start, target, T, pathAlgorithm);
    }
    else if (pathAlgorithm == PathAlgorithm::DSTAR) {
        m_pathCalc = std::make_shared<DstarLite>(name, start, target);
    }
    else if (pathAlgorithm == PathAlgorithm::FLOYDWARSHALL) {
        m_pathCalc = std::make_shared<FloydWarshallPathCalculation>(name, start, target);
    }
    else if (pathAlgorithm == PathAlgorithm::MPCCOBYLA) {
        m_pathCalc = std::make_shared<MpcController>(name, start, target, N, lambda, bounds);
    }
}

/**
 * @brief Car::getPath returns the current path
 * @return
 */
Path Car::getPath() {
    return m_pathCalc->getSystemFunction()->getPath();
}

/**
 * @brief Car::getPath returns current path as const ref
 * @return
 */
const Path& Car::getPath() const {
    return m_pathCalc->getSystemFunction()->getPath();
    //return m_mpcControl->getSystemFunction()->getPath();
}

/**
 * @brief Car::getStart returns discrete start position
 * @return
 */
PathItem Car::getStart() const {
    return m_start;
}

/**
 * @brief Car::getCurrentState current discrete position in the intersection field
 * @return PathItem with current position
 */
PathItem Car::getCurrentState() const {
    return m_pathCalc->getSystemFunction()->getCurrentState();
    //return m_mpcControl->getSystemFunction()->getCurrentState();
}

/**
 * @brief Car::getCurrentStateContinuous returns continuous state position
 * @return
 */
std::vector<double> Car::getCurrentStateContinuous() const {
    return m_pathCalc->getSystemFunction()->getCurrentContinuousState();
    //return m_mpcControl->getSystemFunction()->getCurrentContinuousState();
}

/**
 * @brief Car::getName
 * @return
 */
QString Car::getName() const {
    return m_name;
}

/**
 * @brief Car::calcOcpObjective calls the internal MPC-Controller and optimize in the <b>discrete setting</b>.
 * @param start  start position
 */
PathItem Car::calcOcpObjective(const PathItem& start) {
    return m_pathCalc->optimize(start);
}

/**
 * @brief Car::calcOcpObjectiveContinuous calls the optimization in the <b>continuous setting</b>.
 * @param N horizon length
 * @param start x0
 * @param t0 start time
 * @param T sampling step
 * @return optimal control as vector
 */
std::vector<std::vector<double> > Car::calcOcpObjectiveContinuous(const std::vector<double> &start, const double &t0, const double &T) {
    return m_pathCalc->optimizeContinous(start, t0, T);
}

/**
 * @brief Car::hasTargetReached evaluates if the car has reached the target
 * @return true, if current position equals target, otherwise false
 */
bool Car::hasTargetReached() const {
    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
        return (m_pathCalc->getSystemFunction()->getCurrentState() == m_target);
    }
    else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        return VectorHelper::norm2(VectorHelper::sub(m_pathCalc->getSystemFunction()->getCurrentContinuousState() , m_pathCalc->getTargetContinuous()) ) < 0.1;
    }
}

/**
 * @brief Car::reservePrelimSolution reserve the prelim. solution and if it succeeds,
 * it changes to the new state and add this to the path
 * @return
 */
bool Car::reservePrelimSolution() {
    m_currentSolutionReserved = m_pathCalc->getSystemFunction()->reservePrelimSolution(m_pathCalc->getCurrentPrelimSolution());
    return m_currentSolutionReserved;
}

//TODO: refactor Car::m_path, should only include Car::mpcFunction::systemState::m_path
/**
 * @brief Car::applyNextState in <b>discrete scenarios</b> transfers the car to the next state if the solution can be applied
 */
void Car::applyNextState() {
    //now take the new state
    if (m_currentSolutionReserved) {
        m_pathCalc->getSystemFunction()->applyNextState(m_pathCalc->getCurrentPrelimSolution());
        m_path.addPathItem(m_pathCalc->getCurrentPrelimSolution());
    }
}

/**
 * @brief Car::applyNextState in <b> continuous scenarios</b> transfers the car to the next state
 * @param u optimal control
 * @param t0 start time
 * @param T sampling step
 */
void Car::applyNextState(const std::vector<double> &u, const double &t0, const double &T) {
    m_pathCalc->getSystemFunction()->applyNextState(u, t0, T);
}

/**
 * @brief Car::currentPrelimSolutionInvalid for discrete scenarios verify, if solution is applicable
 * @return
 */
bool Car::isCurrentPrelimSolutionValid() const {
   return m_currentSolutionReserved;
}

/**
 * @brief Car::getTarget returns the discrete target position
 * @return PathItem with discrete cell indices
 */
PathItem Car::getTarget() const {
   return m_target;
}

/**
 * @brief Car::getTargetContinous returns the continuous target position
 * @return coordinates as continuous variables
 */
std::vector<double> Car::getTargetContinous() const {
    return m_pathCalc->getTargetContinuous();
}

/**
 * @brief Car::getCurrentAbsDistance gets the absolute of the difference of the current distance
 * time will not be considered
 * @return
 */
double Car::getCurrentAbsDistance() const {
    return std::sqrt(std::pow(m_target.getX() - m_pathCalc->getSystemFunction()->getCurrentState().getX(), 2)
            + std::pow(m_target.getY() - m_pathCalc->getSystemFunction()->getCurrentState().getY(), 2));
}

/**
 * @brief Car::getCurrentStageCosts calculates the current costs for the car in the current stage
 * @return
 */
double Car::getOpenLoopCosts() const {
    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
        return CostFunction::calcCurrentStateCosts(m_pathCalc->getSystemFunction()->getCurrentState(), m_pathCalc->getCurrentPrelimSolution(), m_target, m_pathCalc->getLambda());
    }
    else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        return m_pathCalc->getOpenLoopCosts();
    }
}

/**
 * @brief Car::getClosedLoopCosts returns closed loop costs
 * @return
 */
double Car::getClosedLoopCosts() const {
    return m_pathCalc->getClosedLoopCosts();
}

/**
 * @brief Car::getInitialControl returns the start control
 * @return
 */
std::vector<double> Car::getInitialControl(const double &t0, const double& T) {
   return m_pathCalc->getInitialControl(t0, T);
}


/**
 * @brief Car::getMessage parse the incoming message and
 * assign this to the car
 * @param msg
 * @return
 */
/*CarComm::Cell Car::getMsg(const std::string& msg) {
    CarComm::Cell parsedMsg;
    parsedMsg.ParseFromString(msg);
    if (parsedMsg.IsInitialized()) {
        std::string id = parsedMsg.car();
        CarInformation carInfo(id);
        carInfo.setLastState(PathItem(parsedMsg.k(), parsedMsg.m(), parsedMsg.t()));
        carInfo.setGlobalLiveTime(parsedMsg.globallivetime());
        carInfo.setGlobalWaitTime(parsedMsg.globalwaittime());
        carInfo.setReservationRequests(parsedMsg.reserverequestsnextcell());
        carInfo.setWaitTimeNextCell(parsedMsg.waittimenextcell());
        carInfo.setExternalReservationRequests(parsedMsg.externalrequests());
        m_pathCalc->getSystemFunction()->setCarInfo(id, carInfo);
    }
    return parsedMsg;
}*/

/**
 * @brief Car::getNeighbours calculates the neighbours with <= distance given by parameter
 * and returns them in a list
 * @param distance given distance, standard is 1
 * @return list with neighbours distance <= distance
 */
std::vector<std::shared_ptr<Car> > Car::getNeighbours(const unsigned int& distance) {
    std::vector<std::shared_ptr<Car> > neighbourList;
    PathItem& curPos = m_path.back();
    for (const std::shared_ptr<Car> car : GlobalCarList::getInstance()) {
        if (curPos.isNeighbouredTo(car->getCurrentState(), distance)) {
            //connect to neighbours - @TODO: not the best, as the Qt Macro cannot handle the shared ptr, therefore shared_ptr.get()
            connect(shared_from_this().get(), SIGNAL(sendMsg(std::shared_ptr<const Car>, const std::string)), car.get(), SLOT(getMsg(const std::string)));
            neighbourList.push_back(car);
            m_countNeighbourCars++;
        }
    }
    m_neighbourCars = neighbourList;

    return neighbourList;
}

/**
 * @brief Car::sendCostsToNeighbours sends local parameters (global wait time, reservation requests, wait time for next cell and the external reservation requests to the neighbours
 * with given distance (standard=1)
 * @param distance max. distance which neighbours should receive it
 */
void Car::sendCostsToNeighbours(const int& distance) {
    /*std::vector<std::shared_ptr<Car> > neighbourList = */getNeighbours(distance);
    /*CarComm::Cell msg;
    msg.set_k(m_pathCalc->getSystemFunction()->getCurrentState().getX());
    msg.set_m(m_pathCalc->getSystemFunction()->getCurrentState().getY());
    msg.set_car(m_name.toLatin1().data());
    msg.set_globallivetime(m_pathCalc->getSystemFunction()->getGlobalLiveTime());
    msg.set_globalwaittime(m_pathCalc->getSystemFunction()->getGlobalWaitTime());
    msg.set_reserverequestsnextcell(m_pathCalc->getSystemFunction()->getReservationRequests());
    msg.set_waittimenextcell(m_pathCalc->getSystemFunction()->getWaitTimeNextCell());
    msg.set_externalrequests(m_pathCalc->getSystemFunction()->getExternalReservationRequests());
    //One-Signal-many-Slot: therefore only one call necessary
    std::string serializedMsg;
    msg.SerializeToString(&serializedMsg);
    sendMsg(shared_from_this(), serializedMsg);
*/
}

/**
 * @brief Car::setGlobalLiveTime for each car as long as exists in the simulation
 * @param timeStep current time
 */
void Car::setGlobalLiveTime(const double& timeStep) {
    m_pathCalc->getSystemFunction()->setGlobalLiveTime(timeStep);
}

/**
 * @brief Car::resetConnectionToNeighbours resets the connection to neighbours
 * if, e.g., the solutions are applied and next time step is set
 */
void Car::resetConnectionToNeighbours() {
    for (std::shared_ptr<Car> neighbour : m_neighbourCars) {
        //disconnect from neighbours - @TODO: not the best solution, as the Qt Macro cannot handle the shared ptr, therefore shared_ptr.get()
        disconnect(shared_from_this().get(), SIGNAL(sendMsg(std::shared_ptr<Car>,std::string)), neighbour.get(), SLOT(getMsg(std::string)));
    }
    m_countNeighbourCars = 0;
}

/**
 * @brief Car::getCountNeighbouredCars return the number of current neighboured cars
 * @return number current neighboured cars which are connected (signal/slot)
 */
int Car::getCountNeighbouredCars() const {
    return m_countNeighbourCars;
}

/**
 * @brief Car::calcCostsForNeighbours calculates the costs for the neigbouring cars to generate the cost map
 * @return
 */
std::map<std::string, double> Car::calcCostsForNeighbours() {
    m_neighbourCostMap = m_pathCalc->calcCostsForNeighbours();
    return m_neighbourCostMap;
}

/**
 * @brief Car::sortAscendingCostsForNeighbours maps the cost map sorted by carids to a multimap to sort by the cost values
 * @param neighbourCostMap cost map to sort
 * @return sorted cost map
 */
std::multimap<double, std::string> Car::sortAscendingCostsForNeighbours(const std::map<std::string, double> &neighbourCostMap) const {
    std::multimap<double, std::string> costMultiMap;
    for(auto it : neighbourCostMap) {
        costMultiMap.insert(std::pair<double, std::string>(it.second, it.first));
    }
    return costMultiMap;
}

/**
 * @brief Car::initializeCosts initialize the costs of each car and send them to the neighbours in the first simulation step
 */
void Car::initializeCosts() {
   m_pathCalc->initializeCosts();
   sendCostsToNeighbours();
}

/**
 * @brief Car::getAscendingNeighbourCostMap returns the multimap with costs as key from belonging neighbours
 * @return multimap
 */
std::multimap<double, std::string> Car::getAscendingNeighbourCostMap() const {
    return m_neighbourSortedCosts;
}

/**
 * @brief Car::setAscendingNeighbourCostMap sets the multimap with costs as key from belonging neighbours
 * @param map
 */
void Car::setAscendingNeighbourCostMap(const std::multimap<double, std::string>& map) {
    m_neighbourSortedCosts = map;
}

/**
 * @brief Car::identifyReservationConflictsWithNeighbours
 */
void Car::identifyReservationConflictsWithNeighbours() {
    //TODO: hier fortfahren
}

/**
 * @brief Car::formulateConstraintsForNextCar gets the current cells which should be reserved by the current prediction
 * and constructs the necessary continuous constraints which are inserted in the OCP of the following cars
 * The constraints are now timedependant, which means, that if the position is hold at $\ft_0\f$, then it onlys
 * looks for k=0
 * @param u current control
 * @param t0 start time
 * @param T sampling step
 * @param N horizon length
 * @param firstCar, if it is first car, last prediction step has not to be doubled
 * @param commScheme communicate only the new or changed constraints
 * @return
 */
std::vector<Constraint> Car::formulateConstraintsForNextCar(const std::vector<std::vector<double> > &u, const double& t0, const double& T,
                                                            const size_t &N, const bool& firstCar, const double& radius, const double& dmin, const CommunicationScheme& commScheme) {

    m_occupiedCells.clear();
    std::vector<Constraint> constraintVec;
    //get the prediction with future states that the optimizer obtained
    std::vector<std::vector<double> > x =
            m_pathCalc->getSystemFunction()->getHolonomicSystemTrajectory(m_pathCalc->getSystemFunction()->getCurrentContinuousState(), u, t0, T, N);
    //get the cells which the trajectory should map to
    if (commScheme == CommunicationScheme::FULL || commScheme == CommunicationScheme::DIFFERENTIAL
            || commScheme == CommunicationScheme::MINMAXINTERVAL || commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        Path cellToReserve = m_pathCalc->getSystemFunction()->mapPredictionToCells(x, t0, T);
        for (PathItem& pathItem : cellToReserve) {
            m_occupiedCells.insert(pathItem.getX(), pathItem.getY());
        }
        //for which prediction step the constraint should hold
        unsigned int step = 0;
        for (const PathItem &pathItem : cellToReserve) {
            //construct the constraint with the middlepoint of the cell as center point
            //and make the constraint timedependant
            constraintVec.emplace_back(Constraint({InterSection::getInstance()->getCellSize() * ((double)pathItem.getX() + 0.5),
                                                   InterSection::getInstance()->getCellSize() * ((double)pathItem.getY() + 0.5)},
                                                  pathItem.getTime(), SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin, m_controlBounds.second));
            step++;
            //m_occupiedCells.insert(pathItem.getX(), pathItem.getY());
        }
        if (!firstCar) {
            constraintVec.emplace_back(Constraint({InterSection::getInstance()->getCellSize() * ((double)cellToReserve.back().getX() + 0.5),
                                                   InterSection::getInstance()->getCellSize() * ((double)cellToReserve.back().getY() + 0.5)},
                                                  cellToReserve.back().getTime() + T, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin, m_controlBounds.second ) );
        }
    }//--SpacialSet::Quantised
    else if (commScheme == CommunicationScheme::CONTINUOUS) {
        double timePred = t0;
        for (const std::vector<double> contPredStep : x) {
            constraintVec.emplace_back(Constraint({contPredStep.at(0), contPredStep.at(1)},
                                                  timePred, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin, m_controlBounds.second));
            timePred +=T;
        }
        if (!firstCar) {
            timePred += T;
            constraintVec.emplace_back(Constraint({x.back().at(0), x.back().at(1)}, timePred, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin, m_controlBounds.second ) );
        }
    }//--SpacialSet::Continuous

    //DEBUG
    /*for (auto it = constraintVec.cbegin(); it != constraintVec.cend(); it++) {
        qDebug() << "Prediction: " << this->m_name << " " << it->getConstraintTime() << "("
            << it->getCenterPoint().at(0) << "," << it->getCenterPoint().at(1) << ")";
    }*/
    //--DEBUG
    if (commScheme == CommunicationScheme::DIFFERENTIAL) {
        //difference here now the constraints, that only new or changed constraints are broadcasted
        std::vector<Constraint> diffConstraints = differenceConstraints(constraintVec, m_communicatedConstraints);
        for (auto it = diffConstraints.cbegin(); it != diffConstraints.end(); it++) {
            m_communicatedConstraints.emplace_back(*it);
        }
        m_countCommunicatedConstraints += diffConstraints.size();
        m_differentConstraints = diffConstraints.size();
        return diffConstraints;
    }
    else {
        m_countCommunicatedConstraints += constraintVec.size();
        m_differentConstraints = constraintVec.size();
        return constraintVec;
    }
}

/**
 * @brief Car::calculateMinMaxConstraintForNextCar using the Interval Superposition Principle, we calculate here the min and max constraints for the cars
 * @param u control imposed for the car to calculate trajectory
 * @param t0 start time
 * @param T sampling interval
 * @param N prediction horizon for the trajectory
 * @param firstCar is this the first car?
 * @param radius radius of the cell
 * @param dmin minimum diameter of robots
 * @param scheme communication scheme
 * @return minimum and maximum constraint as vector (better would be pair, but this is not possible due to compatibility reasons for the other communication scheme)
 */
std::vector<Constraint> Car::calculateMinMaxConstraintForNextCar(const std::vector<std::vector<double> > &u, const double& t0, const double& T, const size_t &N,
                                                                 const bool& firstCar, const double& radius, const double& dmin, const CommunicationScheme& scheme) {
    //full or reduced communication does not matter here
    //for easier calculations, we continue here with full communication
    std::vector<Constraint> constraints = formulateConstraintsForNextCar(u, t0, T, N, firstCar, radius, dmin, CommunicationScheme::FULL);
    std::vector<double> minCenterPoint = {0.0, 0.0}, maxCenterPoint = {0.0, 0.0};
    std::vector<Constraint> minMaxConstraints;
    //first value has to be set, otherwise minimum will always be 0,0
    bool firstValue = true;
    for (auto itConstraint = constraints.begin(); itConstraint != constraints.end(); itConstraint++) {
        if (firstValue) {
            minCenterPoint = itConstraint->getCenterPoint();
            maxCenterPoint = itConstraint->getCenterPoint();
            firstValue = false;
        }
        else {
            //x_min,new < x_min,old
            if (itConstraint->getCenterPoint().at(0) < minCenterPoint.at(0)) {
                minCenterPoint[0] = itConstraint->getCenterPoint().at(0);
            }
            //y_min,new < y_minold
            if (itConstraint->getCenterPoint().at(1) < minCenterPoint.at(1)) {
                minCenterPoint[1] = itConstraint->getCenterPoint().at(1);
            }
            //x_max,new > x_max,old
            if (itConstraint->getCenterPoint().at(0) > maxCenterPoint.at(0)) {
                maxCenterPoint[0] = itConstraint->getCenterPoint().at(0);
            }
            //y_max,new > y_max,old
            if (itConstraint->getCenterPoint().at(1) > maxCenterPoint.at(1)) {
                maxCenterPoint[1] = itConstraint->getCenterPoint().at(1);
            }
        }
    }
    std::vector<double> uMax = {m_pathCalc->getControlBounds().second, m_pathCalc->getControlBounds().second};
    //minimum time needed to get from start to end point
    size_t minTimeNeeded = getMinimumTimeStepsForDistance(minCenterPoint, maxCenterPoint, uMax, t0, T, N);
    //delta is the time, when the car has to move
    size_t delta = N - minTimeNeeded;
    m_delta = delta;
    //setting number of constraints here, for normal MINMAX-Interval it is (x_max - x_min + 1) * (y_max - y_min + 1)
    m_numberCellsReserved = (std::abs(maxCenterPoint.at(0) - minCenterPoint.at(0)) + radius) / radius * (std::abs(maxCenterPoint.at(1) - minCenterPoint.at(1)) + radius) / radius * (double)N;
    /*if (scheme == CommunicationScheme::MINMAXINTERVAL) {
        m_numberCellsReserved = (std::abs(maxCenterPoint.at(0) - minCenterPoint.at(0)) + radius) / radius * (std::abs(maxCenterPoint.at(1) - minCenterPoint.at(1)) + radius) / radius * (double)N;
    }
    //calculate here directions for moving intervals
    else */
    if (scheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        size_t numberConstraintsSkipped = getNumberOfMovingIntervalConstraints(delta, minCenterPoint, maxCenterPoint, m_pathCalc->getSystemFunction()->getCurrentContinuousState(),
                                                                               constraints.back().getCenterPoint(), radius, t0, T, N);
        if (numberConstraintsSkipped > m_numberCellsReserved) {
            std::cout << "Constraints number does not match: original " << m_numberCellsReserved << ", skipped: " << numberConstraintsSkipped << std::endl;
        }
        m_numberCellsReserved -= numberConstraintsSkipped;
        std::pair<double, double> signs = getDirectionOfTrajectory(getCurrentStateContinuous(), u, t0, T, N);
        minCenterPoint[0] *= signs.first;
        minCenterPoint[1] *= signs.second;

    }
    minMaxConstraints.emplace_back(Constraint(minCenterPoint, t0 + T, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin));
    minMaxConstraints.emplace_back(Constraint(maxCenterPoint, t0 + T, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin));
    return minMaxConstraints;
}

/**
 * @brief Car::getDirectionOfTrajectory return the direction of the trajectory considering a positive coordinate grid with origin (0,0), "++" means,
 * @param currentState
 * @param u
 * @param t0
 * @param T
 * @param N
 * @return
 */
std::pair<double, double> Car::getDirectionOfTrajectory(const std::vector<double>& currentState, const std::vector<std::vector<double> >& u, const double &t0, const double& T, const size_t& N) const {
    std::pair<double, double> signs = {1.0, 1.0};
    std::vector<std::vector<double> > x =
            m_pathCalc->getSystemFunction()->getHolonomicSystemTrajectory(currentState, u, t0, T, N);
    if (x.front().at(0) > x.back().at(0)) {
        signs.first = -1.0;
    }
    if (x.front().at(1) < x.back().at(1)) {
        signs.second = -1.0;
    }
    return signs;
}

/**
 * @brief Car::constructConstraintFromMinMaxConstraints reconstructs the constraints from a given interval, respectively minimum and maximum constraint
 * the drawback is here mainly
 * @param minMaxConstraints
 * @param t0
 * @param T
 * @param N
 * @param radius
 * @param dmin
 */
void Car::constructConstraintFromMinMaxConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T, const size_t& N,
                                                   const double& radius, const double& dmin, const CommunicationScheme& scheme) {
    std::multimap<QString, Constraint> constructedConstraints;
    //iterate over the full constraints vector, which includes the min-max constraints
    for (auto itConstraint = constraints.begin(); itConstraint != constraints.end();) {
        if (itConstraint->first != this->m_name) {
            std::vector<Constraint> minMaxConstraints;
            //now, get the range for min-max constraints for each car
            //with the range, get all constraints from one car
            auto itConstraintRange = constraints.equal_range(itConstraint->first);
            for (auto itCarConstraint = itConstraintRange.first; itCarConstraint != itConstraintRange.second; ++itCarConstraint) {
                minMaxConstraints.push_back(itCarConstraint->second);
            }
            //now evaluate, which is min and which is max constraint
            //std::pair<double, double> signs = {1.0, 1.0};
            std::vector<double> minPoint = {0.0, 0.0}, maxPoint = {0.0, 0.0};
            if (minMaxConstraints.size() < 2) {
                minPoint = minMaxConstraints.at(0).getCenterPoint();
                maxPoint = minMaxConstraints.at(0).getCenterPoint();
            }
            else {
                //x_min-x_max
                if (std::abs(minMaxConstraints.at(0).getCenterPoint().at(0)) < std::abs(minMaxConstraints.at(1).getCenterPoint().at(0))) {
                    minPoint[0] = std::abs(minMaxConstraints.at(0).getCenterPoint().at(0));
                    maxPoint[0] = minMaxConstraints.at(1).getCenterPoint().at(0);
                }
                else {
                    //signs.first = -1.0;
                    minPoint[0] = minMaxConstraints.at(1).getCenterPoint().at(0);
                    maxPoint[0] = std::abs(minMaxConstraints.at(0).getCenterPoint().at(0));
                }
                //y_min-y_max
                if (std::abs(minMaxConstraints.at(0).getCenterPoint().at(1)) < std::abs(minMaxConstraints.at(1).getCenterPoint().at(1))) {
                    minPoint[1] = std::abs(minMaxConstraints.at(0).getCenterPoint().at(1));
                    maxPoint[1] = minMaxConstraints.at(1).getCenterPoint().at(1);
                }
                else {
                    //signs.second = -1.0;
                    minPoint[1] = minMaxConstraints.at(1).getCenterPoint().at(1);
                    maxPoint[1] = std::abs(minMaxConstraints.at(0).getCenterPoint().at(1));
                }
            }
            //evaluate start and endpoint
            if (scheme == CommunicationScheme::MINMAXINTERVALMOVING) {
                std::vector<double> startPoint, endPoint;
                std::vector<double> firstPoint = minMaxConstraints.at(0).getCenterPoint(); //point which has the signs
                //right up to left down
                if (firstPoint.at(0) < 0.0 && firstPoint.at(1) < 0.0) {
                    startPoint = {maxPoint.at(0), minPoint.at(1)};
                    endPoint = {minPoint.at(0), maxPoint.at(1)};
                }
                //left up to right down
                else if (firstPoint.at(0) > 0.0 && firstPoint.at(1) < 0.0) {
                    startPoint = {minPoint.at(0), minPoint.at(1)};
                    endPoint = {maxPoint.at(0), maxPoint.at(1)};
                }
                //left down to right up
                else if (firstPoint.at(0) > 0.0 && firstPoint.at(1) > 0.0) {
                    startPoint = {minPoint.at(0), maxPoint.at(1)};
                    endPoint = {maxPoint.at(0), minPoint.at(1)};
                }
                //right down to left up
                else if (firstPoint.at(0) < 0.0 && firstPoint.at(1) > 0.0) {
                    startPoint = {maxPoint.at(0), maxPoint.at(1)};
                    endPoint = {minPoint.at(0), minPoint.at(1)};
                }
                std::vector<double> uMax = {m_pathCalc->getControlBounds().second, m_pathCalc->getControlBounds().second};
                std::vector<double> xNext = m_pathCalc->getSystemFunction()->getHolonomicSystem(startPoint, uMax, t0, t0 + T);
                //minimum time needed to get from start to end point
                size_t minTimeNeeded = getMinimumTimeStepsForDistance(startPoint, endPoint, uMax, t0, T, N);
                //delta is the time, when the car has to move
                size_t delta = N - minTimeNeeded;
                /*if (itConstraint->first == getName()) {
                    m_delta = delta;
                }*/
                size_t constraintSkipped = 0;
                for (double x = minPoint.at(0); x <= maxPoint.at(0); x+= radius) {
                    for (double y = minPoint.at(1); y <= maxPoint.at(1); y += radius) {
                        double t = t0;
                        for (size_t i = 0; i < N; i++) {
                            if (i > delta) {
                                //distance of end point to the point to levae out has to be bigger then the time interval, the robot needs * (N-i) for the needed steps
                                if (VectorHelper::norm2(VectorHelper::sub(endPoint, {x, y})) < (N - i) * VectorHelper::norm2(VectorHelper::sub(startPoint, xNext))) {
                                    constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({x, y}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                                }
                                else {
                                    constraintSkipped++;
                                }
                            }
                            else {
                                constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({x, y}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                            }
                            t += T;
                        }
                    }
                }
            }
            else if (scheme == CommunicationScheme::MINMAXINTERVAL) {
                /*double t = t0;
                //start condition
                if (minPoint.at(0) == maxPoint.at(0) && minPoint.at(1) == maxPoint.at(1)) {
                    t = t0;
                    for (size_t i = 0; i < N; i++) {
                        constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({minPoint.at(0), minPoint.at(1)}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                        t += T;
                    }
                }
                //only y dimension
                else if (minPoint.at(0) == maxPoint.at(0) && minPoint.at(1) != maxPoint.at(1)) {
                    for (double y = minPoint.at(1); y < maxPoint.at(1); y+=radius) {
                        //the interval is constant over time interval [n, n+N]
                        t = t0;
                        for (size_t i = 0; i < N; i++) {
                            constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({minPoint.at(0), y}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                            t += T;
                        }
                    }
                }
                else {
                    //iterate over x-dimension from x_min to x_max
                    for (double x = minPoint.at(0); x <= maxPoint.at(0); x+=radius) {
                        if (minPoint.at(1) == maxPoint.at(1)) {
                            t = t0;
                            for (size_t i = 0; i < N; i++) {
                                constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({x, minPoint.at(1)}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                                t += T;
                            }
                        }
                        for (double y = minPoint.at(1); y <= maxPoint.at(1); y+=radius) {
                            //the interval is constant over time interval [n, n+N]
                            t = t0;
                            for (size_t i = 0; i < N; i++) {
                                constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({x, y}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                                t += T;
                            }
                        }
                    }
                }*/
                for (double x = minPoint.at(0); x <= maxPoint.at(0); x+= radius) {
                    for (double y = minPoint.at(1); y <= maxPoint.at(1); y += radius) {
                        double t = t0;
                        for (size_t i = 0; i < N; i++) {
                            constructedConstraints.insert(std::pair<QString, Constraint>(itConstraint->first, Constraint({x, y}, t, SystemFunctionUsage::CONTINUOUS, N, T, radius, dmin)));
                            t += T;
                        }
                    }
                }
            }
            itConstraint = itConstraintRange.second;
        }//not the own constraints
        else {
            itConstraint++;
        }
    }
    setCurrentConstraints(constructedConstraints, t0, T);
}

/**
 * @brief Car::getNumberOfMovingIntervalConstraints calculates the number of necessary moving collision constraints, which have to be set
 * @param minTimeMove
 * @param minPoint
 * @param maxPoint
 * @param startPoint
 * @return number of constraints which may be skipped
 */
size_t Car::getNumberOfMovingIntervalConstraints(const size_t &minTimeMove, const std::vector<double>& minPoint, const std::vector<double>& maxPoint, const std::vector<double>& startPoint,
                                                 const std::vector<double>& endPoint, const double& radius, const double& t0, const double& T, const size_t &N) const {
    std::vector<double> uMax = {m_pathCalc->getControlBounds().second, m_pathCalc->getControlBounds().second};
    std::vector<double> xNext = m_pathCalc->getSystemFunction()->getHolonomicSystem(startPoint, uMax, t0, t0 + T);
    size_t constraintSkipped = 0;
    for (double x = minPoint.at(0); x <= maxPoint.at(0); x+= radius) {
        for (double y = minPoint.at(1); y <= maxPoint.at(1); y += radius) {
            for (size_t i = 0; i < N; i++) {
                if (i > minTimeMove) {
                    //distance of end point to the point to levae out has to be bigger then the time interval, the robot needs * (N-i) for the needed steps
                    if (VectorHelper::norm2(VectorHelper::sub(endPoint, {x, y})) >= (N - i) * VectorHelper::norm2(VectorHelper::sub(startPoint, xNext))) {
                        constraintSkipped++;
                    }
                }
            }
        }
    }
    return constraintSkipped;
}

/**
 * @brief Car::getMinimumTimeStepsForDistance
 * @param start
 * @param end
 * @param uMax
 * @param t0
 * @param T
 * @param N
 * @return
 */
size_t Car::getMinimumTimeStepsForDistance(const std::vector<double>& start, const std::vector<double>& end, const std::vector<double>& uMax,
                                           const double &t0, const double &T, const size_t &N) const {
    std::vector<double> xNext = m_pathCalc->getSystemFunction()->getHolonomicSystem(start, uMax, t0, t0 + T);
    double distOnce = VectorHelper::norm2(VectorHelper::sub(xNext, start));
    double distStartEnd = VectorHelper::norm2(VectorHelper::sub(start, end));
    size_t minSteps = std::floor(distStartEnd / distOnce);
    return minSteps;
}

/**
 * @brief Car::setCurrentConstraints
 * @param constraints
 */
void Car::setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T){
   m_pathCalc->setCurrentConstraints(constraints, t0, T);
}

/**
 * @brief Car::createGlobalConstraints creates the global constraints stored in MpcController for each car according to the available state space
 * @param lb lower bound, standard is {0.0, 0.0}
 * @param ub upper bound, standard is {InterSectionParameters::k, InterSectionParameters::m}
 */
void Car::createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) {
    m_pathCalc->createGlobalConstraints(lb, ub);
}

/**
 * @brief Car::createDirectionalConstraints creates dicrectional constraints according to the intersection scenario, when cars are not allowed
 * to reach opposite lane
 * @param interSectionWidth width of intersection
 * @param interSectionHeight height of intersection
 * @param cellSize current chosen cell size
 * @param t0 start time
 * @param N horizon length
 * @param T sampling step
 * @param dmin minimum distance
 * @param maxDynamics maximum control which could be imposed
 */
void Car::createDirectionalConstraints(const unsigned int& interSectionWidth, const unsigned int& interSectionHeight,
                                       const double &cellSize, const double &t0, const int &N, const double &T,
                                       const double &dmin, const double &maxDynamics) {
    m_pathCalc->initializeDirectionalConstraints(interSectionWidth, interSectionHeight, cellSize, t0, N, T, dmin, maxDynamics);

}

/**
 * @brief Car::removeOldPredictions removes the old prediction before starting the new optimization
 * @param constraintMap constraint Map with all predictions
 * @return
 */
std::multimap<QString, Constraint> Car::removeOldPredictions(const std::multimap<QString, Constraint>& constraints) const {
    std::multimap<QString, Constraint> constraintMap = constraints;
    for (auto it = constraintMap.begin(); it != constraintMap.end(); ) {
        if (it->first == getName()) {
            it = constraintMap.erase(it);
        }
        else {
            it++;
        }
    }
    return constraintMap;
}

/**
 * @brief Car::getCurrentPrediction as continuous vector
 * @return
 */
std::vector<double> Car::getCurrentPrediction() const {
    return m_pathCalc->getCurrentPrediction();
}

void Car::clearPrediction(std::vector<std::vector<double> >& pred) {
    /*for (auto itPred = pred.begin(); itPred != pred.end(); itPred++) {
        for (auto itPredVec = (*itPred).begin(); itPredVec != (*itPred).end(); itPredVec++) {
            (*itPredVec) = 0.0;
        }
    }*/
    for (size_t i = 0; i < pred.size(); i++) {
        for (size_t j = 0; j < pred.at(i).size(); j++) {
            pred.at(i).at(j) = 0.0;
        }
    }
    m_pathCalc->clearPrediction();
}

/**
 * @brief Car::differenceConstraints looks, if there are any new constraints or constraints changed (constraint timestamp/centerpoint)
 * and returns the changed constraints as vector
 * @param constraintFirst first constraint queue which should be evaluated
 * @param constraintSecond second constraint queue to compare with
 * @return difference of the constraints according to the last time step
 */
std::vector<Constraint> Car::differenceConstraints(const std::vector<Constraint>& constraintsFirst, const std::vector<Constraint>& constraintsSecond) const {
    std::vector<Constraint> diffConstraints;
    for (auto it = constraintsFirst.begin(); it != constraintsFirst.end(); it++) {
        bool foundEqual = false;
        for (auto comIt = constraintsSecond.begin(); comIt != constraintsSecond.end(); comIt++) {
            if (*comIt == *it) {
                foundEqual = true;
                break;
            }
        }
        if (!foundEqual) {
            diffConstraints.push_back(*it);
        }
    }
    return diffConstraints;
}

/**
 * @brief Car::getCountCommunicatedConstraints returns number of communicated constraints
 * @return
 */
unsigned int Car::getCountCommunicatedConstraints() const {
    return m_countCommunicatedConstraints;
}

/**
 * @brief Car::addCountCommunicatedConstraints add the number of communicated constraints
 * @param constr
 */
void Car::addCountCommunicatedConstraints() {
    //difference here now the constraints, that only new or changed constraints are broadcasted
    m_countCommunicatedConstraints += m_differentConstraints;
}

/**
 * @brief Car::setControlRange set the control range for u
 * @param lb
 * @param ub
 */
void Car::setControlRange(const double& lb, const double& ub) {
    m_pathCalc->setControlRange(lb, ub);
}

/**
 * @brief Car::getPredictedTrajectory get for the calculated optimal control the predicted trajectory
 * @return
 */
std::vector<std::vector<double> > Car::getPredictedTrajectory(const std::vector<double>& currentState, const std::vector<double> &prediction, const double& t0, const double& T, const size_t& N) const {
    std::vector<std::vector<double> > predictedTraject = m_pathCalc->getSystemFunction()->getHolonomicSystemTrajectory(currentState, VectorHelper::reshapeXd(prediction), t0, T, N);
    return predictedTraject;
}

Path Car::getPredictedTrajectoryCells(const std::vector<double> &currentState, const std::vector<double>& prediction, const double &t0, const double &T, const size_t &N, const double& radius) const {
    return m_pathCalc->getSystemFunction()->mapPredictionToCells(getPredictedTrajectory(currentState, prediction, t0, T, N), t0, T, radius);
}

/**
 * @brief Car::getOccupiedCells returns the occupied cells for the whole prediction horizon
 * @param openLoop: if true, return reserved cells over whole prediction horizon, otherwise only current position (closed-loop)
 * @return Map with reserved cells
 */
QMultiMap<int, int> Car::getOccupiedCells(const bool& openLoop) const {
    return m_occupiedCells;
}

bool Car::testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const {
    return m_pathCalc->testValidityConstraints(constraints, controlVector);
}

std::vector<Constraint> &Car::getCurrentConstraints() {
    return m_pathCalc->getCurrentConstraints();
}

void Car::clearAllConstraints() {
    m_pathCalc->clearAllConstraints();
    m_countCommunicatedConstraints = 0;

}

std::shared_ptr<PathCalculation> Car::getPathCalculator() const {
    return m_pathCalc;
}

/**
 * @brief Car::testActiveConstraints tests, if the current sets of constraints is active for the car
 * @param constraints
 * @param prediction
 * @param t0
 * @param T
 * @param N
 * @return
 */
bool Car::testActiveConstraints(const std::vector<Constraint> &constr, const std::vector<double>& prediction, const double& t0, const double& T, const double& N) const {
    std::vector<Constraint> constraints = constr;
    bool isActive = false;
    for (Constraint& constraint : constraints) {
        constraint.setActualSystem(this->getPathCalculator()->getSystemFunction(), t0, T, N);
        std::vector<double> grad;
        if (constraint.operator ()(prediction, grad, nullptr) >= 0.0) {
            isActive = true;
            break;
        }
    }
    return isActive;
}

/**
 * @brief Car::getDelta returns the delta for the time, until the car has to go (moving intervals)
 * @return
 */
size_t Car::getDelta() const {
    return m_delta;
}

/**
  * @brief Car::setConstraintsFromPredecessors sets the constraints from all predecessors
  * @param constraints
  */
void Car::setConstraintsFromPredecessors(std::multimap<QString, Constraint>& constraints, std::list<std::shared_ptr<Car> >& predecessors, const double& t0, const double& T) {
     std::multimap<QString, Constraint> constraintsPred;
     for (auto itPred = predecessors.begin(); itPred != predecessors.end(); itPred++) {
         std::shared_ptr<Car> carPred = (*itPred);
         auto itRange = constraints.equal_range((*itPred)->getName());
         //DEBUG
         /*for (auto it = itRange.first; it != itRange.second; it++) {
             std::pair<QString, Constraint> constrPair = *(it);
             qDebug() << "car " << (*itPred)->getName() << ": (" << constrPair.second.getCenterPoint().front() << ", " << constrPair.second.getCenterPoint().back() << ")" << endl;
         }*/
         //--DEBUG
         constraintsPred.insert(itRange.first, itRange.second);
     }
     m_pathCalc->setCurrentConstraints(constraintsPred, t0, T);
}

/**
 * @brief Car::getNumberCellsReserved returns the current number of reserved cells
 * @return
 */
size_t Car::getNumberCellsReserved() const {
    return m_numberCellsReserved;
}
