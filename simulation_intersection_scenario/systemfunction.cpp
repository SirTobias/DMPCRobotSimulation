#include "systemfunction.h"
#include "intersection.h"
#include "../simulation-core/vectorhelper.h"

#include <QtCore/QDebug>

constexpr double SystemFunction::boundaryTol;

/**
 * @brief SystemFunction::SystemFunction
 * @param path given start point
 */
SystemFunction::SystemFunction(const QString& car, const Path &path) :
    m_path(path),
    m_car(car),
    m_globalLiveTime(0),
    m_globalWaitTime(0),
    m_reservationRequests(0),
    m_waitTimeNextCell(0),
    m_externalReservationRequests(0),
    m_systemFuncType(SystemFunctionUsage::DISCRETE),
    m_startPos({0,0}),
    m_globalTime(0.0)
{
}

/**
 * @brief SystemFunction::SystemFunction continuous system function with given
 * start
 * @param car
 * @param startPos given start vector
 */
SystemFunction::SystemFunction(const QString &car, const std::vector<double> startPos) :
  //m_path(path),
  m_car(car),
  m_globalLiveTime(0),
  m_globalWaitTime(0),
  m_reservationRequests(0),
  m_waitTimeNextCell(0),
  m_externalReservationRequests(0),
  m_systemFuncType(SystemFunctionUsage::CONTINUOUS),
  m_startPos(startPos),
  m_globalTime(0.0)
{
    m_currentPos.push_back(m_startPos);
}

/**
 * @brief SystemFunction::getStart gives back the discrete version, get the first item, the start
 * @return PathItem as start position
 */
PathItem SystemFunction::getStart() const {
    return m_path.front();
}

/**
 * @brief SystemFunction::getStartContinuous get the first item, the start
 * @return std::vector as start position
 */
std::vector<double> SystemFunction::getStartContinuous() const {
    return m_startPos;
}

/**
 * @brief SystemFunction::getHolonomicSystemState integrates the equation dx = u with explicit Euler-method and gives back dx for a given delta x
 * @param x0 start value
 * @param u control
 * @param t0 start interval
 * @param tEnd end interval
 * @return dx as vector
 */
std::vector<double> SystemFunction::getHolonomicSystem(const std::vector<double> x0, const std::vector<double> &u, const double &t0, const double &tEnd) const {
    //use Euler-Form to approximate this for a given interval
    std::vector<double> x = x0;
    double deltaT = (tEnd - t0) / m_intervalSplit;
    for (double i = t0; i < tEnd; i+= deltaT) {
        for (unsigned int j = 0; j < x0.size(); j++) {
            //x.at(j) = x.at(j) + deltaT * x.at(j) * u.at(j);
            x.at(j) += deltaT * u.at(j);
        }
    }
    return x;
}

/**
 * @brief SystemFunction::getHolonomicSystemTrajectory calculates the system trajectory from start x0 with control u over a given horizon N
 * from t0 with sampling step T
 * @param x0 start value
 * @param u control
 * @param t0 start time
 * @param T sampling step
 * @param N horizon
 * @return x : vector of states
 */
std::vector<std::vector<double> > SystemFunction::getHolonomicSystemTrajectory(const std::vector<double> x0,
                                                                               const std::vector<std::vector<double> > &u, const double &t0,
                                                                               const double &T, const size_t &N) const {

    std::vector<std::vector<double> > x;
    x.push_back(x0);
    for (size_t i = 0; i < N - 1; i++) {
        x.push_back(getHolonomicSystem(x.at(i), u.at(i), t0 + (double)i*T, t0 + (double)(i+1)*T));
    }
    return x;
}

/**
 * @brief SystemFunction::getCurrentState get the last item, current position
 * @return PathItem of current position
 */
PathItem SystemFunction::getCurrentState() const {
    return m_path.back();
}

/**
 * @brief SystemFunction::getCurrentContinuousState
 * @return
 */
std::vector<double> SystemFunction::getCurrentContinuousState() const {
    return m_currentPos.back();

}

/**
 * @brief SystemFunction::prelimReserveCell reserve the intersection cell preliminary,
 * but still it is not applied
 * @param control
 * @return
 */
PathItem SystemFunction::prelimReserveCell(const PathItem &control) {
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    PathItem reservedCell = control;
    //count how often it is tried to reserve the next same cell (m_waitTimeNextCell)
    //and how often it is tried to get to another position (m_reservationRequests)
    if (m_lastReserved == control) {
        m_waitTimeNextCell++;
    }
    else {
        m_waitTimeNextCell = 0;
    }
    m_reservationRequests++;
    if (interSect) {
        reservedCell.setTime(interSect->tryReservePrelimTimeForCar(m_car, control.getX(), control.getY(), control.getTime()));
        m_prelimPath.push_back(reservedCell);
    }
    return reservedCell;
}

/**
 * @brief SystemFunction::clearOwnPrelimPath should clear after each optimization step
 * the own preliminary path items
 */
void SystemFunction::clearPrelimPath() {
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    if (interSect) {
        interSect->clearPrelimPathOfCar(m_car, m_prelimPath);
    }
    m_prelimPath.clear();
}

/**
 * @brief SystemFunction::reservePrelimSolution and if it succeeds,
 * it changes to the new state and add this to the path
 * @param pathItem
 * @return
 */
bool SystemFunction::reservePrelimSolution(const PathItem& pathItem) {
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    m_reserved = false;
    if (interSect) {
        m_reserved = interSect->reserveTimeForCar(this->m_car, pathItem.getX(), pathItem.getY(), pathItem.getTime());
    }
    return m_reserved;
}

/**
 * @brief SystemFunction::applyNextState if the solution can be applied, the car can get to the next state
 */
void SystemFunction::applyNextState(const PathItem& pathItem) {
    //now set the new state
    if (m_reserved) {
        m_path.addPathItem(pathItem);
        m_waitTimeNextCell = 0;
        if (pathItem != m_path.back()) {
            m_reservationRequests = 0;
        }
    }
    setGlobalTime(m_path.back().getTime());
    if (m_path.size() > 1) {
        if (m_path.at(m_path.size() - 2).getX() == m_path.back().getX() && m_path.at(m_path.size() - 2).getY() == m_path.back().getY()) {
            int64_t globalWaitTime = getGlobalWaitTime();
            setGlobalWaitTime(++globalWaitTime);
        }
        else {
            setGlobalWaitTime(0);
        }
    }
}

/**
 * @brief SystemFunction::applyNextState continuous system and reserves the appropriate cell for the current contin. position
 * @param umin control to be applied
 * @param t0 start time
 * @param T sampling step
 */
void SystemFunction::applyNextState(const std::vector<double> &umin, const double &t0, const double &T) {
    m_currentPos.push_back(getHolonomicSystem(m_currentPos.back(), umin, t0, T));
    //DEBUG
    //qDebug() << "apply state: " << this->m_car << " " << t0 << "(" << m_currentPos.back().at(0) << ","
    //         << m_currentPos.back().at(1) << ")";
    //--DEBUG
    //apply the cell mapped for the current position
    std::shared_ptr<InterSectionCell> cell = InterSection::getInstance()->getCellFromCoordinates(m_currentPos.back());
    if (cell) {
        reservePrelimSolution(PathItem(cell->getX(), cell->getY(), m_globalTime));
    }
    setGlobalTime(t0 + T);
    if (m_currentPos.size() > 1) {
        if (m_currentPos.at(m_currentPos.size() - 2).at(0) == m_currentPos.back().at(0) && m_currentPos.at(m_currentPos.size() - 2).at(1) == m_currentPos.back().at(1)) {
            int64_t globalWaitTime = getGlobalWaitTime();
            setGlobalWaitTime(++globalWaitTime);
        }
        else {
            setGlobalWaitTime(0);
        }
    }
}

/**
 * @brief SystemFunction::getPossibleTimeForPrelimReservation
 * @param item
 * @return
 */
double SystemFunction::getPossibleTimeForPrelimReservation(const PathItem& item) const {
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    double prelimPossibleTime = -1.0;
    if (interSect) {
        prelimPossibleTime = interSect->getPossiblePrelimReserveTimeForCar(item.getX(), item.getY(), item.getTime());
    }
    return prelimPossibleTime;
}

/**
 * @brief SystemFunction::setPossiblePrelimTimeForPath
 * @param path
 * @return
 */
Path SystemFunction::setPossiblePrelimTimeForPath(const Path& path) const {
    Path curPath = path;
    curPath.normalizeForTime(m_path.back());
    for (PathItem& item : curPath) {
        item.setTime(getPossibleTimeForPrelimReservation(item));
    }
    //m_prelimPath = curPath;
    return curPath;
}

/**
 * @brief SystemFunction::getPath get curent system path as const ref
 * @return
 */
Path& SystemFunction::getPath() {
    return m_path;
}

/**
 * @brief SystemFunction::getPath get current system path as copy
 * @return
 */
Path SystemFunction::getPath() const {
    return m_path;
}

/**
 * @brief SystemFunction::getPrelimPath return preliminary path as const ref
 * @return prelim path
 */
Path& SystemFunction::getPrelimPath() {
    return m_prelimPath;
}

/**
 * @brief getPrelimPath return preliminary path as copy
 * @return prelim path
 */
Path SystemFunction::getPrelimPath() const {
    return m_prelimPath;
}

/**
 * @brief SystemFunction::getGlobalLiveTime
 * @return
 */
int64_t SystemFunction::getGlobalLiveTime() const {
    return m_globalLiveTime;
}

/**
 * @brief SystemFunction::setGlobalLiveTime
 * @param t
 */
void SystemFunction::setGlobalLiveTime(const double &t) {
    m_globalLiveTime = t;
}

/**
 * @brief SystemFunction::getGlobalWaitTime
 * @return
 */
int64_t SystemFunction::getGlobalWaitTime() const {
    return m_globalWaitTime;
}

/**
 * @brief SystemFunction::setGlobalWaitTime
 * @param t
 */
void SystemFunction::setGlobalWaitTime(const int64_t &t) {
    m_globalWaitTime = t;
}

/**
 * @brief SystemFunction::getReservationRequests
 * @return
 */
int64_t SystemFunction::getReservationRequests() const {
    return m_reservationRequests;
}

/**
 * @brief SystemFunction::setReservationRequests
 * @param r
 */
void SystemFunction::setReservationRequests(const int64_t &r) {
    m_reservationRequests = r;
}

/**
 * @brief SystemFunction::getWaitTimeNextCell
 * @return
 */
int64_t SystemFunction::getWaitTimeNextCell() const {
    return m_waitTimeNextCell;
}

/**
 * @brief SystemFunction::setWaitTimeNextCell
 * @param t
 */
void SystemFunction::setWaitTimeNextCell(const int64_t &t) {
    m_waitTimeNextCell = t;
}

/**
 * @brief SystemFunction::getExternalReservationRequests
 * @return
 */
int64_t SystemFunction::getExternalReservationRequests() const {
   return m_externalReservationRequests;
}

/**
 * @brief SystemFunction::setExternalReservationRequests
 * @param r
 */
void SystemFunction::setExternalReservationRequests(const int64_t &r) {
    m_externalReservationRequests = r;
}

/**
 * @brief SystemFunction::setPrelimPath set preliminary path
 * @param path
 */
void SystemFunction::setPrelimPath(const Path& path) {
    m_prelimPath = path;
}

/**
 * @brief SystemFunction::setCarInfo set the given CarInfo for the car with the given id
 * @param id for which car the carinfo should be set
 * @param carInfo given carinfo
 */
void SystemFunction::setCarInfo(const std::string& id, const CarInformation& carInfo) {
    //find the appropriate car id in the information
    auto carKey = m_carInfo.find(id);
    if (carKey != m_carInfo.end()) {
        //and set for the appropriate car the updated information
        auto carIt = carKey->second;
        //TODO: test, if this is a reference and modified
        carIt = carInfo;
    }
}

/**
 * @brief SystemFunction::countCarInfos returns amount of neighboured cars
 * @return size_t as count of carinfo from neighboured cars
 */
size_t SystemFunction::countCarInfos() const {
    return m_carInfo.size();
}

/**
 * @brief SystemFunction::getCarInfo returns CarInformation at certain position
 * @param pos iterposition at underlying structure
 * @return reference on CarInformation
 */
const CarInformation& SystemFunction::getCarInfo(const size_t& pos) const {
    std::map<std::string, CarInformation>::const_iterator it = m_carInfo.begin();
    size_t begin = 0;
    while (begin < pos && it != m_carInfo.end()) {
        it++;
        begin++;
    }
    return it->second;
}

/**
 * @brief SystemFunction::setGlobalTime sets the global time for the system function
 * @param t
 */
void SystemFunction::setGlobalTime(const double &t) {
    this->m_globalTime = t;
}

int64_t SystemFunction::getGlobalTime() const
{
    return m_globalTime;
}

const CarInformation& SystemFunction::getCarInfoEnd() const {
    return m_carInfo.end()->second;
}

/**
 * @brief SystemFunction::predictionToCells will map the predictions to the cells which should be reserved.
 * These are given back as a path containing the path items which define the cells to be blocked for a certain time
 * TODO: also reserve intermediate cells in case of cellsize < control
 * @param x vector of states
 * @param t0 start time for reservation
 * @param radius optional parameter radius, if diameter of robot should be considered
 * @return path with the cells which should be reserved
 */
Path SystemFunction::mapPredictionToCells(const std::vector<std::vector<double> > &x, const double& t0, const double& T, const double &radius) const {
    Path path;
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    double tInterval = t0;
    for (unsigned int i = 0; i < x.size(); i++) {
        std::vector<double> curState = x.at(i);
        std::shared_ptr<InterSectionCell> iCell = interSect->getCellFromCoordinates(curState);
        if (iCell) {
            path.addPathItem(PathItem(iCell->getX(), iCell->getY(), tInterval));
            //look up, if one of the coordinate is at one boundary
            //collects all ceil or floored states that have to be reserved, especially, if
            //diagonal paths are required
            std::vector<std::vector<double> > roundedStates;
            //case ceil(i)-i
            //all ceiled is for diagonal paths
            int allCeiled = 0;
            for (unsigned int j = 0; j < curState.size(); j++) {
                if (std::ceil(curState.at(j)) - curState.at(j) < SystemFunction::boundaryTol) {
                    std::vector<double> curStateCeiled = curState;
                    curStateCeiled.at(j) = std::ceil(curState.at(j));
                    roundedStates.push_back(curStateCeiled);
                    allCeiled++;
                }
            }
            //path diagonal is also necessary
            if (allCeiled == curState.size() - (unsigned int)1) {
                roundedStates.emplace_back(VectorHelper::ceil(curState));
            }
            //case floor(i)-i
            int allFloored = 0;
            for (unsigned int j = 0; j < curState.size(); j++) {
                if (curState.at(j) - std::floor(curState.at(j)) < SystemFunction::boundaryTol) {
                    std::vector<double> curStateFloored = curState;
                    curStateFloored.at(j) = std::floor(curState.at(j));
                    roundedStates.push_back(curStateFloored);
                    allFloored++;
                }
            }
            //path diagonal is also necessary
            if (allFloored == curState.size() - (unsigned int)1) {
                roundedStates.emplace_back(VectorHelper::floor(curState));
            }
            if (radius > 0.0) {
                //(x+r, y+r)
                std::vector<double> xRad = {x.at(i).at(0) + radius, x.at(i).at(1) + radius};
                std::shared_ptr<InterSectionCell> rCell = interSect->getCellFromCoordinates(xRad);
                if (iCell != rCell && rCell != nullptr) {
                    path.addPathItem(PathItem(rCell->getX(), rCell->getY(), tInterval));
                }
                //(x+r, y-r)
                xRad = {x.at(i).at(0) + radius, x.at(i).at(1) - radius};
                rCell = interSect->getCellFromCoordinates(xRad);
                if (iCell != rCell && rCell != nullptr) {
                    path.addPathItem(PathItem(rCell->getX(), rCell->getY(), tInterval));
                }
                //(x+r, y-r)
                xRad = {x.at(i).at(0) - radius, x.at(i).at(1) - radius};
                rCell = interSect->getCellFromCoordinates(xRad);
                if (iCell != rCell && rCell != nullptr) {
                    path.addPathItem(PathItem(rCell->getX(), rCell->getY(), tInterval));
                }
                xRad = {x.at(i).at(0) - radius, x.at(i).at(1) + radius};
                rCell = interSect->getCellFromCoordinates(xRad);
                if (iCell != rCell && rCell != nullptr) {
                    path.addPathItem(PathItem(rCell->getX(), rCell->getY(), tInterval));
                }
            }
            //dynamic is greater than cell size, therefore we have to reserve the intermediate cells
            /*if (InterSection::getInstance()->getCellSize() < m_intervalControlDynamic.at(0) || InterSection::getInstance()->getCellSize()< m_intervalControlDynamic.at(1)) {
                if (i < x.size() - 1) {
                    std::vector<double> gradient = VectorHelper::sub(x.at(i+1), curState);
                    std::vector<double> lastState = curState;
                    std::vector<bool> direction = VectorHelper::direction(gradient);
                    for (double j = 0; j * InterSection::getInstance()->getCellSize() < m_intervalControlDynamic.at(1); j+= InterSection::getInstance()->getCellSize()) {
                        lastState = VectorHelper::add(lastState, VectorHelper::mult(gradient, InterSection::getInstance()->getCellSize()));
                        if (VectorHelper::stepSizeInInterval(x.at(i+1), lastState, direction)) { //stepsize not too large
                            std::shared_ptr<InterSectionCell> intermediateCell = interSect->getCellFromCoordinates(lastState);
                            intermediateCell->reserveTimeForCar(this->m_car, t0 + tInterval);
                            path.addPathItem(PathItem(intermediateCell->getX(), intermediateCell->getY(), t0 + tInterval));
                        }
                        else {//stepsize too large, reserve last step
                            std::shared_ptr<InterSectionCell> intermediateCell = interSect->getCellFromCoordinates(x.at(i+1));
                            intermediateCell->reserveTimeForCar(this->m_car, t0 + tInterval);
                            path.addPathItem(PathItem(intermediateCell->getX(), intermediateCell->getY(), t0 + tInterval));
                        }
                    }
                }
            }
            else {*/
                //TODO: should be done in a separate step
                iCell = interSect->getCellFromCoordinates(x.at(i));
                iCell->reserveTimeForCar(this->m_car, t0+tInterval);
            //}
        }

        tInterval += T;
    }
    return path;
}

/**
 * @brief SystemFunction::getSystemFunctionType
 * @return
 */
SystemFunctionUsage SystemFunction::getSystemFunctionType() const {
    return m_systemFuncType;
}
/**
 * @brief SystemFunction::setIntervalControlDynamic
 * @param vec
 */
void SystemFunction::setIntervalControlDynamic(const std::vector<double>& vec) {
    m_intervalControlDynamic = vec;
}

std::vector<double> SystemFunction::getPos(const size_t& N) const {
    return m_currentPos.at(N);
}
