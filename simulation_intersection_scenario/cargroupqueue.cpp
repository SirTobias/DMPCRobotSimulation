#include "cargroupqueue.h"

#include <QDebug>


CarGroupQueue::CarGroupQueue() :
    m_queueType(CarGroupQueueType::FIXED)
{

}

/**
 * @brief CarGroupQueue::CarGroupQueue constructs the CarGroupQueue direct from a 2D-vector
 * @param cars
 */
CarGroupQueue::CarGroupQueue(const std::vector<std::vector<std::shared_ptr<Car> > > &cars) :
    m_queueType(CarGroupQueueType::FIXED),
    m_cars(cars)
{
    for (const std::shared_ptr<Car>& car : getOrderSeq()) {
        m_carTreeSuccessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
        m_carTreePredecessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
    }
}

/**
 * @brief CarGroupQueue::CarGroupQueue constructs the CarGroupQueue from a 1D-vector
 * @param cars
 */
CarGroupQueue::CarGroupQueue(const std::vector<std::shared_ptr<Car> >& cars) :
    m_queueType(CarGroupQueueType::FIXED)
{
    for (const std::shared_ptr<Car>& car : cars) {
        m_cars.at(0).push_back(car);
    }
    for (const std::shared_ptr<Car>& car : cars) {
        m_carTreeSuccessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
        m_carTreePredecessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
    }
}

void CarGroupQueue::setType(const CarGroupQueueType& type) {
    m_queueType = type;
}

void CarGroupQueue::appendCar(const std::shared_ptr<Car>& car) {
    m_cars.at(0).push_back(car);
}

/**
 * @brief CarGroupQueue::appendCar
 * @param pos
 * @param car
 */
void CarGroupQueue::appendCar(const std::shared_ptr<Car>& car, const size_t& pos) {
    while (m_cars.size() - 1 < pos) {
        m_cars.push_back(std::vector<std::shared_ptr<Car> >());
    }
    m_cars.at(pos).push_back(car);
}

/**
 * @brief CarGroupQueue::storeOrder
 * @param cars
 */
void CarGroupQueue::storeOrder(const std::vector<std::shared_ptr<Car> >& cars, const size_t& pos) {
    while (m_cars.size() < pos) {
        m_cars.push_back(std::vector<std::shared_ptr<Car> >());
    }
    for (const std::shared_ptr<Car>& car : cars) {
        m_cars.at(pos).push_back(car);
    }
}

/**
 * @brief CarGroupQueue::getOrder
 * @return
 */
std::vector<std::vector<std::shared_ptr<Car> > > CarGroupQueue::getOrder() const {
    return m_cars;
}

/**
 * @brief CarGroupQueue::getOrder
 * @return
 */
std::vector<std::shared_ptr<Car> > CarGroupQueue::getOrderSeq() const {
    std::vector<std::shared_ptr<Car> > seqVec;
    for (auto itCars = m_cars.begin(); itCars != m_cars.end(); itCars++) {
        for (auto itCar = (*itCars).begin(); itCar != (*itCars).end(); itCar++) {
            seqVec.push_back(*itCar);
        }
    }
    return seqVec;
}

/**
 * @brief CarGroupQueue::findVector
 * @param car
 * @return
 */
std::vector<std::shared_ptr<Car> >& CarGroupQueue::findVector(const std::shared_ptr<Car>& car) {
    auto elemCol = m_cars.begin();
    bool found = false;
    while (elemCol != m_cars.end() && !found) {
        for (auto itElemRow = (*elemCol).begin(); itElemRow != (*elemCol).end(); itElemRow++) {
            if ((*itElemRow) == car) {
                found = true;
                break;
            }
        }
        if (!found) {
            elemCol++;
        }
    }
    return (*elemCol);
}

/**
 * @brief CarGroupQueue::push_back
 * @param car
 */
void CarGroupQueue::push_back(const std::shared_ptr<Car>& car) {
    if (m_cars.empty()) {
        m_cars.push_back(std::vector<std::shared_ptr<Car> >());
    }
    m_cars.at(0).push_back(car);
    m_carTreeSuccessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
    m_carTreePredecessors.insert(std::pair<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > >(car, std::list<std::shared_ptr<Car> >()));
}

/**
 * @brief CarGroupQueue::size
 * @return
 */
size_t CarGroupQueue::size() const {
    size_t size = 0;
    for (auto itCars = m_cars.begin(); itCars != m_cars.end(); itCars++) {
        size += (*itCars).size();
    }
    return size;
}

size_t CarGroupQueue::rowSize() const {
    return m_cars.size();
}

/**
 * @brief CarGroupQueue::removeCar from a given row
 * @param vec
 * @param car
 * @return
 */
std::vector<std::shared_ptr<Car> >::iterator CarGroupQueue::removeCar(std::vector<std::shared_ptr<Car> >& vec, const std::shared_ptr<Car>& car) {
    std::vector<std::shared_ptr<Car> >::iterator col = vec.end();
    bool deleted = false;
    //auto itElemCol = m_cars.begin();
    auto itElemCar = vec.begin();
    while (itElemCar != vec.end() && !deleted) {
        if (*itElemCar == car) {
            col = vec.erase(itElemCar);
            deleted = true;
            break;
        }
        else {
            itElemCar++;
        }
    }
    return col;
}

/**
 * @brief CarGroupQueue::removeCar removes a car
 * @param vec
 * @param car
 * @return
 */
std::vector<std::shared_ptr<Car> >::iterator CarGroupQueue::removeCar(const std::shared_ptr<Car>& car) {
    std::vector<std::shared_ptr<Car> >::iterator col = (*m_cars.begin()).begin();
    bool deleted = false;
    auto itElemCol = m_cars.begin();
    while (itElemCol != m_cars.end() && !deleted) {
        auto itElemCar = (*itElemCol).begin();
        while (itElemCar != (*itElemCol).end() && !deleted) {
            if (*itElemCar == car) {
                col = (*itElemCol).erase(itElemCar);
                deleted = true;
                break;
            }
            else {
                itElemCar++;
            }
        }
        itElemCol++;
    }
    return col;
}


std::vector<std::shared_ptr<Car> >& CarGroupQueue::getRow(const size_t& pos) {
    Q_ASSERT_X(pos < m_cars.size(), typeid(this).name(), "pos > row");
    return m_cars.at(pos);
}

/**
 * @brief CarGroupQueue::removeEmptyRows remove the empty rows
 */
void CarGroupQueue::removeEmptyRows() {
    auto itRow = m_cars.begin();
    while (itRow != m_cars.end()) {
        if ((*itRow).empty()) {
            itRow = m_cars.erase(itRow);
        }
        else {
            ++itRow;
        }
    }
}

/**
 * @brief CarGroupQueue::clear clear all rows
 */
void CarGroupQueue::clear() {
    auto itRow = m_cars.begin();
    while (itRow != m_cars.end()) {
        (*itRow).clear();
        itRow++;
    }
}

/**
 * @brief CarGroupQueue::moveCar erases a car from the current row and insert into the new row newRow
 * @param car
 * @param newRow newRow to insert
 * @return iterator for oldRow
 */
std::vector<std::shared_ptr<Car> >::iterator CarGroupQueue::moveCar(const std::shared_ptr<Car>& car, const size_t &newRow) {
    auto itRow = removeCar(car);
    /*std::vector<std::shared_ptr<Car> >& row = findVector(car);
    auto itOldRow = row.begin();
    while((*itOldRow) != car && itOldRow != row.end()) {
        itOldRow++;
    }
    auto itNew = row.erase(itOldRow);*/
    appendCar(car, newRow);

    return itRow;
}

/**
 * @brief CarGroupQueue::getPosForRow evaluate the position of the row
 * @param row given row
 * @return return position
 */
size_t CarGroupQueue::getPosForRow(std::vector<std::shared_ptr<Car> > & row) const {
    size_t pos = 0;
    while (pos < m_cars.size() && m_cars.at(pos) != row) {
        pos++;
    }
    return pos;
}

/**
 * @brief CarGroupQueue::moveCarPreviousRow moves the car to the previous row
 * @param car
 */
void CarGroupQueue::moveCarPreviousRow(const std::shared_ptr<Car>& car) {
    auto row = findVector(car);
    size_t position = getPosForRow(row);
    if (position > 0) {
        moveCar(car, --position);
    }
}

/**
 * @brief CarGroupQueue::moveCarNextRow transfer the car to the next row
 * @param car
 */
std::vector<std::shared_ptr<Car> >::iterator CarGroupQueue::moveCarToNextRow(const std::shared_ptr<Car>& car) {
    auto row = findVector(car);
    size_t position = getPosForRow(row);
    return moveCar(car, ++position);
}

bool CarGroupQueue::hasPreviousRow(std::vector<std::shared_ptr<Car> >& row) const {
    size_t position = getPosForRow(row);
    return (position > 0);
}
bool CarGroupQueue::hasNextRow(std::vector<std::shared_ptr<Car> >& row) const {
    size_t position = getPosForRow(row);
    return (position < m_cars.size());
}

/**
 * @brief CarGroupQueue::getPreviousRow
 * @param row
 * @return
 */
std::vector<std::shared_ptr<Car> >& CarGroupQueue::getPreviousRow(std::vector<std::shared_ptr<Car> >& row) {
    size_t position = getPosForRow(row);
    Q_ASSERT_X(position > 0, typeid(this).name(), "previous row < 0");
    return m_cars.at(--position);
}

/**
 * @brief CarGroupQueue::getNextRow
 * @param row
 * @return
 */
std::vector<std::shared_ptr<Car> >& CarGroupQueue::getNextRow(std::vector<std::shared_ptr<Car> >& row) {
    size_t position = getPosForRow(row);
    Q_ASSERT_X(position > m_cars.size(), typeid(this).name(), "row > size");
    return m_cars.at(++position);
}

/**
 * @brief CarGroupQueue::findCarInVector finds a given car in vector vec, returns true, if found, otherwise false
 * @param vec vector to look inside
 * @param car car to look up
 * @return
 */
bool CarGroupQueue::findCarInVector(const std::vector<std::shared_ptr<Car> >& vec, const std::shared_ptr<Car>& car) const {
    bool found = false;
    auto itCar = vec.begin();
    while (itCar != vec.end() && !found) {
        if ((*itCar) == car) {
            found = true;
            break;
        }
        else {
            itCar++;
        }
    }
    return found;
}

/**
 * @brief CarGroupQueue::addSuccessor
 * @param car
 * @param succ
 */
void CarGroupQueue::addSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& succ) {
    if (!contains(m_carTreeSuccessors.at(car), succ)) {
        m_carTreeSuccessors.at(car).push_back(succ);
        m_carTreePredecessors.at(succ).push_back(car);
    }
    size_t execLevel = getExecLevelAccordingPreds(succ);
    if (hasSuccessor(succ)) {
        moveSuccessor(succ);
    }
    else {
        moveCar(succ, execLevel);
    }
}

/**
 * @brief CarGroupQueue::hasSuccessor
 * @param car
 * @return
 */
bool CarGroupQueue::hasSuccessor(const std::shared_ptr<Car>& car) const {
    return (!m_carTreeSuccessors.at(car).empty());
}

/**
 * @brief CarGroupQueue::hasPredecessors
 * @param car
 * @return
 */
bool CarGroupQueue::hasPredecessors(const std::shared_ptr<Car>& car) const {
    return (!m_carTreePredecessors.at(car).empty());
}

/**
 * @brief CarGroupQueue::moveSuccessor moves car and all affected successors to the correct execution level
 * @param car
 */
void CarGroupQueue::moveSuccessor(const std::shared_ptr<Car>& car) {
    for (auto itSuc = m_carTreeSuccessors.at(car).begin(); itSuc != m_carTreeSuccessors.at(car).end(); itSuc++) {
        moveSuccessor((*itSuc));
    }
    size_t execLevel = getExecLevelAccordingPreds(car);
    moveCar(car, execLevel);
}

/**
 * @brief CarGroupQueue::removeSuccessor removes from car the successor suc, if car has predecessors, it is inserted there as successor, yielding same level as car,
 * for all successors of suc, they have to be connected also to the former predecessor of suc
 * @param car
 * @param suc
 */
void CarGroupQueue::removeSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& suc) {
    //TODO: insert into row at correct dependency
    m_carTreeSuccessors.at(car).remove(suc);
    m_carTreePredecessors.at(suc).remove(car);
    if (hasPredecessors(car)) {
        //grand-parents are now parents (set parent for predecessor as direct predecessor)
        auto predecessors = getDirectPredecessors(car);
        for (auto itPred : predecessors) {
            //only insert, if predecessor not already inherited the successor
            if (!contains(m_carTreeSuccessors.at(itPred), suc)) {
                std::shared_ptr<Car> tempCar = itPred;
                m_carTreeSuccessors.at(itPred).push_back(suc);
                m_carTreePredecessors.at(suc).push_back(itPred);
            }
        }

    }
    size_t execLevel = getExecLevelAccordingPreds(suc);
    if (hasSuccessor(suc)) {
        //childs becoming also direct successors from car
        auto successors = getDirectSuccessors(suc);
        for (auto itSuc : successors) {
            //only insert, if successor does not already have the predecessor as parent
            if (!contains(m_carTreePredecessors.at(itSuc), car)) {
                std::shared_ptr<Car> tempCar = itSuc;
                m_carTreeSuccessors.at(car).push_back(itSuc);
                m_carTreePredecessors.at(itSuc).push_back(car);
            }
        }
        moveSuccessor(suc);
    }
    else {
        moveCar(suc, execLevel);
    }
}

/**
 * @brief CarGroupQueue::getExecLevelAccordingPreds counts the level until root
 * @param car
 * @param level
 * @return
 */
size_t CarGroupQueue::getExecLevelAccordingPreds(const std::shared_ptr<Car>& car, const size_t& level) const {
    size_t curLevel = level;
    auto listPreds = m_carTreePredecessors.at(car);
    size_t nextLevel = curLevel;
    if (!listPreds.empty()) {
        nextLevel++;
    }
    for (auto itPredCar = listPreds.begin(); itPredCar != listPreds.end(); itPredCar++) {
        std::shared_ptr<Car> tempCar = *itPredCar;
        nextLevel = getExecLevelAccordingPreds((*itPredCar), nextLevel);
        if (nextLevel > curLevel) {
            curLevel = nextLevel;
        }
    }
    return curLevel;
}
/**
 * @brief CarGroupQueue::getDirectSuccessors
 * @param car
 * @return
 */
std::list<std::shared_ptr<Car> >& CarGroupQueue::getDirectSuccessors(const std::shared_ptr<Car>& car) {
    return m_carTreeSuccessors.at(car);
}
/**
 * @brief CarGroupQueue::getDirectPredecessors
 * @param car
 * @return
 */
std::list<std::shared_ptr<Car> >& CarGroupQueue::getDirectPredecessors(const std::shared_ptr<Car>& car) {
    return m_carTreePredecessors.at(car);
}

/**
 * @brief CarGroupQueue::computeRecursivePredecessors
 * @param car
 * @return
 */
std::list<std::shared_ptr<Car> >& CarGroupQueue::computeRecursivePredecessors(const std::shared_ptr<Car>& car, std::list<std::shared_ptr<Car> >& predecessors) const {
    if (hasPredecessors(car)) {
        for (auto itPred = m_carTreePredecessors.at(car).begin(); itPred != m_carTreePredecessors.at(car).end(); itPred++) {
            computeRecursivePredecessors((*itPred), predecessors);
            predecessors.push_back((*itPred));
        }
    }
    /*if (predecessors.size() > 0) {
        qDebug() << "List:";
    }
    for (auto recCar : predecessors) {
        qDebug() << recCar->getName();
    }*/
    return predecessors;
}

/**
 * @brief CarGroupQueue::computeRecursiveSuccessors
 * @param car
 * @return
 */
std::list<std::shared_ptr<Car> >& CarGroupQueue::computeRecursiveSuccessors(const std::shared_ptr<Car>& car, std::list<std::shared_ptr<Car> > &successors) const {
    if (hasSuccessor(car)) {
        for (auto itSucc = m_carTreeSuccessors.at(car).begin(); itSucc != m_carTreeSuccessors.at(car).end(); itSucc++) {
            std::list<std::shared_ptr<Car> > recCars = computeRecursiveSuccessors((*itSucc), successors);
            successors.insert(successors.end(), recCars.begin(), recCars.end());
            successors.push_back((*itSucc));
        }
    }
    return successors;
}

/**
 * @brief CarGroupQueue::isSuccessor
 * @param car
 * @param suc
 * @return
 */
bool CarGroupQueue::isSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& suc) const {
    bool found = false;
    for (auto itSuc = m_carTreeSuccessors.at(car).begin(); itSuc != m_carTreeSuccessors.at(car).end(); itSuc++) {
        found = isSuccessor((*itSuc), suc);
        if ((*itSuc) == suc) {
            found = true;
        }
        if (found) {
            break;
        }
    }
    return found;
}
/**
 * @brief CarGroupQueue::isPredecessor looks recursively, if pred is predecessor of car
 * @param car
 * @param pred
 * @return
 */
bool CarGroupQueue::isPredecessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& pred) const {
    bool found = false;
    for (auto itPred = m_carTreePredecessors.at(car).begin(); itPred != m_carTreePredecessors.at(car).end(); itPred++) {
        found = isPredecessor((*itPred), pred);
        if ((*itPred) == pred) {
            found = true;
        }
        if (found) {
            break;
        }
    }
    return found;
}

/**
 * @brief CarGroupQueue::isInRelationTo tests, if car and it car are either in relation of successor/predecessor
 * @param car
 * @param itCar
 * @return
 */
bool CarGroupQueue::isInRelationTo(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& itCar) const {
    if ( isSuccessor(car, itCar) || isPredecessor(car, itCar)
         || isSuccessor(itCar, car) || isPredecessor(itCar, car) ) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief getRoot returns the root node of the longest path (highest execution level)
 * @param car
 * @return
 */
std::shared_ptr<Car> CarGroupQueue::getRoot(const std::shared_ptr<Car>& car) {
    auto tempCar = car;
    if (hasPredecessors(tempCar)) {
        auto predecessors = getDirectPredecessors(tempCar);
        size_t execLevel = 0;
        auto highestExecLevelCar = predecessors.front();
        for (auto itPred : predecessors) {
            if (getExecLevelAccordingPreds(itPred) > execLevel) {
                highestExecLevelCar = itPred;
                execLevel = getExecLevelAccordingPreds(itPred);
            }
        }
        highestExecLevelCar = getRoot(highestExecLevelCar);
        return highestExecLevelCar;
    }
    else {
        return tempCar;
    }
}

/**
 * @brief CarGroupQueue::showCarGroupTree displays the succeeding cars from the latest root
 * @param car
 */
void CarGroupQueue::showCarGroupTree(const std::shared_ptr<Car>& car) {
    if (hasSuccessor(car)) {
        auto succ = getDirectSuccessors(car);
        for (auto itSuc : succ) {
            qDebug() << "car " << car->getName() << "is succeeded by " << itSuc->getName();
            showCarGroupTree(itSuc);
        }
    }
}

/**
 * @brief CarGroupQueue::contains returns true, if car is in list, otherwise false
 * @param list list to search
 * @param car to look for
 * @return true, if found, else false
 */
bool CarGroupQueue::contains(std::list<std::shared_ptr<Car> >& list, const std::shared_ptr<Car>& car) const {
    bool found = false;
    for (auto itList = list.cbegin(); itList != list.cend(); itList++) {
        if ((*itList) == car) {
            found = true;
            break;
        }
    }
    return found;
}

/**
 * @brief CarGroupQueue::getCarById get the car by the id
 * @param id
 * @return
 */
std::shared_ptr<Car> CarGroupQueue::getCarById(const QString& id) {
    std::shared_ptr<Car> carFound = nullptr;
    for (const std::shared_ptr<Car>& car : getOrderSeq()) {
        if (car->getName() == id) {
            carFound = car;
            break;
        }
    }
    return carFound;
}

/**
 * @brief CarGroupQueue::insertCar insert car in the given row at pos
 * @param car
 * @param row
 * @param pos
 */
void CarGroupQueue::insertCar(const std::shared_ptr<Car>& car, const size_t& row, const size_t& pos) {
    auto itRow = m_cars.at(row).begin();
    size_t countPos = 0;
    while (countPos != pos && itRow != m_cars.at(row).end()) {
        countPos++;
        itRow++;
    }
    if (itRow != m_cars.at(row).end()) {
        m_cars.at(row).insert(itRow, car);
    }
    else if (itRow == m_cars.at(row).end()) {
        m_cars.at(row).push_back(car);
    }
}

void CarGroupQueue::swap(const int& row1, const int& col1, const int& row2, const int& col2) {
    std::shared_ptr<Car> tmpCar = m_cars.at(row1).at(col1);
    m_cars[row1][col1] = m_cars.at(row2).at(col2);
    m_cars[row2][col2] = tmpCar;
}
