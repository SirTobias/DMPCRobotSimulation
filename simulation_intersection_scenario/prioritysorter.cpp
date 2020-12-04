#include "prioritysorter.h"

#include <QDebug>

#include <future>
#include <memory>

/**
 * @brief PrioritySorter::PrioritySorter
 * @param criteria
 */
PrioritySorter::PrioritySorter(const PriorityCriteria &criteria) :
    m_criteria(criteria)
{

}

/**
 * @brief PrioritySorter::sortAfterPriority sort after the chosen priority criteria
 * @param cars a vector of vector of cars to have independent priority order of several groups of cars
 * @param contin solution for the cars
 * @param constraints given constraints
 * @param t0 start time
 * @param T sampling step
 * @param N horizon
 * @param radius radius for one robot car
 */
void PrioritySorter::sortAfterPriority(CarGroupQueue& cars, const std::map<QString, std::vector<std::vector<double> >>& contin,
                                       const std::multimap<QString, Constraint>& constraints,
                                       const double& t0, const double& T, const size_t& N, const double& radius) {
    std::map<QString, std::vector<std::vector<double> > > continSol = contin;
    if (m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTS) {
        auto carsSeq = cars.getOrderSeq();
        std::sort(carsSeq.begin(), carsSeq.end(), compareMinClosedLoopCostsLess);
        cars.clear();
        cars.storeOrder(carsSeq);
    }
    else if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTS) {
        auto carsSeq = cars.getOrderSeq();
        std::sort(carsSeq.begin(), carsSeq.end(), compareMinOpenLoopCostsLess);
        cars.clear();
        cars.storeOrder(carsSeq);
    }
    else if (m_criteria == PriorityCriteria::MAXCLOSEDLOOPCOSTS) {
        auto carsSeq = cars.getOrderSeq();
        std::sort(carsSeq.begin(), carsSeq.end(), compareMaxClosedLoopCostsGreater);
        cars.clear();
        cars.storeOrder(carsSeq);
    }
    else if (m_criteria == PriorityCriteria::MAXOPENLOOPCOSTS) {
        auto carsSeq = cars.getOrderSeq();
        std::sort(carsSeq.begin(), carsSeq.end(), compareMaxOpenLoopCostsGreater);
        cars.clear();
        cars.storeOrder(carsSeq);
    }
    else if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY) {
        //get first the unconstrained solution
        if (t0 == 0.0) {
            continSol = getUnconstrainedSol(cars, t0, T);
        }

        //now evaluate the intersection of trajectories
        size_t rowSize = cars.rowSize();
        //dependent cars, which must not be sorted
        std::map<size_t, std::shared_ptr<Car> > dependent;
        //std::cout << "time: " << t0 << std::endl;
        for (size_t i = 0; i < rowSize; i++) {
            auto carRow = cars.getRow(i);
            //position of current car and the other one
            size_t countPos = 0, countOtherPos = 0;
            auto itCar = carRow.begin();
            while (itCar != carRow.end()) {
                if (countPos < dependent.size()) {
                    countPos = dependent.size();
                }
                countOtherPos = countPos;
                //position of other examined car
                bool foundConflicts = false;
                Path cellsForX = (*itCar)->getPredictedTrajectoryCells((*itCar)->getCurrentStateContinuous(),
                                                                       VectorHelper::reshapeXdTo1d(continSol.at((*itCar)->getName())), t0, T, N, radius);
                //(*itCar)->addCountCommunicatedConstraints();
                if (carRow.size() > 1) {
                    //test all other vehicles in vector
                    auto itOtherCar = carRow.begin();
                    while (itOtherCar != carRow.end()) {
                        bool localCarConflict = false;
                        if ((*itCar) != *itOtherCar) {
                            Path otherCells = (*itOtherCar)->getPredictedTrajectoryCells((*itOtherCar)->getCurrentStateContinuous(),
                                                                                         VectorHelper::reshapeXdTo1d(continSol.at((*itOtherCar)->getName())), t0, T, N, radius);
                            //(*itOtherCar)->addCountCommunicatedConstraints();
                            //if conflicts are found, keep it
                            if (!cellsForX.findSimilarities(otherCells).empty()) {
                                foundConflicts = true;
                                localCarConflict = true;
                                dependent.insert(std::pair<int, std::shared_ptr<Car> >(countOtherPos, *(itOtherCar)));
                                cars.removeCar(*itOtherCar);
                                itOtherCar = carRow.erase(itOtherCar);
                            }
                        }
                        countOtherPos++;
                        if (!localCarConflict) {
                            itOtherCar++;
                        }
                    }//--while
                }
                if (!foundConflicts) {
                    itCar++;
                }
                else {
                    dependent.insert(std::pair<int, std::shared_ptr<Car> >(countPos, (*itCar)));
                    cars.removeCar((*itCar));
                    itCar = carRow.erase(itCar);
                }
                countPos++;
            }//--one car
            //now sort the dependent lists after priority
            if (m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY) {
                for (std::vector<std::shared_ptr<Car> >& sortVector : cars) {
                    std::sort(sortVector.begin(), sortVector.end(), compareMinClosedLoopCostsLess);
                }
            }
            if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY) {
                for (std::vector<std::shared_ptr<Car> >& sortVector : cars) {
                    std::sort(sortVector.begin(), sortVector.end(), compareMinOpenLoopCostsLess);
                }
            }
            //now insert back the blocked cars
            for (auto itDep = dependent.begin(); itDep != dependent.end(); itDep++) {
                cars.insertCar(itDep->second, i, itDep->first);
            }
        }//went over all rows
        //remove empty rows
        cars.removeEmptyRows();
    }
    else if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY ||
             m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
        size_t rowSize = cars.rowSize();
        //in the start, get the unconstrained solution for each car
        if (t0 == 0.0) {
            continSol = getUnconstrainedSol(cars, t0, T);
        }
        else {
            for (size_t i = 0; i < rowSize; i++) {
                auto carRow = cars.getRow(i);
                for (auto car : carRow) {
                    continSol[car->getName()] = VectorHelper::reshapeXd(car->getCurrentPrediction());
                }
            }
        }
        if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY) {
            std::vector<std::shared_ptr<Car> > movedCars;
            for (size_t i = 0; i < rowSize; i++) {
                std::vector<std::shared_ptr<Car> > movedCarsRow;
                auto carRow = cars.getRow(i);
                //first, test for conflicts regarding cells
                auto car = carRow.begin();
                while (car != carRow.end()) {
                    //evaluate conflicts in own row
                    Path cellsForX = (*car)->getPredictedTrajectoryCells((*car)->getCurrentStateContinuous(), VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N, radius);
                    (*car)->addCountCommunicatedConstraints();
                    //now test current row for the other cars
                    auto itCar = carRow.begin();
                    while (itCar != carRow.end()) {
                        if ((*car) != (*itCar)) {
                            Path cellsOther = (*itCar)->getPredictedTrajectoryCells((*itCar)->getCurrentStateContinuous(),
                                                                                    VectorHelper::reshapeXdTo1d(continSol.at((*itCar)->getName())), t0, T, N, radius);
                            (*itCar)->addCountCommunicatedConstraints();
                            if (!cellsForX.findSimilarities(cellsOther).empty()) {
                                //test with priority rule and add to list
                                //iterator over row (hierarchy)
                                auto maxCar = getCarWithHigherCosts((*car), (*itCar));
                                movedCarsRow.push_back(maxCar);
                                if (maxCar == (*car)) {
                                    carRow.erase(car);
                                    break;
                                }
                                else {
                                    carRow.erase(itCar);
                                }
                            }
                            else {
                                itCar++;
                            }
                        }
                        else {
                            itCar++;
                        }
                    }//--iterate over cars in one row
                    car++;
                }//--iterator over row (hierarchy)
                //move car to next row
                for (auto car : movedCarsRow) {
                    cars.moveCarToNextRow(car);
                    movedCars.push_back(car);
                }
            }//--iterate over row
            //now test for active constraints (for level 1 to x)
            for (size_t i = rowSize - 1; i > 0; i--) {
                auto carRow = cars.getRow(i);
                auto car = carRow.begin();
                while (car != carRow.end() && !cars.findCarInVector(movedCars, (*car))) {
                    bool foundConflict = false;
                    //now look for the upper rows (first, evaluate active constraints)
                    auto upRow = cars.getPreviousRow(carRow);
                    for (auto itCar = upRow.begin(); itCar != upRow.end(); itCar++) {
                        auto carConstraints = getConstraintsFromMap(constraints, (*itCar)->getName());
                        foundConflict = (*car)->testActiveConstraints(carConstraints, VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N);
                        if (foundConflict) {
                            break;
                        }
                    }
                    //no conflicts, so can move up a row
                    if (!foundConflict) {
                        if (cars.hasPreviousRow(carRow)) {
                            cars.moveCarPreviousRow((*car));
                            carRow.erase(car);
                        }
                    }//--conflict-free, may move up
                    else {
                        car++;
                    }
                }
            }
        }//--Hierarchy
        else if ( m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
            std::vector<std::shared_ptr<Car> > movedCars;
            for (size_t i = 0; i < rowSize; i++) {
                auto carRow = cars.getRow(i);
                auto car = carRow.begin();
                //evaluate conflicts for all rows
                while (car != carRow.end()) {
                    bool carHasMoved = false;
                    Path cellsForX = (*car)->getPredictedTrajectoryCells((*car)->getCurrentStateContinuous(), VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N, radius);
                    for (size_t j = 0; j < rowSize; j++) {
                        auto carRowj = cars.getRow(j);
                        auto itCar = carRowj.begin();
                        while (itCar != carRowj.end()) {
                            if ((*car) != (*itCar) && !cars.isInRelationTo((*car), (*itCar)) ) {
                                Path cellsOther = (*itCar)->getPredictedTrajectoryCells((*itCar)->getCurrentStateContinuous(),
                                                                                        VectorHelper::reshapeXdTo1d(continSol.at((*itCar)->getName())), t0, T, N, radius);
                                if (!cellsForX.findSimilarities(cellsOther).empty()) {
                                    //test with priority rule and add to list
                                    //iterator over row (hierarchy)
                                    auto maxCar = getCarWithHigherCosts((*car), (*itCar));
                                    movedCars.push_back(maxCar);
                                    if (maxCar == (*car)) {
                                        if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
                                            cars.addSuccessor((*itCar), (*car));
                                            carHasMoved = true;
                                        }
                                        carRow.erase(car);
                                        //if current row and row j are identical, has to be deleted also
                                        /*if (i == j) {
                                            auto itDel = carRow.begin();
                                            while (itDel != carRow.end() && (*itDel) != (*car)) {
                                                itDel++;
                                            }
                                            carRowj.erase(itDel);
                                            //DEBUG
                                            qDebug() << "car moved: " << maxCar->getName() << endl;
                                        }*/
                                        break;
                                    }
                                    else {
                                        if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
                                            cars.addSuccessor((*car), (*itCar));
                                        }
                                        carRowj.erase(itCar);
                                        //if current row and row j are identical, has to be deleted also
                                        if (i == j) {
                                            auto itDel = carRow.begin();
                                            while (itDel != carRow.end() && (*itDel) != (*car)) {
                                                itDel++;
                                            }
                                            carRow.erase(itDel);
                                        }
                                    }
                                }
                                else {
                                    itCar++;
                                }
                            }
                            else {
                                itCar++;
                            }
                        }//--iterate over cars in one row in j
                    }//--iterate over all rows (j)
                    //only move iterator, if it has not moved
                    if (!carHasMoved) {
                        car++;
                    }
                }//--hold one car for i
            }//--iterate over rows (i)
            //now again test for direct successors, if they still have active constraints
            for (size_t i = rowSize - 1; i > 0; i--) {
                auto carRow = cars.getRow(i);
                auto car = carRow.begin();
                while (car != carRow.end() && !cars.findCarInVector(movedCars, (*car))) {
                    auto predList = cars.getDirectPredecessors((*car));
                    auto predCar = predList.begin();
                    while (predCar != predList.end() && predList.size() > 0) {
                        std::shared_ptr<Car> tempCar = *predCar;
                        auto carConstraints = getConstraintsFromMap(constraints, (*predCar)->getName());
                        bool foundConflict = (*car)->testActiveConstraints(carConstraints, VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N);
                        //DEBUG
                        //auto rootCar = cars.getRoot(*car);
                        //cars.showCarGroupTree(rootCar);
                        //--DEBUG
                        if (!foundConflict) {
                            //in std::list, iterator has first to be set to next element, then elements before iterator can be deleted
                            auto removePredIt = predCar;
                            predCar++;
                            cars.removeSuccessor((*removePredIt), (*car));
                            predList.remove((*removePredIt));
                            //DEBUG
                            /*qDebug() << "PredListSize: " << predList.size() << endl;
                            qDebug() << "List for car" << (*car)->getName();
                            for (auto itPredList = predList.begin(); itPredList != predList.end(); itPredList++) {
                                qDebug() << "Predecessors: " << (*itPredList)->getName();
                            }
                            rootCar = cars.getRoot(*car);
                            cars.showCarGroupTree(rootCar);*/
                            //--DEBUG
                        }
                        else {
                            predCar++;
                        }
                    }
                    //test now, if the execution level changes (current execution level counts)
                    /*size_t carExecLevel = cars.getExecLevelAccordingPreds((*car));
                    size_t curRowPos = cars.getPosForRow(cars.findVector(*car));
                    if (curRowPos != carExecLevel) {
                        cars.moveCar((*car), carExecLevel);
                    }*/
                    car++;
                }//--car
            }//--iterate over rows
        }//--tree-based
    cars.removeEmptyRows();
    }
    else if (m_criteria == PriorityCriteria::FIXED) {
        //nothing
    }
}

/**
 * @brief PrioritySorter::testCurrentCarsForCurrentRow tests the car with current solution for the current row if there is any conflict in the hierarchy and moves the car
 * @param cars Queue with cars
 * @param continSol
 * @param car row to test
 * @param radius safety distance
 * @param t0 start time
 * @param T
 * @param N
 * @return QStringList with cars removed
 */
QStringList PrioritySorter::testCurrentCarsForCurrentRow(CarGroupQueue& cars, const std::map<QString, std::vector<std::vector<double> >> continSol,
                                                 std::vector<std::shared_ptr<Car> >& carRow, const double& radius, const double& t0, const double& T, const size_t& N)  {
    QStringList removedCars;
    if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY) {
        auto car = carRow.begin();
        while (car != carRow.end()) {
            bool isCarRemoved = false;
            Path cellsForX = (*car)->getPredictedTrajectoryCells((*car)->getCurrentStateContinuous(), VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N, radius);
            //now test current row for the other cars
            auto itCar = carRow.begin();
            while ( itCar != carRow.end()) {
                if ( (*itCar) != (*car) && !cars.isInRelationTo((*car), (*itCar)) ) {
                    Path cellsOther = (*itCar)->getPredictedTrajectoryCells((*itCar)->getCurrentStateContinuous(),
                                                                            VectorHelper::reshapeXdTo1d(continSol.at((*itCar)->getName())), t0, T, N, radius);
                    if (!cellsForX.findSimilarities(cellsOther).empty()) {
                        std::shared_ptr<Car> carToMove = getCarWithHigherCosts((*itCar), (*car));
                        removedCars.append(carToMove->getName());
                        cars.moveCarToNextRow(carToMove);
                        if (carToMove == (*car)) {
                            carRow.erase(car);
                            isCarRemoved = true;
                            break;
                        }
                        else {
                            carRow.erase(itCar);
                        }
                    }
                    else {
                        itCar++;
                    }
                }
                else {
                    itCar++;
                }
            }
            if (!isCarRemoved) {
                car++;
            }
        }
    }
    else if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
        //we have to test for all cars
        auto car = carRow.begin();
        //test the row for all cars
        while (car != carRow.end()) {
            bool isMoved = false;
            Path cellsForX = (*car)->getPredictedTrajectoryCells((*car)->getCurrentStateContinuous(), VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), t0, T, N, radius);
            for (size_t i = 0; i < cars.rowSize(); i++) {
                auto itRow = cars.getRow(i);
                if (!isMoved) {
                    auto itCar = itRow.begin();
                    //iterate now over all rows
                    while (itCar != itRow.end()) {
                        if ((*itCar) != (*car) && continSol.find((*itCar)->getName()) != continSol.end() && !cars.isInRelationTo((*car), (*itCar)) ) {
                            Path cellsOther = (*itCar)->getPredictedTrajectoryCells((*itCar)->getCurrentStateContinuous(),
                                                                                    VectorHelper::reshapeXdTo1d(continSol.at((*itCar)->getName())), t0, T, N, radius);
                            if (!cellsForX.findSimilarities(cellsOther).empty() ) {
                                std::shared_ptr<Car> carToMove = getCarWithHigherCosts((*itCar), (*car));

                                removedCars.append(carToMove->getName());
                                if (carToMove == (*car)) {
                                    cars.addSuccessor((*itCar), (*car));
                                    carRow.erase(car);
                                    //car = carRow.begin(); //workaround!!!
                                    isMoved = true;
                                    break;
                                }
                                else {
                                    cars.addSuccessor((*car), (*itCar));
                                    itRow.erase(itCar);
                                }
                            }
                            else {
                                itCar++;
                            }
                        }
                        else {
                            itCar++;
                        }
                    }//--iterate over row j
                } //--moved
            }//--iterate over rows (j)
            if (!isMoved) {
                car++;
            }
        }//--iterate over row of cars
    }
    return removedCars;
}

/**
 * @brief PrioritySorter::getCarWithHigherCosts given two cars, returns the one with lower costs (lower priority) according to the chosen priority criteria m_criteria
 * @param car1
 * @param car2
 * @return
 */
std::shared_ptr<Car> PrioritySorter::getCarWithHigherCosts(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) const {
    std::vector<std::shared_ptr<Car> > cars = {car1, car2};
    if (m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTS || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY
            || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY || m_criteria == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
        std::sort(cars.begin(), cars.end(), compareMinClosedLoopCostsLess);
    }
    else if (m_criteria == PriorityCriteria::MINOPENLOOPCOSTS || m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY
             || m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY || m_criteria == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE) {
        std::sort(cars.begin(), cars.end(), compareMinOpenLoopCostsLess);
    }
    else if (m_criteria == PriorityCriteria::MAXCLOSEDLOOPCOSTS) {
        std::sort(cars.begin(), cars.end(), compareMaxClosedLoopCostsGreater);
    }
    else if (m_criteria == PriorityCriteria::MAXOPENLOOPCOSTS) {
        std::sort(cars.begin(), cars.end(), compareMaxOpenLoopCostsGreater);

    }
    // cars is local vector
    return cars.back();
}


/**
 * @brief PrioritySorter::findCarInDeorderPriorityMap returns the vector of car if is is in one of the priority lists in the deordering map. The idea is to handle seperate lists
 * for each group of depending cars
 * @param deorderAndPriorityMap
 * @param car
 * @return
 */
std::vector<std::shared_ptr<Car> > PrioritySorter::findCarInDeorderPriorityMap(CarGroupQueue &deorderAndPriorityMap, const std::shared_ptr<Car>& car) {
    return deorderAndPriorityMap.findVector(car);
}

/**
 * @brief PrioritySorter::compareMinClosedLoopCostsLess compares the minimum closed loop costs
 * @param car1
 * @param car2
 * @return true, if costs of car1 < car2
 */
bool PrioritySorter::compareMinClosedLoopCostsLess(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) {
    return car1->getClosedLoopCosts() < car2->getClosedLoopCosts();
}

/**
 * @brief PrioritySorter::compareMinOpenLoopCostsLess compares the minimum open loop costs
 * @param car1
 * @param car2
 * @return true, if costs of car1 < car2
 */
bool PrioritySorter::compareMinOpenLoopCostsLess(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) {
    return car1->getOpenLoopCosts() < car2->getOpenLoopCosts();
}

/**
 * @brief PrioritySorter::compareMaxClosedLoopCostsGreater compares the maximum closed loop costs
 * @param car1
 * @param car2
 * @return true, if costs of car1 > car2
 */
bool PrioritySorter::compareMaxClosedLoopCostsGreater(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) {
    return car1->getClosedLoopCosts() > car2->getClosedLoopCosts();
}


/**
 * @brief PrioritySorter::compareMaxOpenLoopCostsGreater compares the maximum open loop costs
 * @param car1
 * @param car2
 * @return true, if costs of car1 > car2
 */
bool PrioritySorter::compareMaxOpenLoopCostsGreater(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) {
    return car1->getOpenLoopCosts() > car2->getOpenLoopCosts();
}

/**
 * @brief PrioritySorter::getTextForCriteria
 * @return
 */
QStringList PrioritySorter::getTextForCriteria() {
    QStringList criteriaList;
    for (int i = 0; i < getPriorityCriteriaMapSize(); i++) {
        criteriaList << priorityCriteriaMap[i].text;
    }
    return criteriaList;
}

/**
 * @brief PrioritySorter::getPriorityCriteria returns the chosen priority criteria
 * @return
 */
PriorityCriteria PrioritySorter::getPriorityCriteria() const {
    return m_criteria;
}

/**
 * @brief PrioritySorter::getTextForChosenCriteria returns a string for the chosen priority criteria
 * @param criteria
 * @return QString with chosen criteria
 */
QString PrioritySorter::getTextForChosenCriteria(const PriorityCriteria& criteria) {
    QString ret = QString::null;
    for (int i = 0; i < getPriorityCriteriaMapSize(); i++) {
        if (criteria == priorityCriteriaMap[i].criteria) {
            ret = priorityCriteriaMap[i].text;
        }
    }
    return ret.toLower();
}

std::map<QString, std::vector<std::vector<double> > > PrioritySorter::getUnconstrainedSol(const CarGroupQueue& cars, const double& t0, const double& T) const {
    std::map<QString, std::vector<std::vector<double> > > continSol;
    //calculate the OCP without any constraints
    for (std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        car->clearAllConstraints();
        car->createGlobalConstraints();
        auto optControl = std::async(std::launch::async, &Car::calcOcpObjectiveContinuous, car, car->getInitialControl(t0, T), t0, T);
        continSol[car->getName()] = optControl.get();
    }
    return continSol;
}

/**
 * @brief PrioritySorter::getConstraintsFromMap returns the constraint from the multimap
 * @param constraints
 * @param car
 * @return
 */
std::vector<Constraint> PrioritySorter::getConstraintsFromMap(const std::multimap<QString, Constraint>& constraints, const QString& car) const {
    auto carConstraints = constraints.equal_range(car);
    std::vector<Constraint> constraintList;
    for (auto itConstraint = carConstraints.first; itConstraint != carConstraints.second; itConstraint++) {
        constraintList.push_back(itConstraint->second);
    }
    return constraintList;
}
