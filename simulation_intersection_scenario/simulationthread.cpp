#include "simulationthread.h"
#include "globalcarlist.h"
#include "intersectionparameters.h"

#include <QtTest/QTest>
#include <QtCore/QStringList>
#include <QtCore/QVariantList>
#include <QtCore/QMap>
#include <QtCore/QStringBuilder>


/**
 * @brief SimulationThread::SimulationThread initializes a separate thread to run calculations
 * @param m width of intersection
 * @param k height of intersection
 * @param maxCars maximum number of cars in intersection
 * @param N
 * @param lambda
 * @param parent parent QObject
 */
SimulationThread::SimulationThread(const int &width, const int &height, const int &maxCars, const size_t &N, const double& T, const double &lambda,
                                   const std::pair<double, double> &bounds, const std::pair<double, double> &gridSize, QObject *parent,
                                   const double &robotDiameter, const PriorityCriteria &priority, const CommunicationScheme &commScheme, const PathAlgorithm& pathAlgorithm) : QThread(parent),
    k(width-1),
    m(height-1),
    m_N(N),
    maxCars(maxCars),
    lambda(lambda),
    Pause(false),
    targetReached(false),
    countSteps(0),
    //d_db(nullptr),
    m_t0(0.0),
    m_T(T),
    m_controlBounds(bounds),
    m_gridSize(gridSize),
    m_currentGridSize(gridSize.first),
    m_firstSimRun(true),
    m_commScheme(commScheme),
    m_radius(robotDiameter),
    m_priority(priority),
    m_pathAlgorithm(pathAlgorithm),
    m_numberOfCars(0)

{
    /*if (priority == PriorityCriteria::FIXED || priority == PriorityCriteria::MAXCLOSEDLOOPCOSTS
        || priority == PriorityCriteria::MAXOPENLOOPCOSTS || priority == PriorityCriteria::MINCLOSEDLOOPCOSTS
            || priority == PriorityCriteria::MINOPENLOOPCOSTS) {
        m_cars.setType(CarGroupQueueType::FIXED);
    }
    else if (priority == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY
             || priority == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY ) {
        m_cars.setType(CarGroupQueueType::SETBASED);
    }*/
    qRegisterMetaType<std::vector<std::vector<double> >>("std::vector<std::vector<double> >");
    qRegisterMetaType<QMap<int,int> >("QMap<int,int>");
    interSection = std::make_shared<InterSection>(k, m, m_currentGridSize);
    eval.disableTitle(true);
    debugFile.setFileName("debugOut.txt");

    if (!debugFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        std::cout << "cannot open debug file" << std::endl;
    }

    outStream.setDevice(&debugFile);
    //parent->getDbThread();
    //set up connection to databse
    //d_db = new DataBaseCore();
    //if (!d_db->connect("localhost","simulation_intersection","Homyum","123456"))break;
    //d_db->connect("localhost","simulation_intersection","Homyum","123456");

    outStream << "Car-ID, Start, PathItem(n,k,m), Target(n,k,m)" << endl;

}

/** @brief set Pause flag to true
 */
void SimulationThread::pause()
{
    Pause = true;

}

/** @brief Resumes simulation, wakes up thread
 */
void SimulationThread::resume()
{
    Pause = false;
    pauseSimulation.wakeOne();
}

/** @brief build intersection and start thread running
 */
void SimulationThread::startSimulation()
{
    qDebug() << "horizon length: " << m_N;
    if (InterSectionParameters::varyCellSize == 0) {
        //interSection->setCellSize(1.0);
        makeCarsAndIntersection();
        //initialize first costs here
        //should only be done in the beginning - countSteps == 0
        Q_ASSERT(countSteps == 0);
        if (InterSectionParameters::directComm == 1 ) {
           for (std::shared_ptr<Car>& car : m_cars.getOrderSeq()) {
               car->initializeCosts();
           }
        }

        start(LowPriority);
    }//--fixed grid size
    //dynamic grid size - start first run and then connect finished with startMultipleRuns
    else if (InterSectionParameters::varyCellSize == 1) {
        eval.setCellSize(m_currentGridSize);
        qDebug() << "gridsize: " << m_currentGridSize;
        if (m_firstSimRun) {
            makeCarsAndIntersection();
            if (m_cars.size() > 3) {
                m_cars.swap(0, 1, 0, 2);
            }
            connect (this, SIGNAL(simFinished()), this, SLOT(startMultipleSimRuns()));
        }
        m_firstSimRun = false;
        start(LowPriority);

    }//--dynamic grid size via control range
}

/**
 * @brief SimulationThread::startMultipleSimRuns
 */
void SimulationThread::startMultipleSimRuns() {
    eval.setCostsContinuousInfinity(eval.sumCostsOfCarToInfinity(eval.getCostsContinuous(CostType::CLOSEDLOOP)), CostType::CLOSEDLOOP);
    eval.setCostsContinuousInfinity(eval.sumCostsOfCarToInfinity(eval.getCostsContinuous(CostType::OPENLOOP)), CostType::OPENLOOP);
    std::map<unsigned int, unsigned int> commEffort = eval.getCommConstraintsPerStep();
    //evaluates first the full communication, then the differential communication
    if (m_commScheme == CommunicationScheme::FULL) {
        eval.setCommEffortWholeSimulationSum(eval.getCommConstraintsForWholeSim(commEffort));
    }
    else if (m_commScheme == CommunicationScheme::DIFFERENTIAL) {
        eval.setDiffCommEffortWholeSimulationSum(eval.getCommConstraintsForWholeSim(commEffort));
    }
    std::map<QString, double> culmCarClosedLoopCosts = eval.getCostsContinuousInfinity(CostType::CLOSEDLOOP);
    std::map<QString, double> culmCarOpenLoopCosts = eval.getCostsContinuousInfinity(CostType::OPENLOOP);
    eval.setCostsContinuousInfinity(culmCarClosedLoopCosts, CostType::CLOSEDLOOP);
    eval.setCostsContinuousInfinity(culmCarOpenLoopCosts, CostType::OPENLOOP);
    eval.addCommEffortClosedLoopPerformance(eval.communicationEffortCostsPerformance(commEffort, culmCarClosedLoopCosts));
    eval.addCommEffortOpenLoopPerformance(eval.communicationEffortCostsPerformance(commEffort, culmCarOpenLoopCosts));
    eval.setGridSizeToCosts(eval.getCostsToCellSize(CostType::CLOSEDLOOP), CostType::CLOSEDLOOP);
    eval.setGridSizeToCosts(eval.getCostsToCellSize(CostType::OPENLOOP), CostType::OPENLOOP);
    //TODO: clean up simulation-runtime-based costs and comm. costs
    //catch up roundoff-limit
    QApplication::processEvents();
    if (m_currentGridSize < m_gridSize.second && m_commScheme != CommunicationScheme::CONTINUOUS) {
        if (m_currentGridSize == 2.0) {
            m_currentGridSize += 1.0;
        }
        else {
            m_currentGridSize += 0.5;
        }
        m_currentGridSize = std::floor(m_currentGridSize * (unsigned int)10) / (double)10;
        //TODO: redraw cells
        //emit drawCells(getGridHeight(), getGridWidth());
        makeCarsAndIntersection();
        startSimulation();
    }
    //run the simulations with IntervalMoving principle
    else if ( m_currentGridSize == m_gridSize.second && m_commScheme == CommunicationScheme::MINMAXINTERVAL) {
        eval.clearStatisticsAfterIteratedCellSizes();
        m_commScheme = CommunicationScheme::MINMAXINTERVALMOVING;
        m_currentGridSize = m_gridSize.first;
        makeCarsAndIntersection();
        startSimulation();
    }
    //run the whole simulations now with differential communication
    else if ( m_currentGridSize == m_gridSize.second && m_commScheme == CommunicationScheme::FULL) {
        eval.clearStatisticsAfterIteratedCellSizes();
        m_commScheme = CommunicationScheme::DIFFERENTIAL;
        m_currentGridSize = m_gridSize.first;
        makeCarsAndIntersection();
        startSimulation();
    }
    //run now the simulation with continuous communcation and without any gridsize
    else if (m_currentGridSize == m_gridSize.second && m_commScheme == CommunicationScheme::DIFFERENTIAL) {
        eval.plotDifferentialCommunicationAndNormalCommunicationToCellSize(true, true);
        m_commScheme = CommunicationScheme::CONTINUOUS;
        m_currentGridSize = 0.0;
        makeCarsAndIntersection();
        startSimulation();
    }
    else {
        eval.plotCommEffortCostsInfinity(CostType::CLOSEDLOOP);
        eval.plotCommEffortCostsInfinity(CostType::OPENLOOP);
        eval.plotCommEffortToCellSize();
        eval.plotCostToCellSize(CostType::CLOSEDLOOP);
        eval.plotCostToCellSize(CostType::OPENLOOP);
        eval.plotCulmCostsOverAllCellSizes(this->m_N, CostType::CLOSEDLOOP);
        eval.plotCulmCostsOverAllCellSizes(this->m_N, CostType::OPENLOOP);
        if (m_commScheme != CommunicationScheme::CONTINUOUS) {
            eval.plotConsumedCellDifference(maxCars, m_N);
        }
        if (m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY
                || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY
                || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE) {
            eval.plotMaxPriorityQueueLength(maxCars, m_N, m_priority.getTextForChosenCriteria(m_priority.getPriorityCriteria()));
            eval.plotNumberOfPriorityQueues(maxCars, m_N, m_priority.getTextForChosenCriteria(m_priority.getPriorityCriteria()));
        }
        //if (InterSectionParameters::intersectionalScenario == 1) {
            eval.plotPredictionDifference(m_N, true);
            eval.plotGridPredictionDifference(m_N, true);
            eval.plotPredictionDifferenceOverCellSizes(m_N);
            eval.plotOccupancyGridDifferenceOverCellSizes(m_N);
        //}
        eval.showPlots();
        eval.exportPlots("PDF");
        //TODO: OUTPUT for table.txt - communication effort
        std::map<QString, Plot2d*> plots = eval.plots();
        for (auto itPlot = plots.begin(); itPlot != plots.end(); itPlot++) {
            if (itPlot->first.contains("comm")) {
                eval.exportPlotToText(itPlot->first + ".txt", itPlot->second);
            }
        }
    }
}

/** @brief demonstrate: calculate one step at a time
 * finish execution when targets have been reached
 */
void SimulationThread::run()
{
    //--DEBUG

    mutex.lock();
    while (!targetReached) {
        outStream << "Step: " << countSteps << endl;
        emit steps(countSteps);
        interSection->copyCurrentPositionsToPreliminaries();
        updateCellReservations();
        //take the first step for solution
        std::map<QString, PathItem> nextTargets;
        std::map<QString, std::vector<std::vector<double> > > continTargets = calculateStep(nextTargets);

        //DEBUG
        debugFile.flush();
        //--DEBUG

        evaluateStep(nextTargets, continTargets);
        //if all cars have reached their target, we are done
        if ( (m_cars.size() == 0 && m_waitCars.size() == 0) || (InterSectionParameters::intersectionalScenario == 1 && InterSectionParameters::stochasticArrival == 1 && countSteps > 120)) {
            targetReached = true;
        }
        countSteps++;
        //shift to next timestep
        setGlobalTime(getGlobalTime() + m_T);
        //m_t0 += m_T;

    }//--while - targetreached

    m_output.insertMulti("carname", d_carName);
    m_output.insertMulti("state", d_state);
    m_output.insertMulti("time", d_time);
    m_output.insertMulti("x", d_x);
    m_output.insertMulti("y", d_y);
    emit StartDBThread(m_output);

    debugFile.close();
    mutex.unlock();

    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        eval.plotAppliedContControl(m_N, false);
        eval.plotCosts(m_N,CostType::OPENLOOP, true, false);
        eval.plotCosts(m_N,CostType::CLOSEDLOOP, true, false);
        eval.plotPredictionDifference(m_N, true);
        eval.plotGridPredictionDifference(m_N, true);
        eval.plotDeltaOfCellSize(maxCars, m_N, m_currentGridSize);
        if (InterSectionParameters::stochasticArrival == 1) {
            eval.plotOccupancyCellsHeatMap(countSteps, m_numberOfCars, m_waitCars.size(), m_currentGridSize, m_distParams);
        }
        if (InterSectionParameters::varyCellSize == 0) {
            eval.showPlots();
            eval.exportPlots("PDF");
        }
    }
    emit simFinished();


}

/** @brief calculate the next step each car will take
 * @param nextTargets
 */
std::map<QString, std::vector<std::vector<double> >> SimulationThread::calculateStep(std::map<QString, PathItem>& nextTargets)
{
    if (m_constraints.empty()) {
        m_constraints = appendConstraintsFromPosition(m_cars);
    }
    else {
        m_constraints = deleteOldConstraints(m_constraints);
    }
    //stochastic arrival for cars
    if (InterSectionParameters::stochasticArrival == 1 && InterSectionParameters::intersectionalScenario == 1) {
        createInterArrivalCars();
        m_cars = insertCarsFromWaitingQueue(m_waitCars, m_cars);
    }
    std::map<QString, std::vector<std::vector<double> >> continSol;
    for (std::shared_ptr<Car>& car : m_cars.getOrderSeq()) {
        car->clearAllConstraints();
        car->createGlobalConstraints();
        continSol[car->getName()] = VectorHelper::reshapeXd(car->getInitialControl(m_t0, m_T));
    }
    bool firstCar = true;
    if (m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY
            || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY
            || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY
            || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY
            || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE
            || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
        m_priority.sortAfterPriority(m_cars, continSol, m_constraints, m_t0, m_T, m_N, m_radius);
    }
    else if (m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTS || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTS) {
        m_priority.sortAfterPriority(m_cars, continSol, m_constraints);
    }
    size_t rowSize = m_cars.rowSize();
    for (size_t i = 0; i < rowSize; i++) {
        auto carRow = m_cars.getRow(i);
        std::multimap<QString, Constraint> constraintsForRow;
        auto car = carRow.begin();
        while (car != carRow.end()) {
            if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
                nextTargets[(*car)->getName()] = (*car)->calcOcpObjective((*car)->getCurrentState());
                (*car)->setCurrentConstraints(m_constraints, m_t0, m_T);

                //currentConstr = car->formulateConstraintsForNextCar(nextTargets[car->getName()], getGlobalTime(), m_T, m_N, firstCar,
                //        this->m_currentGridSize, this->m_radius, m_commScheme);
                //firstCar = false;
                //m_constraints = insertFormulatedConstraints(car->getName(), m_constraints, currentConstr);
            }
            else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
                //for each row (cluster) the constraints are deleted
                if ((m_commScheme != CommunicationScheme::DIFFERENTIAL) && (m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY
                    || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY
                    || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE
                    || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE)) {
                    if (firstCar) {
                        m_constraints.clear();
                    }
                }
                //remove predictions from own car, because new prediction will be calculated
                //TODO: has to be done later, because for difference communication purpose
                //m_constraints = car->removeOldPredictions(m_constraints);
                //set current constraints from other cars except own
                if (m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
                    (*car)->constructConstraintFromMinMaxConstraints(m_constraints, m_t0, m_T, m_N, this->m_currentGridSize, this->m_radius, this->m_commScheme);
                }
                else {
                    if (m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE
                            || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE) {
                        std::list<std::shared_ptr<Car> > predList;
                        m_cars.computeRecursivePredecessors((*car), predList);
                        (*car)->setConstraintsFromPredecessors(m_constraints, predList, m_t0, m_T);
                    }
                    else if (m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTS
                             ||m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTS ) {
                        (*car)->setCurrentConstraints(m_constraints,m_t0, m_T);
                    }
                    else {
                        if (m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY
                                || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY) {
                            std::list<std::shared_ptr<Car> > predList;
                            m_cars.computeRecursivePredecessors((*car), predList);
                            (*car)->setConstraintsFromPredecessors(m_constraints, predList, m_t0, m_T);
                        }
                        else {
                            //if (!firstCar) {
                                (*car)->setCurrentConstraints(m_constraints,m_t0, m_T);
                            //}
                        }

                    }
                }
                continSol[(*car)->getName()] = (*car)->calcOcpObjectiveContinuous(VectorHelper::reshapeXdTo1d(continSol.at((*car)->getName())), getGlobalTime(), m_T);

                emit updateCarGUIReservationsClear((*car)->getName());
                //formulate own constraints

                std::vector<Constraint> currentConstr;
                if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL) {
                    currentConstr = (*car)->formulateConstraintsForNextCar(continSol[(*car)->getName()], getGlobalTime(), m_T, m_N, firstCar, this->m_currentGridSize, this->m_radius, m_commScheme);
                }
                else if (m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
                    currentConstr = (*car)->calculateMinMaxConstraintForNextCar(continSol[(*car)->getName()], getGlobalTime(), m_T, m_N, firstCar, this->m_currentGridSize, this->m_radius, m_commScheme);
                }
                else if (m_commScheme == CommunicationScheme::CONTINUOUS) {
                    //we do not use a grid here, therefore avoid grid size
                    currentConstr = (*car)->formulateConstraintsForNextCar(continSol[(*car)->getName()], getGlobalTime(), m_T, m_N, firstCar, 0.0, this->m_radius, m_commScheme);
                }
                firstCar = false;
                if ( m_priority.getPriorityCriteria() != PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY
                        && m_priority.getPriorityCriteria() != PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY
                        && m_priority.getPriorityCriteria() != PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE
                        && m_priority.getPriorityCriteria() != PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE
                        /*&& m_priority.getPriorityCriteria() != PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY
                        && m_priority.getPriorityCriteria() != PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY*/) {
                    m_constraints = insertFormulatedConstraints((*car)->getName(), m_constraints, currentConstr);
                }
                else {
                    constraintsForRow = insertFormulatedConstraints((*car)->getName(), constraintsForRow, currentConstr);
                }
            }

            //direct communication with neighbours
            //current costs are sent to neighbours
            if (InterSectionParameters::directComm == 1) {
                (*car)->sendCostsToNeighbours();
            }
            if (Pause) {
                pauseSimulation.wait(&mutex);
            }
            if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
                //emit here for GUI Update the new position
                emit updateCarGUI((*car)->getName(), (*car)->getCurrentState().getX(), (*car)->getCurrentState().getY());
            }
            else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
                //emit here for the GUI update the new position
                emit updateCarGUI((*car)->getName(), (*car)->getCurrentStateContinuous().at(0), (*car)->getCurrentStateContinuous().at(1));
                //emit the current prediction
                emit updateCarGUIPrediction((*car)->getName(), (*car)->getPredictedTrajectory((*car)->getCurrentStateContinuous(), (*car)->getCurrentPrediction(),m_t0, m_T, m_N));
                emit updateCarGUIDynamicReservations((*car)->getName(), (*car)->getOccupiedCells());
                //DEBUG
                //qDebug() << "emitted position for car "<< car->getName() <<  ": " << car->getCurrentStateContinuous().at(0) << "," << car->getCurrentStateContinuous().at(1);
                //--DEBUG
            }
            msleep(100);
            car++;
        }//--car for carRow
        //when one row in the prioriy queue is solved, for the next independent row the former constraint do not matter
        if (m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY) {
            m_constraints.clear();
        }//add the constraints for the next hierarchy level
        else if (m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY
                 || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY
                 || m_priority.getPriorityCriteria() == PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE
                 || m_priority.getPriorityCriteria() == PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE) {
            //remove constraints from cars, which are shifted to the later hierarchy
            QStringList removedCars = m_priority.testCurrentCarsForCurrentRow(m_cars, continSol, carRow, this->m_radius, m_t0, m_T, m_N);
            if (!removedCars.empty()) {
                for (auto carName : removedCars) {
                    constraintsForRow = removeConstraintsOfMovedCars(carName, constraintsForRow);
                    //clear the prediction, as this is invalid
                    if (m_cars.getCarById(carName)) {
                        m_cars.getCarById(carName)->clearPrediction(continSol.at(carName));
                    }
                }
                //update row size
                rowSize = m_cars.rowSize();
            }
            m_constraints.insert(constraintsForRow.begin(), constraintsForRow.end());
            constraintsForRow.clear();
        }
    }
    m_cars.removeEmptyRows();

    //if (InterSectionParameters::intersectionalScenario == 1) {
        //calculate the difference from current continuous predictions to the previous one
        std::map<QString, std::vector<std::vector<double> > > predictedStates = eval.determineCurrentPredictionStates(m_cars, m_t0, m_T, m_N);
        eval.addDiffPredictions(eval.calculateDiffInPredictions(predictedStates, eval.getPrediction()));
        eval.addCountDiffPredictions(eval.countDiffInPredictions(predictedStates, eval.getPrediction()));
        eval.saveCurrentDiffPredictionsforCellSize(m_currentGridSize, eval.getDiffPredictions());
        //and save the new state
        eval.saveCurrentPrediction(eval.determineCurrentPredictionStates(m_cars, m_t0, m_T, m_N));
        //now calculate difference from occupancy grid
        eval.addCountDiffOccupancyGrid(eval.countDiffInOccupancyGrids(m_cars, eval.getOccupancyGrid()));
        eval.addDiffOccupancyGrid(eval.calculateDiffInOccupancyGrids(m_cars, eval.getOccupancyGrid()));
        eval.saveCurrentDiffOccupancyGridsForCellSize(m_currentGridSize, eval.getDiffOccupancyGrids());
        eval.saveCurrentOccupiedCells(m_cars);
    //}
    return continSol;
}

/** @brief evaluate the intersection after step is taken
 * @param nextTargets
 */
void SimulationThread::evaluateStep(std::map<QString, PathItem> &nextTargets, const std::map<QString, std::vector<std::vector<double> > > &continSol)
{
    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
        //try to reserve all preliminary solutions
        std::vector<std::shared_ptr<Car> > carsWithInvalidSolution;
        //interSection->printReservations();
        for (std::shared_ptr<Car>& car : m_cars.getOrderSeq()) {
            bool isValid = car->reservePrelimSolution();
            if (!isValid) {
                carsWithInvalidSolution.push_back(car);
            }
        }
        //for invalid solution, which cannot be reserved, reoptimize
        //at first, recopy the reserved positions
        if (carsWithInvalidSolution.size() > 0) {
            while (carsWithInvalidSolution.size() > 0) {
                interSection->copyCurrentPositionsToPreliminaries();
                for (std::shared_ptr<Car>& car : carsWithInvalidSolution) {
                    nextTargets[car->getName()] = car->calcOcpObjective(car->getCurrentState());
                }
                //apply again the optimized solution and remove cars with valid solutions
                std::vector<std::shared_ptr<Car> >::iterator it = carsWithInvalidSolution.begin();
                while ( it != carsWithInvalidSolution.end()) {
                    bool isValid = (*it)->reservePrelimSolution();
                    if (isValid) {
                        carsWithInvalidSolution.erase(it);
                    }
                    else {
                        it++;
                    }
                }

            }//--while - invalid solutions
        }//-- invalid solutions
    }//--discrete steps
    else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        for (std::shared_ptr<Car>& car : m_cars.getOrderSeq()) {
            //apply solution (u(0)) for continuous purpose
            car->applyNextState(continSol.at(car->getName()).at(0), getGlobalTime(), getGlobalTime() + m_T);
        }
    }


    //store current function values, which should all be valid
    for (std::shared_ptr<Car>& car : m_cars.getOrderSeq()) {
        //DEBUG
        /*if (car->getName() == "car0") {
            QString str = "";
            std::cout << "PathItemNext: " << nextTargets.at(car->getName()).getTime() << "," << nextTargets.at(car->getName()).getX()
                         << nextTargets.at(car->getName()).getY();
        }*/
        //--DEBUG
        //TODO: Ausgabe für contin. Simulation
        if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
            outStream << car->getName() << ": current State: "
                      << car->getCurrentState().getTime() << "," << car->getCurrentState().getX() << ","
                      << car->getCurrentState().getY()
                      << " PathItem: "
                      << nextTargets.at(car->getName()).getTime() << "," << nextTargets.at(car->getName()).getX() << ","
                      << nextTargets.at(car->getName()).getY()
                      << " Target: "
                      << car->getTarget().getTime() << "," << car->getTarget().getX() << ","
                      << car->getTarget().getY() << endl;
        }
        //update global wait time
        car->setGlobalLiveTime(m_t0 + m_T);

        //TODO: Ausgabe für contin. Simulation
        if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
            //store current state of car
            d_carName.push_back(car->getName());
            d_state.push_back(1); //state = 1, which means the pathitem is start point
            d_time.push_back((qint64)car->getCurrentState().getTime());
            d_x.push_back((qint64)car->getCurrentState().getX());
            d_y.push_back((qint64)car->getCurrentState().getY());

            d_carName.push_back(car->getName());
            d_state.push_back(0); // state = 0, which means the pathitem is final target
            d_time.push_back((qint64)car->getTarget().getTime());
            d_x.push_back((qint64)car->getTarget().getX());
            d_y.push_back((qint64)car->getTarget().getY());
            //store nextTargets pathitem of cars
            d_carName.push_back(car->getName());
            d_state.push_back(2); //state =2, which means the pathitem represents next target
            d_time.push_back((qint64)nextTargets.at(car->getName()).getTime());
            d_x.push_back((qint64)nextTargets.at(car->getName()).getX());
            d_y.push_back((qint64)nextTargets.at(car->getName()).getY());

            //get the car to the next discrete state
            car->applyNextState();
        }
        else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
            //d_state.push_back(car->getMpcController()->getSystemFunction()->getCurrentContinuousState());
        }
        //update global wait time
        car->setGlobalLiveTime(m_t0 + m_T);

    }//--cars

    //evaluate costs per car after one step
    //@TODO: would be better, if this is calculated after a car reaches its
    //target, therefore the evaluation could be made in an extra thread
    //and the simulation can continue concurrently
    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
        eval.calculateDistanceCostsPerStep(m_cars, countSteps);
        //eval.setTimeDepCostsForTimeStep(cars);
        eval.saveContCostsPerCar(m_cars);
    }
    else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        eval.saveCurrentCommunicatedConstraints(m_cars, countSteps);
        eval.saveContAppliedControl(m_cars, continSol);
        eval.saveContCostsPerCar(m_cars);
        eval.saveMaxPriorityQueueLength(m_cars, countSteps);
        eval.saveNumberOfPriorityQueues(m_cars, countSteps);
        eval.saveCurrentDeltaForCar(m_cars, countSteps);
        eval.saveCurrentCellReservations(m_cars, m_commScheme, m_currentGridSize, countSteps );
    }

    //remove cars, which has reached their target
    auto cars = m_cars.getOrderSeq();
    auto it = cars.begin();
    int row = 0; //to remove car from listWidget
        while ( it != cars.end()) {
            if ((*it)->hasTargetReached()) {
                //save path length
                eval.setPathSizePerCar((*it)->getName(), (*it)->getPath().size());
                //eval.calculateFunctionValuesPerStep(it, countSteps);

                QString keyName = (*it)->getName();
                m_cars.removeCar(*it);
                //remove car from GUI
                if (Pause)
                    pauseSimulation.wait(&mutex);
                emit removeCarfromGUI(keyName, row);
            }
            else {
                row++;
            }
            it++;
        }
}

/** @brief Tells GUI to update cell with new number of reservations
 */
void SimulationThread::updateCellReservations()
{
    if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL || m_commScheme == CommunicationScheme::MINMAXINTERVAL  || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        for (unsigned int i = 0; i < interSection->getGridWidth(); i++)
        {
            for (unsigned int j = 0; j < interSection->getGridHeight(); j++)
            {
                unsigned int reserved = interSection->getNumberOfReservations(i, j);
                if (Pause) //if pause loop is set
                {
                    pauseSimulation.wait(&mutex);
                }

                emit updateCellGUI(i, j, reserved);
            }
        }
    }
}

/** @brief Tells GUI to create and display intersection and cars
 */
void SimulationThread::makeCarsAndIntersection()
{
    if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL || m_commScheme == CommunicationScheme::MINMAXINTERVAL  || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        interSection->buildGrid(interSection->getHeight(), interSection->getWidth(), m_currentGridSize);
    }
    mutex.lock();
    //clean up
    //set back variable, otherwise simulation does not restart
    targetReached = false;
    countSteps = 0;
    m_t0 = 0;
    m_constraints.clear();
    eval.clearStatisticsAfterOneRun();
    //--clean up
    double startPos = 0.5;
    //
    int countDivisor = 0;
    for (unsigned int i = 0; i < maxCars; i++) {
        //variable start positions
        PathItem pathItem;
        PathItem pathCarTarget;
        QString nameCar = QString("car") % QString::number(i);
        std::shared_ptr<Car> car = nullptr;
        if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
            if (i % 4 == 0) {
                //leftside
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, 0, 0, 0.0);
                pathItem.setCoordinates(0, 0, reserveTime);
                //target rightside
                pathCarTarget.setCoordinates(k, 0, 0);
            }
            else if (i % 4 == 1) {
                //rightside
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, k, m, 0.0);
                pathItem.setCoordinates(0, m, reserveTime);
                //target leftside
                pathCarTarget.setCoordinates(0, 0, 0);
            }
            else if (i % 4 == 2) {
                //upside
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, 0, m, 0.0);
                pathItem.setCoordinates(k, 0, reserveTime);
                //target downside
                pathCarTarget.setCoordinates(k, m, 0);
            }
            else {
                //downside
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, k, 0, 0.0);
                pathItem.setCoordinates(k, m, reserveTime);
                //target upside
                pathCarTarget.setCoordinates(0, m, 0);
            }
            car = std::make_shared<Car>(nameCar, pathItem, pathCarTarget, m_N, lambda, m_pathAlgorithm, m_T, m_controlBounds);
            m_numberOfCars++;

        }
        else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
            std::vector<double> start, target;
            if (InterSectionParameters::intersectionalScenario == 0) {

                //normal vice-versa start
                /*if (i % 2 == 0) {
                    //left side of the starts - targets on the right side
                    //format of vector is (x_1, x_2)
                    start = {0.5, 0.85 + startPos};
                    target = { (double)k - 0.5, 0.85 + startPos };
                }
                else if (i % 2 == 1) {
                    start = {(double)k - 0.5, 0.85 + startPos};
                    target = {0.5, 0.85 + startPos};
                    if (m_currentGridSize > 2.0) {
                        startPos += m_currentGridSize;
                    }
                    else {
                        startPos += 3.0;
                    }
                }*/
                //cross-wise
                //change to 1.5, 10.5 because of Mohamed's values
                if (i % 4 == 0) {
                    start = {0.5 + 1.0, startPos + 1.0};
                    target = {(double)k - 0.5 - 1.0, (double)m - startPos - 1.0};
                }
                else if (i % 4 == 1) {
                    start = {(double)k - 0.5 - 1.0, (double)m - startPos - 1.0};
                    target = {0.5 + 1.0, startPos + 1.0};
                }
                else if (i % 4 == 2) {
                    start = {startPos + 1.0, (double)m - 0.5 - 1.0};
                    target = {(double)k - 0.5 - 1.0, startPos + 1.0};
                }
                else if (i % 4 == 3) {
                    start = {(double)k - 0.5 - 1.0, startPos + 1.0};
                    target = {0.5 + 1.0, (double)m - startPos - 1.0};
                }
                car = std::make_shared<Car>(nameCar, start, target, m_N, lambda, m_pathAlgorithm, m_T, m_controlBounds);
                m_numberOfCars++;
                if (m_commScheme != CommunicationScheme::CONTINUOUS) {
                    std::shared_ptr<InterSectionCell> cell = interSection->getCellFromCoordinates(start);
                    if (cell) {
                        cell->reserveTimeForCar(car->getName(), 0);
                    }
                }
                //take here the standard constraints
                car->createGlobalConstraints();
            }
            else if (InterSectionParameters::intersectionalScenario == 1) { //start position in each entry points (2x2)
                //start i = 0,0
                //static constraint margin between cars and safety margin
                double constraintDistance = Constraint::getOverallMargin(this->m_currentGridSize, m_radius, m_controlBounds.second, m_T) + 0.2;
                if (i % 4 == 0) {
                    start = {startPos, startPos};
                    target = {startPos, (double)m - startPos};

                    //different positions for carid > 4
                    if (i >= 4) {
                        start.at(0) += (double)countDivisor * constraintDistance;
                        target.at(0) += (double)countDivisor * constraintDistance;
                    }
                }
                else if (i % 4 == 1) {
                    start = {k - startPos, startPos};
                    target = {startPos, startPos};
                    if (i >= 4) {
                        start.at(1) += (double)countDivisor * constraintDistance;
                        target.at(1) += (double)countDivisor * constraintDistance;
                    }
                }
                else if (i % 4 == 2) {
                    start = {k - startPos, m - startPos};
                    target = {k - startPos, startPos};
                    if (i >= 4) {
                        start.at(0) -= (double)countDivisor * constraintDistance;
                        target.at(0) -= (double)countDivisor * constraintDistance;
                    }
                }
                else if (i % 4 == 3) {
                    start = {startPos, m - startPos};
                    target = {k - startPos, m - startPos};
                    if (i >= 4) {
                        start.at(1) -= (double)countDivisor * constraintDistance;
                        target.at(1) -= (double)countDivisor * constraintDistance;
                    }
                }
                car = std::make_shared<Car>(nameCar, start, target, m_N, lambda, m_pathAlgorithm, m_T, m_controlBounds);
m_numberOfCars++;
                std::shared_ptr<InterSectionCell> cell = interSection->getCellFromCoordinates(start);
                cell->reserveTimeForCar(car->getName(), 0);
                //take here the standard constraints
                car->createGlobalConstraints();
                //for intersection scenario add directional constraints
                car->createDirectionalConstraints(interSection->getWidth(), interSection->getHeight(),
                                                  m_currentGridSize, m_t0, m_N, m_T, m_radius, m_controlBounds.second);
            }
            if (i % 4 == 3) {
                countDivisor++;
            }
            //take here the standard constraints
            car->createGlobalConstraints();
        }
        else if (i % 4 == 2) {
            //upside
            if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL
                    || m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, 0, m, 0.0);
                pathItem.setCoordinates(0, m, reserveTime);
                //target downside
                pathCarTarget.setCoordinates(0, 0, 0);
            }
        }
        else {
            //downside
            if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL
                    || m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
                double reserveTime = interSection->reserveNextFreeTimeForCar(nameCar, k, 0, 0.0);
                pathItem.setCoordinates(k, 0, reserveTime);
                //target upside
                pathCarTarget.setCoordinates(k, m, 0.0);
            }
        }
        m_cars.push_back(car);
        GlobalCarList::getInstance().push_back(car);
        if (Pause) //if pause loop is set
        {
            pauseSimulation.wait(&mutex);
        }

        if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::DISCRETE) {
            emit addCarGUI(nameCar, car->getCurrentState().getX(), car->getCurrentState().getY(),
                       car->getTarget().getX(), car->getTarget().getY());
        }
        else if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
            emit addCarGUI(nameCar, car->getCurrentStateContinuous().at(0), car->getCurrentStateContinuous().at(1),
                       car->getTargetContinous().at(0), car->getTargetContinous().at(1));
        }
        //tell gui window to add intersection cells
        if (m_commScheme == CommunicationScheme::FULL || m_commScheme == CommunicationScheme::DIFFERENTIAL
                || m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
            for (unsigned int i = 0; i < interSection->getGridWidth(); i++)
            {
                for (unsigned int j = 0; j < interSection->getGridHeight(); j++)
                {
                    if (Pause) //if Pause loop is set
                    {
                        pauseSimulation.wait(&mutex);
                    }
                    emit addCellGUI(i, j, interSection->getNumberOfReservations(i,j));
                }
            }
        }
    }
    if (InterSectionParameters::intersectionalScenario == 1 && InterSectionParameters::stochasticArrival == 1) {
        if (interSection) {

            /*DistParam distParam;
            distParam.distribFunc = DistributionFunction::POISSON;
            distParam.meanArrivalTime = 1200.0;
            distParam.metric = TimeMetric::SECONDS;
            distParam.samplingTime = m_T;*/
            m_distParams = {
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 30.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 30.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 30.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 120.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 120.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 120.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T},
                {DistributionFunction::POISSON, 60.0, TimeMetric::SECONDS, m_T}
            };
            interSection->setEntryPointsStandardLanes(m_distParams);
        }
    }
    mutex.unlock();
}

/**
 * @brief SimulationThread::placeCarInStartPosition places the created car in the start position, either append this directly in the car-vector,
 * if intersection is available, otherwise in the car queue (m_waitCars)
 * @param entryPoint entryPoint of the Intersection
 * @param startMargin margin from the boundary
 */
void SimulationThread::placeCarInStartPosition(const unsigned int& entryPoint, const double& startMargin) {
    std::shared_ptr<InterSectionCell> entryCell = interSection->getEntryPoint(entryPoint);
    //start/target vectors
    std::vector<double> start, target;
    //top left
    if (entryCell && entryCell->getX() < std::floor((double)k / (m_currentGridSize * 2.0)) && entryCell->getY() < 1) {
        start = {entryCell->getX() + 0.5, entryCell->getY() + 0.5};
        target = {entryCell->getX() + 0.5, (double)m/m_currentGridSize - startMargin};

    }
    //top right
    else if (entryCell && entryCell->getX() >= std::ceil((double)k / (m_currentGridSize * 2.0)) && entryCell->getY() < std::floor((double)m / (m_currentGridSize * 2.0)) ) {
        start = {entryCell->getX() + 0.5, entryCell->getY() + 0.5};
        target = {0.5, entryCell->getY() + 0.5};
    }
    //bottom right
    else if (entryCell && entryCell->getX() >= std::ceil((double)k / (m_currentGridSize * 2.0))  && entryCell->getY() > std::ceil((double)m / (m_currentGridSize * 2.0)) ) {
        start = {entryCell->getX() + 0.5, entryCell->getY() + 0.5};
        target = {entryCell->getX(), 0.5};
    }
    //bottom left
    else if (entryCell && entryCell->getX() < 1 && entryCell->getY() >= std::ceil((double)m / (m_currentGridSize * 2.0)) ) {
        start = {entryCell->getX() + 0.5, entryCell->getY() + 0.5};
        target = {k - 0.5, entryCell->getY()};
    }
    QString carId = QString("car") + QString::number(maxCars++);
    std::shared_ptr<Car> car = std::make_shared<Car>(carId, start, target, m_N, lambda, m_pathAlgorithm, m_T, m_controlBounds);
    m_numberOfCars++;
    //add car to GUI
    if (InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
        emit addCarGUI(car->getName(), car->getCurrentStateContinuous().at(0), car->getCurrentStateContinuous().at(1),
                   car->getTargetContinous().at(0), car->getTargetContinous().at(1));
    }
    if (entryCell) {
        //take here the standard constraints
        car->createGlobalConstraints();
        //test, if start position is not colliding with any constraints
        car->setCurrentConstraints(m_constraints, getGlobalTime(), m_T);
        bool valid = car->testValidityConstraints(car->getCurrentConstraints(),
                                                                      car->getInitialControl(m_t0, m_T));
        //if invalid, is has to wait
        if (!valid) {
            car->clearAllConstraints();
            m_waitCars[car] = getGlobalTime() + m_T;
        }
        else {
            //for intersection scenario add directional constraints
            car->createDirectionalConstraints(interSection->getWidth(), interSection->getHeight(),
                                              m_currentGridSize, getGlobalTime(), m_N, m_T, m_radius,
                                              m_controlBounds.second);

            std::vector<Constraint> constraints = car->formulateConstraintsForNextCar(
                        VectorHelper::reshapeXd(car->getInitialControl(getGlobalTime(), m_T)),
                        getGlobalTime(), m_T, m_N, false, m_currentGridSize,
                        m_radius, m_commScheme);
            m_constraints = insertFormulatedConstraints(car->getName(), m_constraints, constraints);
            m_cars.push_back(car);
        }
    }
}

/**
  * @brief SimulationThread::appendConstraintsFromPosition takes the current position of the cars and
  * lists them as constraint map
  * @param cars
  * @return
  */
 std::multimap<QString, Constraint> SimulationThread::appendConstraintsFromPosition(const CarGroupQueue& cars) {
     std::multimap<QString, Constraint> posConstraints;
     //here it will always be n0, as the position here is the initial condition (should be addded for the full horizon)
     for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
       // std::shared_ptr<InterSectionCell> curCell = InterSection::getInstance()->getCellFromCoordinates(car->getCurrentStateContinuous());
         std::shared_ptr<InterSectionCell> curCell ;
         if(InterSectionParameters::sysFuncUsage == SystemFunctionUsage::CONTINUOUS) {
            curCell  = InterSection::getInstance()->getCellFromCoordinates(car->getCurrentStateContinuous());
         } else {
            PathItem as =  car->getCurrentState();
             curCell = InterSection::getInstance()->getInterSectionCell(as.getX(),as.getY());
         }

         double currentTime = m_t0;
         if (m_commScheme == CommunicationScheme::DIFFERENTIAL || m_commScheme == CommunicationScheme::FULL) {
             for (unsigned int i = 0; i < m_N ; i++) {
             posConstraints.insert(
                     std::pair<QString, Constraint>(car->getName(), Constraint({InterSection::getInstance()->getCellSize() * ((double)curCell->getX() + 0.5),
                                                                                 InterSection::getInstance()->getCellSize() * ((double)curCell->getY() + 0.5)},
                                                                                 currentTime, SystemFunctionUsage::CONTINUOUS, m_N, m_T, this->m_currentGridSize,
                                                                               this->m_radius, m_radius + 0.5) ) );//TODO: investigate current radius!!!
             currentTime += m_T;
             }
         }
         else if (m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) { //need only one constraint
             posConstraints.insert(
                     std::pair<QString, Constraint>(car->getName(), Constraint({InterSection::getInstance()->getCellSize() * ((double)curCell->getX() + 0.5),
                                                                                 InterSection::getInstance()->getCellSize() * ((double)curCell->getY() + 0.5)},
                                                                                 m_t0, SystemFunctionUsage::CONTINUOUS, m_N, m_T, this->m_currentGridSize,
                                                                               this->m_radius, m_radius + 0.5) ) );//TODO: investigate current radius!!!
         }
         else if (m_commScheme == CommunicationScheme::CONTINUOUS) {
             for (unsigned int i = 0; i < m_N ; i++) {
             posConstraints.insert(
                     std::pair<QString, Constraint>(car->getName(), Constraint({car->getCurrentStateContinuous().at(0), car->getCurrentStateContinuous().at(1)},
                                                                                 currentTime, SystemFunctionUsage::CONTINUOUS, m_N, m_T, this->m_currentGridSize,
                                                                               this->m_radius, m_radius + 0.5) ) );//TODO: investigate current radius!!!
             currentTime += m_T;
             }
         }
     }
     return posConstraints;
 }

 /**
  * @brief SimulationThread::getGlobalTime returns t0
  * @return t0
  */
 double SimulationThread::getGlobalTime() const {
     return this->m_t0;
 }

 /**
  * @brief SimulationThread::setGlobalTime sets t0
  * @param t
  */
 void SimulationThread::setGlobalTime(const double& t) {
     this->m_t0 = t;
 }

 /**
  * @brief SimulationThread::deleteOldConstraints delete the constraints which has a smaller timestamp than the global one
  * @param constraintMap
  * @return
  */
 std::multimap<QString, Constraint> SimulationThread::deleteOldConstraints(const std::multimap<QString, Constraint>& constraintMap) const {
     std::multimap<QString, Constraint> constraints = constraintMap;
     for (auto it = constraints.begin(); it != constraints.end(); ) {
         if (it->second.getConstraintTime() < getGlobalTime()) {
             it = constraints.erase(it);
         }
         else {
             ++it;
         }
     }
     //DEBUG
     /*for (auto it = constraints.begin(); it != constraints.end(); it++) {
         qDebug() << "Car: " << it->first << it->second.getConstraintTime() << " (" << it->second.getCenterPoint().at(0) << "," << it->second.getCenterPoint().at(1) << ")";
     }*/
     //--DEBUG
     qDebug() << "globalTime: " << getGlobalTime();
     return constraints;
 }

 /**
  * @brief SimulationThread::findEqualConstraint returns true, if the given constraint is already in m_constraints
  * @param constraint constraints to look for
  * @return true, if constraint is already in m_constraints, false, otherwise
  */
 bool SimulationThread::findEqualConstraint(const std::multimap<QString, Constraint>& constraintList, const Constraint& constraint) const {
     bool isIncluded = false;
     for (auto it = constraintList.begin(); it != constraintList.end(); it++) {
         if (it->second == constraint) {
             isIncluded = true;
             break;
         }
     }
     return isIncluded;
 }

 /**
  * @brief SimulationThread::removeConstraintWithEqualTimeStamp
  * @param car
  * @param constraintList
  * @param constraint
  * @return
  */
 bool SimulationThread::removeConstraintWithEqualTimeStamp(std::multimap<QString, Constraint>& constraintList, const QString& car, const Constraint& constraint) const {
     bool removedConstraint;

     for (auto it = constraintList.begin(); it != constraintList.end(); ) {
         if (car ==  it->first && constraint.getConstraintTime() == it->second.getConstraintTime()) {
             it = constraintList.erase(it);
             removedConstraint = true;
         }
         else {
             ++it;
         }
     }
     return removedConstraint;
 }

 /**
  * @brief SimulationThread::insertFormulatedConstraints
  * @param car
  * @param constaintList list with constraints
  * @param constraints new constraints
  * @return
  */
 std::multimap<QString, Constraint> SimulationThread::insertFormulatedConstraints(const QString& car, const std::multimap<QString, Constraint>& constraintList,
                                                                                  const std::vector<Constraint> &constraints) const {
     //with the interval scheme, all previous constraints can be deleted, as the interval is always newly set
     std::multimap<QString, Constraint> newConstraintsList = constraintList;
     if (m_commScheme == CommunicationScheme::MINMAXINTERVAL || m_commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
         for (auto it = newConstraintsList.begin(); it != newConstraintsList.end(); ) {
             if (it->first == car) {
                 it = newConstraintsList.erase(it);
             }
             else {
                 ++it;
             }
         }
     }
     for (auto it = constraints.begin(); it != constraints.end(); it++) {
         if (m_commScheme != CommunicationScheme::MINMAXINTERVAL && m_commScheme != CommunicationScheme::MINMAXINTERVALMOVING) {
             removeConstraintWithEqualTimeStamp(newConstraintsList, car, *it);
         }
         newConstraintsList.insert( std::pair<QString, Constraint>(car, *it) );
     }
     //DEBUG
     /*for (auto it = m_constraints.begin(); it != m_constraints.end(); it++) {
         qDebug() << "Car: " << it->first << "Constraint: (" << it->second.getCenterPoint().at(0) << "," << it->second.getCenterPoint().at(1) << ")";
     }*/
     //--DEBUG
     return newConstraintsList;
 }

 /**
  * @brief SimulationThread::getGridWidth returns the amount of gridcells in width depending on cell size
  * @return
  */
 int SimulationThread::getGridWidth() const {
     return interSection->getGridWidth();
 }

 /**
  * @brief SimulationThread::getGridHeight returns the amount of gridcells in height depending on cell size
  * @return
  */
 int SimulationThread::getGridHeight() const {
     return interSection->getGridHeight();
 }

 /**
  * @brief SimulationThread::getOverallConstraintMargin returns the overall margin for one constraint
  * @return
  */
 double SimulationThread::getOverallConstraintMargin() const {
     Constraint constraint ({InterSection::getInstance()->getCellSize() + 0.5,
                 InterSection::getInstance()->getCellSize() + 0.5},
                 m_t0, SystemFunctionUsage::CONTINUOUS, m_N, m_T, this->m_currentGridSize,
                 this->m_radius, m_radius + 0.5);
     return constraint.getOverallMargin();
 }

 /**
  * @brief SimulationThread::getCurrentCellSize returns the current cell size
  * @return the current cell size
  */
 double SimulationThread::getCurrentCellSize() const {
     return m_currentGridSize;
 }

 /**
 * @brief createInterArrivalCars create new cars and place them in the entry points
 * @param cars vector with existing cars
 * @param numberCreateCars number of cars to be created
 */
void SimulationThread::createInterArrivalCars() {
    if (interSection) {
        //get vector for new arriving cars for all entry points of the intersection
        std::vector<unsigned int> newArrivedCars;
        for (unsigned int i = 0; i < interSection->numberEntryPoints(); i++) {
            newArrivedCars.push_back(interSection->getAmountOfCarsForNextTime(i));
        }
        //iterate over the vector of entry points
        for (unsigned int i = 0; i < newArrivedCars.size(); i++) {
            //create now the cars
            for (unsigned int j = 0 ; j < newArrivedCars.at(i); j++) {
                placeCarInStartPosition(i, 0.5);
            }
        }
    }

 }

/**
 * @brief SimulationThread::insertCarsFromWaitingQueue inserts the cars in the vector of active cars (cars), if their time for the reserved intersection cell would be achived next
 * @param waitCars cars which are waiting
 * @param cars current cars in intersection
 * @return full vector with old and new arrived cars in intersection
 */
CarGroupQueue SimulationThread::insertCarsFromWaitingQueue(std::map<std::shared_ptr<Car>, double> &waitCars,
                                                                               const CarGroupQueue& cars) {
    CarGroupQueue oldCars = cars;

    for (auto it = waitCars.begin(); it != waitCars.end();) {
        //in the next time instant the car can be placed
        if (it->second == m_t0) {
            //take here the standard constraints
            it->first->createGlobalConstraints();
            //test, if start position is not colliding with any constraints
            it->first->setCurrentConstraints(m_constraints, m_t0, m_T);
            bool valid = it->first->testValidityConstraints(
                        it->first->getCurrentConstraints(),
                        it->first->getInitialControl(m_t0, m_T));
            //if valid, it can go
            if (valid) {
                oldCars.push_back(it->first);
                //for intersection scenario add directional constraints
                it->first->createDirectionalConstraints(interSection->getWidth(), interSection->getHeight(),
                                                  m_currentGridSize, getGlobalTime(), m_N, m_T, m_radius,
                                                  m_controlBounds.second);

                std::vector<Constraint> constraints = it->first->formulateConstraintsForNextCar(
                            VectorHelper::reshapeXd(it->first->getInitialControl(getGlobalTime(), m_T)),
                            getGlobalTime(), m_T, m_N, false, m_currentGridSize,
                            m_radius, m_commScheme);
                m_constraints = insertFormulatedConstraints(it->first->getName(), m_constraints, constraints);
                it = waitCars.erase(it);
            }
            else {
                it->first->clearAllConstraints();
                it++;
            }
        }
        else {
            it++;
        }
    }
    return oldCars;
}

/**
 * @brief SimulationThread::removeConstraintsOfMovedCars removes constraints from cars, which are not there anymore
 * @param carName car which has been removed
 * @param constraintMap constraint map
 * @return cleared constraint map
 */
std::multimap<QString, Constraint> SimulationThread::removeConstraintsOfMovedCars(const QString& carName, const std::multimap<QString, Constraint> &constraintMap) {
    std::multimap<QString, Constraint> reducedConstraints;
    for (auto itConstraint = constraintMap.begin(); itConstraint != constraintMap.end(); itConstraint++) {
        if (itConstraint->first != carName) {
            reducedConstraints.insert((*itConstraint));
        }
    }
    return reducedConstraints;
}
