#ifndef EVALUATION_H
#define EVALUATION_H

#include "cargroupqueue.h"
#include "plot2d.h"
#include "prioritysorter.h"
#include "distparam.h"

enum class CostType {
    OPENLOOP = 0,
    CLOSEDLOOP = 1
};

class Evaluation
{
public:
    Evaluation();
    void setParentWidget(QWidget *parent);
    void calculatePathValues(const unsigned int& amountCars);
    void setPathSizePerCar(const QString& car, const unsigned int& pathSteps);
    void calculateDistanceCostsPerStep(const CarGroupQueue &cars, const unsigned int &step);
    //void setTimeDepCostsForTimeStep(const std::vector<std::shared_ptr<Car> > &cars);

    double getMeanPathLength() const;
    unsigned int getMinPathLength() const;
    unsigned int getMaxPathLength() const;
    const std::map<int, double> &getCulmCostsPerStep() const;
    const std::vector<double>& getMeanFunctionValues() const;
    void saveContAppliedControl(const CarGroupQueue &cars, const std::map<QString, std::vector<std::vector<double> > >& control);
    void saveContCostsPerCar(const CarGroupQueue &cars);
    void saveCurrentCommunicatedConstraints(const CarGroupQueue &cars, unsigned int &step);
    std::map<QString, double> sumCostsOfCarToInfinity(const std::map<QString, std::vector<double> >& costsContinuous) const;
    std::pair<unsigned int, double> communicationEffortCostsPerformance(std::map<unsigned int, unsigned int>& commEffort, std::map<QString, double> culmCosts);
    std::pair<double, unsigned int> getCommConstraintsForWholeSim(const std::map<unsigned int, unsigned int> &commEffort) const;
    std::pair<double, double> getCostsToCellSize(const CostType &costType) const;
    void disableTitle(const bool& disableTitle);
    unsigned int countPredictionDifference(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const;
    double calculatePredictionDifference(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const;
    std::map<QString, double> calculateDiffInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                      const std::map<QString, std::vector<std::vector<double> > >& pred2) const;
    unsigned int calculatePredictionEqualityLength(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const;
    std::map<QString, unsigned int> countDiffInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                      const std::map<QString, std::vector<std::vector<double> > >& pred2) const;
    std::map<QString, unsigned int> calculateEqualityInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                      const std::map<QString, std::vector<std::vector<double> > >& pred2) const;
    void saveCurrentPrediction(const std::map<QString, std::vector<std::vector<double> > >& pred);
    unsigned int countOccupancyDifference(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const;
    std::map<QString, unsigned int> countDiffInOccupancyGrids(const CarGroupQueue &cars,
                                                                 const std::map<QString, QMap<int, int> >& occupiedCells) const;
    std::map<QString, unsigned int> calculateEqualityInOccupancyGrid(const CarGroupQueue &cars,
                                                                     const std::map<QString, QMap<int, int> >& occupiedCells) const;
    std::map<QString, unsigned int> calculateDiffInOccupancyGrids(const CarGroupQueue &cars,
                                                                 const std::map<QString, QMap<int, int> >& occupiedCells) const;
    unsigned int calculateOccupancyDiff(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const;
    unsigned int calculateOccupancyEqual(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const;
    void saveCurrentOccupiedCells(const CarGroupQueue &cars, const bool &openLoop = true);
    void saveMaxPriorityQueueLength(CarGroupQueue& cars, const unsigned int& step);
    void saveNumberOfPriorityQueues(CarGroupQueue& cars, const unsigned int& step);
    void saveCurrentDeltaForCar(CarGroupQueue& cars, const unsigned int& step);
    //void saveOccupiedCells(const std::shared_ptr<InterSection>& intersect, std::vector<std::shared_ptr<Car> > &cars);
    //Clear after each simulation run
    void clearStatisticsAfterOneRun();
    //Clear statistics after one run over all cell sizes
    void clearStatisticsAfterIteratedCellSizes();


    //Plots
    void plotAppliedContControl(const size_t& N = InterSectionParameters::N, const bool &accumulated = false);
    void plotCosts(const size_t &N, const CostType& costType, const bool &accumulated = false, const bool &withCulmCosts = false);
    void plotCommEffortCostsInfinity(const CostType &costType);
    void plotCommEffortToCellSize();
    void plotCostToCellSize(const CostType &costType);
    void plotDifferentialCommunicationAndNormalCommunicationToCellSize(const bool &full = true, const bool &diff = true);
    void plotCulmCostsOverAllCellSizes(const size_t& N, const CostType &costType);
    void plotPredictionDifference(const size_t &N, const bool &culmulative);
    void plotGridPredictionDifference(const size_t& N, const bool &culmulative);
    void plotPredictionDifferenceOverCellSizes(const size_t &N);
    void plotOccupancyGridDifferenceOverCellSizes(const size_t& N);
    void plotOccupancyCellsHeatMap(const unsigned int& time, const unsigned int &numberVehicles,
                                   const unsigned int &numberVehiclesWaiting, const double& cellSize,
                                   const std::vector<DistParam> &distparams);
    void plotMaxPriorityQueueLength(const size_t& numberCars, const size_t& N, const QString& prioritySort);
    void plotNumberOfPriorityQueues(const size_t& numberCars, const size_t& N, const QString &prioritySort);
    void plotDeltaOfCellSize(const size_t &numberCars, const size_t &N, const double& cellSize);
    void plotConsumedCellDifference(const size_t& numberCars, const size_t &N);
    void showPlots();
    void exportPlots(const QString &type);

    //helper Functions
    std::pair<double, double> getMinAndMax(const QMultiMap<double, double>& map) const;

    //get-set
    std::map<QString, std::vector<double> > getCostsContinuous(const CostType &costType) const;
    void setCostsContinuous(std::map<QString, std::vector<double> >& costsContinuous, const CostType &costType);
    void setCostsContinuousInfinity(const std::map<QString, double>& costsInf, const CostType &costType);
    void addCommEffortClosedLoopPerformance(const std::pair<unsigned int, double>& pair);
    void addCommEffortOpenLoopPerformance(const std::pair<unsigned int, double>& pair);
    std::map<unsigned int, unsigned int> getCommConstraintsPerStep() const;
    std::map<QString, double> getCostsContinuousInfinity(const CostType &costType) const;
    void setCellSize(const double& cellSize);
    void setCommEffortWholeSimulationSum(const std::pair<double, unsigned int> &pair);
    void setDiffCommEffortWholeSimulationSum(const std::pair<double, unsigned int> &pair);
    void setGridSizeToCosts(const std::pair<double, double>& pair, const CostType &costType);
    const std::map<QString, std::vector<std::vector<double> > >& getPrediction() const;
    std::map<QString, std::vector<std::vector<double> > > determineCurrentPredictionStates(const CarGroupQueue &cars, const double &t0, const double &T, const int &N) const;
    void addCountDiffPredictions(const std::map<QString, unsigned int> &pred);
    void addDiffPredictions(const std::map<QString, double> &pred);
    const std::map<QString, std::vector<double> >& getDiffPredictions() const;
    void saveCurrentDiffPredictionsforCellSize(const double& cellSize, const std::map<QString, std::vector<double> >& diffPredictions);
    const std::map<QString, QMap<int, int> >& getOccupancyGrid() const;
    void addCountDiffOccupancyGrid(const std::map<QString, unsigned int> &diffOccupancyGrid);
    void addDiffOccupancyGrid(const std::map<QString, unsigned int> &diffOccupancyGrid);
    const std::map<QString, std::vector<unsigned int> >& getDiffOccupancyGrids() const;
    void saveCurrentDiffOccupancyGridsForCellSize(const double& cellSize, const std::map<QString, std::vector<unsigned int> >& diffOccupancyGrids);
    void saveCurrentCellReservations(CarGroupQueue& cars, const CommunicationScheme &intervalType, const double cellsize, const unsigned int &step);
    void setPriorityCriteria(const PriorityCriteria& prio);
    std::map<QString, Plot2d*> plots() const;
    //serialisation
    friend QDataStream& operator<<(QDataStream& os, Evaluation& eval);
    void exportPlotToText(const QString& fileName, const Plot2d* plot);
private:
    unsigned int getNextValidColor(const unsigned int& index);
    static QString getInlineSuperSubscriptStyle();


    ///pathsteps of all cars
    unsigned int m_pathSteps;
    ///path step per each car (cumulated path length for each car, after reaching target)
    std::map<QString, int> m_pathStepPerCar;
    ///costs per step (key: actual step, value: cumulated costs for each car)
    std::map<int, double> m_culmDistanceCostsPerStep;
    ///alpha value of each car for each applied step (valid solution)
    std::map<QString, std::vector<double> > m_alphaValues;
    ///alpha values for each step without minimum of prior time step
    std::map<QString, std::vector<double> > m_alphaNValues;
    /*///costs per step per car for each step via the open loop cost functional (time dependent function)
    std::map<QString, std::vector<double> > m_openLoopCostsPerStep;
    ///costs per step per car for each step via the closed loop cost functional (time dependent function)
    std::map<QString, std::vector<double> > m_closedLoopCostsPerStep;*/

    ///mean function values over all cars in one time step
    std::vector<double> m_meanAlphaValues;
    ///alpha mean values for each time step without minimum of prior time step over all cars
    /// this means that \sum_{k=0}^{S}(V_n - V_{n-1})/l_{n-1}
    std::vector<double> m_alphaNMeanValues;
    ///this vector contains the splitted values, that means, that first the sum is taken
    /// and then alpha is taken over the splitted sums of sum over V_{n-1}, sum over V_n and sum overl_{n-1}
    /// to get \alpha_n = \frac{sum_{k=0}^{S}V_{n} - sum_{k=0}^{S}V_{n-1}}{sum_{k=0}^{S}l_{n-1}
    std::vector<double> m_alphaNSplittedValues;
    ///saves u(0) for each car in each timestamp
    std::map<QString, std::vector<std::vector<double> > > m_controlContinuous;
    ///saves closed-loop costs for each car in each timestamp
    std::map<QString, std::vector<double> > m_closedLoopCosts;
    ///saves open-loop costs for each car in each timestamp
    std::map<QString, std::vector<double> > m_openLoopCosts;
    ///closed-loop performance for each car
    std::map<QString, double> m_closedLoopCostsContinuousInfinity;
    ///open-loop performance for each car
    std::map<QString, double> m_openLoopCostsContinuousInfinity;
    ///set the relation between communication effort and the resulting closed-loop-performance
    std::multimap<unsigned int, double> m_commEffortClosedLoopPerformance;
    ///set the relation between communication effort and the resulting open-loop-performance
    std::multimap<unsigned int, double> m_commEffortOpenLoopPerformance;
    ///for different cellsizes the accumulated open-loop costs over each timestep
    std::map<double, std::map<double, double> > m_culmOpenLoopCostsOverTimestepsOverCellsize;
    ///for different cellsizes the accumulated closed-loop costs over each timestep
    std::map<double, std::map<double, double> > m_culmClosedLoopCostsOverTimestepsOverCellsize;
    ///mean path length
    double m_meanPathLength;
    ///minimal path length
    unsigned int m_minPathLength;
    ///maximal path length
    unsigned int m_maxPathLength;
    ///plots
    std::map<QString, Plot2d*> m_plots;
    ///choose colour
    unsigned int m_colorIndex;
    QList<QColor> m_colors;
    ///communication effort of constraints in each time step
    std::map<unsigned int, unsigned int> m_commConstraintsPerStep;
    ///current cellsize of the intersection grid
    double m_cellSize;
    ///commeffort for whole simuation run
    unsigned int m_commEffortWholeSimulation;
    ///map for which cellsize which commeffort
    std::map<double, unsigned int> m_gridSizeCommEffort;
    ///cellsize / closed loop costs
    std::map<double, double> m_gridSizeClosedLoopCostsMap;
    ///cellsize / closed loop costs
    std::map<double, double> m_gridSizeOpenLoopCostsMap;
    ///cellsize / differential communication effort
    std::map<double, double> m_gridSizeDifferentialCommEffort;
    ///for a certain time step for each car the current prediction
    std::map<QString, std::vector<std::vector<double> > > m_predictions;
    ///difference in continuous predictions \f$ z_p(\cdot; z_p^0) := \left(0, x_p(0), y_p(0),\cdots,N, x_p(N),y_p(N)\right)\f$
    std::map<QString, std::vector<double> > m_diffPredictions;
    ///count difference of continuous predictions
    std::map<QString, std::vector<unsigned int> > m_countDiffPredictions;
    ///count the maximum equality for each car comparing the current and last time prediction
    std::map<QString, std::vector<unsigned int> > m_equalPredictions;
    ///current occupancy grid for each car in the current time instant
    std::map<QString, QMap<int, int> > m_occupancyGrid;
    ///difference in occupancy grid \f$I_p(n) = \left(n+k, a_p^u\left(k;z_p^0\right), \cdots, b_p^u\left(k;z_p^0\right) \right)_{k \in [0:N] }\f$
    std::map<QString, std::vector<unsigned int> > m_diffOccupancyGrid;
    ///count the difference in the predictions of the occupancy grid
    std::map<QString, std::vector<unsigned int> > m_countDiffOccupancyGrid;
    ///measures the maximum equality for each car comparing current and last time prediction
    std::map<QString, std::vector<unsigned int> > m_equalOccupancyGrid;
    ///saves all the differencePredictions for each car for the current cell size
    std::map<double, std::map<QString, std::vector<double> > > m_diffPredictionsOverCellSize;
    ///saves all the occupancy grid predictions for each car for the current cell size
    std::map<double, std::map<QString, std::vector<unsigned int> > > m_diffOccupancyGridsOverCellSize;
    ///saves the number of occupance for each cell over all time steps
    std::vector<std::vector<int>> m_occupiedCells;
    ///saves the maximum priority queue length for each time instant for each cell size
    std::map<double ,std::map<unsigned int, unsigned int> > m_maxPriorityQueueLength;
    ///saves the number of priority queues for each time instant for each cell size
    std::map<double, std::map<unsigned int, size_t> > m_numberPriorityQueues;
    ///save the delta values (differences between min time needed and horizon for dynamics) for each car over time n
    std::map<QString, std::vector<size_t> > m_deltaOverTime;
    ///saves the consumption of space between interval fixed and interval moving principle
    /// for both methods, distinguished by QString, for each cell size (double) the consumed space is measured (int) over time
    std::map<CommunicationScheme, std::map<double, std::vector<int> > > m_intervalTypeConsumedCellSizes;
    ///chosen priority criteria
    PriorityCriteria m_prio;
    ///title should be skipped
    bool m_skipTitle;
};

#endif // EVALUATION_H
