#ifndef CAR_H
#define CAR_H
#include "pathcalculation.h"
#include "costfunction.h"
#include "messages.pb.h"
#include "../simulation-core/vectorhelper.h"
#include "../simulation-core/simulationobject.h"


/**
 * @brief The Car class describes the automobile which should cross the intersection.
 * It contains at first a path.
 */
class Car : public SimulationObject, public std::enable_shared_from_this<Car>
{
    Q_OBJECT
public:
    Car(const QString& name, const PathItem& start, const PathItem &target, const size_t& N, const double &lambda, const PathAlgorithm& pathAlgorithm, const double& T, const std::pair<double, double> &bounds = {-1.0, 1.0});
    Car(const QString& name, const std::vector<double>& start, const std::vector<double> &target, const size_t &N, const double& lambda, const PathAlgorithm& pathAlgorithm, const double& T, const std::pair<double, double> &bounds = {-1.0, 1.0});
    PathItem calcOcpObjective(const PathItem &start);
    std::vector<std::vector<double> > calcOcpObjectiveContinuous(const std::vector<double> &start, const double &t0, const double &T);
    bool reservePrelimSolution();
    bool isCurrentPrelimSolutionValid() const;
    double getCurrentAbsDistance() const;
    double getOpenLoopCosts() const;
    double getClosedLoopCosts() const;
    int getCountNeighbouredCars() const;
    QMultiMap<int, int> getOccupiedCells(const bool &openLoop = true) const;

    bool hasTargetReached() const;
    QString getName() const;
    PathItem getStart() const;
    PathItem getCurrentState() const;
    std::vector<double> getCurrentStateContinuous() const;
    std::vector<double> getInitialControl(const double &t0, const double &T);
    Path getPath();
    const Path &getPath() const;
    PathItem getTarget() const;
    std::vector<double> getTargetContinous() const;
    std::vector<std::shared_ptr<Car> > getNeighbours(const unsigned int &distance = 1);
    void sendCostsToNeighbours(const int& distance = 1);
    void setGlobalLiveTime(const double &timeStep);
    void resetConnectionToNeighbours();
    std::map<std::string, double> calcCostsForNeighbours();
    std::multimap<double, std::string> sortAscendingCostsForNeighbours(const std::map<std::string, double> &neighbourCostMap) const;
    void initializeCosts();
    std::multimap<double, std::string> getAscendingNeighbourCostMap() const;
    void setAscendingNeighbourCostMap(const std::multimap<double, std::string>& map);
    void identifyReservationConflictsWithNeighbours();
    void applyNextState();
    void applyNextState(const std::vector<double> &u, const double &t0, const double &T);
    std::vector<Constraint> formulateConstraintsForNextCar(const std::vector<std::vector<double> > &u, const double &t0, const double &T,
                                                           const size_t &N, const bool& firstCar, const double& radius = 1.1, const double& dmin = 0.5, const CommunicationScheme &commScheme = CommunicationScheme::FULL);
    std::vector<Constraint> calculateMinMaxConstraintForNextCar(const std::vector<std::vector<double> > &u, const double& t0, const double& T, const size_t &N,
                                                                     const bool& firstCar, const double& radius, const double& dmin, const CommunicationScheme &scheme);
    void setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double &t0, const double &T);
    void constructConstraintFromMinMaxConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T, const size_t& N,
                                                  const double &radius, const double &dmin, const CommunicationScheme& scheme);
    void createGlobalConstraints(const std::vector<double>& lb = {0.0, 0.0}, const std::vector<double>& ub = {(double)InterSectionParameters::k, (double)InterSectionParameters::m});
    std::multimap<QString, Constraint> removeOldPredictions(const std::multimap<QString, Constraint>& constraints) const;
    std::vector<double> getCurrentPrediction() const;
    void clearPrediction(std::vector<std::vector<double> > &pred);
    std::vector<std::vector<double> > getPredictedTrajectory(const std::vector<double> &currentState, const std::vector<double>& prediction, const double &t0, const double &T, const size_t &N) const;
    Path getPredictedTrajectoryCells(const std::vector<double> &currentState, const std::vector<double>& prediction, const double &t0, const double &T, const size_t &N, const double &radius) const;
    std::pair<double, double> getDirectionOfTrajectory(const std::vector<double>& currentState, const std::vector<std::vector<double> > &u, const double &t0, const double& T, const size_t& N) const;
    size_t getMinimumTimeStepsForDistance(const std::vector<double>& start, const std::vector<double>& end, const std::vector<double>& uMax, const double &t0, const double &T, const size_t &N) const;
    std::vector<Constraint> differenceConstraints(const std::vector<Constraint>& constraintsFirst, const std::vector<Constraint>& constraintsSecond) const;
    void createDirectionalConstraints(const unsigned int& interSectionWidth, const unsigned int& interSectionHeight, const double &cellSize, const double &t0, const int &N, const double &T,
                                           const double &dmin, const double &maxDynamics);
    unsigned int getCountCommunicatedConstraints() const;
    void addCountCommunicatedConstraints();
    void setControlRange(const double& lb, const double& ub);
    std::vector<Constraint> &getCurrentConstraints();
    void clearAllConstraints();
    bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const;
    bool testActiveConstraints(const std::vector<Constraint> &constraints, const std::vector<double>& prediction, const double& t0, const double& T, const double& N) const;
    std::shared_ptr<PathCalculation> getPathCalculator() const;
    void setConstraintsFromPredecessors(std::multimap<QString, Constraint> &constraints, std::list<std::shared_ptr<Car> > &predecessors, const double &t0, const double &T);
    size_t getNumberOfMovingIntervalConstraints(const size_t& minTimeMove, const std::vector<double> &minPoint, const std::vector<double> &maxPoint, const std::vector<double> &startPoint, const std::vector<double> &endPoint, const double &radius, const double &t0, const double &T, const size_t &N) const;

    size_t getDelta() const;
    size_t getNumberCellsReserved() const;
public slots:
    //CarComm::Cell getMsg(const std::string &msg);
signals:
    void sendMsg(std::shared_ptr<const Car> car, const std::string& msg);
private:
    ///start position
    PathItem m_start;
    ///target position
    PathItem m_target;
    ///contains the path for the intersection (TODO: decide, whether this should be here or leave it in Systemfunction)
    Path m_path;
    ///unique name of the car
    QString m_name;
    ///current preliminary solution for this time is applied
    bool m_currentSolutionReserved;
    ///stores pointers to neighboured cars, is updated in every time step
    std::vector<std::shared_ptr<Car> > m_neighbourCars;
    ///number of neighboured cars
    int m_countNeighbourCars;
    ///calculated costs for every neigboured car
    std::map<std::string, double> m_neighbourCostMap;
    ///sorted costs ascending according to the values
    std::multimap<double, std::string> m_neighbourSortedCosts;
    ///stores the communicated constraints to communicate only the new or changed constraints
    std::vector<Constraint> m_communicatedConstraints;
    ///communicated constraints in one time instant (could be more, if priority has to be negotiated)
    size_t m_countCommunicatedConstraints;
    ///number of different constraints in one time instant, which has to be communicated
    size_t m_differentConstraints;
    ///blocked cell coordinates to communicate to the GUI
    QMultiMap<int, int> m_occupiedCells;
    ///algorithm to calculate trajectory
    std::shared_ptr<PathCalculation> m_pathCalc;
    ///bounded constraints for control
    std::pair<double, double> m_controlBounds;
    ///difference for horizon length and moving effort to get from start to target prediction
    size_t m_delta;
    ///number of constraints / cells reserved
    size_t m_numberCellsReserved;
};

#endif // CAR_H
