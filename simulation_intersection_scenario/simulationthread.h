#ifndef SIMULATIONTHREAD_H
#define SIMULATIONTHREAD_H

#include <QtCore/QObject>
#include <QtCore/QThread>
#include <QtCore/QString>
#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QStringBuilder>

#include "intersection.h"
#include "car.h"
#include "evaluation.h"
#include "globalcarlist.h"
#include "../simulation-core/databasecore.h"
#include "prioritysorter.h"
#include "arrivalcar.h"
#include "enterintersectiondist.h"
#include "cargroupqueue.h"

#include <map>
#include <memory>

/**
 * @brief The SimulationThread class will launch separate thread that runs calculations for the simulation
 */
class SimulationThread : public QThread
{
    Q_OBJECT
public:
    explicit SimulationThread(const int& width = 4, const int& height = 4, const int& maxCars = 20,
                              const size_t& N = InterSectionParameters::N, const double& T = InterSectionParameters::T, const double& lambda = 0.2,
                              const std::pair<double, double>& bounds = {-1.0, 1.0}, const std::pair<double,
                              double>& gridSize = {1.0, 1.0}, QObject *parent = 0, const double& robotDiameter = 0.5,
                              const PriorityCriteria &priority = PriorityCriteria::FIXED, const CommunicationScheme& commScheme = CommunicationScheme::MINMAXINTERVAL, const PathAlgorithm& pathAlgorithm = PathAlgorithm::MPCCOBYLA);
    double getGlobalTime() const;
    void setGlobalTime(const double& t);
    int getGridWidth() const;
    int getGridHeight() const;
    double getOverallConstraintMargin() const;
    double getCurrentCellSize() const;


signals:
    void simFinished();
    void updateCarGUI(const QString& name, const double& x, const double& y);
    void updateCarGUIPrediction(const QString& name, const std::vector<std::vector<double> >& vec);
    void updateCarGUIDynamicReservations(const QString& name, const QMap<int, int>& occupiedCells);
    void updateCarGUIReservationsClear(const QString& name);
    void updateCellGUI(const unsigned int& x, const unsigned int& y, const unsigned int& reservations);
    //void addCarGUI(QString name, unsigned int x, unsigned int y, unsigned int targetx, unsigned int targety);
    void addCarGUI(const QString& name, const double &x, const double &y, const double &targetx, const double &targety);
    void addCellGUI(const unsigned int& x, const unsigned int& y, const unsigned int& reservations);
    void drawGui(const CarGroupQueue& cars, std::shared_ptr<InterSection>& interSection);
    void removeCarfromGUI(const QString& name, const int& location); //car has reach target
    void steps(int);
    void drawCells(const unsigned int& rows, const unsigned int& cols);

    void StartDBThread(QMap<QString, QVariantList> output);

protected:
    void run();

public slots:
    void startSimulation();
    void pause();
    void resume();

private slots:
    void startMultipleSimRuns();

private:
    unsigned int k, m, m_N, maxCars, m_numberOfCars;
    double lambda;
    bool Pause;
    std::shared_ptr<InterSection> interSection;
    //std::vector<std::vector<std::shared_ptr<Car> > > cars;
    CarGroupQueue m_cars;
    std::map<std::shared_ptr<Car>, double> m_waitCars;
    Evaluation eval;
    QMutex mutex;
    QFile debugFile;
    QTextStream outStream;
    QWaitCondition pauseSimulation;
    bool targetReached;
    unsigned int countSteps;

    QMap<QString, QVariantList> m_output;
//    define some variable to store location of the car in real time
    QVariantList d_carName;
    QVariantList d_state;
    QVariantList d_time;
    QVariantList d_x;
    QVariantList d_y;

    ///global optimization parameters
    /// t0 start time
    double m_t0;
    ///sampling parameter
    double m_T;
    ///constraints that are obtained by each car
    std::multimap<QString, Constraint> m_constraints;
    ///control bounds inserted by each car
    std::pair<double, double> m_controlBounds;
    ///bounds for gridsizes (from..to)
    std::pair<double, double> m_gridSize;
    ///current value for grid size
    double m_currentGridSize;
    ///first run of simulation
    bool m_firstSimRun;
    ///communication scheme
    CommunicationScheme m_commScheme;
    ///min. radius between cars
    double m_radius;
    ///priority criteria
    PrioritySorter m_priority;
    ///Path algorithm
    PathAlgorithm m_pathAlgorithm;
    //DataBaseCore* d_db;

    ///distribution parameters
    std::vector<DistParam> m_distParams;

    //simulation methods
    std::map<QString, std::vector<std::vector<double> >> calculateStep(std::map<QString, PathItem> &nextTargets);
    void evaluateStep(std::map<QString, PathItem> &nextTargets, const std::map<QString, std::vector<std::vector<double> >> &continSol = std::map<QString, std::vector<std::vector<double> >>());
    void updateCellReservations();
    void makeCarsAndIntersection();
    void placeCarInStartPosition(const unsigned int &entryPoint, const double& startMargin);
    void createInterArrivalCars();
    CarGroupQueue insertCarsFromWaitingQueue(std::map<std::shared_ptr<Car>, double> &waitCars, const CarGroupQueue &cars);
    std::multimap<QString, Constraint> appendConstraintsFromPosition(const CarGroupQueue &cars);
    std::multimap<QString, Constraint> deleteOldConstraints(const std::multimap<QString, Constraint> &constraintMap) const;
    std::multimap<QString, Constraint> insertFormulatedConstraints(const QString &car, const std::multimap<QString, Constraint>& constraintList, const std::vector<Constraint> &constraints) const;
    bool findEqualConstraint(const std::multimap<QString, Constraint> &constraintList, const Constraint &constraint) const;
    bool removeConstraintWithEqualTimeStamp(std::multimap<QString, Constraint> &constraintList, const QString& car, const Constraint& constraint) const;
    std::multimap<QString, Constraint> removeConstraintsOfMovedCars(const QString &carName, const std::multimap<QString, Constraint> &constraintMap);
};

Q_DECLARE_METATYPE(std::vector<std::vector<double> >)

#endif // SIMULATIONTHREAD_H
