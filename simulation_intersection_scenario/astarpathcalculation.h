#ifndef ASTARPATHCALCULATION_H
#define ASTARPATHCALCULATION_H
#include <memory>
#include "systemfunction.h"
#include "pathcalculation.h"
#include <QtCore/QMap>
#include <QtCore/QPointer>
#include <QtCore/QList>


/**
 * @brief The AStarPathCalculation class provides one implementing interface for path calculation
 */
class AStarPathCalculation : public PathCalculation
{
public:

    AStarPathCalculation(const QString &car, const PathItem& start, const PathItem& target, const double& T, const PathAlgorithm& pathCalc);
    AStarPathCalculation(const QString &car, const std::vector<double>& start, const std::vector<double>& target, const double& T, const PathAlgorithm &pathCalc);
    ~AStarPathCalculation();
    PathAlgorithm getAlg() const;
    std::vector<double> getTargetContinuous() const;
    std::shared_ptr<SystemFunction> getSystemFunction() const;
    Path calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target);
    PathItem optimize(const PathItem &start);
    std::vector<std::vector<double> > optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T);
    double initializeCosts();
    void setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T);
    void createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub);
    void initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                          const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics);
    double estimateCost(const std::shared_ptr<InterSectionCell>& source, const std::shared_ptr<InterSectionCell>& target) const;
    std::vector<double> getCurrentPrediction() const;
    void clearPrediction();
    PathItem getCurrentPrelimSolution() const;
    void setControlRange(const double& lb, const double& ub);
    double getLambda() const ;
    double getOpenLoopCosts() const;
    double getClosedLoopCosts() const;
    std::vector<double> getInitialControl(const double &t0, const double &T);
    std::map<std::string, double> calcCostsForNeighbours();
    std::pair<double, double> getControlBounds() const;
    bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const;
    std::vector<Constraint> &getCurrentConstraints();
    void clearAllConstraints();
    unsigned int position = 0;

private:
    ///system dynamics
    std::shared_ptr<SystemFunction> m_systemFunc;
    ///TODO: allow diagonal, first: yes
    //double calculateHCost(PathItem& source);

    void expandNode(const std::weak_ptr<InterSection>& grid, std::weak_ptr<InterSectionCell> &currentNode) ;
    //std::shared_ptr<InterSectionCell> getLowestCostNode(QList<std::shared_ptr<InterSectionCell> > &list, std::shared_ptr<InterSectionCell>& source) const;
    bool containsSuccessor(QList<std::shared_ptr<InterSectionCell> > &list, std::shared_ptr<InterSectionCell>& cell) const;
    QList<std::shared_ptr<InterSectionCell> > m_openList;
    QList<std::shared_ptr<InterSectionCell> > m_closedList;
    std::shared_ptr<InterSectionCell> m_startNode;
    std::shared_ptr<InterSectionCell> m_targetNode;
    Path m_path;
    QString m_car;
    std::vector<double> m_targetCont;
    // double getGcost()const;
    //double getFCost()const;
    //void calculateFCost(PathItem &source)const;
    //void updateGcost();
    PathItem m_target;
    PathItem m_start;
    PathAlgorithm m_astarType;
    //std::vector<double> m_targetCont; // use in constructor to initialize

    ///constraint which are coming from the previous optimizing cars
    std::vector<Constraint> m_constraints;

};

#endif // ASTARPATHCALCULATION_H
