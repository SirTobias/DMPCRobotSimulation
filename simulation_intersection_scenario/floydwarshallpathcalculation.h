#ifndef FLOYDWARSHALLPATHCALCULATION_H
#define FLOYDWARSHALLPATHCALCULATION_H

#include "systemfunction.h"
#include "pathcalculation.h"

/**
 * @brief The FloydWarshallPathCalculation class
 */


struct param{
  double g;
std::shared_ptr<InterSectionCell> parent;

};

class FloydWarshallPathCalculation : public PathCalculation
{
public:
    FloydWarshallPathCalculation(const QString &car, const PathItem& start, const PathItem& target, const double& T);
    FloydWarshallPathCalculation(const QString &car, const std::vector<double>& start, const std::vector<double>& target);
    PathAlgorithm getAlg() const;
    std::shared_ptr<SystemFunction> getSystemFunction() const;
    Path calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target);
    PathItem optimize(const PathItem &start);
    std::vector<std::vector<double> > optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T);
    double initializeCosts();
    void setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T);
    void createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub);
    void initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                                  const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics);
    std::vector<double> getCurrentPrediction() const;
    void clearPrediction();
    PathItem getCurrentPrelimSolution() const;
    virtual void setControlRange(const double& lb, const double& ub);
    double getLambda() const ;
    double getOpenLoopCosts() const;
    double getClosedLoopCosts() const;
    std::vector<double> getInitialControl(const double &t0, const double &T);
    std::vector<double> getTargetContinuous() const;
    std::map<std::string, double> calcCostsForNeighbours();
    std::pair<double, double> getControlBounds() const;
    bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const;
    std::vector<Constraint> &getCurrentConstraints();
    void clearAllConstraints();

private:

 double estimateCost(const std::shared_ptr<InterSectionCell> &first, const std::shared_ptr<InterSectionCell> &second);
 void expandNode(const std::shared_ptr<InterSection>& grid, std::weak_ptr<InterSectionCell> &currentNode) ;
 void setG(const std::shared_ptr<InterSectionCell> &a , double value);
 void setParent(const std::shared_ptr<InterSectionCell> &a, const std::shared_ptr<InterSectionCell> &b);
 double getG(const std::shared_ptr<InterSectionCell> &a);
 std::shared_ptr<InterSectionCell> getParent(const std::shared_ptr<InterSectionCell> &a);
QString m_car;
PathItem m_start;
PathItem m_target;
QList<std::shared_ptr<InterSectionCell>> m_openList;
QList<std::shared_ptr<InterSectionCell>> m_closedList;
std::shared_ptr<SystemFunction> m_sysFunc;
std::shared_ptr<InterSectionCell> m_startNode;
std::shared_ptr<InterSectionCell> m_targetNode;
Path m_path;
int position=0;
int a;
QMap<std::shared_ptr<InterSectionCell> , param> m_map;

};

#endif // FLOYDWARSHALLPATHCALCULATION_H
