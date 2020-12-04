#ifndef DstarLite_H
#define DstarLite_H

#include <memory>
#include "intersection.h"
#include "systemfunction.h"
#include "pathcalculation.h"
#include "tuple"
#include <QtCore/QMap>
#include <QtCore/QPointer>
#include <QtCore/QList>


struct DStarParam {
    double rhs;
    double g;
   // double key;
    std::tuple <double , double> key;
};

class DstarLite : public PathCalculation
{
public:
    DstarLite(const QString &car, const PathItem& start, const PathItem& target, const double& T);
    DstarLite(const QString &car, const std::vector<double>& start, const std::vector<double>& target);
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
    virtual void setControlRange(const double& lb, const double& ub);
    double getLambda() const ;
    double getOpenLoopCosts() const;
    double getClosedLoopCosts() const;
    std::vector<double> getInitialControl(const double &t0, const double &T);
    std::vector<double> getTargetContinuous() const;
    PathItem getCurrentPrelimSolution() const;
    std::map<std::string, double> calcCostsForNeighbours();
    std::pair<double, double> getControlBounds() const;
    bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const;
    std::vector<Constraint> &getCurrentConstraints();
    void clearAllConstraints();
private:

   // void findPath(std::shared_ptr<InterSection> &grid, std::shared_ptr<InterSectionCell> &source);
    std::tuple<double, double> getKey(const std::shared_ptr<InterSectionCell> &node);
    double getG(const std::shared_ptr<InterSectionCell> &node);
    double getrhs(const std::shared_ptr<InterSectionCell> &node);
    void setKey(const std::shared_ptr<InterSectionCell> &node , const std::tuple<double, double> &value);
    void setG(const std::shared_ptr<InterSectionCell> &node , const double value);
    void setrhs(const std::shared_ptr<InterSectionCell> &node , const double value);
    Path replan(const shared_ptr<InterSection> &grid, PathItem &source, std::shared_ptr<InterSectionCell> &start);
    //Path replan1(const shared_ptr<InterSection> &grid , PathItem &source, std::shared_ptr<InterSectionCell> &start);
    void expandNode(const std::shared_ptr<InterSection> &grid, std::weak_ptr<InterSectionCell> &currentNode, const std::shared_ptr<InterSectionCell> &start);
    std::shared_ptr<InterSectionCell> gradientForPath(const std::shared_ptr<InterSection> &grid,  const std::shared_ptr<InterSectionCell> &presentNode);
    std::shared_ptr<InterSectionCell> gradientForReplanning(const std::shared_ptr<InterSection> &grid,  const std::shared_ptr<InterSectionCell> &presentNode);
    double estimateCost(const std::shared_ptr<InterSectionCell> &first, const std::shared_ptr<InterSectionCell> &second);
    Path m_initialPath; //first path calculated initially
    Path m_finalPath; // path calculated after changes
    QList<std::shared_ptr<InterSectionCell>> m_openList;

    std::shared_ptr<SystemFunction> m_sysFunc;

    PathItem m_start;
    PathItem m_target;
    QString m_car;
    std::shared_ptr<InterSectionCell> m_startnode;
    std::shared_ptr<InterSectionCell> m_targetnode;
    std::shared_ptr<InterSectionCell> m_source;
    std::shared_ptr<InterSectionCell> m_presentNode;
    int position=0;
    int a=0;
    //double m_rhs;
    //double m_gCosts;
    QMap<std::shared_ptr<InterSectionCell>, DStarParam> m_dStarMap;
   // QMap<DStarParam, std::shared_ptr<InterSectionCell> > sortedOpenList;



};

/*inline bool operator<(const DStarParam &a, const DStarParam &b) {
    if (std::get<0>(a.key) == std::get<0>(b.key)) {
        return std::get<1>(a.key) < std::get<1>(b.key);
    }
    else {
        return std::get<0>(a.key) < std::get<0>(b.key);
    }
}*/

#endif // DstarLite_H
