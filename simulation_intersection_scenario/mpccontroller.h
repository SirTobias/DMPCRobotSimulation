#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H


#include "costfunction.h"
#include "systemfunction.h"
#include "constraintdirectional.h"
#include "constraintmin.h"
#include "constraintmax.h"
#include "constraint.h"

#include "pathcalculation.h"

#include <nlopt.hpp>

#include <memory>
#include <vector>

/**
 * @brief The MPCController class is controlling each car as an instance
 * first it is assumed, that the intersection cells are numbered globally by the control sequence
 * from 0 to intersection.size() - 1
 */
class MpcController : public PathCalculation
{
public:
    MpcController(const QString &car, const PathItem& start, const PathItem& target, const size_t& N, const double& lambda, const std::pair<double, double> &bounds = {-1.0, 1.0});
    MpcController(const QString &car, const std::vector<double>& start, const std::vector<double>& target, const size_t &N, const double& lambda, const std::pair<double, double>& bounds = {-1.0, 1.0});
    void calcConstraintsFromPath();
    PathItem optimize(const PathItem &start);
    void addActualSolutionToFunctionValues();
PathAlgorithm getAlg() const;
    Path calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target);


    PathItem getCurrentPrelimSolution() const;
    std::shared_ptr<SystemFunction> getSystemFunction();
    std::shared_ptr<SystemFunction> getSystemFunction() const;
    const std::vector<double> &getFunctionValues() const;
    double getLambda() const;
    double getOpenLoopCosts() const;
    double getClosedLoopCosts() const;
    bool shouldReoptimize(const Path& curPath, const double& costs) const;
    double reoptimize(nlopt::opt &optObject, std::vector<double>& controlVec);
    std::map<std::string, double> calcCostsForNeighbours();
    ///initial value for accepting an intermediate optimization result if reoptimization is necessary
    static double m_lowerAcceptenceBound;
    ///if costs are too high, reoptimize
    static double m_reoptimizeBound;
    double initializeCosts();
    std::vector<std::vector<double> > optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T);
    std::vector<double> getTargetContinuous() const;
    std::vector<double> getInitialControl(const double &t0, const double &T);
    void initializeConstraints(const double &t0, const double &T);
    void setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T);
    std::vector<Constraint> &getCurrentConstraints();
    void clearAllConstraints();
    void initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                          const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics);
    void updateDirectionalConstraints(const std::shared_ptr<SystemFunction>& sysFunc, const double& t0,
                                                     const double& T, const int& N);
    void createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub);
    std::vector<double> getCurrentPrediction() const;
    void clearPrediction();
    bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const;
    //TODO: Refactor
    bool testValidityConstraintsMin(std::vector<ConstraintMin> &constraints, const std::vector<double>& controlVector) const;
    bool testValidityConstraintsMax(std::vector<ConstraintMax> &constraints, const std::vector<double>& controlVector) const;
    void setControlRange(const double& lb, const double& ub);
    std::pair<double, double> getControlBounds() const;
private:
    ///system dynamics
    std::shared_ptr<SystemFunction> m_systemFunc;
    //std::shared_ptr<SystemFunction> m_costFunc;
    ///ID of the car
    QString m_car;
    ///N steps
    unsigned int m_n;
    ///target of the system
    PathItem m_target;
    ///damping factor for adding the control values to the cost function
    double m_lambda;
    ///control path
    Path m_control;
    ///current constraints for the car
    Constraint m_constraint;
    ///minimized value
    std::vector<double> m_functionValues;
    ///actual function value
    double m_actualFunctionValue;
    ///time dependant open loop costs;
    double m_openLoopCosts;
    ///time dependant closed loop costs;
    double m_closedLoopCosts;
    ///constraint which are coming from the previous optimizing cars
    std::vector<Constraint> m_constraints;

    ///directional constraint, to keep cars in right directional lanes
    std::vector<ConstraintDirectional> m_constraintsDirec;

    ///get the lowerAcceptanceBound, when the optimizer should finished in reoptimization
    double calcLowerAcceptanceBound() const;
    ///get the reoptimization bound, when the costs are too high
    double calcReoptimizeBound() const;
    ///reoptimization factor for how much higher is the cost factor
    static constexpr double m_reoptimizationCostFactor = 4.0;
    ///target in continous form
    std::vector<double> m_targetCont;
    ///global constraints
    std::vector<ConstraintMin> m_globalConstraintsMin;
    std::vector<ConstraintMax> m_globalConstraintsMax;
    ///obtained optimal control
    std::vector<double> m_prediction;
    ///control bounds
    double m_controlLowerBound, m_controlUpperBound;
    ///bound init steps
    bool m_boundInitSteps;
};

#endif // MPCCONTROLLER_H
