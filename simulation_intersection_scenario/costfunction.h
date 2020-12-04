#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "path.h"
#include "pathitem.h"
#include "systemfunction.h"
#include "dlib/optimization.h"

typedef dlib::matrix<double,0,1> column_vector;

enum class ROUND {
    DOWN = 0,
    UP = 1
};

/**
 * @brief The CostFunction class implements the current cost function regarding the system function.
 * It is compatible via a wrapper (wrapCostFunctionObject and operator () ) to be included in NLOpt.
 * operator() will return the costs over the horizon n while getStageCosts returns the current distance costs
 */
class CostFunction
{
public:
    CostFunction(const PathItem& start, const PathItem& target, const size_t& N, const double& lambda, std::shared_ptr<SystemFunction> systemFunc);
    CostFunction(const std::vector<double> &x0, std::vector<double> &target, const double& t0, const double &T, const size_t &N, const double &lambda, std::shared_ptr<SystemFunction> systemFunc);
    void setTarget(const PathItem& x);
    double operator()(const column_vector& vec) const;
    void setLambda(const double& lambda);
    double getStageCosts(const PathItem& x, const PathItem& u) const;
    double getStageCosts(const std::vector<double> &x, const std::vector<double> &u) const;
    double getStageCostsQuartic(const std::vector<double> &x, const std::vector<double> &u) const;
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    std::vector<double> getDerivativeCosts(const std::vector<std::vector<double> > &x, const std::vector<double> &xTarget) const;
    PathItem getTarget() const;
    static double wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    static double calcCurrentStateCosts(const PathItem& x, const PathItem& u, const PathItem& target, const double& lambda);
    double calcCurrentWeightedCostsOfGivenCar(const CarInformation& car, const double &lambda);
    double getCurrentOpenLoopCosts() const;
    double getCurrentClosedLoopCosts() const;
    double getCurrentAbsDistance(const PathItem& x) const;
    Path getBetterNeighbours(const std::vector<double>& path) const;
    std::vector<double> round(const std::vector<double>& v, const ROUND& round) const;
    ///cost map of neighboured cars
    std::map<std::string, double> calcCostsForNeighbours();
    double addWeightedFunctionsQuadratic() const;
private:
    ///current position
    PathItem m_start;
    ///target
    PathItem m_target;
    ///continuous target
    std::vector<double> m_targetCont;
    ///horizon N
    unsigned int m_n;
    ///lambda for damping the control impact
    double m_lambda;
    /// current open loop costs (summed up over horizon N)
    double m_openLoopCosts;
    /// current closed loop costs
    double m_closedLoopCosts;
    ///state trajectory
    Path x;
    ///calculated control
    Path u;
    ///system function for the model
    std::shared_ptr<SystemFunction> m_systemFunc;
    ///invalid path occurs if cells are not neighboured
    static constexpr int invalidPathPenalty = 100;
    ///start value
    std::vector<double> m_x0;
    ///start time
    double m_t0;
    ///sampling step
    double m_T;

};



#endif // COSTFUNCTION_H
