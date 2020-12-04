#ifndef PATHCALCULATION_H
#define PATHCALCULATION_H
#include "intersectioncell.h"
#include "systemfunction.h"
#include "constraint.h"
#include "path.h"
#include <memory>

class InterSection;
/**
 * @brief The PathAlgorithm enum class sets the algorithms which should be taken for path calculation
 */
enum class PathAlgorithm : unsigned int __attribute__((visibility("default"))) {
    ASTARCENTRALIZED = 0,
    ASTARDECENTRALZED = 4,
    DSTAR = 1,
    FLOYDWARSHALL = 2,
    MPCCOBYLA = 3
   };

/**
 * @brief The PathCalculation class holds the different algorithms for the path calculation
 */
class PathCalculation
{
public:
    PathCalculation(const PathAlgorithm &alg);
    PathCalculation();
    virtual ~PathCalculation();
    ///returns chosen algorithm
    virtual PathAlgorithm getAlg() const;
    ///returns target in continuous form
    virtual std::vector<double> getTargetContinuous() const = 0;
    ///calculates path for given discrete source and target instance
    virtual Path calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target) = 0;
    ///returns the underlying system dynamics
    virtual std::shared_ptr<SystemFunction> getSystemFunction() const = 0;
    ///calculate the next step in discrete manner
    virtual PathItem optimize(const PathItem &start) = 0;
    ///calculate the next step in continuous manner
    virtual std::vector<std::vector<double> > optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T) = 0;
    ///calculate initial costs
    virtual double initializeCosts() = 0;
    ///insert the current constraint obtained by other cars
    virtual void setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T) = 0;
    ///create global constraints concening the operational space
    virtual void createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) = 0;
    ///if used the directional intersection scenario, this has to be used
    virtual void initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                          const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics) = 0;
    ///get current continuous prediction
    virtual std::vector<double> getCurrentPrediction() const = 0;
    virtual void clearPrediction() = 0;
    ///get current calculated, but not applied yet solution
    virtual PathItem getCurrentPrelimSolution() const = 0;
    ///set the maximum control values
    virtual void setControlRange(const double& lb, const double& ub) = 0;
    ///returns the impose of the friction concerning cost function
    virtual double getLambda() const = 0;
    ///returns open loop costs (current time step + prediction horizon)
    virtual double getOpenLoopCosts() const = 0;
    ///returns closed loop costs (only current time step)
    virtual double getClosedLoopCosts() const = 0;
    ///returns the initialization for the control vector
    virtual std::vector<double> getInitialControl(const double &t0, const double &T) = 0;
    virtual std::map<std::string, double> calcCostsForNeighbours() = 0;
    virtual std::pair<double, double> getControlBounds() const = 0;
    virtual bool testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const = 0;
    virtual std::vector<Constraint> &getCurrentConstraints() = 0;
    virtual void clearAllConstraints() = 0;
protected:
    PathAlgorithm m_alg;
};

#endif // PATHCALCULATION_H
