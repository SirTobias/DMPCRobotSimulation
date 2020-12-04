#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "intersectionparameters.h"
#include "systemfunction.h"
#include <vector>

/**
 * @brief <b>Discrete Scenario</b>: The Constraint class follows the form of nonlinear constraints.
 * The class holds the valid values that are possible for the control functions
 * The control values are mapped as integer values from the grid coordinates of the
 * PathControlMap<br />
 * <b>Continuous Scenario</b>: \f$||\cdot||_{\infty}\f$-norm for the squares, so each constraints
 * should represent a square and the constraints should be given to the following cars, so the constraints have the form
 * \f$(-1)||x_p - \begin{bmatrix}k^C \\ m^C\end{bmatrix}||_{\infty} + radius
 */
class Constraint
{
public:
    Constraint(const SystemFunctionUsage& funcType);
    Constraint();
    Constraint(const double& lowerBound, const double& upperBound, const size_t& N, const SystemFunctionUsage& funcType = SystemFunctionUsage::DISCRETE);
    Constraint(const std::vector<double> &centerPoint, const double& t0, const SystemFunctionUsage &funcType, const size_t& N = InterSectionParameters::N,
               const double &T = InterSectionParameters::T, const double &gridSize = 1.1, const double &dmin = 0.5, const double &maxDynamics = 1.0);
    Constraint(std::vector<double>& constraintVector);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data);
    static double wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    std::vector<double>& getValidValues();
    void setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& curTime, const double &T, const size_t& N);
    double getConstraintTime() const;
    std::vector<double> getCenterPoint() const;
    double getSafetyMarginForRadius() const;
    double getCurrentGridSize() const;
    double getOverallMargin() const;
    static double getOverallMargin(const double& radius, const double& dmin, const double& maxDynamics, const double& T);
    bool operator==(const Constraint& constr) const;
    virtual ~Constraint();
private:
    std::vector<double> m_centerPoint;
protected:
    ///valid time for the constraint
    double m_tConstraint;
    ///step
    double m_T;
    ///actual system which has to obey the states
    std::shared_ptr<SystemFunction> m_actualSystemFunc;
    ///horizon length
    size_t m_N;
    ///current simulation time
    double m_t0;
private:
    ///continuous or discrete system
    SystemFunctionUsage m_funcType;
    std::vector<double> m_validValues;
    ///current gridSize1
    double m_gridSize;
    ///upper boundary for dynamics (minimum cell size)
    double m_maxDynamics;
    ///minimum distance between two cars
    double m_dmin;
};

#endif // CONSTRAINT_H
