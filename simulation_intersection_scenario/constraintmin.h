#ifndef CONSTRAINTMIN_H
#define CONSTRAINTMIN_H

#include "constraint.h"
#include "systemfunction.h"
#include "intersectionparameters.h"

/**
 * @brief The ConstraintMin class defines the constraint \f$x(pos)>=lb\f$
 */
class ConstraintMin : public Constraint
{
public:
    ConstraintMin(const double lb, const unsigned int pos, const SystemFunctionUsage& funcUsage = SystemFunctionUsage::CONTINUOUS);
    static double wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data);
    void setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& t0, const double& T, const size_t& N);
    std::vector<double> getCenterPoint() const;
private:
    ///continuous or discrete system
    SystemFunctionUsage m_funcType;
    double m_lb;
    ///which vector
    unsigned int m_pos;
    ///start time
    double m_t0;
    ///step
    double m_T;
    ///horizon length
    int m_N;
    ///actual system function
    std::shared_ptr<SystemFunction> m_actualSystemFunc;
};

#endif // CONSTRAINTMIN_H
