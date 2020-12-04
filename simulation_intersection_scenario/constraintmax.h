#ifndef CONSTRAINTMAX_H
#define CONSTRAINTMAX_H

#include "constraint.h"
#include "systemfunction.h"
#include "intersectionparameters.h"

class ConstraintMax : public Constraint
{
public:
    ConstraintMax(const double ub, const unsigned int pos, const SystemFunctionUsage& funcUsage = SystemFunctionUsage::CONTINUOUS);
    static double wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data);
    void setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& t0, const double& T, const size_t& N);
    std::vector<double> getCenterPoint() const;
private:
    ///continuous or discrete system
    SystemFunctionUsage m_funcType;
    double m_ub;
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

#endif // CONSTRAINTMAX_H
