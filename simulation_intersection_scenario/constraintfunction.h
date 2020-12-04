#ifndef CONSTRAINTFUNCTION_H
#define CONSTRAINTFUNCTION_H

#include "constraint.h"
#include "systemfunction.h"
#include "memory"
enum class ConstraintType : unsigned int __attribute__((visibility("default"))) {
    EQUAL = 1,
    UNEQUAL = 2
};

/**
 * @brief The ConstraintFunction class holds the constraint values named Constraint with their lower and upper
 * bounds and calculate if a control value sequence is valid (neighbouring fields)
 */
class ConstraintFunction
{
public:
    ConstraintFunction(const ConstraintType& type, const Constraint& constraint, std::shared_ptr<SystemFunction> systemFunc);
    void addConstraint(const Constraint& constraint);
    static double wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data);
private:
    ConstraintType m_type;
    Constraint m_constraint;
    std::shared_ptr<SystemFunction> m_systemFunc;
};

#endif // CONSTRAINTFUNCTION_H
