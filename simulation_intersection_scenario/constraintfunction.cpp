#include "constraintfunction.h"
#include "pathcontrolmap.h"
#include "intersection.h"

/**
 * @brief ConstraintFunction::ConstraintFunction
 * @param type
 */
ConstraintFunction::ConstraintFunction(const ConstraintType &type, const Constraint &constraint, std::shared_ptr<SystemFunction> systemFunc) :
    m_type(type),
    m_constraint(constraint),
    m_systemFunc(systemFunc)
{
}


/**
 * @brief ConstraintFunction::addConstraint adds a single constraint
 * @param constraint
 */
void ConstraintFunction::addConstraint(const Constraint& constraint) {
    m_constraint = constraint;
}

/**
 * @brief ConstraintFunction::wrapConstraintFunctionObject
 * @param u
 * @param grad
 * @param data
 * @return
 */
double ConstraintFunction::wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
   return (*reinterpret_cast<ConstraintFunction*>(data)) (u, grad, data);
}

/**
 * @brief ConstraintFunction::operator () test first, if the control values are valid within the member object Constraint which holds the valid values
 * second, if the first condition holds, it tests, if the control sequence is ascending by u+1, so the intersection cells are neighboured
 * and also fits to the current system state and follows a valid path
 * @param x
 * @param grad
 * @param f_data
 * @return
 */
double ConstraintFunction::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    //first test if the control values are within the bound of the constraints
    double ret = m_constraint.operator ()(u, grad, f_data);
    //values are already invalid
    if (ret > 0.0) {
        return ret += 10.0;
    }
    //values have to be neighboured
    else {
        PathControlMap convertMap(u);
        Path controlPath = convertMap.getPath();
        //prepend the current systemState
        controlPath.insert(controlPath.begin(), m_systemFunc->getCurrentState());
        unsigned int violated = 0;
        if (!controlPath.isNeighboured(&violated)) {
            ret = (double)violated;
        }
    }
    return ret;
}
