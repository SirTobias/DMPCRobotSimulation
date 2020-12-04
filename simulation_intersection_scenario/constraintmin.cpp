#include "constraintmin.h"
#include "../simulation-core/vectorhelper.h"

#include <QtCore/QDebug>

/**
 * @brief ConstraintMin::ConstraintMin solves $\f x(pos) \geq lb\f$
 * @param lb lower bound
 * @param pos
 * @param funcUsage
 */
ConstraintMin::ConstraintMin(const double lb, const unsigned int pos, const SystemFunctionUsage &funcUsage) :
    Constraint(funcUsage),
    m_funcType(funcUsage),
    m_lb(lb),
    m_pos(pos),
    m_t0(0.0),
    m_T(0.0),
    m_N(0)
{
}

/**
 * @brief ConstraintMin::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double ConstraintMin::wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    return (*reinterpret_cast<ConstraintMin*>(data)) (u, grad, data);
}

/**
 * @brief ConstraintMin::operator () gives as unequal constraint back \f$-x(pos) \leq 0 \f$
 * @param u state vector
 * @param grad gradient to give back
 * @param f_data
 * @return
 */
double ConstraintMin::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    double ret = 0.0;
    if (m_funcType == SystemFunctionUsage::DISCRETE) {
        //
    }
    else if (m_funcType == SystemFunctionUsage::CONTINUOUS) {
        if (m_actualSystemFunc) {
            std::vector<std::vector<double> > x = m_actualSystemFunc->getHolonomicSystemTrajectory(m_actualSystemFunc->getCurrentContinuousState(), VectorHelper::reshapeXd(u), m_t0, m_T, m_N);
            std::vector<double> x1d = VectorHelper::reshapeXdTo1d(x);
            if (!grad.empty()) {
                for (unsigned int i = 0; i < x1d.size(); i++) {
                    if (x1d.at(0)*(-1) < 0) {
                        grad[i] = -1.0;
                    }
                    else {
                        grad[i] = 1.0;
                    }
                }
            }
            double min = (-1)*x1d.at(m_pos) + m_lb /*+ 0.5*/; //TODO: tight constraints
            if (min > 0.0) {
                qDebug() << "ConstraintMin at: (" << m_lb << "," << m_lb << ") not valid: " << min;
            }
            return min;
        }//actualSystemFunc
    }
    return ret;
}

/**
 * @brief ConstraintMin::setActualSystem set current System and the parameters needed for trajectory calculation
 * @param systemFunc
 * @param t0
 * @param T
 * @param N
 */
void ConstraintMin::setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& t0, const double& T, const size_t& N) {
    m_actualSystemFunc = systemFunc;
    m_t0 = t0;
    m_T = T;
    m_N = N;
}

/**
 * @brief getCenterPoint
 * @return
 */
std::vector<double> ConstraintMin::getCenterPoint() const {
    return {m_lb, m_lb};
}
