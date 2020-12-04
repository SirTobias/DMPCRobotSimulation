#include "constraintmax.h"
#include "../simulation-core/vectorhelper.h"

#include <QtCore/QDebug>

/**
 * @brief ConstraintMax::ConstraintMax solves $\f x(pos) \geq lb\f$
 * @param lb lower bound
 * @param pos
 * @param funcUsage
 */
ConstraintMax::ConstraintMax(const double ub, const unsigned int pos, const SystemFunctionUsage &funcUsage) :
    m_funcType(funcUsage),
    m_ub(ub),
    m_pos(pos),
    m_t0(0.0),
    m_T(0.0),
    m_N(0),
    Constraint(funcUsage)
{
}

/**
 * @brief ConstraintMax::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double ConstraintMax::wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    return (*reinterpret_cast<ConstraintMax*>(data)) (u, grad, data);
}

/**
 * @brief ConstraintMax::operator () gives as unequal constraint back \f$-x(pos) \leq 0 \f$
 * @param u state vector
 * @param grad gradient to give back
 * @param f_data
 * @return
 */
double ConstraintMax::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    double ret = 0.0;
    if (m_funcType == SystemFunctionUsage::DISCRETE) {
        //
    }
    else if (m_funcType == SystemFunctionUsage::CONTINUOUS) {
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
        double max = x1d.at(m_pos) - m_ub /*+ 0.5*/;
        if (max > 0.0) {
            qDebug() << "ConstraintMin at: (" << m_ub << "," << m_ub << ") not valid: " << max;
        }
        return max;

    }
    return ret;
}

/**
 * @brief ConstraintMax::setActualSystem set current System and the parameters needed for trajectory calculation
 * @param systemFunc
 * @param t0
 * @param T
 * @param N
 */
void ConstraintMax::setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& t0, const double& T, const size_t& N) {
    m_actualSystemFunc = systemFunc;
    m_t0 = t0;
    m_T = T;
    m_N = N;
}

/**
 * @brief ConstraintMax::getCenterPoint
 * @return
 */
std::vector<double> ConstraintMax::getCenterPoint() const {
    return {m_ub, m_ub};
}

