#include "constraint.h"
#include "../simulation-core/vectorhelper.h"
#include <QtCore/QDebug>
#include <cmath>
#include <algorithm>

/**
 * @brief Constraint::Constraint push back in integer values from lower bound to upperbound
 * @param type
 * @param lowerBound
 * @param upperBound
 * @param N
 * @param funcType
 */
Constraint::Constraint(const double &lowerBound, const double &upperBound, const size_t& N, const SystemFunctionUsage &funcType) :
    m_funcType(funcType),
    m_centerPoint({0.0,0.0}),
    m_gridSize(0.0),
    m_actualSystemFunc(nullptr),
    m_tConstraint(0.0),
    m_T(0.0),
    m_N(N),
    m_t0(0.0),
    m_maxDynamics(0.0),
    m_dmin(0.0)
{
    if (m_funcType == SystemFunctionUsage::DISCRETE) {
        m_validValues.reserve(upperBound - lowerBound);
        for (unsigned int i = std::ceil(lowerBound); i < std::floor(upperBound); i++) {
            m_validValues.push_back(i);
        }
    }
    else if (m_funcType == SystemFunctionUsage::CONTINUOUS) {
        //
    }
}

/**
 * @brief Constraint::Constraint
 * @param funcType
 */
Constraint::Constraint(const SystemFunctionUsage &funcType) :
    m_funcType(funcType),
    m_centerPoint({0.0,0.0}),
    m_gridSize(0.0),
    m_actualSystemFunc(nullptr),
    m_tConstraint(0.0),
    m_T(0.0),
    m_N(InterSectionParameters::N),
    m_t0(0.0),
    m_maxDynamics(0.0),
    m_dmin(0.0)
{

}

/**
 * @brief Constraint::Constraint
 * @param centerPoint
 * @param t0
 * @param funcType
 * @param N
 * @param T
 * @param gridSize
 * @param dmin
 * @param maxDynamics minimum grid size
 */
Constraint::Constraint(const std::vector<double> &centerPoint, const double& t0, const SystemFunctionUsage &funcType,
                       const size_t& N, const double& T, const double& gridSize, const double &dmin, const double& maxDynamics) :
    m_funcType(funcType),
    m_centerPoint(centerPoint),
    m_gridSize(gridSize),
    m_actualSystemFunc(nullptr),
    m_tConstraint(t0),
    m_T(T),
    m_N(N),
    m_t0(0.0),
    m_maxDynamics(maxDynamics),
    m_dmin(dmin)
{
    m_gridSize += getSafetyMarginForRadius();
}

/**
 * @brief Constraint::Constraint
 * @param constraintVector assumed as std::vector<double> which contains the valid control values
 */
Constraint::Constraint(std::vector<double>& constraintVector)
{
    m_validValues = constraintVector;
}

/**
 * @brief Constraint::Constraint Default-Constructor
 */
Constraint::Constraint() :
m_funcType(SystemFunctionUsage::DISCRETE)
{
    //
}

/**
 * @brief Constraint::~Constraint
 */
Constraint::~Constraint() {
    //
}

/**
 * @brief Constraint::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double Constraint::wrapConstraintObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    return (*reinterpret_cast<Constraint*>(data)) (u, grad, data);
}

/**
 * @brief Constraint::getValidValues
 * @return
 */
std::vector<double> &Constraint::getValidValues() {
    return m_validValues;
}

/**
 * @brief <b>discrete scenario</b>: Constraint::operator () encapsulate for NLOpt looks up if values of x are valid, returns -1 if it is found
 * otherwise 1 if not found and therefore the constraints are violated
 * <b>continuous scenario</b>: constraints are defined in the form \f$(-1)||\cdot||_{\infty}\f$-norm + m\_radius \leq 0\f$
 * @param x vector with control values
 * @param grad
 * @param f_data
 * @return
 */
double Constraint::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    double ret = -1.0;
    if (m_funcType == SystemFunctionUsage::DISCRETE) {
        for (unsigned int i = 0; i < u.size(); i++) {
            //if not found return
            //TODO: hier erst mal ceil genommen, da immer aufgerundet wird
            /*if (std::find(m_validValues.begin(), m_validValues.end(), std::ceil(u.at(i))) == std::end(m_validValues)) {
                ret = 1;
                break;
            }*/
            for (std::vector<double>::const_iterator it = u.begin(); it != u.end(); ++it) {
                if ( std::find(m_validValues.begin(), m_validValues.end(), std::ceil(u.at(i))) == std::end(m_validValues) ) {
                    if (ret < 0.0) {
                       ret = 1;
                    }
                    else {
                       ret += 1.0;
                    }
                }
            }
        }
    }
    else if (m_funcType == SystemFunctionUsage::CONTINUOUS) {
        std::vector<std::vector<double> > x = m_actualSystemFunc->getHolonomicSystemTrajectory(m_actualSystemFunc->getCurrentContinuousState(), VectorHelper::reshapeXd(u), m_t0, m_T, m_N);
        //std::vector<double> x = m_actualSystemFunc->getHolonomicSystem(m_actualSystemFunc->getCurrentContinuousState().back(), VectorHelper::reshape2d(u), m_t0, m_T);
        double maxNorm = 0.0;
        //for (unsigned int i = 0; i < x.size(); i++) {
        //the constraint has to be checked for the current position n=n0 until n=n0+N, therefore
        //we have to calculate timestep of (constraint - currentTime) to get the right step
        //dealing with smaller time step, means, that we have to norm them to ||1|| to get the right position

        double pos = m_tConstraint*1/m_T - m_t0*1/m_T;
        int position = (int)pos;
            Q_ASSERT_X(position >= 0, typeid(this).name(), "position < 0");
        double kMaxNorm = 0.0;
        if (position >= 0 && position < x.size()) {
            kMaxNorm = VectorHelper::getInfinityNorm(VectorHelper::sub(x.at(position), m_centerPoint));
            kMaxNorm = (-1.0)*kMaxNorm + m_gridSize;
        }
        maxNorm = kMaxNorm;
        if (maxNorm > 0.0) {
            //qDebug() << "Constraint at: (" << this->getCenterPoint().at(0) << "," << this->getCenterPoint().at(1) << ") not valid: " << kMaxNorm;
        }
        else {
            //qDebug() << "Constraint at: (" << this->getCenterPoint().at(0) << "," << this->getCenterPoint().at(1) << ") valid: " << kMaxNorm;
        }
        //}
        //derivative here is the absolute value of the maximum of the vector (x_pi-c), therefore -1 or 1
        if (!grad.empty()) {
            int gradPos = 0;
            for (unsigned int i = 0; i < x.size(); i++) {
                std::vector<double> diff = VectorHelper::sub(x.at(i), m_centerPoint);
                std::vector<double> absDeriv = VectorHelper::getDerivOfAbsValue(diff);
                for (unsigned int j = 0; j < absDeriv.size(); j++) {
                    grad[gradPos] = absDeriv.at(j);
                    gradPos++;
                }
            }
        }
        return maxNorm;
    }
    return ret;
}

/**
 * @brief Constraint::setActualSystem sets the actual system for which the constraint should be applied
 * @TODO: globalen Zeitstempel mitgeben
 * @param systemFunc the current system, this should obey this constraint
 * @param curTime current simulation time
 * @param T length
 * @param N horizon
 */
void Constraint::setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& curTime, const double& T, const size_t& N) {
    m_actualSystemFunc = systemFunc;
    m_t0 = curTime;
    m_T = T;
    m_N = N;
}

/**
 * @brief Constraint::getConstraintTime get the timestamp for which this constraint holds
 * @return timestamp as double
 */
double Constraint::getConstraintTime() const {
    return m_tConstraint;
}

/**
 * @brief Constraint::getCenterPoint
 * @return
 */
std::vector<double> Constraint::getCenterPoint() const {
    return m_centerPoint;
}

bool Constraint::operator==(const Constraint& constr) const {
    return (getCenterPoint() == constr.getCenterPoint() && getConstraintTime() == constr.getConstraintTime());
}

double Constraint::getSafetyMarginForRadius() const {
    double ret = 0.0;
    if (m_dmin > m_T * m_maxDynamics) {
        ret += 2.0 * m_dmin;
    }
    else {
        ret += 2.0 * m_T * m_maxDynamics;
    }
    ret += (m_maxDynamics * m_T * std::cos(M_PI / 4) + 0.01) / 2.0;
    //ret += 0.02;
    return ret;
}

/**
 * @brief Constraint::getOverallMargin get radius + safety margin + epsilon
 * @return
 */
double Constraint::getOverallMargin() const {
    return m_gridSize + getSafetyMarginForRadius();
}

/**
 * @brief Constraint::getOverallMargin returns the minimum distance between two positions in a static constraint function
 * @param radius minimum c to be kept
 * @param dmin radius between robot
 * @param maxDynamics
 * @param T
 * @return overall cellsize including safety margin
 */
double Constraint::getOverallMargin(const double& radius, const double& dmin, const double& maxDynamics, const double& T) {
    double ret = radius;
    if (dmin > T * maxDynamics) {
        ret += 2.0 * dmin;
    }
    else {
        ret += 2.0 * T * maxDynamics;
    }
    ret += (maxDynamics * T * std::cos(M_PI / 4) + 0.01) / 2.0;
    //ret += 0.02;
    return ret;
}

double Constraint::getCurrentGridSize() const {
    return m_gridSize;
}
