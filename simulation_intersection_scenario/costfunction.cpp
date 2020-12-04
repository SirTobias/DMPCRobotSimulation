#include "costfunction.h"
#include "pathcontrolmap.h"
#include "costcriteria.h"
#include "intersectionparameters.h"
#include "../simulation-core/vectorhelper.h"

#include <cmath>


/**
 * @brief CostFunction::CostFunction
 * set the target
 */
CostFunction::CostFunction(const PathItem &start, const PathItem &target, const size_t& N, const double &lambda, std::shared_ptr<SystemFunction> systemFunc) :
    m_start(start),
    m_target(target),
    m_targetCont({0.0}),
    m_n(N),
    m_lambda(lambda),
    m_openLoopCosts(0.0),
    m_closedLoopCosts(0.0),
    m_systemFunc(systemFunc),
    m_x0({0.0}),
    m_t0(0.0),
    m_T(0.0)
{
    //u initialize with start value and rest with 0;

}

CostFunction::CostFunction(const std::vector<double> &x0, std::vector<double> &target, const double& t0, const double &T, const size_t &N,
                           const double &lambda, std::shared_ptr<SystemFunction> systemFunc) :
    m_targetCont(target),
    m_n(N),
    m_lambda(lambda),
    m_openLoopCosts(0.0),
    m_closedLoopCosts(0.0),
    m_systemFunc(systemFunc),
    m_x0(x0),
    m_t0(t0),
    m_T(T)
{

}

/**
 * @brief CostFunction::CostFunction
 * @param systemFunc
 */
/*CostFunction::CostFunction(void *systemFunc) {
    if (systemFunc) {
        m_systemFunc = static_cast<SystemFunction*>(systemFunc);
    }
}*/

/**
 * @brief CostFunction::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double CostFunction::wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    //return (operator()(x, grad, data));
    return (*reinterpret_cast<CostFunction*>(data)) (u, grad);
}

/**
 * @brief CostFunction::setTarget
 * @param x
 */
void CostFunction::setTarget(const PathItem &x) {
    m_target = x;
}

PathItem CostFunction::getTarget() const {
    return m_target;
}

/**
 * @brief CostFunction::operator () function for DLIB optimizer
 * @param vec
 * @return
 */
double CostFunction::operator()(const column_vector& vec) const {
    double costs = 0.0;
    std::vector<PathItem> controlPath;
    long size = vec.size() / 3;
    for (unsigned int i = 0; i < size; i++) {
        PathItem controlItem;
        controlItem.setX(vec(0, i));
        controlItem.setY(vec(1, i));
        controlItem.setTime(vec(2, i));
        controlPath.push_back(controlItem);

    }
    Path x;
    //add start value
    x.push_back(m_start);
    //Now optimize J from i=0 to N-1
    for (unsigned int i = 0; i < m_n - 1; i++) {
        x.push_back(m_systemFunc->prelimReserveCell(x.at(i) + controlPath.at(i)));
        costs += getStageCosts(x.at(i), controlPath.at(i));
    }
    //x.push_back(m_systemFunc->prelimReserveCell(uCopy.at(1)));
    //TODO: difference of control to previous
    //costs = getStageCosts(x.at(0), uCopy.at(1) - uCopy.at(0));
    return costs;
}

/**
 * @brief CostFunction::operator () function for NLOPT optimizer
 * calculates back from control path values to path values and takes the difference for the control values
 * @param x are the control values Control (3,1) -> (3,3)
 * @param grad
 * @param data
 * @return
 */
double CostFunction::operator() (const std::vector<double> &u, std::vector<double> &grad, void* data) {
    if (m_systemFunc->getSystemFunctionType() == SystemFunctionUsage::DISCRETE) {
        //for each optmization step there must be a clean prelim path, otherwise the reservation cells
        //get full of every step
        m_systemFunc->clearPrelimPath();
        double costs = 0.0;
        Path controlDiffuse;
        if (m_systemFunc->getCurrentState().getTime() > 30 && m_systemFunc->getCurrentState().getTime() < 45) {
            std::vector<double> uDiffuse;
            for (const double& uValue : u) {
                uDiffuse.push_back(uValue + 1.0);
            }
            PathControlMap controlDiffuseMap(uDiffuse);
            controlDiffuse = controlDiffuseMap.getPath();
        }


        PathControlMap controlMap(u);
        //converts to the absolute coordinates
        Path controlPath = controlMap.getPath();

        if (controlDiffuse.size() >= m_n) {
            controlPath = controlDiffuse;
        }
        //exception (0,0), u has to make round, because otherwise the optimizer has no chance to reach 0,0
        //because standard is ceil(x,y)
        if (m_target.getY() == 0 && m_target.getX() == 0 ) {
            std::vector<double> uRound;
            bool rounded = false;
            for (const double& uValue : u) {
                if (uValue < 1.0) {
                    rounded = true;
                    uRound.push_back(std::floor(uValue));
                }
                else {
                    uRound.push_back(uValue);
                }
            }
            if (rounded) {
                controlMap.setVector(uRound);
                controlPath = controlMap.getPath();
            }
        }
        //make round,
        if ( getCurrentAbsDistance( PathItem(controlPath.front().getX() - 1, controlPath.front().getY(), controlPath.front().getTime() ) ) <  getCurrentAbsDistance( controlPath.front() ) ) {
            std::vector<double> uRound = round(u, ROUND::DOWN);
            controlMap.setVector(uRound);
            controlPath = controlMap.getPath();
        }
        Path xPath;
        //set time to i+1, because the control is the function to the next state,
        //and the car takes at least t+1 time to get to the next field
        controlPath = m_systemFunc->setPossiblePrelimTimeForPath(controlPath);

        //and look, if the first control value is valid
        Path testValidPath;
        testValidPath.push_back(m_systemFunc->getCurrentState());
        for (PathItem& pathItem : controlPath) {
            testValidPath.push_back(pathItem);
        }
        unsigned int violated = 0;
        if (!testValidPath.isNeighboured(&violated)) {
            costs += (double)(invalidPathPenalty * violated);
        }
        //convert here from optimizer values to path values
        for (unsigned int i = 0; i < m_n - 1; i++) {
            //TODO: incorporate the reserved positions, this should be done in prelimReserveCell in the intersection Cell
            xPath.push_back(m_systemFunc->prelimReserveCell(controlPath.at(i)));
            PathItem controlDiff(controlPath.at(i+1));
            costs += getStageCosts(xPath.at(i), controlDiff - controlPath.at(i));
        }
        //getting the cost function more continous for the optimizer
        //a tendency is given for the next better neighbour which is more cost decreasing
        //this is calculated for the whole path and for each cell the better neighbour is calculated
        //the costs are then added as (x - floor(x))*costs(x_betterNeighbour)
        if (m_systemFunc->getCurrentState().getTime() > 40) {
            Path betterNeighbours = getBetterNeighbours(u);
            std::vector<double> betterNeighboursVec = PathControlMap::convertPathToVector(betterNeighbours);
            for (unsigned int i = 0; i < m_n; ++i) {
                double delta = std::abs(u.at(i) - betterNeighboursVec.at(i));
                costs += getCurrentAbsDistance(betterNeighbours.at(i)) * delta * 10;
            }
        }
        m_openLoopCosts = costs;
    }
    else if (m_systemFunc->getSystemFunctionType() == SystemFunctionUsage::CONTINUOUS) {
        std::vector<std::vector<double> > x;
        //reshape the control to [1 2;3 4;];
        std::vector<std::vector<double> > uReshaped = VectorHelper::reshapeXd(u);
        x = m_systemFunc->getHolonomicSystemTrajectory(m_x0, uReshaped, m_t0, m_T, m_n);
        double costs = 0.0;
        for (unsigned int i = 0; i < m_n; i++) {
            costs += getStageCostsQuartic(x.at(i), uReshaped.at(i));
        }
        m_openLoopCosts = costs;
        m_closedLoopCosts = getStageCostsQuartic(x.at(0), uReshaped.at(0));
        //derivative from cost function
        //derived to x
        if (!grad.empty()) {
            //std::vector<double> x1d = VectorHelper::reshape2d1d(x);
            /*for (unsigned int i = 0; i < x1d.size(); i++) {
                unsigned int targetContPos = i % m_targetCont.size();
                grad[i] = getDerivativeCosts(x1d.at(i), m_targetCont.at(targetContPos));
            }*/
            grad = getDerivativeCosts(x, m_targetCont);
        }
    }
    return m_openLoopCosts;
}

/**
 * @brief CostFunction::getDerivativeCosts derives $\f \frac{dl}{dx}l\left(x,u) = \frac{x}{||x_p-x^{\astar}||}\f$
 * which will be
 * @param x
 * @param u
 * @return
 */
std::vector<double> CostFunction::getDerivativeCosts(const std::vector<std::vector<double> > &x, const std::vector<double> &xTarget) const {
    std::vector<double> grad;
    //Q_ASSERT_X(x.size() == xTarget.size(), typeid(this).name(), "unequal size");
    for (unsigned int i = 0; i < x.size(); i++) {
        std::vector<double> curRow = x.at(i);
        double targeti = 0.0;
        targeti = xTarget.at(i % 2);
        for (unsigned int j = 0; j < curRow.size(); j++) {
            grad.push_back(curRow.at(j) / VectorHelper::norm2(VectorHelper::sub(curRow, xTarget)) );
        }
    }
    return grad;
}


/**
 * @brief CostFunction::getStageCosts get the current stage costs
 * costs are defined as
 * \f||x*_s - x_s(n)||^2 + \lambda * u(n)
 * if weighted functions are added, that is is
 * ||x*_s - x_s(n) + WEIGHTED_FUNCTIONS||^2 +
 * @param x
 * @param u
 * @return
 */
double CostFunction::getStageCosts(const PathItem& x, const PathItem& u) const {
    double costs  = 0.0;
    PathItem item;
    item.setX( m_target.getX() - x.getX() );
    item.setY( m_target.getY() - x.getY() );
    //komponentenweises Quadrieren nach euklidischer Norm
    item.setTime( 1 / (std::pow(m_target.getTime(), 2) - std::pow(x.getTime(), 2)) );
    costs = std::sqrt(std::pow(item.getX(), 2) + std::pow(item.getY(), 2) + std::pow(item.getTime(), 2) + addWeightedFunctionsQuadratic());
    costs += m_lambda * std::sqrt(std::pow(u.getX(), 2) + std::pow(u.getY(), 2) + std::pow(u.getTime(), 2) );
    return costs;
}

/**
 * @brief CostFunction::getStageCosts
 * @param x
 * @param u
 * @return
 */
double CostFunction::getStageCosts(const std::vector<double> &x, const std::vector<double> &u) const {
    double costs = 0.0;
    //euclidian norm without time
    costs += VectorHelper::norm2(VectorHelper::sub(x, m_targetCont));
    double normedControl = VectorHelper::norm2(u);
    costs += m_lambda * normedControl;
    return costs;
}

/**
 * @brief CostFunction::getStageCostsQuadratic
 * @param x
 * @param u
 * @return
 */
double CostFunction::getStageCostsQuartic(const std::vector<double> &x, const std::vector<double> &u) const {
    double costs = 0.0;
    //euclidian norm without time
    //l(x,u) = ||(xp(t)-xp)^4 + 20*y-y_p^2 ||_2 + 0.2||u^2||_2
    costs += VectorHelper::norm2({std::pow(x.at(0) - m_targetCont.at(0), 2), 5*(x.at(1) - m_targetCont.at(1))});
    //costs += VectorHelper::norm2(VectorHelper::sub(x, m_targetCont));
    double normedControl = VectorHelper::norm2({std::pow(u.at(0), 2), std::pow(u.at(1), 2)});
    costs += m_lambda * normedControl;
    return costs;
}

/**
 * @brief calcCurrentStateCosts is currently a helper function to calculate the costs outside the optimizer
 * at first this is implemented without external costs
 * @param x
 * @param u
 * @param target
 * @param lambda
 * @return
 */
double CostFunction::calcCurrentStateCosts(const PathItem& x, const PathItem& u, const PathItem& target, const double& lambda) {
    double costs  = 0.0;
    PathItem item;
    item.setX( target.getX() - x.getX() );
    item.setY( target.getY() - x.getY() );
    item.setTime( std::pow(target.getTime(), 2) - std::pow(x.getTime(), 2) );
    costs = std::sqrt(std::pow(item.getX(), 2) + std::pow(item.getY(), 2) + std::pow(item.getTime(), 2) /*+ addWeightedFunctionsQuadratic()*/);
    costs += lambda * std::sqrt(std::pow(u.getX(), 2) + std::pow(u.getY(), 2) + std::pow(u.getTime(), 2));
    return costs;
}

/**
 * @brief CostFunction::getCurrentAbsDistance returns the geometrical distance in euklidian norm
 * @param x current stage
 * @return distance in euklidian norm
 */
double CostFunction::getCurrentAbsDistance(const PathItem& x) const {
    return std::sqrt(std::pow(m_target.getX() - x.getX(), 2)
        + std::pow(m_target.getY() - x.getY(), 2));
}

/**
 * @brief CostFunction::getCurrentCosts returns the last open loop costs
 * @return
 */
double CostFunction::getCurrentOpenLoopCosts() const {
    return m_openLoopCosts;
}

/**
 * @brief CostFunction::getCurrentCosts returns the last closed loop costs
 * @return
 */
double CostFunction::getCurrentClosedLoopCosts() const {
    return m_closedLoopCosts;
}

/**
 * @brief CostFunction::getBetterNeighbour finds for each value in vector path the neighbour
 * with the lower distance to the target
 * @param path
 * @return
 */
Path CostFunction::getBetterNeighbours(const std::vector<double>& path) const {
    std::vector<double> lowerValues;
    std::vector<double> higherValues;
    for (const double& pathValue : path) {
        lowerValues.push_back(std::floor(pathValue));
        higherValues.push_back(std::ceil(pathValue));
    }
    PathControlMap lowerMap(lowerValues);
    Path lowerPath = lowerMap.getPath();
    PathControlMap higherMap(higherValues);
    Path higherPath = higherMap.getPath();
    double lowerValuesCosts = 0.0, higherValuesCosts = 0.0;
    for (const PathItem& pathItem: lowerPath) {
        lowerValuesCosts += getCurrentAbsDistance(pathItem);
    }
    for (const PathItem& pathItem : higherPath) {
        higherValuesCosts += getCurrentAbsDistance(pathItem);
    }
    if (lowerValuesCosts > higherValuesCosts) {
        return higherPath;
    }
    else {
        return lowerPath;
    }
}

/**
 * @brief CostFunction::round rounds the values of the given vector v up or down
 * @param v vector which values to be rounded
 * @param round given ENUM: DOWN or UP
 * @return rounded vector
 */
std::vector<double> CostFunction::round(const std::vector<double>& v, const ROUND& round) const {
    std::vector<double> roundVec;
    roundVec.reserve(v.size());
    if (round == ROUND::DOWN) {
        for (const double& value : v) {
            roundVec.push_back(std::floor(value));
        }
    }
    else if (round == ROUND::UP) {
        for (const double& value : v) {
            roundVec.push_back(std::ceil(value));
        }
    }
    return roundVec;
}

/**
 * @brief CostFunction::calcCostsForNeighbours calculates the current costs for each neighbour and return it as a map
 * @return map with carId as key and cost as value
 */
std::map<std::string, double> CostFunction::calcCostsForNeighbours() {
    std::map<std::string, double> neighbourCostMap;
    for (size_t i = 0; i < m_systemFunc->countCarInfos(); i++) {
        const CarInformation neighbourInfo = m_systemFunc->getCarInfo(i);
        if (neighbourInfo != m_systemFunc->getCarInfoEnd()) {
            neighbourCostMap[neighbourInfo.getCarId()] = calcCurrentWeightedCostsOfGivenCar(neighbourInfo, InterSectionParameters::lambda);
        }
        else {
            std::cout << typeid(this).name() << "end reached";
        }
    }
    return neighbourCostMap;
}

/**
 * @brief CostFunction::calcCurrentWeightedCosts calculates the weighted costs without control
 * for the given car
 * @param car car as CarInformation (mostly neighbour car)
 * @param lambda given lambda
 * @return
 */
double CostFunction::calcCurrentWeightedCostsOfGivenCar(const CarInformation& car, const double& lambda) {
    double costs = calcCurrentStateCosts(car.getLastState(), PathItem(0,0,0), car.getTarget(), lambda);
    CostCriteria costCrit(car);
    costs += costCrit.getCumulatedCosts();
    return costs;
}

/**
 * @brief CostFunction::addWeightedFunctions for direct communication between the cars
 * the accumulated costs over the several criteria are calculated and returned to be added,
 *  if InterSectionParameters::directComm == 1
 * @return accumulated costs as double
 */
double CostFunction::addWeightedFunctionsQuadratic() const {
    //for direct communication implement here the additional weighted functions
    double costs = 0.0;
    CostCriteria costCrit(m_systemFunc);
    costs += costCrit.getCumulatedQuadraticCosts();
    return costs;
}
