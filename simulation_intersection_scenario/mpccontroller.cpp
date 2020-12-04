#include "mpccontroller.h"
#include "intersection.h"
#include "pathcontrolmap.h"
#include "constraintfunction.h"

#include "../simulation-core/vectorhelper.h"
#include <QtCore/QDebug>

#include <utility>

double MpcController::m_lowerAcceptenceBound = 0.0;
double MpcController::m_reoptimizeBound = 0.0;
/**
 * @brief MpcController::MpcController
 * @param car
 * @param start
 * @param target
 * @param lambda
 * @param T sampling interval
 */
MpcController::MpcController(const QString& car, const PathItem& start, const PathItem& target, const size_t &N, const double &lambda, const std::pair<double, double>& bounds) :
    m_car(car),
    m_n(N),
    m_target(target),
    m_lambda(lambda),
    m_actualFunctionValue(0.0),
    m_openLoopCosts(0.0),
    m_closedLoopCosts(0.0),
    m_controlLowerBound(bounds.first),
    m_controlUpperBound(bounds.second),
    m_boundInitSteps(true)
{
    m_systemFunc = std::make_shared<SystemFunction>(car, Path(start));
    m_systemFunc->setIntervalControlDynamic({m_controlLowerBound, m_controlUpperBound});
    if (m_lowerAcceptenceBound == 0.0) {
        m_lowerAcceptenceBound = calcLowerAcceptanceBound();
    }
    if (m_reoptimizeBound == 0.0) {
        m_reoptimizeBound = calcReoptimizeBound();
    }
}

/**
 * @brief MpcController::MpcController
 * @param car
 * @param start
 * @param target
 * @param lambda
 */
MpcController::MpcController(const QString &car, const std::vector<double>& start, const std::vector<double>& target, const size_t &N, const double& lambda, const std::pair<double, double> &bounds) :
    m_car(car),
    m_n(N),
    m_target(PathItem()),
    m_lambda(lambda),
    m_actualFunctionValue(0.0),
    m_openLoopCosts(0.0),
    m_closedLoopCosts(0.0),
    m_targetCont(target),
    m_controlLowerBound(bounds.first),
    m_controlUpperBound(bounds.second),
    m_boundInitSteps(true)
{
    m_systemFunc = std::make_shared<SystemFunction>(car, start);
    m_systemFunc->setIntervalControlDynamic({m_controlLowerBound, m_controlUpperBound});
}

/**
 * @brief MpcController::getAlg returns current PathAlgorithm
 * @return
 */
PathAlgorithm MpcController::getAlg() const {
    return PathAlgorithm::MPCCOBYLA;
}

/**
 * @brief MpcController::calculatePath
 * @param grid
 * @param source
 * @param target
 * @return
 */
Path MpcController::calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target) {
    //
}

/**
 * @brief MpcController::calcConstraintsFromPath set the constraints, looking up from which start to which target the car go
 * and set the constraints as the opposite direction.
 * It takes the difference between target and start and disclose the opposite direction as intervals for the control value, which are globally unique
 * for each intersection cell
 *
 */
void MpcController::calcConstraintsFromPath() {
    PathItem diffTarget = m_target;
    diffTarget = diffTarget - m_systemFunc->getStart();
    diffTarget.abs();
    //boundaries for direction
    //horizontal
    if (diffTarget.getY() <= 1 && diffTarget.getX() > 1) {
        //left- to rightside
        if (m_target.getY() <= 1) {
            //lowerBound = 0, 0, 0;
            //upperBound = InterSection::getInstance()->getWidth(), (InterSection::getInstance()->getHeight() - 1) / 2, DBL_MAX;
            //NLOpt: 0 to 4*4 = 16/2-1 = 7
            m_constraint = Constraint(0, InterSection::getInstance()->getWidth() * InterSection::getInstance()->getHeight() / 2 - 1, m_n);
        }
        //right- to leftside
        else if (m_target.getY() > 1) {
            //lowerBound = 0, (InterSection::getInstance()->getHeight()) / 2, 0;
            //upperBound = InterSection::getInstance()->getWidth(), InterSection::getInstance()->getHeight(), DBL_MAX;
            //NLOpt: 4*4=16/2 = 8 to 15 = 4*4-1
            m_constraint = Constraint(InterSection::getInstance()->getWidth() * InterSection::getInstance()->getHeight() / 2,
                                      InterSection::getInstance()->getWidth() * InterSection::getInstance()->getHeight() - 1, m_n);
        }
    }
    //vertical
    else if (diffTarget.getX() <= 1 && diffTarget.getY() > 1) {
        //up- to downside
        if (m_target.getX() <= 1) {
            //lowerBound = 0, 0, 0;
            //upperBound = (InterSection::getInstance()->getWidth() - 1) / 2, InterSection::getInstance()->getHeight(), DBL_MAX;
            //NLOpt
            //0,1, 4, 5, 8, 9, ...
            std::vector<double> controlValues;
            for (unsigned int i = 0; i < InterSection::getInstance()->getWidth() * InterSection::getInstance()->getHeight(); i++) {
                //left side (0, 1, 4, 5)
                if (i % InterSection::getInstance()->getWidth() < InterSection::getInstance()->getWidth() / 2) {
                    controlValues.push_back(i);
                }
            }
            m_constraint = Constraint(controlValues);
        }
        //down- to upside
        else if (m_target.getX() > 1) {
            //lowerBound = InterSection::getInstance()->getWidth() / 2, 0, 0;
            //upperBound = InterSection::getInstance()->getWidth(), InterSection::getInstance()->getHeight(), DBL_MAX;
            //NlOpt

            std::vector<double> controlValues;
            for (unsigned int i = 0; i < InterSection::getInstance()->getWidth() * InterSection::getInstance()->getHeight(); i++) {
                //left side (0, 1, 4, 5)
                if (i % InterSection::getInstance()->getWidth() >= InterSection::getInstance()->getWidth() / 2) {
                    controlValues.push_back(i);
                }
            }
            m_constraint = Constraint(controlValues);
        }
    }
}

/**
 * @brief MpcController::optimize
 * @param N
 * @param start
 * @param control
 * @return
 */
PathItem MpcController::optimize(const PathItem& start) {

    //initial value is start position
    if (m_control.empty()) {
        m_control.reserve(m_n);

    }
    //should be cleared if is unvalid
    if (m_control.isInvalid()) {
        m_control.clear();
    }
    for (unsigned int i = m_control.size(); i <= m_n; i++) {
        m_control.push_back(start);
    }

    //NLOpt-Object with Horizon Length or dimension N
    nlopt::opt optObject(nlopt::LN_COBYLA, m_n);
    PathControlMap controlMap(m_control, PathControlMapOptimizer::NOPT);
    std::vector<double> controlVec = controlMap.getVectorNlOpt();
    std::vector<double> gradient;
    //it is not enough to declare it here as a void*-pointer and then give to the NLOpt-instance,
    //is should be a normal type-pointer and then he can cast it to void* itself and back
    //void* systemFunc = m_systemFunc.get();
    //SystemFunction* sysTest = m_systemFunc.get();
    CostFunction costTest(start, m_target, m_n, m_lambda, m_systemFunc);
    optObject.set_min_objective(CostFunction::wrapCostFunctionObject, &costTest);
    //calculate the bounds for the control value
    calcConstraintsFromPath();
    //the constraintfunction takes now all the constraints
    ConstraintFunction constraintFunction(ConstraintType::UNEQUAL, m_constraint, m_systemFunc);
    optObject.add_inequality_constraint(ConstraintFunction::wrapConstraintFunctionObject, &constraintFunction);

    std::vector<double> lowerBound(m_n);
    std::fill(lowerBound.begin(), lowerBound.end(), 0.0);
    optObject.set_lower_bounds(lowerBound);


    std::vector<double> upperBound(m_n);
    std::shared_ptr<InterSection> interSect = InterSection::getInstance();
    double maxValue = interSect->getHeight() * interSect->getWidth() - 1;
    std::fill(upperBound.begin(), upperBound.end(), maxValue);
    optObject.set_upper_bounds(upperBound);

    optObject.set_xtol_rel(1e-4);
    double minValueFound = 0.0;
    nlopt::result res = optObject.optimize(controlVec, minValueFound);
    //here the actual function value is stored, because we take into account only valid solutions
    //and not solutions, which are discarded
    m_actualFunctionValue = minValueFound;
    m_openLoopCosts = costTest.getCurrentOpenLoopCosts();


    //add control item
    //PathControlMap controlBack(controlVec);
    //Path retControl = controlBack.getPath();
    //prelim path is already added in the cost function
    Path systemPrelim = m_systemFunc->getPrelimPath();
    Path retControl = systemPrelim;
    retControl.prependPathItem(m_systemFunc->getCurrentState());
    if (shouldReoptimize(retControl, m_actualFunctionValue)) {
        //reset the vector to current state
        controlVec = controlMap.getVectorNlOpt();
        minValueFound = reoptimize(optObject, controlVec);
        m_actualFunctionValue = minValueFound;
        systemPrelim = m_systemFunc->getPrelimPath();
    }

    //the cost Function clear the last preliminated reserved path, but after the last optimization step, we have to do it here
    m_systemFunc->clearPrelimPath();
    if (m_systemFunc->getCurrentState().getTime() > 40) {
        systemPrelim = costTest.getBetterNeighbours(controlVec);
    }
    //set time to i+1, because the control is the function to the next state,
    //and the car takes at least t+1 time to get to the next field
    retControl = m_systemFunc->setPossiblePrelimTimeForPath(systemPrelim);
    m_systemFunc->setPrelimPath(retControl);
    //only save the first control value, the others should be thrown away
    m_control.clear();

    m_control.push_back(retControl.front());
    return m_control.back();
}

/**
 * @brief MpcController::optimizeContinous solves the OCP over the implemented continuous system dynamic in the class SystemFunction
 * @param N
 * @param controlVec
 * @return
 */
std::vector<std::vector<double> > MpcController::optimizeContinous(const std::vector<double>& controlVec, const double &t0, const double &T) {

    //nlopt::opt continObject(nlopt::LD_LBFGS, controlVec.size());
    nlopt::opt continObject(nlopt::LN_COBYLA, controlVec.size());
    //continObject.set_lower_bounds(-1.0);
    //continObject.set_upper_bounds(1.0);
    continObject.set_lower_bounds(m_controlLowerBound);
    continObject.set_upper_bounds(m_controlUpperBound);

    //continObject.set_ftol_rel(0.00001);
    continObject.set_ftol_abs(0.1);
    continObject.set_xtol_rel(0.001);
    continObject.set_maxeval(10000);

    /*if (m_constraints.size() > 1) {
        std::cout << "current constraint radius: " << m_constraints.at(0).getCurrentGridSize() << std::endl;
        if (m_constraints.at(0).getCurrentGridSize() >= 2.5) {
            continObject.set_ftol_rel(0.01);
            //continObject.set_ftol_abs(0.001);
            //continObject.set_xtol_rel(0.001);
        }
    }*/

    //if (m_boundInitSteps) {
        //continObject.set_maxeval(100);
        m_boundInitSteps = false;
    //}

    //qDebug() << "Car: " << m_car << "(" << m_systemFunc->getCurrentContinuousState().at(0) << "," << m_systemFunc->getCurrentContinuousState().at(1)
    //         << ")" << "to (" << m_targetCont.at(0) << "," << m_targetCont.at(1) << ")";ö
    //update directional constraints
    if (InterSectionParameters::intersectionalScenario == 1)  {
        updateDirectionalConstraints(getSystemFunction(), t0, T, m_n);
    }
    //insert the constraints from previous vehicles
    for (Constraint& constraint : m_constraints) {
        continObject.add_inequality_constraint(Constraint::wrapConstraintObject, &constraint);
        //DEBUG
        //qDebug() << "Constraint: (" << constraint.getCenterPoint().at(0) << "," << constraint.getCenterPoint().at(1) << ")";
        //--DEBUG
    }
    //insert global constraints
    //first minimum constriants
    for (unsigned int i = 0; i < m_globalConstraintsMin.size(); i++) {
        continObject.add_inequality_constraint(ConstraintMin::wrapConstraintObject, &m_globalConstraintsMin.at(i));
    }
    for (unsigned int i = 0; i < m_globalConstraintsMax.size(); i++) {
        continObject.add_inequality_constraint(ConstraintMax::wrapConstraintObject, &m_globalConstraintsMax.at(i));
    }
    for (unsigned int i = 0; i < m_constraintsDirec.size(); i++) {
        continObject.add_inequality_constraint(ConstraintDirectional::wrapConstraintObject, &m_constraintsDirec.at(i));
    }
    CostFunction costFunction(m_systemFunc->getCurrentContinuousState(), m_targetCont, t0, T, m_n, m_lambda, m_systemFunc);
    continObject.set_min_objective(CostFunction::wrapCostFunctionObject, &costFunction);
    double functionValue = 0.0;
    /*if (VectorHelper::smaller(m_systemFunc->getCurrentContinuousState(), m_targetCont) > 0.0) {
        //continObject.set_initial_step({0.0, m_controlLowerBound});
        continObject.set_initial_step(m_controlLowerBound);
    }*/
    std::vector<double> optVec = controlVec;
    //getting stuck
    if (VectorHelper::norm2(optVec) < 0.5 && t0 > 10.0) {
        for (auto i = 0; i < optVec.size(); ++i) {
                optVec[i] = 0.0;
        }
        for (auto i = 0; i < std::ceil(optVec.size() / 2); ++i) {
            if (VectorHelper::norm2(VectorHelper::sub(m_systemFunc->getCurrentContinuousState(), m_globalConstraintsMin.at(0).getCenterPoint()))
                    > VectorHelper::norm2(VectorHelper::sub(m_systemFunc->getCurrentContinuousState(), m_globalConstraintsMax.at(0).getCenterPoint())) ) {
                std::cout << "adapt reverse pattern: " << m_car.toLatin1().data() << ", t : " << t0 << std::endl;
                if (i % 2 == 0) {
                    optVec[i] = -1.0;
                }
                else {
                    optVec[i] = 0.0;
                }
            }
            else {
                std::cout << "adapt forward pattern: " << m_car.toLatin1().data() << ", t : " << t0 << std::endl;
                if (i % 2 == 0) {
                    optVec[i] = 1.0;
                }
                else {
                    optVec[i] = 0.0;
                }
            }
        }
    }
    nlopt::result ret;
    try{ ret = continObject.optimize(optVec, functionValue);}
    catch(nlopt::roundoff_limited) {
        qDebug() << m_car << ": roundoff_limit";
    }
    testValidityConstraints(m_constraints, optVec);
    testValidityConstraintsMax(m_globalConstraintsMax, optVec);
    testValidityConstraintsMin(m_globalConstraintsMin, optVec);
    m_openLoopCosts = functionValue;
    m_closedLoopCosts = costFunction.getCurrentClosedLoopCosts();
    if (ret == nlopt::FAILURE) {
        qDebug() << "nlopt failed for:" << m_car;
    }
    std::vector<std::vector<double> > shapedOptControl = VectorHelper::reshapeXd(optVec);
    m_prediction = optVec;
    return shapedOptControl;
}

/**
 * @brief MpcController::getInitialControl calculates an initial control for \$f\|x^\ast - x(0)\|/10\f$ if last prediction is empty
 * otherwise takes the last prediction and remove the last values by \$f\#val = \frac{c}{u} - 1\f$
 * @param cellSize
 * @return
 */
std::vector<double> MpcController::getInitialControl(const double& t0, const double& T) {
    if (m_prediction.empty() /*|| getTimeDependantCosts() < (double)m_n + 3.0*/) {
        m_prediction.reserve(m_n * InterSectionParameters::vectorDimension);
        std::vector<double> start = VectorHelper::sub(getTargetContinuous(), m_systemFunc->getCurrentContinuousState());
        double diffStartTarget = VectorHelper::smaller(getTargetContinuous(), m_systemFunc->getCurrentContinuousState());
        //skip the last control, therefore add (0,0), that by reaching the target the initial control is not getting infeasible
        for (unsigned int i = 0; i < m_n * InterSectionParameters::vectorDimension - InterSectionParameters::vectorDimension; i++) {
            m_prediction.emplace_back(0.0);
            /*unsigned int iMod = i % InterSectionParameters::vectorDimension;
            if (start.at(iMod) != 0) {
                m_prediction.emplace_back(start.at(iMod) / (100.0 * std::abs(start.at(iMod))));
            }
            else {
                //big step down to get initial control
                if (diffStartTarget < 0.0 && i < m_n / (m_n / 4)) {
                    m_prediction.emplace_back(m_controlLowerBound);
                }
                else {
                    double secondComp = diffStartTarget / 100.0 * std::abs(diffStartTarget);
                    if (std::abs(secondComp) < m_controlUpperBound) {
                        m_prediction.emplace_back(secondComp);
                    }
                    else {
                        m_prediction.emplace_back(m_controlUpperBound);
                    }
                }
            }*/
        }
        //filling the last step of the control with (0,0), to keep the control feasible by reaching the target
        for (unsigned int i = 0; i < InterSectionParameters::vectorDimension; i++) {
            m_prediction.emplace_back(0.0);
        }
        //overwrite with 0
        /*for (unsigned int i = 0; i < m_prediction.size(); i++) {
            m_prediction[i] = 0.0;
        }*/
    }//--prediction empty
    initializeConstraints(t0, T);
    //shift the control to forward (cut first element, append (0,0))
    //unsigned int removeElements = std::ceil(cellSize / m_controlUpperBound - 1);
    //skip this for the initialization
    if (t0 != 0.0) {
        for (unsigned int i = 0; i < InterSectionParameters::vectorDimension; i++) {
            m_prediction.erase(m_prediction.begin());
        }
    }


    int i = m_n * InterSectionParameters::vectorDimension - 1;
    if (t0 != 0.0) {
        for (; i >= m_n * InterSectionParameters::vectorDimension - InterSectionParameters::vectorDimension; i--) {
            m_prediction.emplace_back(0.0);
        }
    }
    //test on feasibility of initial control
    int lowerBound = i - InterSectionParameters::vectorDimension;
    while ((!testValidityConstraints(m_constraints, m_prediction)
            || !testValidityConstraintsMin(m_globalConstraintsMin, m_prediction)
            || !testValidityConstraintsMax(m_globalConstraintsMax, m_prediction)) && i > -1) {
        for (; i > lowerBound; i--) {
            m_prediction[i] = 0.0;
        }
        lowerBound -= InterSectionParameters::vectorDimension;
    }
    return m_prediction;
}

/**
 * @brief MpcController::getCurrentPrelimSolution
 * @return
 */
PathItem MpcController::getCurrentPrelimSolution() const {
    return m_control.back();
}

/**
 * @brief MpcController::getSystemFunction
 * @return
 */
std::shared_ptr<SystemFunction> MpcController::getSystemFunction() {
    return m_systemFunc;
}

/**
 * @brief MpcController::getSystemFunction
 * @return
 */
std::shared_ptr<SystemFunction> MpcController::getSystemFunction() const {
    return m_systemFunc;
}



const std::vector<double>& MpcController::getFunctionValues() const {
    return m_functionValues;
}

/**
 * @brief MpcController::addActualSolutionToFunctionValues
 */
void MpcController::addActualSolutionToFunctionValues() {
    m_functionValues.push_back(m_actualFunctionValue);
}

/**
 * @brief MpcController::getLambda get the lambda fraction
 * @return
 */
double MpcController::getLambda() const {
    return m_lambda;
}

/**
 * @brief MpcController::getOpenLoopCosts which is calculated by the open loop cost functional
 * @return
 */
double MpcController::getOpenLoopCosts() const {
    return m_openLoopCosts;
}

/**
 * @brief MpcController::getOpenLoopCosts which is calculated by the closed loop cost functional
 * @return
 */
double MpcController::getClosedLoopCosts() const {
    return m_closedLoopCosts;
}

/**
 * @brief MpcController::shouldReoptimize looks up, if the optimized path is violating the constraints
 * or the costs are much higher than the acceptable bound, so retry a reoptimization
 * @param curPath current Path to take
 * @param costs last costs for curPath
 * @return true, if should reoptimize, otherwise false
 */
bool MpcController::shouldReoptimize(const Path &curPath, const double &costs) const {
    unsigned int violated = 0;
    if (!curPath.isNeighboured(&violated) || ( m_functionValues.size() > 0
            && costs - m_functionValues.at(m_functionValues.size() - 1) > MpcController::m_reoptimizeBound ) ) {
        return true;
    }
    return false;
}

/**
 * @brief MpcController::reoptimizes the last step and stops at the m_lowerAcceptenceBound
 * @param optObject optimization object reference
 * @param controlVec vector with optimization values which should be optimized
 * @return
 */
double MpcController::reoptimize(nlopt::opt &optObject, std::vector<double>& controlVec) {
    //low to up, e.g. (3,0 to (3,3)
    //set initial step size for (x,y)->(x,y+height), this is vertical path search
    //for getting faster over the gaps (3-7, 7-11, etc.)
    if (m_systemFunc->getStart().getX() == m_target.getX()
            && m_systemFunc->getStart().getY() != m_target.getY()) {
        std::vector<double> yStep(controlVec);
        for (double& y : yStep) {
            //y = (double)InterSection::getInstance()->getHeight() / 3.0;
            y = (double)InterSection::getInstance()->getHeight() - 0.1;
        }
        optObject.set_initial_step(yStep);
    }

    //right to left, e.g. (4,4) to (0,4)
    else if (m_systemFunc->getStart().getY() == m_target.getY()
             && m_systemFunc->getPath().back().getX() > 0 && m_systemFunc->getPath().back().getX() > m_target.getX()) {
        std::vector<double> yStep(controlVec);
        for (double& y : yStep) {
            //y = (double)InterSection::getInstance()->getHeight() / 3.0;
            y = -0.8;
        }
        optObject.set_initial_step(yStep);
    }
    optObject.set_stopval(m_lowerAcceptenceBound / 2);
    double minValueFound = 0.0;
    optObject.optimize(controlVec, minValueFound);
    return minValueFound;
}

/**
 * @brief MpcController::calcLowerAcceptanceBound calculate the lower acceptance boundary
 * depending on with and height of the grid
 * @return
 * @TODO: get upper bound and validate bounds for intersection grid
 */
double MpcController::calcLowerAcceptanceBound() const {
    return CostFunction::calcCurrentStateCosts(PathItem(0,0,0.0), PathItem(1,1,0.0),
                         PathItem(InterSection::getInstance()->getWidth(), InterSection::getInstance()->getHeight(), 0.0),
                         m_lambda) * 1.5;
}

/**
 * @brief MpcController::calcReoptimizeBound calculate the upper boundary, when the result should be
 * reoptimized
 * @return reoptimization boundary
 */
double MpcController::calcReoptimizeBound() const {
    return CostFunction::calcCurrentStateCosts(PathItem(0,0,0.0), PathItem(1,1,0.0),
                         PathItem(InterSection::getInstance()->getWidth(), InterSection::getInstance()->getHeight(), 0.0),
                         m_lambda) * m_reoptimizationCostFactor;
}

/**
 * @brief MpcController::calcCostsForNeighbours calculates the costs for the neighbours and returns them in a map
 * @return map sorted by carid
 */
std::map<string, double> MpcController::calcCostsForNeighbours() {
    CostFunction costNeighbours(m_control.back(), m_target, m_n, m_lambda, m_systemFunc);
    return costNeighbours.calcCostsForNeighbours();
}

/**
 * @brief MpcController::initializeCosts with stagecosts + weighted criterias, if available
 * @return
 */
double MpcController::initializeCosts() {
    CostFunction startCosts(m_systemFunc->getStart(), m_target, m_n, m_lambda, m_systemFunc);
    return startCosts.getStageCosts(m_systemFunc->getStart(), m_control.back());
}

std::vector<double> MpcController::getTargetContinuous() const {
    return m_targetCont;
}

/**
 * @brief MpcController::setCurrentConstraints clears the current dynamic constraints, not the global ones
 * and inserts them except if they are their own
 * @param constraints from the previous cars
 * @param current start time t0
 * @param current sampling instance T
 */
void MpcController::setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T) {
    /*if (scheme == CommunicationScheme::FULL || scheme == CommunicationScheme::CONTINUOUS
            || scheme == CommunicationScheme::MINMAXINTERVAL || scheme == CommunicationScheme::MINMAXINTERVALMOVING) {
*/
        m_constraints.clear();
        for (auto it = constraints.cbegin(); it != constraints.cend(); it++) {
            if (it->first != m_car) {
                m_constraints.push_back(it->second);
            }
        }
    /*}
    else if (scheme == CommunicationScheme::DIFFERENTIAL) {
        //first, remove old constraints
        auto itConstraint = m_constraints.begin();
        while (itConstraint != m_constraints.end()) {
            if (itConstraint->getConstraintTime() <= t0) {
                itConstraint = m_constraints.erase(itConstraint);
            }
            else {
                itConstraint++;
            }
        }
        //now, insert constraints, which are from the other cars and with updated timestamps
        QList<double> newTimeInstants;
        for (auto itConstraint = constraints.begin(); itConstraint != constraints.end(); itConstraint++) {
            if (itConstraint->first != m_car) {
                if (!newTimeInstants.contains(itConstraint->second.getConstraintTime())) {
                    newTimeInstants.push_back(itConstraint->second.getConstraintTime());
                }
            }
        }
    }*/
    initializeConstraints(t0, T);
}

std::vector<Constraint>& MpcController::getCurrentConstraints() {
    return m_constraints;
}

/**
 * @brief MpcController::createGlobalConstraints
 * @param lb lower bound for creating the minimum constraints in form $\f -x\left(pos\right) + lb \leq 0\f$
 * @param ub uppber bound for creating the maximum constraints in form $\f x\left(pos\right) - ub \leq 0\f$
 */
void MpcController::createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) {
    Q_ASSERT_X(lb.size() == ub.size(), typeid(this).name(), "unequal size");
    for (unsigned int i = 0; i < m_n * InterSectionParameters::vectorDimension; i++) {
        unsigned int iMod = i % InterSectionParameters::vectorDimension;
        ConstraintMin minConstr(lb.at(iMod), i, SystemFunctionUsage::CONTINUOUS);
        m_globalConstraintsMin.push_back(minConstr);
        ConstraintMax maxConstr(ub.at(iMod), i, SystemFunctionUsage::CONTINUOUS);
        m_globalConstraintsMax.push_back(maxConstr);
    }
}

/**
 * @brief MpcController::getCurrentPrediction
 * @return
 */
std::vector<double> MpcController::getCurrentPrediction() const {
    return m_prediction;
}

/**
 * @brief MpcController::testValidityConstraints gives back, if one of the given constraints is invalid by the given controlvector calculating
 * the trajectory
 * @param constraints given constraints to be validated
 * @param controlVector given trajectory control
 */
bool MpcController::testValidityConstraints(std::vector<Constraint>& constraints, const std::vector<double>& controlVector) const {
    bool valid = true;
    for (Constraint& constraint : constraints) {
        std::vector<double> grad;
        double val = constraint.operator ()(controlVector, grad, nullptr);
        if (val > 0.0) {
            valid = false;
            //qDebug() << m_car << "Constraint at: (" << constraint.getCenterPoint().at(0) << "," << constraint.getCenterPoint().at(1) << ") not valid: " << val;
            break;
        }
    }
    return valid;
}

bool MpcController::testValidityConstraintsMin(std::vector<ConstraintMin>& constraints, const std::vector<double>& controlVector) const {
    bool valid = true;
    for (ConstraintMin& constraint : constraints) {
        std::vector<double> grad;
        double val = constraint.operator ()(controlVector, grad, nullptr);
        if (val > 0.0) {
            valid = false;
            qDebug() << m_car << "Constraint at: (" << constraint.getCenterPoint().at(0) << "," << constraint.getCenterPoint().at(1) << ") not valid: " << val;
        }
    }
    return valid;
}

bool MpcController::testValidityConstraintsMax(std::vector<ConstraintMax>& constraints, const std::vector<double>& controlVector) const {
    bool valid = true;
    for (ConstraintMax& constraint : constraints) {
        std::vector<double> grad;
        double val = constraint.operator ()(controlVector, grad, nullptr);
        if (val > 0.0) {
            valid = false;
            qDebug() << m_car << "Constraint at: (" << constraint.getCenterPoint().at(0) << "," << constraint.getCenterPoint().at(1) << ") not valid: " << val;
        }
    }
    return valid;
}

/**
 * @brief MpcController::setControlRange
 * @param lb
 * @param ub
 */
void MpcController::setControlRange(const double& lb, const double& ub) {
    m_controlLowerBound = lb;
    m_controlUpperBound = ub;
}

/**
 * @brief MpcController::initializeConstraints dynamic and also min/max-constraints
 * @param t0
 * @param T
 * @param N
 */
void MpcController::initializeConstraints(const double &t0, const double& T) {
    //insert the constraints from previous vehicles
    for (Constraint& constraint : m_constraints) {
        constraint.setActualSystem(m_systemFunc, t0, T, m_n);
    }
    //insert global constraints
    //first minimum constriants
    for (unsigned int i = 0; i < m_globalConstraintsMin.size(); i++) {
        m_globalConstraintsMin.at(i).setActualSystem(m_systemFunc, t0, T, m_n);
    }
    for (unsigned int i = 0; i < m_globalConstraintsMax.size(); i++) {
        m_globalConstraintsMax.at(i).setActualSystem(m_systemFunc, t0, T, m_n);
    }
}

/**
 * @brief MpcController::clearAllConstraints
 */
void MpcController::clearAllConstraints() {
   m_constraints.clear();
   m_globalConstraintsMax.clear();
   m_globalConstraintsMin.clear();
   m_constraintsDirec.clear();
}

/**
 * @brief MpcController::getControlBounds
 * @return
 */
std::pair<double, double> MpcController::getControlBounds() const {
   return std::make_pair(m_controlLowerBound, m_controlUpperBound);
}

/**
 * @brief MpcController::initializeDirectionalConstraints
 * @param interSectionWidth
 * @param interSectionHeight
 * @param cellSize
 * @param t0
 * @param N
 * @param T
 * @param dmin minmum distance of cars
 * @param maxDynamics maximum speed
 */
void MpcController::initializeDirectionalConstraints(const unsigned int& interSectionWidth, const unsigned int& interSectionHeight,
                                                     const double &cellSize, const double &t0, const int &N, const double &T,
                                                     const double &dmin, const double &maxDynamics) {
    if (m_constraintsDirec.empty()) {
        m_constraintsDirec = ConstraintDirectional::createConstraintsForCar(m_systemFunc->getStartContinuous(), interSectionWidth,
                                                                        interSectionHeight, cellSize, t0, N, T, dmin,
                                                                        maxDynamics);
    }
}

/**
 * @brief MpcController::updateDirectionalConstraints
 * @param sysFunc SytemFunction (Dynamics) to be set
 * @param t0 constraint start time
 * @param T sampling step
 * @param N prediction horizon
 */
void MpcController::updateDirectionalConstraints(const std::shared_ptr<SystemFunction>& sysFunc, const double& t0, const double& T, const int& N) {
    for (ConstraintDirectional& constraint : m_constraintsDirec) {
        constraint.setActualSystem(sysFunc, t0, T, N);
    }
}

/**
 * @brief MpcController::clearPrediction clears the prediction to have a fresh start for the next value
 */
void MpcController::clearPrediction() {
    for (double& val : m_prediction) {
        val = 0.0;
    }
}
