#include "astarpathcalculation.h"
#include "intersection.h"

/**PathCalculation
 * @brief AStarPathCalculation::AStarPathCalculation
 * @param car
 * @param start
 * @param target
 * @param T sampling interval
 * @param pathCalc astar variant: PathCalculation::ASTARCENTRALIZED or PathCalculation::ASTARDECENTRALIZED
 */
AStarPathCalculation::AStarPathCalculation(const QString &car, const PathItem &start, const PathItem &target, const double &T, const PathAlgorithm &pathCalc) :
    m_car(car),
    m_target(target),
    m_start(start),
    m_astarType(pathCalc)
{
    //qDebug() << "First Constructor Called: ";
    std::cout<<" First constructor called hogya he"<<std::endl;
    //std::cout<<" Car Name: " << car <<std::endl;
    std::cout<<" Path Item Values: " << start.getX()<<" and "<<start.getY() <<std::endl;
    m_systemFunc = std::make_shared<SystemFunction>(m_car,Path(start));
}

AStarPathCalculation::AStarPathCalculation(const QString &car, const std::vector<double>& start, const std::vector<double>& target, const double &T, const PathAlgorithm &pathCalc):
    m_targetCont(target),
    m_astarType(pathCalc)
{
    // qDebug() << "Second Constructor Called: ";
    std::cout<<"Second constructor Called: "<< std::endl;
}

/**
 * @brief AStarPathCalculation::~AStarPathCalculation
 */
AStarPathCalculation::~AStarPathCalculation() {

}

/**
 * @brief AStarPathCalculation::getAlg
 * @return
 */
PathAlgorithm AStarPathCalculation::getAlg() const {
    return PathAlgorithm::ASTARCENTRALIZED;
}

/**
 * @brief getSystemFunction
 * @return
 */
std::shared_ptr<SystemFunction> AStarPathCalculation::getSystemFunction() const {
    return m_systemFunc;

}

/**
 * @brief AStarPathCalculation::calculateHCost calculates the euclidian distance between given source and target
 * @param source start node, from where the route begins
 * @param target end node, whre the route should end
 * @return
 */
double AStarPathCalculation::estimateCost(const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target) const {
    std::shared_ptr<InterSectionCell> targetP (target);
    std::shared_ptr<InterSectionCell> sourceP (source);
    return std::sqrt(std::pow(targetP->getX() - sourceP->getX(), 2) + std::pow(targetP->getY() - sourceP->getY(), 2));

   // return 0;
}

/*double AStarPathCalculation::estimateCost(const std::weak_ptr<InterSectionCell>& source, const std::weak_ptr<InterSectionCell>& target) const {
    if (!source.expired() && !target.expired()) {
        std::shared_ptr<InterSectionCell> targetP (target);
       std::shared_ptr<InterSectionCell> sourceP (source);
       double nextFreeTime=0;
       if(targetP.get()){
       nextFreeTime= targetP->getNextFreeTime(get);
       }
        return std::sqrt(std::pow(targetP->getX() - sourceP->getX(), 2) + std::pow(targetP->getY() - sourceP->getY(), 2) + std::pow(nextFreeTime - sourceP->getNextFreeTime(),2))
    }
    return 0;
}*/

/*double AStarPathCalculation::calculateHCost(PathItem& source) {

    return std::sqrt(std::pow(m_target.getX() - source.getX(), 2) + std::pow(m_target.getY() - source.getY(), 2));
}*/
/**
 * @brief AStarPathCalculation::calculatePath
 * @param source
 * @param target
 * @return
 */
// for centralized case
Path AStarPathCalculation::calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target){
    //Q_ASSERT_X(grid.expired() || source.expired() || target.expired(), typeid(this).name(), "expired grid or nodes");

    if (m_astarType == PathAlgorithm::ASTARCENTRALIZED){

        m_startNode = source;
        m_targetNode = target;

        //remove these 3
        //std::shared_ptr<InterSectionCell> sourceP(source);
        //std::shared_ptr<InterSectionCell> targetP(target);
        //std::shared_ptr<InterSection> gridP(grid);

       // std::shared_ptr<InterSectionCell> node= make_shared<InterSectionCell>(1,2);
        //QList<weak_ptr<InterSectionCell>> adu = gridP->getNeighbours(node,1);

     std::shared_ptr<InterSectionCell> startPoint = grid->getInterSectionCell(m_start.getX(),m_start.getY());



        //initializecost();
        source -> setTentativeGCost(0.0);
        source -> setFCost(estimateCost(source,target));// put weak pointers here


        m_openList.append(source);
        //PathItem temp= PathItem(sourceP->getX(),sourceP->getY(),static_cast<double>(m_systemFunc->getGlobalLiveTime()));// do i need conversion
        // for global time ?
        Path m_pathint;
        double globalTime = m_systemFunc->getGlobalTime();


        while (!m_openList.isEmpty()){


            //std::shared_ptr<InterSectionCell> currentNode = getLowestCostNode(m_openList, sourceP);
            //qSort(m_openList.begin(), m_openList.end(),  [](const InterSectionCell* a, const InterSectionCell* b) -> bool { return a->getFCost() < b->getFCost(); });
            //qSort(m_openList.begin(), m_openList.end(), compare);
            qSort(m_openList.begin(), m_openList.end(),[](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return a->getFCost() < b->getFCost(); });
            std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
            //int64_t ad= m_systemFunc->getGlobalLiveTime();
            //double time= static_cast<double>(ad);
            //double nextFreeTimeLocal = currentNode->reserveNextFreeTimeForCar(m_car,time);

            if (currentNode != startPoint){  // i dont want start point to be in parth

                globalTime+= InterSectionParameters::T ;
        //double nextFreeTimeLocal = currentNode->reserveNextFreeTimeForCar(m_car,i);
        double nextFreeTimeLocal = m_systemFunc->getPossibleTimeForPrelimReservation(PathItem(currentNode->getX(), currentNode->getY(), globalTime));

                //car is already in intersection
                //if not, is has to wait
                if (!m_pathint.empty()) {
                    while( globalTime != nextFreeTimeLocal ){

                        PathItem stayhere = m_pathint.back();
                        m_pathint.push_back(PathItem(stayhere.getX(), stayhere.getY(), globalTime));
                        m_systemFunc->prelimReserveCell(m_pathint.back());
                        globalTime+= InterSectionParameters::T ;
                    }
                }


                m_pathint.push_back(PathItem(currentNode->getX(),currentNode->getY(),nextFreeTimeLocal));
                m_systemFunc->prelimReserveCell(m_pathint.back());

            }

        Q_ASSERT_X(currentNode, typeid(this).name(), "open list is not reachable");
                   //target node is reached

                   if (currentNode) {
                       if (currentNode == target) {

                           return m_pathint;
                       }
                       else {
                           m_closedList.append(currentNode);
                           m_openList.removeFirst();

                           std::weak_ptr<InterSectionCell>currentNodeW(currentNode);// maybe do it before open and closed list
                           expandNode(grid, currentNodeW);
                       }

        }
        }
        return m_pathint;

    }
    else{

        //m_startNode = source;
        m_targetNode = target;
        //std::shared_ptr<InterSectionCell> sourceP(source);
        //std::shared_ptr<InterSectionCell> targetP(target);
        //std::shared_ptr<InterSection> gridP(grid);

        Path intpath;

        std::shared_ptr<InterSectionCell> startPoint = grid->getInterSectionCell(m_start.getX(),m_start.getY());
        std::weak_ptr<InterSectionCell> sourceWeak(source);

        if(source == startPoint){

            source -> setTentativeGCost(0.0); // not needed by the way, already 0
            source -> setFCost(estimateCost(source,target));
            //m_openList.append(source);
            m_closedList.append(source);
            expandNode(grid,sourceWeak);
        }

        else{
            m_openList.removeFirst();
            m_closedList.append(source);
            expandNode(grid,sourceWeak);
        }

        qSort(m_openList.begin(), m_openList.end(),[](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return a->getFCost() < b->getFCost();});
        std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
        double desiredtime= m_systemFunc->getGlobalTime() + InterSectionParameters::T;

        //double nextFreeTimeLocal = currentNode->reserveNextFreeTimeForCar(m_car,static_cast<double>(m_systemFunc->getGlobalLiveTime()));
        double nextFreeTimeLocal = m_systemFunc->getPossibleTimeForPrelimReservation(PathItem(currentNode->getX(), currentNode->getY(), currentNode->getNextFreeTime(desiredtime)));

        if (desiredtime == nextFreeTimeLocal ){

            PathItem ret = PathItem(currentNode->getX(),currentNode->getY(),nextFreeTimeLocal);
            intpath.push_back(ret);
            m_systemFunc->prelimReserveCell(intpath.back());

            m_path.push_back(ret);
        }
        else{

            PathItem ret = m_path.back();

            PathItem stayhere = PathItem(ret.getX(), ret.getY(), desiredtime);
            intpath.push_back(stayhere);
            m_systemFunc->prelimReserveCell(intpath.back());

            m_path.push_back(stayhere);

        }


        return intpath;
}
}

/**
 * @brief the expanding method for all successor nodes and appends them to the openlist if
 * - successor node is found for the first time
 * - the found path generates lower costs
 * - TODO: get searchdirection (is defined as <left,right,diagonal>
 * is called recursive for each successor node
 * @param currentNode fron this and the neighbours all nodes will be expanded
 * @param target to get the right search direction
 * @return
 */

//make grid shared
void AStarPathCalculation::expandNode(const std::weak_ptr<InterSection>& grid, std::weak_ptr<InterSectionCell> &currentNode) {
    //Q_ASSERT_X(grid.expired() || currentNode.expired(), typeid(this).name(), "expired grid or nodes");
    std::shared_ptr<InterSection> gridP(grid);
    std::shared_ptr<InterSectionCell> currentNodeP(currentNode);

    //TOBIAS: please indent the code to common standards (here, you may use Qt Creator suggestions)
    if (currentNodeP.get()) {
        //at first get list of neighbours
        QList<std::weak_ptr<InterSectionCell> > neighbours = gridP->getNeighbours(currentNode, 1);// maybe get neighbours
        //now iterate over neighbours
        for (QList<std::weak_ptr<InterSectionCell> >::iterator itNeighbourList = neighbours.begin(); itNeighbourList != neighbours.end(); itNeighbourList++) {
            std::shared_ptr<InterSectionCell> currentNeighbourP(*(itNeighbourList));// ??

            double tentativeGSummedCosts;

            /*if (m_astarType == PathAlgorithm::ASTARCENTRALIZED){

             tentativeGSummedCosts = estimateCost(m_startNode, currentNodeP) + estimateCost(currentNodeP, currentNeighbourP);
}
            else{

              tentativeGSummedCosts = estimateCost(currentNodeP, currentNeighbourP);
            }*/
            tentativeGSummedCosts = estimateCost(currentNodeP, currentNeighbourP);
            double fCost = tentativeGSummedCosts + estimateCost(currentNeighbourP, m_targetNode);
            //look, if closed list contains successor
            bool isContained = containsSuccessor(m_closedList, currentNeighbourP);
            //if closed list contains current nodes, no further examination
            if (isContained) {
                continue;
            }

            //if successor-node is not in the open list
            else if(!m_openList.contains(currentNeighbourP)){
                currentNeighbourP ->setTentativeGCost(tentativeGSummedCosts);
                currentNeighbourP ->setFCost(fCost);
                currentNeighbourP->setParent(currentNode);
                m_openList.append(currentNeighbourP);
            }

            // if successor node is int the open list but new gcost is not better
            else if (m_openList.contains(currentNeighbourP) && fCost >= currentNeighbourP ->getFCost()) {
                continue;
            }
            // successor in the open list with better gcost available
            else {
                //successor-node is better
                //check first, if reservation is possible
                //TODO: give start time here, when it should be reserved (perhaps as start from calculatePath(...))
                currentNeighbourP ->setTentativeGCost(tentativeGSummedCosts);
                currentNeighbourP ->setFCost(fCost);
                currentNeighbourP->setParent(currentNode);
                int index = m_openList.indexOf(currentNeighbourP);
                m_openList.replace(index,currentNeighbourP);
                //double nextFreeTime = currentNeighbourP->getNextFreeTime();
                //m_path.push_back(PathItem(currentNeighbourP->getX(), currentNeighbourP->getY(), nextFreeTime));

            }
        }
    }
}



/*else {
                //calculate costs for the new path:
                //first: value of predecessor (since start?) + costs for the current found node (currentSuccessor)
                //TODO: note and weight if waiting for the next free cell is appropriate or a note as an obstacle would be better
                //at first take constant value like t+2 as supremum (look up in reserved cells)

                //check psuedo code of astar. use top function for costs.
                double tentativeGSummedCosts = estimateCost(m_startNode, currentNode) + estimateCost(currentNode, currentNeighbourP);

                    double fCost = tentativeGSummedCosts + estimateCost(currentNeighbourP, m_target);

                //if successor-node is already on openlist but new path not better than the old one - do nothing
                if (m_openList.contains(currentNodeP) && tentativeSummedCosts >= estimateCost(m_startNode, currentNodeP)) {
                    continue;
                }
                else {
                    //successor-node is better
                    //check first, if reservation is possible
                    //TODO: give start time here, when it should be reserved (perhaps as start from d(...))
                    double nextFreeTime = currentNeighbourP->getNextFreeTime();
                    m_path.push_back(PathItem(currentNeighbourP->getX(), currentNeighbourP->getY(), nextFreeTime));

                }
            }
        }
    }
    //TODO
    return targetP;
    */





/**
 * @brief AStarPathCalculation::getLowestCostNode
 * @param list
 * @param source
 * @return
 */
/*std::shared_ptr<InterSectionCell> AStarPathCalculation::getLowestCostNode(QList<std::shared_ptr<InterSectionCell> >& list, std::shared_ptr<InterSectionCell> &source) const {
    double costs = DBL_MAX;
    std::shared_ptr<InterSectionCell> cheapestNode(list.first());
    for (QList<std::shared_ptr<InterSectionCell> >::iterator itOpenList = list.begin(); itOpenList != list.end(); itOpenList++) {
        std::shared_ptr<InterSectionCell> currentNode(*itOpenList);
        double currentCosts = calculateHCost(source, currentNode) < costs;
        if (currentCosts < costs) {
            cheapestNode = currentNode;
            costs = currentCosts;
        }
    }
    return cheapestNode;
}*/

/**
 * @brief AStarPathCalculation::containsSuccessor
 * @param list
 * @param cell
 * @return
 */
bool AStarPathCalculation::containsSuccessor(QList<std::shared_ptr<InterSectionCell> > &list, std::shared_ptr<InterSectionCell>& cell) const {
    bool isContained = false;
    for (QList<std::shared_ptr<InterSectionCell> >::iterator itClosedList = list.begin(); itClosedList != list.end(); itClosedList++) {
        if (cell == std::shared_ptr<InterSectionCell>(*(itClosedList))) {
            isContained = true;

        }
    }
    return isContained;
}

/*double AStarPathCalculation::getGcost() const
{

}

void AStarPathCalculation::calculateFCost(PathItem &source){
    fCost = gCost + calculateHCost(source);
}


double AStarPathCalculation::getFcost() const {
    return fCost;
}

void AStarPathCalculation::updateGcost(const std::weak_ptr<InterSectionCell>& cell, QList<std::weak_ptr<InterSectionCell>>& neighbours)
{
    if (!cell.expired()) {
        std::shared_ptr<InterSectionCell> cellP(cell);
        // generate list of neighbours with start point and then for every top element in the list(maybe in calc path function)
        for (QList<std::weak_ptr<InterSectionCell> >::const_iterator it = neighbours.begin(); it != neighbours.end(); it++) {
            std::shared_ptr<InterSectionCell> neighbourCur(*it);
            if (neighbourCur->getX() == cellP->getX() || neighbourCur->getY() == cellP->getY()) {

                neighbourCur->setGValue(1.0); // idea to do
                            }
            else {
                                neighbourCur->setGValue(1.4);
                            }
                        }
                    }
}
*/

/**
 * @brief AStarPathCalculation::optimize
 * @param start
 * @return
 */
PathItem AStarPathCalculation::optimize(const PathItem &start) {

    std::shared_ptr<InterSection> grid= InterSection::getInstance();
    //1. we want constant or non constant getintersectioncell ?
    //2. which way is correct?. second way is giving error. i dont know why
    std::shared_ptr<InterSectionCell> startNode= grid->getInterSectionCell(start.getX(),start.getY());
    //std::shared_ptr<InterSectionCell> startNode= InterSection::getInterSectionCell(start.getX(),start.getY());

    std::shared_ptr<InterSectionCell> targetNode= grid->getInterSectionCell(m_target.getX(),m_target.getY());



    if (m_astarType == PathAlgorithm::ASTARCENTRALIZED) {


        //double counter = InterSectionParameters::T;

        //TOBIAS: here, later start times are possible for later arriving cars, therefore, size
        //of m_path has also to be checked
        if (start.getTime() == 0.0 || m_path.size() == 0) {

            //if i dont append start pathitem in m_path (astar). returning pathitems can become easy
            m_path = calculatePath(grid, startNode, targetNode );

            Q_ASSERT_X(m_path.size() > position, typeid(this).name(), "m_path size not matching position");
            PathItem returns = m_path.at(position);
            return returns;
            //return m_path[counter];
        }
        else{

            position++;
            PathItem ret = m_path.at(position);

            return ret;
        }

    }
    else {
        return calculatePath(grid, startNode,targetNode).back();

    }

}

// either we can make a global pointer of current node and set it before entering optimize
/*std::shared_ptr<InterSectionCell> currentNode= make_shared<InterSectionCell> (start.getX(),start.getY());
    std::weak_ptr<InterSectionCell> currentNodeExpansion(currentNode); //necessary ?
    std::shared_ptr<InterSection> grid= InterSection::getInstance();

    m_closedList.append(currentNode);
    m_openList.removeFirst();
    expandNode(grid,currentNodeExpansion);

    qSort(m_openList.begin(),m_openList.end());

    std::shared_ptr<InterSectionCell> desiredNode= m_openList.first();
    double nextFreeTime = desiredNode->reserveNextFreeTimeForCar(m_car,static_cast<double>(m_systemFunc->getGlobalLiveTime()));
    PathItem toReturn= PathItem(desiredNode->getX(),desiredNode->getY(),nextFreeTime);
    m_path.push_back(toReturn);// optional

    return toReturn;
}
*/


    /*    qSort(m_openList.begin(), m_openList.end());
        std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
        double nextFreeTimeLocal = currentNode->reserveNextFreeTimeForCar(m_car,static_cast<double>(m_systemFunc->getGlobalLiveTime()));
        PathItem pathItemToBeReturned= PathItem(currentNode->getX(),currentNode->getY(),nextFreeTimeLocal);
        m_path.push_back(pathItemToBeReturned);
        Q_ASSERT_X(currentNode, typeid(this).name(), "open list is not reachable");

        if (currentNode) {
                m_closedList.append(currentNode);
                m_openList.removeFirst();
                //TODO: dämlicher workaround, überlegen, ob generell die member als shared_ptr gespeichert werden
                std::weak_ptr<InterSectionCell>currentNodeW(currentNode);
                std::shared_ptr<InterSection> grid = InterSection::getInstance();
                expandNode(grid, currentNodeW);
                // recursive call to optimize method //optimize for source
           // }
        }
        return pathItemToBeReturned;
     }*/

/**
 * @brief AStarPathCalculation::optimizeContinous
 * @param controlVec
 * @param t0
 * @param T
 * @return
 */
std::vector<std::vector<double> > AStarPathCalculation::optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T) {
    //std::vector<std::vector<double>> adnan;

    //return adnan;
}

/**
 * @brief AStarPathCalculation::initializeCosts
 * @return
 */

double AStarPathCalculation::initializeCosts() {
    //
    return estimateCost(m_startNode,m_targetNode);
}
/**
 * @brief AStarPathCalculation::setCurrentConstraints sets here the obstacles
 * @param constraints
 */
void AStarPathCalculation::setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double &t0, const double &T) {
    //

}

/**
 * @brief AStarPathCalculation::createGlobalConstraints
 * @param lb
 * @param ub
 */
void AStarPathCalculation::createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) {

    //
}

/**
 * @brief AStarPathCalculation::initializeDirectionalConstraints
 * @param interSectionWidth
 * @param interSectionHeight
 * @param cellSize
 * @param t0
 * @param N
 * @param T
 * @param dmin
 * @param maxDynamics
 */
void AStarPathCalculation::initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                                            const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics) {
    //

}

/**
 * @brief AStarPathCalculation::getCurrentPrediction
 * @return
 */
std::vector<double> AStarPathCalculation::getCurrentPrediction() const {
    std::vector<double> adnan;
    //adnan.push_back(0.0);
    return adnan;
}

/**
 * @brief AStarPathCalculation::setControlRange
 * @param lb
 * @param ub
 */
void AStarPathCalculation::setControlRange(const double& lb, const double& ub) {
    double a = lb;
    double b = ub;
}

std::vector<double> AStarPathCalculation::getTargetContinuous() const {
    return m_targetCont;
}
/**
 * @brief AStarPathCalculation::getLambda
 * @return
 */
double AStarPathCalculation::getLambda() const {
    return 0.0;
}
/**
 * @brief AStarPathCalculation::getOpenLoopCosts
 * @return
 */
double AStarPathCalculation::getOpenLoopCosts() const {
    return 0.0;
}

/**
 * @brief AStarPathCalculation::getClosedLoopCosts
 * @return
 */
double AStarPathCalculation::getClosedLoopCosts() const {
    return 0.0;
}

/**
 * @brief AStarPathCalculation::getInitialControl
 * @param t0
 * @param T
 * @return
 */
std::vector<double> AStarPathCalculation::getInitialControl(const double &t0, const double &T) {
    std::vector<double> adnan;
    adnan.push_back(0.0);
    adnan.push_back(0.0);

    return adnan;
}

/**
 * @brief AStarPathCalculation::getCurrentPrelimSolution
 * @return
 */
PathItem AStarPathCalculation::getCurrentPrelimSolution() const {

    if (m_astarType == PathAlgorithm::ASTARCENTRALIZED) {

        return m_path.at(position);
    }

    else {
        return m_path.back();
    }
    //return m_path.at(m_systemFunc->getGlobalTime());
    //return m_path.back();
    //return m_path[i];  make i a member int variable and increment for each optimization step
    //return m_sysFunc->m_path.back();
}

std::map<std::string, double> AStarPathCalculation::calcCostsForNeighbours() {
    std::map<std::string,double> adnan;

    adnan.insert(std::make_pair("car1",0.0));
    adnan.insert(std::make_pair("car2",0.0));

    return adnan;
}

std::pair<double, double> AStarPathCalculation::getControlBounds() const {

    return std::make_pair(-1.0, 1.0 );
}

bool AStarPathCalculation::testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const {
    return false;
}

std::vector<Constraint> &AStarPathCalculation::getCurrentConstraints() {
    //
}

void AStarPathCalculation::clearAllConstraints() {

}

void AStarPathCalculation::clearPrediction() {
    //
}
