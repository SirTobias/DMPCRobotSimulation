#include "floydwarshallpathcalculation.h"
#include "intersection.h"

/**
 * @brief FloydWarshallPathCalculation::FloydWarshallPathCalculation
 * @param car
 * @param start
 * @param target
 * @param T sampling interval
 */
FloydWarshallPathCalculation::FloydWarshallPathCalculation(const QString &car, const PathItem &start, const PathItem &target, const double &T) :
m_car(car),
m_start(start),
m_target(target)
{
m_sysFunc = std::make_shared<SystemFunction>(m_car,Path(start));

for(int i=0; i < InterSection::getInstance() ->getWidth(); i++){
    for(int j=0; j < InterSection::getInstance() ->getHeight(); j++){

     std::shared_ptr<InterSectionCell> temp =InterSection::getInstance()->getInterSectionCell(i,j);
             //temp->setTentativeGCost(5000);

            // std::weak_ptr<InterSectionCell> tempW(temp);
             //tempW = temp->getParent();
             //std::shared_ptr<InterSectionCell> hell(tempW);
             //if(!hell){
               //  hell.reset();

             //}
            // std::shared_ptr<InterSectionCell> a;

            param parameters;
            parameters.g = 5000;

             m_map.insert(InterSection::getInstance() -> getInterSectionCell(i,j) , parameters);
             m_openList.append(temp);
    }
}
}

/**
 * @brief FloydWarshallPathCalculation
 * @param car
 * @param start
 * @param target
 */
FloydWarshallPathCalculation::FloydWarshallPathCalculation(const QString &car, const std::vector<double>& start, const std::vector<double>& target) {

}

std::map<std::string, double> FloydWarshallPathCalculation::calcCostsForNeighbours() {
    //
}

std::pair<double, double> FloydWarshallPathCalculation::getControlBounds() const {

}

/**
 * @brief FloydWarshallPathCalculation::getAlg
 * @return
 */
PathAlgorithm FloydWarshallPathCalculation::getAlg() const {
    return PathAlgorithm::FLOYDWARSHALL;
}

/**
 * @brief FloydWarshallPathCalculation::getSystemFunction
 * @return
 */
std::shared_ptr<SystemFunction> FloydWarshallPathCalculation::getSystemFunction() const {
    return m_sysFunc;
}

double FloydWarshallPathCalculation::estimateCost(const std::shared_ptr<InterSectionCell> &first, const std::shared_ptr<InterSectionCell> &second)
{
    return std::sqrt(std::pow(second->getX() - first->getX(), 2) + std::pow(second->getY() - first->getY(), 2));
}



void FloydWarshallPathCalculation::expandNode(const std::shared_ptr<InterSection> &grid, std::weak_ptr<InterSectionCell> &currentNode)
{
   std::shared_ptr<InterSectionCell> currentNodeP(currentNode);

   if(currentNodeP.get()){

       QList<std::weak_ptr<InterSectionCell>> neighbourList = grid->getNeighbours(currentNode,1);

       for(auto itNeighbourList= neighbourList.begin(); itNeighbourList!= neighbourList.end(); itNeighbourList++){

           std::shared_ptr<InterSectionCell> currentNeighbourP(*(itNeighbourList));

           // should i only find distance from neighbour to start node everytime ??
           //double tentativeCost = estimateCost(currentNeighbourP,currentNodeP) + currentNodeP->getTentativeGCost();
           double tentativeCost = estimateCost(currentNeighbourP,currentNodeP) + getG(currentNodeP);

           if(m_closedList.contains(currentNeighbourP)){
               continue;
           }

           //else if(m_openList.contains(currentNeighbourP) && currentNeighbourP->getTentativeGCost() <= tentativeCost){
             else if(m_openList.contains(currentNeighbourP) && getG(currentNeighbourP) <= tentativeCost){
               continue;
           }

           else{

               //currentNeighbourP->setTentativeGCost(tentativeCost);
               setG(currentNeighbourP,tentativeCost);
               //currentNeighbourP->setParent(currentNode);
               setParent(currentNeighbourP,currentNodeP);
              // m_map.insert(currentNeighbourP,currentNodeP);
           }
       }
   }
}

void FloydWarshallPathCalculation::setG(const std::shared_ptr<InterSectionCell> &a, double value)
{
param parameters = m_map.value(a);
parameters.g = value;
m_map.insert(a,parameters);

}

void FloydWarshallPathCalculation::setParent(const std::shared_ptr<InterSectionCell> &a, const std::shared_ptr<InterSectionCell> &b)
{
param parameters = m_map.value(a);
parameters.parent = b;
m_map.insert(a,parameters);
}

double FloydWarshallPathCalculation::getG(const std::shared_ptr<InterSectionCell> &a)
{
param parameters = m_map.value(a);
double g = parameters.g;
return g;
}

std::shared_ptr<InterSectionCell> FloydWarshallPathCalculation::getParent(const std::shared_ptr<InterSectionCell> &a)
{
param parameters = m_map.value(a);
std::shared_ptr<InterSectionCell> node = parameters.parent;
return node;
}

/**
 * @brief FloydWarshallPathCalculation::calculatePath
 * @param grid
 * @param source
 * @param target
 * @return
 */
Path FloydWarshallPathCalculation::calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source,
                                                 const std::shared_ptr<InterSectionCell> &target) {

    //std::shared_ptr<InterSectionCell> a = std::make_shared<InterSectionCell>(3,4);
    //int j;
    m_startNode = source;
    m_targetNode = target;

   // source->setTentativeGCost(0);
    setG(m_startNode,0);
   // m_closedList.append(source);

    while(!m_openList.isEmpty()){

    //qSort(m_openList.begin(), m_openList.end(),[](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return a->getTentativeGCost() < b->getTentativeGCost(); });

        qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return getG(a) < getG(b); });
    std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
    m_openList.removeFirst();
    m_closedList.append(currentNode);

    std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
    expandNode(grid,currentNodeW);


    }
    // path planning here
    Path temp;
    QList<std::shared_ptr<InterSectionCell>> pointers;
    std::shared_ptr<InterSectionCell> present(m_targetNode);

    double globalTime = m_sysFunc->getGlobalTime() + InterSectionParameters::T ;

    while(present != m_startNode){
        /*std::weak_ptr<InterSectionCell> parent;
        parent = present->getParent();
        std::shared_ptr<InterSectionCell> parentShared(parent);
        PathItem ret = PathItem (parentShared->getX(), parentShared->getY(), 0);
        temp.push_back(ret);
        present = parentShared;*/

        //std::shared_ptr<InterSectionCell> parent = m_map.value(present);

        //PathItem ret = PathItem(present->getX() , present->getY() , globalTime);
        //temp.push_back(ret);
        pointers.append(present);

        std::shared_ptr<InterSectionCell> parent = getParent(present);
        present = parent;
        //globalTime+= InterSectionParameters::T ;


        //present = parent;

    }


    std::reverse(pointers.begin(),pointers.end());

    for(auto it= pointers.begin() ; it!= pointers.end() ; it++){

        std::shared_ptr<InterSectionCell> current(*(it));
        PathItem abc = PathItem(current->getX() , current->getY() , globalTime);

        double nextFreeTime = m_sysFunc->getPossibleTimeForPrelimReservation(abc);

        if(nextFreeTime != globalTime){
            if (temp.size() > 0) { //is already on the intersection
                PathItem stayHere = temp.back();
                temp.push_back(PathItem(stayHere.getX() , stayHere.getY() , globalTime));
                m_sysFunc->prelimReserveCell(temp.back());
                globalTime+= InterSectionParameters::T ;
            } else {
                PathItem startItem(current->getX(), current->getY(), nextFreeTime);
                m_sysFunc->prelimReserveCell(startItem);
                temp.push_back(startItem);
            }
        }

        temp.push_back(PathItem(abc.getX() , abc.getY() , nextFreeTime));
        m_sysFunc->prelimReserveCell(temp.back());
        globalTime+= InterSectionParameters::T ;
    }

    return temp;
}

/**
 * @brief FloydWarshallPathCalculation::optimize
 * @param start
 * @return
 */
PathItem FloydWarshallPathCalculation::optimize(const PathItem &start) {

    std::shared_ptr<InterSection> grid = InterSection::getInstance();
    std::shared_ptr<InterSectionCell> startNode = grid->getInterSectionCell(m_start.getX(), m_start.getY());
    std::shared_ptr<InterSectionCell> targetNode = grid->getInterSectionCell(m_target.getX(), m_target.getY());

    a = position;

    if (start.getTime() == 0 || m_path.empty()){
       m_path = calculatePath(grid,startNode,targetNode);



    PathItem ret = m_path.at(position);
   // double nextFreeTime = m_sysFunc->getPossibleTimeForPrelimReservation(ret);
return ret;
    }

    else {
        position++ ;
        PathItem ret = m_path.at(position);
        return ret;

    }
}

/**
 * @brief FloydWarshallPathCalculation::optimizeContinous
 * @param controlVec
 * @param t0
 * @param T
 * @return
 */
std::vector<std::vector<double> > FloydWarshallPathCalculation::optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T) {
    //
}

/**
 * @brief FloydWarshallPathCalculation::initializeCosts
 * @return
 */
double FloydWarshallPathCalculation::initializeCosts() {
    //
}

/**
 * @brief FloydWarshallPathCalculation::setCurrentConstraints
 * @param constraints
 */
void FloydWarshallPathCalculation::setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double &t0, const double &T) {
    //
}

/**
 * @brief FloydWarshallPathCalculation::createGlobalConstraints
 * @param lb
 * @param ub
 */
void FloydWarshallPathCalculation::createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) {
    //
}

/**
 * @brief FloydWarshallPathCalculation::initializeDirectionalConstraints
 * @param interSectionWidth
 * @param interSectionHeight
 * @param cellSize
 * @param t0
 * @param N
 * @param T
 * @param dmin
 * @param maxDynamics
 */
void FloydWarshallPathCalculation::initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                              const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics) {
    //
}

/**
 * @brief FloydWarshallPathCalculation::getCurrentPrediction
 * @return
 */
std::vector<double> FloydWarshallPathCalculation::getCurrentPrediction() const {
    //
}

/**
 * @brief FloydWarshallPathCalculation::setControlRange
 * @param lb
 * @param ub
 */
void FloydWarshallPathCalculation::setControlRange(const double& lb, const double& ub) {
    //
}

std::vector<double> FloydWarshallPathCalculation::getTargetContinuous() const {

}
/**
 * @brief FloydWarshallPathCalculation::getLambda
 * @return
 */
double FloydWarshallPathCalculation::getLambda() const {
    //
}
/**
 * @brief FloydWarshallPathCalculation::getOpenLoopCosts
 * @return
 */
double FloydWarshallPathCalculation::getOpenLoopCosts() const {
    //
}

/**
 * @brief FloydWarshallPathCalculation::getClosedLoopCosts
 * @return
 */
double FloydWarshallPathCalculation::getClosedLoopCosts() const {
    //
}

/**
 * @brief FloydWarshallPathCalculation::getInitialControl
 * @param t0
 * @param T
 * @return
 */
std::vector<double> FloydWarshallPathCalculation::getInitialControl(const double &t0, const double &T) {
    //
}

/**
 * @brief FloydWarshallPathCalculation::getCurrentPrelimSolution
 * @return
 */
PathItem FloydWarshallPathCalculation::getCurrentPrelimSolution() const {
    return m_path.at(position);
}

bool FloydWarshallPathCalculation::testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const {
    return false;
}

std::vector<Constraint> &FloydWarshallPathCalculation::getCurrentConstraints() {
    //return m_pathCalc->getCurrentConstraints();
}
void FloydWarshallPathCalculation::clearAllConstraints() {
    //
}

void FloydWarshallPathCalculation::clearPrediction() {
    //
}
