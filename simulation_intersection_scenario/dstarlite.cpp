#include "dstarlite.h"
#include "algorithm"
#include "iomanip"

#include <QDebug>

/**
 * @brief DstarLite::DstarLite
 * @param car
 * @param start
 * @param target
 * @param T sampling interval
 */
DstarLite::DstarLite(const QString &car, const PathItem &start, const PathItem &target, const double &T) :
    m_car(car),
    m_start(start),
    m_target(target)

{
    m_sysFunc= std::make_shared<SystemFunction> (m_car,Path(start));
    for (int i = 0; i < InterSection::getInstance()->getWidth(); i++) {
        for (int j = 0; j < InterSection::getInstance()->getWidth(); j++) {
            DStarParam param;
            param.g = 5000;
            param.key = std::make_tuple(100,100);
            param.rhs = 5000;
            m_dStarMap.insert(InterSection::getInstance()->getInterSectionCell(i,j), param);
        }
    }
    //DStarParam currentParam = m_dStarMap.value(source);
    //currentParam.gCost = 5000.0;
    //m_dStarMap.insert(source, currentParam);
}

DstarLite::DstarLite(const QString &car, const std::vector<double>& start, const std::vector<double>& target) {

}

/**
 * @brief DstarLite::getAlg
 * @return
 */
PathAlgorithm DstarLite::getAlg() const {
    return PathAlgorithm::DSTAR;
}

/**
 * @brief DstarLite::getSystemFunction
 * @return
 */
std::shared_ptr<SystemFunction> DstarLite::getSystemFunction() const {
    return m_sysFunc;
}

double DstarLite::estimateCost(const std::shared_ptr<InterSectionCell> &first, const std::shared_ptr<InterSectionCell> &second)
{
    return std::sqrt(std::pow(second->getX() - first->getX(), 2) + std::pow(second->getY() - first->getY(), 2));
}

/**
 * @brief DstarLite::calculatePath
 * @param grid
 * @param source
 * @param target
 * @return
 */
Path DstarLite::calculatePath(const std::shared_ptr<InterSection> grid, const std::shared_ptr<InterSectionCell> &source, const std::shared_ptr<InterSectionCell> &target) {

    m_targetnode= target;
    m_startnode = grid->getInterSectionCell(m_start.getX(),m_start.getY());
    m_source = source;
    //std::shared_ptr<InterSectionCell> here = grid->getInterSectionCell(12,0);

    // if (m_source == m_startnode){  // calculate the initial complete path


    //target->setRhs(0.0);
    setrhs(target,0);
    //target->setG(5000);
    //double min = std::min(target->getG() , target->getRhs());
    double min = std::min(getG(target) , getrhs(target));
    //target->setKey(min + estimateCost(target,m_source));
    double k1 = min + estimateCost(target,m_source);
    std::tuple<double,double> first(k1,min);

    setKey(target , first);



    m_openList.append(target);
    // sortedOpenList.insert(m_dStarMap.value(target), target);

    //while((m_openList.first()->getKey() < source->getKey()) || source->getG() != source->getRhs()) {
    while((getKey(m_openList.first()) < getKey(source)) || getG(source) != getrhs(source)){



        //double b = std::get<1> (getKey(InterSection::getInstance()->getInterSectionCell(12,0)));
        std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
        m_openList.removeFirst();
        // int a = m_openList.indexOf(currentNode);
        //m_openList.removeAt(a);

        // decide when to remove the element. at start or end

        if (currentNode){

            //if (currentNode->getG() > currentNode->getRhs()){ // over-consistent ?
            if(getG(currentNode) > getrhs(currentNode)){
                //currentNode->setG(currentNode->getRhs());
                setG(currentNode,getrhs(currentNode));
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,source);

            }

            else{ // under-consistent?

                //currentNode->setG(5000);
                setG(currentNode , 5000);
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,source);
                double min = std::min(getG(currentNode) , getrhs(currentNode));
                double k1 = min + estimateCost(currentNode,m_source);
                std::tuple<double,double> first(k1,min);

                setKey(currentNode, first);
                m_openList.append(currentNode); // put it back in the open list
            }

            // this is the correct position to sort the list
            //m_dStarMap.value(a).key;
            //QMap<DStarParam, std::shared_ptr<InterSectionCell> > sortedOpenList;
            /*for (size_t i = 0; i < m_openList.size(); i++) {
                sortedOpenList.insert(m_dStarMap.value(m_openList.at(i)), m_openList.at(i));
            }*/
            // sortedOpenList.first(); //gives InterSectionCell
            // sortedOpenList.firstKey(); //gives DStarParam

            /* for (auto itSortedMap = sortedOpenList.begin(); itSortedMap != sortedOpenList.end(); itSortedMap++) {
                qDebug() << std::get<0>(itSortedMap.key().key) << "," << std::get<1>(itSortedMap.key().key);
            }*/


            //qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::   shared_ptr<InterSectionCell> b) -> bool { return getKey(a) < getKey(b); });
            //qSort(sortedOpenList.begin(), sortedOpenList.end(),[&](const DStarParam a, const DStarParam b) -> bool { return std::get<0>(a.first.key) < std::get<0>(b.first.key); });)
            qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool
            {
                if( std::get<0> (getKey(a)) == std::get<0>(getKey(b)))
                    return std::get<1>(getKey(a)) < std::get<1>(getKey(b));

                else
                    return std::get<0> (getKey(a)) < std::get<0>(getKey(b)) ; });
        }
    }

    // generate path.... make this a function
    double globalTime = 0.0;
    m_presentNode = m_startnode; // needed initialization to run the loop.
    while (m_presentNode != m_targetnode) {
        // std::shared_ptr<InterSectionCell> here = grid->getInterSectionCell(13,1);

        m_presentNode = gradientForPath(grid,m_presentNode);// node with the least g value
        globalTime += InterSectionParameters::T ;
        m_initialPath.push_back(PathItem(m_presentNode->getX() , m_presentNode->getY() , globalTime));
        // m_sysFunc->prelimReserveCell(m_initialPath.back()); // what if 2 cars reserve same cell for same time at planning

        //  wht if cell is blocked since start ?
    }



    return m_initialPath;

}

/**
 * @brief DstarLite::optimize
 * @param start
 * @return
 */
PathItem DstarLite::optimize(const PathItem &start) {

    std::shared_ptr<InterSection> grid= InterSection::getInstance();

    std::shared_ptr<InterSectionCell> startNode = grid->getInterSectionCell(start.getX(),start.getY());

    std::shared_ptr<InterSectionCell> targetNode = grid->getInterSectionCell(m_target.getX(),m_target.getY());

    position=a;

    if (start.getTime() == 0 || m_finalPath.empty()) {

        m_finalPath = calculatePath(grid , startNode, targetNode);
    }
    // std::shared_ptr<InterSectionCell> here = grid->getInterSectionCell(12,0);

    PathItem toreturn = m_finalPath.at(position);

    // double time = m_sysFunc->getGlobalTime() + InterSectionParameters::T;
    double nextfreetime = m_sysFunc->getPossibleTimeForPrelimReservation(toreturn);

    if (toreturn.getTime() == nextfreetime){
        a++;
        m_sysFunc->prelimReserveCell(toreturn);
        return toreturn;
    }

    else{
        // replan here

        PathItem blocked = m_finalPath.at(position); // decide whether to take here or outside of this function
        m_finalPath.clear();
        position=0;
        a=0;
        m_initialPath.clear();

        m_finalPath = replan(grid , blocked, startNode);
        toreturn = m_finalPath.at(position);
        m_sysFunc->prelimReserveCell(toreturn);
        a++;
        return toreturn;
        // 1. delete the list of path
        //2. dont forget to reset position to 0 in new list
    }



}


// update vertex according to algorithm
void DstarLite::expandNode(const std::shared_ptr<InterSection> &grid, std::weak_ptr<InterSectionCell> &currentNode, const std::shared_ptr<InterSectionCell> &start)
{

    //std::shared_ptr<InterSection> gridP(grid);
    std::shared_ptr<InterSectionCell> currentNodeP(currentNode);

    if (currentNodeP.get()){

        QList<std::weak_ptr<InterSectionCell>> neighbours = grid->getNeighbours(currentNode,1);

        for(QList<std::weak_ptr<InterSectionCell>>::iterator it = neighbours.begin(); it!= neighbours.end(); it++){

            std::shared_ptr<InterSectionCell> currentNeighbourP(*(it));

            // double rhsValue = currentNodeP->getG() + estimateCost(currentNodeP,currentNeighbourP);
            double rhsValue = getG(currentNodeP) + estimateCost(currentNodeP , currentNeighbourP);


            if (currentNeighbourP == currentNodeP){
                continue;
            }
            if (currentNeighbourP == m_targetnode){
                continue;
            }

            else if (m_openList.contains(currentNeighbourP)){
                int index = m_openList.indexOf(currentNeighbourP);
                m_openList.removeAt(index);
                //continue;
            }

            // if already consistent , continue.... still not sure about this approach
            else if ((getG(currentNeighbourP) == getrhs(currentNeighbourP)) && getrhs(currentNeighbourP) <= rhsValue){
                continue;
            }

            else {// main condition

                //currentNeighbourP->setRhs(rhsValue); // should i put the condition to check if g and rhs are not equal now
                setrhs(currentNeighbourP,rhsValue);
                //double min = std::min(currentNeighbourP->getG() , currentNeighbourP->getRhs());
                double min = std::min(getG(currentNeighbourP) , getrhs(currentNeighbourP));
                // should replace m_startnode to something else because of replanning
                //currentNeighbourP->setKey(min + estimateCost(currentNeighbourP , start)) ;
                double k1 = min + estimateCost(currentNeighbourP,start);
                std::tuple<double,double> first(k1,min);
                setKey(currentNeighbourP ,first);
                m_openList.append(currentNeighbourP);

            }
            //
        }

    }
}

std::shared_ptr<InterSectionCell> DstarLite::gradientForPath(const std::shared_ptr<InterSection> &grid, const std::shared_ptr<InterSectionCell> &presentNode)
{
    std::shared_ptr<InterSectionCell> myNode = presentNode;
    QList<std::weak_ptr<InterSectionCell>> neighbours = grid->getNeighbours(presentNode , 1);
    for(QList<std::weak_ptr<InterSectionCell>>::iterator it = neighbours.begin(); it!= neighbours.end(); it++){

        std::shared_ptr<InterSectionCell> currentNeighbourPGradient(*(it));

        if(m_openList.contains(currentNeighbourPGradient)){
            continue;
        }

        else if (currentNeighbourPGradient == presentNode ){
            continue;
        }

        else if(getG(currentNeighbourPGradient) < getG(myNode)){
            myNode = currentNeighbourPGradient;
        }

    }
    m_presentNode = myNode;
    return m_presentNode;
}

/*std::shared_ptr<InterSectionCell> DstarLite::gradientForReplanning(const std::shared_ptr<InterSection> &grid,const std::shared_ptr<InterSectionCell> &presentNode)
{
    std::shared_ptr<InterSectionCell> myNode = presentNode;
    QList<std::weak_ptr<InterSectionCell>> neighbours = grid->getNeighbours(presentNode , 1);
    for(QList<std::weak_ptr<InterSectionCell>>::iterator it = neighbours.begin(); it!= neighbours.end(); it++){

        std::shared_ptr<InterSectionCell> currentNeighbourPGradient(*(it));

        if (currentNeighbourPGradient == presentNode ){
            continue;
        }

        else if(getG(currentNeighbourPGradient) < getG(myNode) ){
            myNode = currentNeighbourPGradient;
        }

    }
    m_presentNode = myNode;
    return m_presentNode;
}*/

/*Path DstarLite::replan1(const shared_ptr<InterSection> &grid, PathItem &source, std::shared_ptr<InterSectionCell> &start)
{

    std::shared_ptr<InterSectionCell> blocked = grid->getInterSectionCell(source.getX() , source.getY());

    setrhs(blocked,5000);
    double min = std::min(getG(blocked) , getrhs(blocked));
    double k1 = min + estimateCost(blocked,start);
    std::tuple<double,double> first(k1,min);
    setKey(blocked ,first);
    // setKey(blocked,(min + estimateCost(blocked,start)));
    // m_openList.append(blocked);

    // for incoming edges. till line #301
    QList<std::weak_ptr<InterSectionCell>> neighbours = grid->getNeighbours(blocked,1);

    for(QList<std::weak_ptr<InterSectionCell>>::iterator it = neighbours.begin(); it!= neighbours.end(); it++){

        std::shared_ptr<InterSectionCell> currentNeighbourP(*(it));

        if(currentNeighbourP == blocked){
            continue;
        }

        m_presentNode = gradientForReplanning(grid,currentNeighbourP);// incoming node check

        if (m_presentNode == blocked){
            m_openList.append(blocked);
            m_presentNode = gradientForPath(grid,currentNeighbourP);
            int index = m_openList.indexOf(blocked);
            m_openList.removeAt(index); // remove blocked node for another neighbour to iterate also on it

            //currentNeighbourP->setRhs(m_presentNode->getG() + estimateCost(m_presentNode , currentNeighbourP));
            setrhs(currentNeighbourP,(getG(m_presentNode) + estimateCost(m_presentNode,currentNeighbourP)));

            if (getG(currentNeighbourP) == getrhs(currentNeighbourP)){
                continue;
            }
            else {
                double min = std::min(getG(currentNeighbourP) , getrhs(currentNeighbourP));
                double k1 = min + estimateCost(currentNeighbourP,start);
                std::tuple<double,double> first(k1,min);
                setKey(currentNeighbourP ,first);
                // setKey(currentNeighbourP,(min + estimateCost(currentNeighbourP,start)));
                m_openList.append(currentNeighbourP);

            }

        }

        else {
            continue;
        }



    }

    // for outgoing edges now

    //double minOfStart = std::min(start->getG() , start->getRhs());
    //start->setKey(min + estimateCost(blocked,start));
    m_openList.append(blocked);
    qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return getKey(a) < getKey(b); });

    while((getKey(m_openList.first()) < getKey(start)) || getG(start) != getrhs(start)){

        std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
        m_openList.removeFirst();
        // decide when to remove the element. at start or end

        if (currentNode){

            if(getG(currentNode) > getrhs(currentNode)){
                //currentNode->setG(currentNode->getRhs());
                setG(currentNode,getrhs(currentNode));
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,start);

            }

            else{ // under-consistent?
                setG(currentNode , 5000);
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,start);
                double min = std::min(getG(currentNode) , getrhs(currentNode));
                double k1 = min + estimateCost(currentNode,start);
                std::tuple<double,double> first(k1,min);
                setKey(currentNode ,first);
                // setKey(currentNode, (min + estimateCost(currentNode ,m_source)));
                m_openList.append(currentNode); // put it back in the open list
            }



            // this is the correct position to sort the list
            qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool { return getKey(a) < getKey(b); });
        }
    }

    double globalTime = m_sysFunc->getGlobalTime();
    m_presentNode = start;
    while (m_presentNode != m_targetnode) {

        m_presentNode = gradientForPath(grid,m_presentNode);// node with the least g value
        globalTime += InterSectionParameters::T ;
        m_initialPath.push_back(PathItem(m_presentNode->getX() , m_presentNode->getY() , globalTime));
        // m_sysFunc->prelimReserveCell(m_initialPath.back());

    }
    return m_initialPath;
}*/


Path DstarLite::replan(const shared_ptr<InterSection> &grid, PathItem &source, std::shared_ptr<InterSectionCell> &start)
{
    std::shared_ptr<InterSectionCell> blocked = grid->getInterSectionCell(source.getX() , source.getY());

    // no need to calculate rhs or g or key for blocked. its blocked. maybe set garbage values
    // only calculate rhs values of affected cells.


    double a = getrhs(blocked) + 1;
    //double a1= std::ceil(a);
    double b = getrhs(blocked) + 1.4;
    //double b1= std::floor(b);

    // fill blocked with garbage values. no use
    setrhs(blocked,4000);
    setG(blocked,4500);
    double min = std::min(getG(blocked) , getrhs(blocked));
    double k1 = min + estimateCost(blocked,start);
    std::tuple<double,double> k(k1,min);
    setKey(blocked,k);

// manual first step for update vertex till line #526:D
    QList<std::weak_ptr<InterSectionCell>> neighbours = grid->getNeighbours(blocked,1);

    for(QList<std::weak_ptr<InterSectionCell>>::iterator it = neighbours.begin(); it!= neighbours.end(); it++){

        std::shared_ptr<InterSectionCell> currentNeighbourP(*(it));
        double x= getrhs(currentNeighbourP);

        // these nodes calculated their rhs value based on blocked node. so update their parent and calculate new values

        //if(abs ((getrhs(currentNeighbourP)==a || getrhs(currentNeighbourP)==b)<0.03) && m_openList.contains(currentNeighbourP)){
        //if((abs(x==a || x==b) < 0.03) && m_openList.contains(currentNeighbourP)){

        // this if if finding which node calculated its rhs value through blocked
        if(((abs(x-a)<0.03) ||(abs(x-b) < 0.03)) && currentNeighbourP!=start && currentNeighbourP!=blocked){
            setrhs(currentNeighbourP,5000);
            std::tuple<double,double> k(100,100);
            setKey(currentNeighbourP,k);
            //expandNode(grid,currentNeighbourW,start);
            if(m_openList.contains(currentNeighbourP)){

                int index= m_openList.indexOf(currentNeighbourP);
                m_openList.removeAt(index);
            }
            else{
                m_openList.append(currentNeighbourP);
            }
        }


        else if(currentNeighbourP == start){
            setrhs(currentNeighbourP,5000);
            double min = std::min(getG(currentNeighbourP) , getrhs(currentNeighbourP));
            double k1 = min + estimateCost(currentNeighbourP,start);
            std::tuple<double,double> k(k1,min);
            setKey(currentNeighbourP,k);
            m_openList.append(currentNeighbourP);

        }
        else if (currentNeighbourP == blocked){
            continue ;
        }


        else{
            continue;
        }
    }

    qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool
    {
        if( std::get<0> (getKey(a)) == std::get<0>(getKey(b)))
            return std::get<1>(getKey(a)) < std::get<1>(getKey(b));

        else
            return std::get<0> (getKey(a)) < std::get<0>(getKey(b)) ; });

    while((getKey(m_openList.first()) < getKey(start)) || getG(start) != getrhs(start)){

        std::shared_ptr<InterSectionCell> currentNode = m_openList.first();
        m_openList.removeFirst();
        // decide when to remove the element. at start or end

        if (currentNode){

            if(getG(currentNode) > getrhs(currentNode)){
                //currentNode->setG(currentNode->getRhs());
                setG(currentNode,getrhs(currentNode));
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,start);

            }

            else{ // under-consistent?
                setG(currentNode , 5000);
                std::weak_ptr<InterSectionCell> currentNodeW(currentNode);
                expandNode(grid,currentNodeW,start);
                double min = std::min(getG(currentNode) , getrhs(currentNode));
                double k1 = min + estimateCost(currentNode,start);
                std::tuple<double,double> first(k1,min);
                setKey(currentNode ,first);
                // setKey(currentNode, (min + estimateCost(currentNode ,m_source)));
                m_openList.append(currentNode); // put it back in the open list
            }

            qSort(m_openList.begin(), m_openList.end(),[&](const std::shared_ptr<InterSectionCell> a, const std::shared_ptr<InterSectionCell> b) -> bool
            {
                if( std::get<0> (getKey(a)) == std::get<0>(getKey(b)))
                    return std::get<1>(getKey(a)) < std::get<1>(getKey(b));

                else
                    return std::get<0> (getKey(a)) < std::get<0>(getKey(b)) ; });


        }
    }
    double globalTime = m_sysFunc->getGlobalTime();
    m_presentNode = start;
    while (m_presentNode != m_targetnode) {

        m_presentNode = gradientForPath(grid,m_presentNode);// node with the least g value
        globalTime += InterSectionParameters::T ;
        m_initialPath.push_back(PathItem(m_presentNode->getX() , m_presentNode->getY() , globalTime));
        // m_sysFunc->prelimReserveCell(m_initialPath.back());

    }
    return m_initialPath;
}


/**
 * @brief DstarLite::optimizeContinous
 * @param controlVec
 * @param t0
 * @param T
 * @return
 */

std::vector<std::vector<double> > DstarLite::optimizeContinous(const std::vector<double> &controlVec, const double &t0, const double &T) {
    //
}


/**
 * @brief DstarLite::initializeCosts
 * @return
 */
double DstarLite::initializeCosts() {
    //
}

/**
 * @brief DstarLite::setCurrentConstraints
 * @param constraints
 */
void DstarLite::setCurrentConstraints(const std::multimap<QString, Constraint> &constraints, const double& t0, const double& T) {
    //
}

/**
 * @brief DstarLite::createGlobalConstraints
 * @param lb
 * @param ub
 */
void DstarLite::createGlobalConstraints(const std::vector<double>& lb, const std::vector<double>& ub) {
    //
}

/**
 * @brief DstarLite::initializeDirectionalConstraints
 * @param interSectionWidth
 * @param interSectionHeight
 * @param cellSize
 * @param t0
 * @param N
 * @param T
 * @param dmin
 * @param maxDynamics
 */
void DstarLite::initializeDirectionalConstraints(const unsigned int &interSectionWidth, const unsigned int &interSectionHeight, const double &cellSize,
                                                 const double &t0, const int &N, const double &T, const double &dmin, const double &maxDynamics) {
    //
}

/**
 * @brief DstarLite::getCurrentPrediction
 * @return
 */
std::vector<double> DstarLite::getCurrentPrediction() const {
    //
}

/**
 * @brief DstarLite::setControlRange
 * @param lb
 * @param ub
 */
void DstarLite::setControlRange(const double& lb, const double& ub) {
    //
}

std::vector<double> DstarLite::getTargetContinuous() const {

}
/**
 * @brief DstarLite::getLambda
 * @return
 */
double DstarLite::getLambda() const {
    //
}
/**
 * @brief DstarLite::getOpenLoopCosts
 * @return
 */
double DstarLite::getOpenLoopCosts() const {
    //
}

/**
 * @brief DstarLite::getClosedLoopCosts
 * @return
 */
double DstarLite::getClosedLoopCosts() const {
    //
}

/**
 * @brief DstarLite::getInitialControl
 * @param t0
 * @param T
 * @return
 */
std::vector<double> DstarLite::getInitialControl(const double &t0, const double &T) {
    //
}


/**
 * @brief DstarLite::getCurrentPrelimSolution
 * @return
 */
PathItem DstarLite::getCurrentPrelimSolution() const {
    return m_finalPath.at(position);
}

std::map<std::string, double> DstarLite::calcCostsForNeighbours() {
    //
}

std::pair<double, double> DstarLite::getControlBounds() const {

}

std::tuple<double,double> DstarLite::getKey(const std::shared_ptr<InterSectionCell> &node)
{
    DStarParam currentParam = m_dStarMap.value(node);
    std::tuple<double,double> key = currentParam.key;
    return key;
}

double DstarLite::getG(const std::shared_ptr<InterSectionCell> &node)
{
    DStarParam currentParam = m_dStarMap.value(node);
    double g = currentParam.g;
    return g;
}

double DstarLite::getrhs(const std::shared_ptr<InterSectionCell> &node)
{
    DStarParam currentParam = m_dStarMap.value(node);
    double rhs = currentParam.rhs;
    return rhs;
}

void DstarLite::setKey(const std::shared_ptr<InterSectionCell> &node, const std::tuple<double,double> &value)
{
    DStarParam currentParam = m_dStarMap.value(node);
    currentParam.key = value;
    m_dStarMap.insert(node, currentParam);
}

void DstarLite::setG(const std::shared_ptr<InterSectionCell> &node, const double value)
{
    DStarParam currentParam = m_dStarMap.value(node);
    currentParam.g = value;
    m_dStarMap.insert(node, currentParam);

}

void DstarLite::setrhs(const std::shared_ptr<InterSectionCell> &node, const double value)
{
    DStarParam currentParam = m_dStarMap.value(node);
    currentParam.rhs = value;
    m_dStarMap.insert(node, currentParam);
}

bool DstarLite::testValidityConstraints(std::vector<Constraint> &constraints, const std::vector<double>& controlVector) const {
    return false;
}

std::vector<Constraint> &DstarLite::getCurrentConstraints() {
    //
}

void DstarLite::clearAllConstraints() {
    //
}

void DstarLite::clearPrediction() {

}
