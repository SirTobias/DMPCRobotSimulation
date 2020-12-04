#include "intersection.h"
#include "astarpathcalculation.h"

#include <QtTest/QTest>

/**
 * @brief InterSection::InterSection constructs the intersection with the intersection grid
 * constructing a \f$\frac{width}{cellSize}*\frac{height}{cellsize}\f$ intersection grid
 * @param width of the grid
 * @param height of the grid
 * @param cellSize cellSize of one cell (only squared are allowed)
 * @param alg path algorihm to choose (TODO)
 */
InterSection::InterSection(const unsigned int &width, const unsigned int &height, const double cellSize) :
    m_width(width),
    m_height(height),
    m_gridSizeWidth(std::ceil((double)width / cellSize)),
    m_gridSizeHeight(std::ceil((double)height / cellSize)),
    m_cellSize(cellSize)
{
    interSection = std::shared_ptr<InterSection>(this);
}

/**
 * @brief InterSection::buildGrid constructs the grid according to \f$ \frac{width}{cellSize}*\frac{height}{cellSize}\f$
 * @param width width of grid (k)
 * @param height height of grid (m)
 * @param cellSize (cellsize, have to be > 0
 */
void InterSection::buildGrid(const unsigned int &width, const unsigned int &height, const double cellSize) {
    if (m_gridMap) {
        m_gridMap.reset();
    }
    m_cellSize = cellSize;
    Q_ASSERT_X(m_width > 0, typeid(this).name(), "width must > 0");
    Q_ASSERT_X(m_height > 0, typeid(this).name(), "height must > 0");
    Q_ASSERT_X(m_cellSize > 0, typeid(this).name(), "cellsize must > 0");
    m_gridMap = std::make_shared<InterSectionGrid>();
    //if grid size varies, -x_min,x_max stays constant, so amount of grid cells varies
    m_gridSizeWidth = std::ceil((double)width / cellSize);
    m_gridSizeHeight = std::ceil((double)height / cellSize);
    for (unsigned int i = 0; i < m_gridSizeWidth; i++) {
        m_gridMap->insert(i, QHash<unsigned int, std::shared_ptr<InterSectionCell> >());
        for (unsigned int j = 0; j < m_gridSizeHeight; j++) {
            m_gridMap->operator [](i).insert(j, std::make_shared<InterSectionCell>(i, j));
        }
    }
}

/**
 * @brief getNeighbours get a map with references on the cells which are neighboured around the given cell
 * @param cell InterSectionCell which should be examined
 * @param radius max. distance to neighbours (Moore-Neighbourhood)
 * @return list with InterSectionCells surrounding given cell
 */
QList<std::weak_ptr<InterSectionCell> > InterSection::getNeighbours(const std::weak_ptr<InterSectionCell>& cell, const unsigned int& radius) const {
    QList<std::weak_ptr<InterSectionCell> > neighbourMap;
    if (!cell.expired()) {
        std::shared_ptr<InterSectionCell> cellP(cell);
        unsigned int x = cellP->getX();
        unsigned int y = cellP->getY();
        //Sonderfall: x||y == 0
        if (x == 0 || y == 0) {
            if (x == 0 && y == 0) {
                for (unsigned int i = x; i <= x+radius; i++) { // i!=y+radius
                    for (unsigned int j = y; j <= y+radius; j++) {// j!=x+radius
                        if (m_gridMap->contains(i)) {
                            QHash<unsigned int, std::shared_ptr<InterSectionCell> > row = m_gridMap->value(i);
                            if (row.contains(j)){ //&& i == y && j == x) {
                                neighbourMap.append(row.value(j));
                            }//contains cell
                        }//contains row
                    }//col
                }//row
            }//x && y == 0
            else if (x == 0) {
                for (unsigned int i = x ; i <= x+radius; i++) {// same
                    for (unsigned int j = y-1; j <= y+radius; j++) {// same
                        if (m_gridMap->contains(i)) {
                            QHash<unsigned int, std::shared_ptr<InterSectionCell> > row = m_gridMap->value(i);
                            if (row.contains(j)) { //&& i == y && j == x) {
                                neighbourMap.append(row.value(j));
                            }//contains cell
                        }//contains row
                    }//col
                }//row
            } //x == 0
            else if (y == 0) {
                for (unsigned int i = x-1; i <= x+radius; i++) {// same
                    for (unsigned int j = y; j <= y+radius; j++) {// same
                        if (m_gridMap->contains(i)) {
                            QHash<unsigned int, std::shared_ptr<InterSectionCell> > row = m_gridMap->value(i);
                            if (row.contains(j)){ //&& i == y && j == x) {
                                neighbourMap.append(row.value(j));
                            }//contains cell
                        }//contains row
                    }//col
                }//row
            }//y==0
        }//x || y == 0
        else {
            //first look, if row is included, then get the cell
            for (unsigned int i = x - 1; i <= x+radius; i++) {// same
                for (unsigned int j = y - 1; j <= y+radius; j++) { //same
                    if (m_gridMap->contains(i)) {
                        QHash<unsigned int, std::shared_ptr<InterSectionCell> > row = m_gridMap->value(i);
                        if (row.contains(j)){// && i == y && j == x) {
                            neighbourMap.append(row.value(j));
                        }//contains cell
                    }//contains row
                }//col
            }//row
        }//normal case
    }
    return neighbourMap;
}

/**
 * @brief InterSection::isNeighboured tests if two cells are neighboured
 * @param first cell to examine
 * @param second cell to examine
 * @return true, if neighboured, otherwise false
 */
bool InterSection::isNeighboured(const std::weak_ptr<InterSectionCell>& first, const std::weak_ptr<InterSectionCell>& second) const {
    bool ret = false;
    if (!first.expired() && !second.expired()) {
        std::shared_ptr<InterSectionCell> firstP(first);
        std::shared_ptr<InterSectionCell> secondP(second);
        QList<std::weak_ptr<InterSectionCell> > neighbourMap = getNeighbours(first, 1);
        for (QList<std::weak_ptr<InterSectionCell> >::const_iterator it = neighbourMap.begin(); it != neighbourMap.end(); it++) {
            std::shared_ptr<InterSectionCell> neighbourCur(*it);
            if (neighbourCur->getX() == secondP->getX() && neighbourCur->getY() == neighbourCur->getY()) {
                ret = true;
                break;
            }
        }
    }
    return ret;
}

/**
 * @brief InterSection::setEntryPoints, first, the standard case with a 2x2-intersection, half of the lanes are allowed
 * @param distParam Parameter for distribution function, mean time,
 */
void InterSection::setEntryPointsStandardLanes(const std::vector<DistParam>& distParam) {
    unsigned int counter = 0;
    //top left
    for (int i = 0; i < std::floor((double)m_gridSizeWidth / 2.0); i++) {
        m_entryPoints[counter] = getInterSectionCell(i, 0);
        counter++;
    }
    //top right
    for (int i = 0; i < std::floor((double)m_gridSizeHeight / 2.0); i++) {
        m_entryPoints[counter] = getInterSectionCell(m_gridSizeWidth - 1, i);
        counter++;
    }
    //bottom right
    for (int i = std::ceil((double)m_gridSizeWidth / 2.0 ); i < m_gridSizeWidth; i++) {
        m_entryPoints[counter] = getInterSectionCell(i, m_gridSizeHeight - 1);
        counter++;
    }
    //bottom left
    for (int i = std::ceil((double)m_gridSizeHeight / 2.0 ); i < m_gridSizeHeight; i++) {
        m_entryPoints[counter] = getInterSectionCell(0, i);
        counter++;
    }
    for (unsigned int i = 0; i < numberEntryPoints(); i++) {
        if (distParam.size() > i) {
            m_enterInterSectDist.push_back(ArrivalCar(distParam.at(i)));
        }
        else {
            qDebug() << "not enough distribution parameter, using first as default";
            m_enterInterSectDist.push_back(ArrivalCar(distParam.at(0)));
        }
    }
}

/**
 * @brief InterSection::getEntryPoint
 * @param index index of the to obtained entry point
 * @return
 */
std::shared_ptr<InterSectionCell> InterSection::getEntryPoint(const unsigned int& index) {
    return m_entryPoints.at(index);
}

/**
 * @brief InterSection::numberEntryPoints
 * @return the number of entry points
 */
unsigned int InterSection::numberEntryPoints() const {
    return m_entryPoints.size();
}

/**
 * @brief InterSection::getSuccessors determine the following successors from the start node to the target node
 * normally in a grid these are the vertical, horizontal and diagonal cell in direction to the target (3 Cells)
 * @param start from this intersection cell the expansion starts
 * @param target the path to this cell should be found
 * @return path, containing intersection cells
 */
QList<std::weak_ptr<InterSectionCell> > InterSection::getSuccessors(const std::weak_ptr<InterSectionCell>& start, const std::weak_ptr<InterSectionCell>& target) const {
    QList<std::weak_ptr<InterSectionCell> > successors;
    if (!start.expired() && !target.expired()) {
        std::shared_ptr<InterSectionCell> startP(start);
        std::shared_ptr<InterSectionCell> targetP(target);
        //ASSERT: target != start
        Q_ASSERT_X(startP->getX() == targetP->getX() && startP->getY() == targetP->getY(), typeid(this).name(), "start == target");
        //Differenzen zur Richtung bestimmen
        //double diffX = targetP->getX() - startP->getX();
        //double diffY = targetP->getY() - startP->getY();
        if (targetP->getX() > startP->getX()) {
            //diagonal path positive
            if (targetP->getY() > startP->getY()) {
                //TODO
            }
            //horizontal path negative
            else if (targetP->getY() < startP->getY()) {
                //TODO
            }

        }
        //diagonal path negative
        else if (targetP->getX() < startP->getX()) {
            //TODO
        }
    }
    return successors;
}

/**
 * @brief InterSection::reserveTimeForCar tries to reserve given time of car at intersection cell(k,m)
 * return false, if time cannot be reserved
 * @param car string of car
 * @param k x-coordinate
 * @param m y-coordinate
 * @param t time
 * @return true, if it is reserved, otherwise false
 */
bool InterSection::reserveTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t) {
    bool reserved = false;
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    if (interSectionCell) {
        reserved = interSectionCell->reserveTimeForCar(car, t);
    }
    return reserved;
}

/**
 * @brief InterSection::reserveTimeForCar reserve next free time at intersection cell (k,m) for the given car
 * @param k x-coordinate
 * @param m y-coordinate
 * @param t time
 * @return true, if a next time can be reserved, otherwise false
 */
double InterSection::reserveNextFreeTimeForCar(const QString& car, const unsigned int& k, const unsigned int& m, const double& t) {
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    double reservedTime = -1;
    if (interSectionCell) {
        reservedTime = interSectionCell->reserveNextFreeTimeForCar(car, t);
    }
    return reservedTime;
}

/**
 * @brief tryReserveTimeForCar reserve preliminary the next cell for the car
 * @param car ID of the car
 * @param k first coordinate (width)
 * @param m second coordinate (height)
 * @param t time that should be reserved
 * @return time that is reserved, if no time available, -1.0 is returned
 */
double InterSection::tryReserveTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t) {
    double reservedTime = -1.0;
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    if (interSectionCell) {
        //time is already reserved, nothing to to anymore
        if (interSectionCell->isTimeAlreadyReserved(car, t)) {
            return interSectionCell->getTimeForCar(car, t);
        }
        //get next free time and reserve preliminary
        double nextFreeTime = interSectionCell->getNextFreeTime(t);
        if (nextFreeTime > t && nextFreeTime < DBL_MAX) {
            reservedTime = interSectionCell->reservePremlimNextTime(car, nextFreeTime);
        }
    }
    return reservedTime;
}

/**
 * @brief tryReserveTimeForCar reserve preliminary time for a car, important for taking an optimization step
 * this reservation is not final dependant on disturbation or reservation from other cars
 * @param car identity of the car
 * @param k row coordinate
 * @param m column coordinate
 * @param t time
 * @return
 */
double InterSection::tryReservePrelimTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t) {
    double reservedTime = -1;
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    if (interSectionCell) {
        //time is reserved in preliminary or in the normal reserved queue
        if (interSectionCell->isPrelimTimeAlreadyReserved(car, t)) {
            return interSectionCell->getPrelimTimeForCar(car, t);
        }
        else if (interSectionCell->isTimeAlreadyReserved(car, t)) {
            return interSectionCell->getTimeForCar(car, t);
        }
        //get next free time and reserve preliminary
        double nextFreeTime = interSectionCell->getNextFreePrelimTime(t);
        //greater equal because the car can reserve the cell just in time
        if (nextFreeTime >= t && nextFreeTime < DBL_MAX) {
            reservedTime = interSectionCell->reservePremlimNextTime(car, nextFreeTime);
        }
    }
    return reservedTime;
}

/**
 * @brief InterSection::getPossibleReserveTimeForCar
 * @param k
 * @param m
 * @param t
 * @return
 */
double InterSection::getPossibleReserveTimeForCar(const unsigned int& k, const unsigned int& m, const double& t) const {
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    double nextFreeTime = -1.0;
    if (interSectionCell) {
        nextFreeTime = interSectionCell->getNextFreeTime(t);
    }
    return nextFreeTime;
}

/**
 * @brief InterSection::getPossiblePrelimReserveTimeForCar
 * @param k
 * @param m
 * @param t
 * @return
 */
double InterSection::getPossiblePrelimReserveTimeForCar(const unsigned int& k, const unsigned int& m, const double& t) const {
    std::shared_ptr<InterSectionCell> interSectionCell = getInterSectionCell(k, m);
    double nextFreeTime = -1.0;
    if (interSectionCell) {
        nextFreeTime = interSectionCell->getNextFreePrelimTime(t);
    }
    else if (k >= getWidth() || m >= getHeight()) {
        nextFreeTime = 1000;
    }
    return nextFreeTime;
}

/**
 * @brief InterSection::isTimeAlreadyReserved
 * @param cell
 * @param car
 * @return
 */
bool InterSection::isTimeAlreadyReserved(std::shared_ptr<InterSectionCell> cell, const QString& car, const double& time) const {
    //Q_ASSERT_X(cell == NULL)
    return cell->isTimeAlreadyReserved(car, time);
}

/**
 * @brief InterSection::getGrid
 * @return
 */
std::shared_ptr<InterSectionGrid> InterSection::getGrid() const {
    return m_gridMap;
}

/**
 * @brief InterSection::getInstance
 * @return
 */
std::shared_ptr<InterSection> InterSection::getInstance() {
    if (interSection == nullptr) {
        //TODO: hier noch variable Parameter festlegen
        interSection = std::make_shared<InterSection>(4, 4, 1.0);
    }
    return interSection;
}

/**
 * @brief InterSection::getInterSectionCell returns the according intersection cell with (k,m)
 * if cellsize > 1, it will be mapped by k/c and m/c
 * @param k width
 * @param m height
 * @return
 */
std::shared_ptr<InterSectionCell> InterSection::getInterSectionCell(const unsigned int& k, const unsigned int& m) {
    //TODO: Q_ASSERT_X - falls Zelle nicht vorhanden
    std::shared_ptr<InterSectionCell> cell = nullptr;
    //gridsize bigger, less cells available
    /*if (m_gridSizeWidth < m_width|| m_gridSizeHeight < m_height) {
        unsigned int kReduced = std::ceil((double)k / m_cellSize);
        unsigned int mReduced = std::ceil((double)m / m_cellSize);
        if (getGrid() && getGrid()->contains(kReduced) && getGrid()->operator [](kReduced).contains(mReduced)) {
            cell = getGrid()->operator[](kReduced).operator[](mReduced);
        }
    //}
    //else */
    if (getGrid() && getGrid()->contains(k) && getGrid()->operator [](k).contains(m)) {
        cell = getGrid()->operator[](k).operator[](m);
    }
    return cell;
}

/**
 * @brief InterSection::getInterSectionCell const version of InterSection::getInterSectionCell
 * if cellsize > 1, it will be mapped by k/c and m/c
 * @param k width
 * @param m height
 * @return
 */
std::shared_ptr<InterSectionCell> InterSection::getInterSectionCell(const unsigned int& k, const unsigned int& m) const {
    //TODO: Q_ASSERT_X - falls celle nicht vorhanden
    std::shared_ptr<InterSectionCell> cell = nullptr;
    /*if (m_gridSizeWidth < m_width|| m_gridSizeHeight < m_height) {
        unsigned int kReduced = std::ceil((double)k / m_cellSize);
        unsigned int mReduced = std::ceil((double)m / m_cellSize);
        if (getGrid() && getGrid()->contains(kReduced) && getGrid()->operator[] (kReduced).contains(mReduced)) {
            cell = getGrid()->operator[](kReduced).operator[](mReduced);
        }
    }*/
    //else
    if (getGrid() && getGrid()->contains(k) && getGrid()->operator [](k).contains(m)) {
        cell = getGrid()->operator[](k).operator[](m);
    }
    return cell;
}

/**
 * @brief InterSection::getHeight
 * @return
 */
unsigned int InterSection::getHeight() const {
   return m_height;
}

/**
 * @brief InterSection::getWidth
 * @return
 */
unsigned int InterSection::getWidth() const {
    return m_width;
}

/**
 * @brief InterSection::getGridHeight
 * @return
 */
unsigned int InterSection::getGridHeight() const {
    return m_gridSizeHeight;
}

/**
 * @brief InterSection::getGridWidth
 * @return
 */
unsigned int InterSection::getGridWidth() const {
    return m_gridSizeWidth;
}

/**
 * @brief InterSection::clearPrelimPathOfCar
 * @param car
 * @param path
 */
void InterSection::clearPrelimPathOfCar(const QString& car, const Path& path) {
    for (const PathItem& pathItem : path) {
        std::shared_ptr<InterSectionCell> cell = getInterSectionCell(pathItem.getX(), pathItem.getY());
        if (cell) {
            cell->removePrelimPathFromCar(car);
        }
    }
}

void InterSection::copyCurrentPositionsToPreliminaries() {
    for (unsigned int k = 0; k < m_width; k++) {
        for (unsigned int m = 0; m < m_height; m++) {
            std::shared_ptr<InterSectionCell> cell = getInterSectionCell(k, m);
            if (cell) {
                cell->replacePrelimWithReservedPositions();
            }
        }
    }
}

/** get number of reservations in a cell
 * @param k
 * @param m
 * @return
 */
unsigned int InterSection::getNumberOfReservations(const unsigned int k, const unsigned int m)
{
    std::shared_ptr<InterSectionCell> cell = getInterSectionCell(k, m);
    return cell->getNumOfReservations();
}

/**
 * @brief InterSection::printReservations
 */
void InterSection::printReservations() const {
    for (unsigned int k = 0; k < m_width; k++) {
        for (unsigned int m = 0; m < m_height; m++) {
            std::cout << "Cell: " << k << ", " << m << std::endl;
            std::shared_ptr<InterSectionCell> cell = getInterSectionCell(k, m);
            const QVector<ReservationCell>& reservQueue = cell->getReservationCells();
            std::cout << "reserved: " << std::endl;
            for (const ReservationCell& resCell: reservQueue) {
                std::cout << "(" << resCell.getCar().toStdString() << ", " << resCell.getTime() << "), ";
            }
            std::cout << std::endl;
            const QVector<ReservationCell>& prelimQueue = cell->getPrelimReservationCells();
            std::cout << "prelim.: " << std::endl;
            for (const ReservationCell& preCell : prelimQueue) {
                std::cout << "(" << preCell.getCar().toStdString() << ", " << preCell.getTime() << "), ";
            }
            std::cout << std::endl;
        }
    }
}

/**
 * @brief InterSection::getCellFromCoordinates returns the cell which is mapped to the coordinates
 * regarding u as the direction to regard the boundary problem
 * so if the coordinates are on the boundary of
 * @param x the coordinates in x_1,x_2-form
 * @return reference to the appropriate intersection cell
 */
std::shared_ptr<InterSectionCell> InterSection::getCellFromCoordinates(const std::vector<double> &x) {
    int w = std::floor(x.at(0) / getCellSize());
    int h = std::floor(x.at(1) / getCellSize());
    return getInterSectionCell(w, h);
}

/**
 * @brief InterSection::setCellSize
 * @param cellSize
 */
void InterSection::setCellSize(const double& cellSize) {
    m_cellSize = cellSize;
}

/**
 * @brief getCellSize
 * @return
 */
double InterSection::getCellSize() const {
    return m_cellSize;
}

unsigned int InterSection::getAmountOfCarsForNextTime(const unsigned int &entryPoint) {
    //Q_ASSERT_X()
    return m_enterInterSectDist.at(entryPoint).arrivedCarsPerMinute();
}
