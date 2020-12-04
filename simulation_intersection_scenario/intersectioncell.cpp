#include "intersectioncell.h"

#include "intersectionparameters.h"

/**
 * @brief InterSectionCell::InterSectionCell
 * @param x position horizontal
 * @param y position vertical
 */
InterSectionCell::InterSectionCell(const unsigned int &x, const unsigned int &y) :
    m_x(x),
    m_y(y),
    tentativeGCost(0.0),
    fCost(0.0)
    //rhs(5000.0),
    //g(5000.0),
    //key(100.0)
{
}

/**
 * @brief InterSectionCell::isTimeReserved
 * looks up for a given time, if the cell is already reserved
 * @param time time for which the function should look up
 * @return
 */
bool InterSectionCell::isTimeReserved(const double &time) const {
    QVector<ReservationCell>::const_iterator it = m_reserved.begin();
    while (it != m_reserved.end()) {
        if ((*it).getTime() == time) {
            return true;
            break;
        }
        it++;
    }
    return false;
}

/**
 * @brief InterSectionCell::getTimeForCar
 * @param car string of the car
 * @return
 */
double InterSectionCell::getTimeForCar(const QString& car, const double &t) const {
    QVector<ReservationCell>::const_iterator it = m_reserved.begin();
    double time = -1.0;
    while (it != m_reserved.end()) {
        if ((*it).getCar() == car && (*it).getTime() >= t) {
            time = (*it).getTime();
            break;
        }
        it++;
    }
    return time;
}

/**
 * @brief getPrelimTimeForCar
 * @param car
 * @return
 */
double InterSectionCell::getPrelimTimeForCar(const QString& car, const double& t) const {
    QVector<ReservationCell>::const_iterator it = m_prelimReserved.begin();
    double time = -1;
    while (it != m_prelimReserved.end()) {
        if ((*it).getCar() == car && (*it).getTime() >= t) {
            time = (*it).getTime();
            break;
        }
        it++;
    }
    return time;
}

/**
 * @brief InterSectionCell::isPrelimTimeReserved looks up, if for a given time
 * the cell is reserved
 * @param time given time which is been asked for
 * @return true, if time slot is free, otherwise false
 */
bool InterSectionCell::isPrelimTimeReserved(const double &time) const {
    QVector<ReservationCell>::const_iterator it = m_prelimReserved.begin();
    while (it != m_prelimReserved.end()) {
        if ((*it).getTime() == time) {
            return true;
            break;
        }
        it++;
    }
    return false;
}

/**
 * @brief InterSectionCell::isTimeAlreadyReserved returns true, if a time greaterequal than given time is reserved
 * @param car
 * @param time reserved time greaterequal than this (lower bound
 * @return
 */
bool InterSectionCell::isTimeAlreadyReserved(const QString& car, const double &time) const {
    QVector<ReservationCell>::const_iterator it = m_reserved.begin();
    bool found = false;
    while (it != m_reserved.end()) {
        if ((*it).getCar() == car && (*it).getTime() >= time) {
            found = true;
            break;
        }
        it++;
    }
    return found;
}

/**
 * @brief InterSectionCell::isPrelimTimeAlreadyReserved if a preliminary time >= than given time is reserved
 * @param car
 * @param time lower bound
 * @return true, if a prelim. time which is greater than given time is found, otherwise false
 */
bool InterSectionCell::isPrelimTimeAlreadyReserved(const QString& car, const double& time) const {
    QVector<ReservationCell>::const_iterator it = m_prelimReserved.begin();
    bool found = false;
    while (it != m_prelimReserved.end()) {
        if ((*it).getCar() == car && (*it).getTime() >= time) {
            found = true;
            break;
        }
        it++;
    }
    return found;
}

/**
 * @brief InterSectionCell::reserveTimeForCar reserve time slot for given car
 * @param car for which the time should be reserved
 * @param time given time, which should be reserved
 * @return true, if time is reserved, false, if time is already reserved
 */
bool InterSectionCell::reserveTimeForCar(const QString &car, const double &time) {
    QVector<ReservationCell>::const_iterator it = m_reserved.begin();
    while (it != m_reserved.end()) {
        if ((*it).getTime() == time) {
            return false;
            break;
        }
        it++;
    }
    m_reserved.append(ReservationCell(car, time));
    return true;
}

/**
 * @brief removeCar
 * @param car
 * @return true, if car was found and is removed, otherwise false
 */
bool InterSectionCell::removeCar(const QString &car) {
    QVector<ReservationCell>::const_iterator it = m_reserved.begin();
    bool removed = false;
    unsigned int count = 0;
    while (it != m_reserved.end()) {
        if ((*it).getCar() == car) {
            m_reserved.removeAt(count);
            removed = true;
            break;
        }
        it++;
        count++;
    }
    return removed;
}

/**
 * @brief InterSectionCell::getNextFreeTime
 * @param start
 * @return
 */
double InterSectionCell::getNextFreeTime(const double &start) const {
    double firstFreeTime = start;
    while (isTimeReserved(firstFreeTime)) {
        firstFreeTime += InterSectionParameters::T;
    }
    return firstFreeTime;
}

/**
 * @brief InterSectionCell::getNextFreePrelimTime
 * @param start
 * @return
 */
double InterSectionCell::getNextFreePrelimTime(const double &start) const {
    double firstFreeTime = start;
    while (isPrelimTimeReserved(firstFreeTime)) {
        firstFreeTime += InterSectionParameters::T;
    }
    return firstFreeTime;
}

/**
 * @brief InterSectionCell::getX
 * @return
 */
double InterSectionCell::getX() const {
    return m_x;
}

/**
 * @brief InterSectionCell::getY
 * @return
 */
double InterSectionCell::getY() const {
    return m_y;
}

/**
 * @brief InterSectionCell::reserveNextFreeTimeForCar
 * @param car
 * @param time
 * @return
 */
double InterSectionCell::reserveNextFreeTimeForCar(const QString& car, const double &time) {
    double firstFreeTime = getNextFreeTime(time);
    if (firstFreeTime != -1.0) {
        reserveTimeForCar(car, firstFreeTime);
    }
    return firstFreeTime;
}

/**
 * @brief InterSectionCell::reservePremlimNextTime
 * @param car
 * @param time
 * @return
 */
bool InterSectionCell::reservePremlimNextTime(const QString& car, const double& time) {
    QVector<ReservationCell>::const_iterator it = m_prelimReserved.begin();
    while (it != m_prelimReserved.end()) {
       if ((*it).getTime() == time) {
           return false;
           break;
       }
       it++;
    }
    m_prelimReserved.append(ReservationCell(car, time));
    return true;
}

/**
 * @brief InterSectionCell::operator ==
 * @param first
 * @param second
 * @return
 */
bool InterSectionCell::operator==(const std::shared_ptr<InterSectionCell> compCell) const{
    //if (!compCell.expired()) {
        //std::shared_ptr<InterSectionCell> compCellP(compCell);
        return (getX() == compCell->getX()) && (getY() == compCell->getY());
   // }
   // return false;
}


void InterSectionCell::removePrelimPathFromCar(const QString& car) {
    QVector<ReservationCell>::iterator it = m_prelimReserved.begin();
    while (it != m_prelimReserved.end()) {
        if ((*it).getCar() == car) {
            ReservationCell& carDel = (*it);
            int index = m_prelimReserved.indexOf(carDel);
            m_prelimReserved.remove(index);
        }
        else {
            it++;
        }
    }
}

/**
 * @brief InterSectionCell::replacePrelimWithReservedPositions removes all items from m_prelimReserved and copy the items from m_reserved to it
 * @return amount of copied items
 */
unsigned int InterSectionCell::replacePrelimWithReservedPositions() {
    m_prelimReserved.clear();
    m_prelimReserved.reserve(m_reserved.size());
    for (ReservationCell& reserve : m_reserved) {
        m_prelimReserved.push_back(reserve);
    }
    return m_prelimReserved.size();
}

/** @brief will get the number of reservations the cell has
 *  @return int
 */
unsigned int InterSectionCell::getNumOfReservations()
{
    return m_reserved.size();
}

/**
 * @brief InterSectionCell::getReservationCells get the reservation queue
 * @return reservation queue
 */
const QVector<ReservationCell>& InterSectionCell::getReservationCells() const {
    return m_reserved;
}

/**
 * @brief InterSectionCell::getPrelimReservationCells get the prelimiated reservation queue
 * @return prelimination queue
 */
const QVector<ReservationCell> &InterSectionCell::getPrelimReservationCells() const {
    return m_prelimReserved;
}

double InterSectionCell::getTentativeGCost() const
{
    return tentativeGCost;
}

void InterSectionCell::setTentativeGCost(double value)
{
    tentativeGCost = value;
}

double InterSectionCell::getFCost() const
{
    return fCost;
}

void InterSectionCell::setFCost(double value)
{
    fCost = value;
}

std::weak_ptr<InterSectionCell> InterSectionCell::getParent() const
{
    return parent;
}

void InterSectionCell::setParent(const std::weak_ptr<InterSectionCell> &value)
{
    parent = value;
}

/*double InterSectionCell::getRhs() const
{
    return rhs;
}

void InterSectionCell::setRhs(double value)
{
    rhs = value;
}

double InterSectionCell::getG() const
{
    return g;
}

void InterSectionCell::setG(double value)
{
    g = value;
}

double InterSectionCell::getKey() const
{
    return key;
}

void InterSectionCell::setKey(double value)
{
    key = value;
}

bool compare(const std::shared_ptr<InterSectionCell> left, const std::shared_ptr<InterSectionCell> right)
{
    return left->getFCost() < right->getFCost();
}*/




