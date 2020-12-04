#include "pathitem.h"
#include <typeinfo>
#include <cmath>

/**
 * @brief PathItem::PathItem
 */
PathItem::PathItem() :
    m_x(0),
    m_y(0),
    m_t(0)
{
}

/**
 * @brief PathItem
 * @param x
 * @param y
 * @param t
 */
PathItem::PathItem(const int64_t &x1, const int64_t &x2, const double& t) :
  m_x(x1),
  m_y(x2),
  m_t(t)
{

}

/**
 * @brief PathItem::setCoordinates
 * @param x
 * @param y
 * @param t
 */
void PathItem::setCoordinates(const int64_t &x1, const int64_t &x2, const double& t) {
    m_x = x1;
    m_y = x2;
    m_t = std::ceil((int64_t)t);
}

/*void PathItem::setCoordinates(const std::vector<const unsigned int>& x, const double& t) {
    //Q_ASSERT_X(x.size() < 2, typeid(this).name(), "vector " + typeid(x).name() + " too small");
    m_x1 = x.at(0);
    m_x2 = x.at(1);
    m_t = t;
}*/

/**
 * @brief PathItem::getX
 * @return
 */
int64_t PathItem::getX() const {
    return m_x;
}

/**
 * @brief PathItem::getY
 * @return
 */
int64_t PathItem::getY() const {
    return m_y;
}

/**
 * @brief PathItem::getTime
 * @return
 */
double PathItem::getTime() const {
    return m_t;
}

/**
 * @brief PathItem::setX
 * @param x
 */
void PathItem::setX(const int64_t &x) {
    m_x = x;
}

/**
 * @brief PathItem::setY
 * @param y
 */
void PathItem::setY(const int64_t &y) {
    m_y =y;
}

/**
 * @brief PathItem::setX
 * is necessary for the optimizer to round to the upper or lower values
 * @param x
 */
void PathItem::setX(const double& x) {
    if (x > (double)m_x) {
        m_x = std::ceil(x);
    }
    else if (x < (double)m_x) {
        m_x = std::floor(x);
    }
}

/**
 * @brief PathItem::setY
 * @param y
 */
void PathItem::setY(const double& y) {
    if (y > (double)m_y) {
        m_y = std::ceil(y);
    }
    else if (y < (double)m_y) {
        m_y = std::floor(y);
    }
}

/**
 * @brief PathItem::setTime
 * @param t
 */
void PathItem::setTime(const double& t) {
    m_t = std::ceil((int64_t)t);
}

/**
 * @brief PathItem::operator - difference for boundaries, important for cost function
 * @param item
 * @return
 */
PathItem& PathItem::operator-(const PathItem& item) {
    m_x -= item.getX();
    m_y -= item.getY();
    m_t -= item.getTime();
    return *this;
}

/**
 * @brief PathItem::operator +
 * @param item
 * @return
 */
PathItem& PathItem::operator+(const PathItem& item) {
    m_x += item.getX();
    m_y += item.getY();
    m_t += item.getTime();
    return *this;
}

/**
 * @brief PathItem::abs
 */
void PathItem::abs() {
    m_x = std::abs(m_x);
    m_y = std::abs(m_y);
    m_t = std::abs(m_t);
}

/**
 * @brief PathItem::operator == returns true, if the coordinates (x,y) AND time are equal,
 * time is not necessary
 * @param item to compare
 * @return true, if (t,x,y) are equal, else false
 */
bool PathItem::operator==(const PathItem& item) const {
    return (getX() == item.getX() && getY() == item.getY() && getTime() == item.getTime());
}


/**
 * @brief PathItem::operator != returns false, if the coordinates (x,y) are equal,
 * time is not necessary
 * @param item item to compare
 * @return false, if (x,y) are equal, else false
 */
bool PathItem::operator!=(const PathItem& item) const {
    return (getX() != item.getX() || getY() != item.getY());
}

/**
 * @brief PathItem::isNeighbouredTo tests, if a given PathItem item is neigboured
 * with maximal distance given as parameter
 * @param item item to examine
 * @param distance maximal distance (<=)
 * @return
 */
bool PathItem::isNeighbouredTo(const PathItem& item, const unsigned int& distance) const {
    if (std::abs(getX() - item.getX()) <= distance && std::abs(getY() - item.getY()) <= distance ) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief operator <<
 * @param os
 * @param obj
 * @return
 */
/*std::ostream& operator<<(std::ostream& os, const PathItem& obj) {
    os << "PathItem: ";
    os << obj.getTime() << ", " << obj.getX() << ", " << obj.getY() << std::endl;
    return os;
}*/

/**
 * @brief QTextStream::operator <<
 * @param os
 * @param obj
 * @return
 */
/*QTextStream & QTextStream::operator<<(QTextStream& os, const PathItem& obj) {
    os << "PathItem: ";
    os << obj.getTime() << ", " << obj.getX() << ", " << obj.getY() << endl;
}*/
