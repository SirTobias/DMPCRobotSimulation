#ifndef PATHITEM_H
#define PATHITEM_H
#include <stdint.h>
#include <QtCore/QTextStream>


/**
 * @brief The PathItem class holds a position and time for a path. The path itself consists of many PathItems
 */
class PathItem
{
public:
    PathItem();
    PathItem(const int64_t& x1, const int64_t& x2, const double& t);
    void setCoordinates(const int64_t& x1, const int64_t& x2, const double& t);
    //void setCoordinates(const std::vector<const unsigned int>& x, const double& t);
    int64_t getX() const;
    int64_t getY() const;
    double getTime() const;
    void setX(const int64_t& x);
    void setY(const int64_t& y);
    void setX(const double& x);
    void setY(const double& y);
    void setTime(const double& t);
    void abs();
    PathItem& operator-(const PathItem& item);
    PathItem& operator+(const PathItem& item);
    bool operator==(const PathItem& item) const;
    bool operator!=(const PathItem& item) const;
    bool isNeighbouredTo(const PathItem& item, const unsigned int &distance = 1) const;
    //friend std::ostream& operator<<(std::ostream& os, );

private:
    ///x-coord for the grid
    int64_t m_x;
    ///y-coord for the grid
    int64_t m_y;
    ///time when the grid is taken
    double m_t;
};

//QTextStream& QTextStream::operator<<(QTextStream& os, const PathItem& obj);

#endif // PATHITEM_H
