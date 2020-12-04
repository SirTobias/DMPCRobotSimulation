#ifndef RESERVATIONCELL_H
#define RESERVATIONCELL_H
#include <QtCore/QString>

/**
 * @brief stores the information for which time a car the cell has reserved
 */
class ReservationCell
{
public:
    ReservationCell(const QString &car, const double &time);
    ReservationCell();
    QString getCar() const;
    double getTime() const;
    bool operator==(const ReservationCell& cell) const;
private:
    ///ID of the car
    QString m_car;
    ///time for which the cell is reserved
    double m_time;
};

#endif // RESERVATIONCELL_H
