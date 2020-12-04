#include "reservationcell.h"

/**
 * @brief reservationCell::reservationCell
 * @param car
 * @param time
 */
ReservationCell::ReservationCell(const QString &car, const double &time) :
    m_car(car),
    m_time(time)
{
}

/**
 * @brief ReservationCell::ReservationCell
 */
ReservationCell::ReservationCell() :
    m_car(""),
    m_time(0)
{

}

/**
** * @brief reservationCell::getCar
** * @return
*/
QString ReservationCell::getCar() const {
    return m_car;
}

/**
 * @brief reservationCell::getTime
 * @return
 */
double ReservationCell::getTime() const {
    return m_time;
}

bool ReservationCell::operator==(const ReservationCell& cell) const {
    return (getCar() == cell.getCar() && getTime() == cell.getTime());
}
