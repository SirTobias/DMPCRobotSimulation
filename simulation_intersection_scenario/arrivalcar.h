#ifndef ARRIVALCAR_H
#define ARRIVALCAR_H

#include "distparam.h"
#include <random>


class ArrivalCar
{
public:
    ArrivalCar(const DistParam& distParam);
    double arrivedCarsPerMinute();
private:
    ///creator function for arrival events
    std::poisson_distribution<int> m_distribution;
    ///mersanne twister for random numbers
    std::mt19937 m_merTwister;
    ///mean arrival numbe of cars per hour
    double m_meanArrivalTime;
    ///current time
    double m_t0;

};

#endif // ARRIVALCAR_H
