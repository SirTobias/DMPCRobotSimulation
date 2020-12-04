#include "arrivalcar.h"
#include <time.h>


/**
 * @brief ArrivalCar::ArrivalCar
 * @param distrib choose the distribution function (here first, only poisson process)
 * @param meanArrivalTime mean amount of cars per hour
 * @param metric time metric
 */
ArrivalCar::ArrivalCar(const DistParam &distParam) :
    m_meanArrivalTime(distParam.meanArrivalTime)
{
    std::random_device rd;
    m_merTwister = std::mt19937(rd());
    ///cars per minute
    if (distParam.distribFunc == DistributionFunction::EXP) {
        //m_distribution = std::exponential_distribution<double>(meanArrivalTime);
        //TODO: make template of parameter for later purposes
    }
    else if (distParam.distribFunc == DistributionFunction::POISSON) {
        double lambda = distParam.meanArrivalTime / ((double)distParam.metric * (1.0 / distParam.samplingTime));
        m_distribution = std::poisson_distribution<int>(lambda);
    }
}

/**
 * @brief ArrivalCar::arrivedCarsPerMinute returns the number of arrived cars for the specified interval created in the constructor
 * @return number of cars to create
 */
double ArrivalCar::arrivedCarsPerMinute() {
    double numberCars = (double)m_distribution.operator ()(m_merTwister);
    return numberCars;
}
