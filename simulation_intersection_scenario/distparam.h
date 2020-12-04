#ifndef DISTPARAM_H
#define DISTPARAM_H

/**
 * @brief The DistributionFunction enum
 */
enum class DistributionFunction {
    POISSON = 0,
    EXP = 1
};

/**
 * @brief The TimeMetric enumerates which metric is the divisor, so e.g.
 * x cars arriving in 1 minute, this will be \f$ \lambda = \frac{\mu * time instant }{TimeMetric * \frac{1}{T}}\f$,
 * where \mu is the expectation value, TimeMetric the time interval and T the sampling interval
 */
enum class TimeMetric {
    SECONDS = 3600,
    DEZSECONDS = 10,
    MINUTES = 60,
    HOURS = 1
};

/**
 * @brief The DistParam struct holds the parameter for the distribution function
 */
struct DistParam {
    DistributionFunction distribFunc;
    double meanArrivalTime;
    TimeMetric metric;
    double samplingTime;
};


#endif // DISTPARAM_H
