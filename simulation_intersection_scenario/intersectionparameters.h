#ifndef INTERSECTIONPARAMETERS_H
#define INTERSECTIONPARAMETERS_H

#include <cstddef>

/**
 * @brief The SystemFunctionUsage enum distinguish on the one hand the discrete trajectory case or the continuous case
 *
 */
enum class SystemFunctionUsage {
    DISCRETE = 0,
    CONTINUOUS = 1
};

/**
 * @brief The CommunicationScheme enum distinguish either the Full quantised communication scheme, the differential scheme which uses
 * updated cells or the Min/Max-Cell
 */
enum class CommunicationScheme {
    FULL = 0,
    DIFFERENTIAL = 1,
    MINMAXINTERVAL = 2,
    MINMAXINTERVALMOVING = 3,
    CONTINUOUS = 4
};

/**
 * @brief The SpacialSet enum describes either the usage of quantisation or continuous communication
 */
enum class SpacialSet {
    QUANTISED = 1,
    CONTINUOUS = 2
};

class InterSectionParameters {
public:
static constexpr unsigned int maxCars = 4;
static constexpr unsigned int k = 12;
static constexpr unsigned int m = 12;
static constexpr size_t N = 12;
static constexpr double T = 0.5;
static constexpr unsigned int vectorDimension = 2;
static constexpr double lambda = 0.2;
static constexpr double weightGlobalLiveTime = 0.2;
static constexpr double weightGlobalWaitTime = 0.2;
static constexpr double weightReservRequests = 0.2;
static constexpr double weightTimeNextCell = 0.2;
static constexpr double weightExternalReservRequests = 0.2;
static constexpr unsigned int directComm = 0;
static constexpr SystemFunctionUsage sysFuncUsage = SystemFunctionUsage::CONTINUOUS;
static constexpr unsigned int varyCellSize = 1;
static constexpr double robotDiameter = 0.5;
static constexpr unsigned int intersectionalScenario = 0;
static constexpr unsigned int stochasticArrival = 0;
};

#endif // INTERSECTIONPARAMETERS_H
