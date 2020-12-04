#ifndef LINEARCONGRUENTIALGENERATOR_H
#define LINEARCONGRUENTIALGENERATOR_H
#include <stdint.h>

#include "simsharedlib.h"

/**
 * @brief The linearCongruentialGenerator class is the implementation for an linear congruential number generator
 *
 */
class SIM_CORE_EXPORT LinearCongruentialGenerator
{
public:
    LinearCongruentialGenerator(const uint64_t &seed, const uint64_t &startInterval, const uint64_t &endInterval);
    ~LinearCongruentialGenerator();
    void init(const uint64_t &seed = 0, const uint64_t &startInterval = 0, const uint64_t &endInterval = 0);
    void createArrayWithPrim(uint64_t *&mod, const int &length);
    double getSeededStartValue();
    double getRandom(const bool &forceSumUp = false);
    int getState();
protected:
    double getCpuLoad();
    uint64_t smallestValue( uint64_t* &arr,  unsigned int &length);
private:
    ///how many primes
    static const int primesFileCount;
    ///how many tabs
    static const int primesFileTabs;
    //where the prime startsvertreten will.
    static const int primesFileStart;
    /// n (state variable)
    unsigned int state;
    /// m must be prime
    uint64_t* modul;
    ///a
    double* factor;
    ///b - must be coprime
    uint64_t increment;
    ///interval
    uint64_t startInterval;
    ///interval
    uint64_t endInterval;
    ///difference-interval
    uint64_t diffInterval;
    ///result
    double* result;
    ///current Position in result array
    unsigned int curResultPosition;
};

#endif // LINEARCONGRUENTIALGENERATOR_H
