#ifndef VECTORHELPER_H
#define VECTORHELPER_H

#include "simsharedlib.h"
#include <vector>

class SIM_CORE_EXPORT VectorHelper : public std::vector<double>
{
public:
    VectorHelper();
    static std::vector<std::vector<double > > reshapeXd(const std::vector<double> &vec, const size_t shapeFactor = 2);
    static std::vector<double> reshapeXdTo1d(const std::vector<std::vector<double > > &vec);
    static double getInfinityNorm(const std::vector<double> &vec);
    static std::vector<double> sub(const std::vector<double> &v1, const std::vector<double> &v2);
    static std::vector<double> add(const std::vector<double> &v1, const std::vector<double> &v2);
    static std::vector<double> mult(const std::vector<double> &vec1, const std::vector<double> &vec2);
    static std::vector<double> mult(const std::vector<double> &vec1, const double &scal);
    static std::vector<double> ceil(const std::vector<double> &vec);
    static std::vector<double> floor(const std::vector<double> &vec);
    static double norm2(const std::vector<double> &vec);
    static std::vector<double> getDerivOfAbsValue(const std::vector<double> &vec);
    static double smaller(const std::vector<double>& vec1, const std::vector<double>& vec2);
    static std::vector<bool> direction(const std::vector<double> &vec);
    static bool stepSizeInInterval(const std::vector<double> &vec1, const std::vector<double> &vec2, const std::vector<bool>& directions);
    static std::vector<double> getVectorValues(const std::vector<double> &vec, const unsigned int &startPos, const unsigned int &length);
    static std::vector<double> shiftStep(const std::vector<double>& vec, const size_t &steps, const double& initVal);
    static std::vector<double> setSubVector(const std::vector<double>& fullVector, const std::vector<double>& subVector, const size_t& startPos);
};

#endif // VECTORHELPER_H
