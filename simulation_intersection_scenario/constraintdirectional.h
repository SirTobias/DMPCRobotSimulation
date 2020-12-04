#ifndef CONSTRAINTDIRECTIONAL_H
#define CONSTRAINTDIRECTIONAL_H

#include "constraint.h"

#include <vector>

class ConstraintDirectional : public Constraint
{
public:
    ConstraintDirectional(const std::vector<double> &centerPoint, const double& t0, const SystemFunctionUsage &funcType, const int& N = InterSectionParameters::N,
                          const double &T = InterSectionParameters::T, const double &radius = 1.1, const double &dmin = 0.5, const double &maxDynamics = 1.0);
    static std::vector<ConstraintDirectional> createConstraintsForCar(const std::vector<double>& startPos,
                                             const unsigned int& interSectionWidth, const unsigned int& interSectionHeight,
                                             const double &cellSize, const double &t0, const int &N, const double &T,
                                             const double &dmin, const double &maxDynamics);
    void setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& curTime, const double &T, const int& N);
};

#endif // CONSTRAINTDIRECTIONAL_H
