#include "constraintdirectional.h"

#include <cmath>

ConstraintDirectional::ConstraintDirectional(const std::vector<double> &centerPoint, const double &t0,
                                             const SystemFunctionUsage &funcType, const int &N, const double &T,
                                             const double &radius, const double &dmin, const double &maxDynamics) :
    Constraint(centerPoint, t0, funcType, N, T, radius, dmin, maxDynamics)
{

}

/**
 * @brief ConstraintDirectional::createConstraintsForCar creates the directional constraints in dependence of the start position of the Car
 * and map this according to \f$ f(n,a,b) = (c*(a+0.5), c*(b+0.5)\f$, based on 0-index
 * @param startPos
 * @param interSectionWidth
 * @param interSectionHeight
 * @param cellSize cell width
 * @param t0 start time
 * @param N prediction horizon length
 * @param T sampling step
 * @param dmin minimum vehicle distance
 * @param maxDynamics
 * @return vector of directional constraints
 */
std::vector<ConstraintDirectional> ConstraintDirectional::createConstraintsForCar(const std::vector<double>& startPos,
                                             const unsigned int& interSectionWidth, const unsigned int& interSectionHeight,
                                             const double& cellSize, const double& t0, const int& N, const double& T,
                                             const double& dmin, const double& maxDynamics) {
    double widthStart = 0.0;
    double widthMax = 0.0;
    double heightStart = 0.0;
    double heightMax = 0.0;
    std::vector<ConstraintDirectional> constraintDirecVec;
    //case e.g. x_p = (0,6)
    if (startPos.at(0) < std::floor((double)interSectionWidth / 2.0)
            && startPos.at(1) >= std::floor((double)interSectionHeight / 2.0)) {

        widthStart = 0.5;
        widthMax = std::floor((double)interSectionWidth / 2.0);
        heightStart = 0.5;
        heightMax = std::floor((double)interSectionHeight / 2.0);
    }
    //case e.g. x_p = (6,6)
    else if (startPos.at(0) >= std::floor((double)interSectionWidth / 2.0)
            && startPos.at(1) >= std::floor((double)interSectionHeight / 2.0)) {
        widthStart = 0.5;
        widthMax = std::floor((double)interSectionWidth / 2.0);
        heightStart = std::floor((double)interSectionHeight / 2.0);
        heightMax = (double)interSectionHeight;
    }
    //case e.g. x_p = (6.0, 0.0)
    else if (startPos.at(0) >= std::floor((double)interSectionWidth / 2.0)
            && startPos.at(1) < std::floor((double)interSectionHeight / 2.0)) {
        widthStart = std::floor((double)interSectionWidth / 2.0);
        widthMax = (double)interSectionWidth;
        heightStart = std::floor((double)interSectionHeight / 2.0);
        heightMax = (double)interSectionHeight;
    }
    //case e.g. x_p = (0,0)
    else if (startPos.at(0) < std::floor((double)interSectionWidth / 2.0)
        && startPos.at(1) < std::floor((double)interSectionHeight / 2.0)) {
        widthStart = std::floor((double)interSectionWidth / 2.0);
        widthMax = (double)interSectionWidth;
        heightStart = 0.5;
        heightMax = std::floor((double)interSectionHeight / 2.0);
    }
    //map this according to
    for (double width = widthStart; width < widthMax; width+= cellSize) {
        for (double height = heightStart ; height < heightMax; height += cellSize) {
            constraintDirecVec.push_back(ConstraintDirectional({width, height}, t0, SystemFunctionUsage::CONTINUOUS, N, T, cellSize, dmin, maxDynamics) );
        }
    }
    return constraintDirecVec;
}

/**
 * @brief ConstraintDirectional::setActualSystem and updates the constraint time += T
 * @param systemFunc
 * @param curTime
 * @param T
 * @param N
 */
void ConstraintDirectional::setActualSystem(const std::shared_ptr<SystemFunction> &systemFunc, const double& curTime, const double &T, const int& N) {
    m_actualSystemFunc = systemFunc;
    m_t0 = curTime;
    m_tConstraint += T;
    m_T = T;
    m_N = N;
}
