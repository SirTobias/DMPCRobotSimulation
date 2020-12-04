#include "pathcalculation.h"

/**
 * @brief PathCalculation::PathCalculation
 */
PathCalculation::PathCalculation() :
    m_alg(PathAlgorithm::MPCCOBYLA)
{

}

/**
 * @brief PathCalculation::PathCalculation
 */
PathCalculation::PathCalculation(const PathAlgorithm &alg) :
    m_alg(alg)
{
}

/**
 * @brief PathCalculation::~PathCalculation
 */
PathCalculation::~PathCalculation() {

}

PathAlgorithm PathCalculation::getAlg() const {
    return m_alg;
}

/**
 * @brief calculatePath
 * @param source
 * @param target
 * @return
 */
/*Path calculatePath(const InterSectionCell &source, const InterSectionCell &target) const {

}*/
