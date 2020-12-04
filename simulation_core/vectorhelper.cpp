#include "vectorhelper.h"

#include <QtCore/QDebug>
#include <cmath>

#include <typeinfo>

VectorHelper::VectorHelper()
{
}

/**
 * @brief VectorHelper::reshapeXd converts a given vector to a size x shapeFactor length matrix
 * @param vec to be reshaped
 * @param shapeFactor length of rows
 * @return
 */
std::vector<std::vector<double > > VectorHelper::reshapeXd(const std::vector<double> &vec, const size_t shapeFactor) {
    std::vector<std::vector<double > > vecXd;
    std::vector<double> curRow;
    curRow.reserve(shapeFactor);
    for (size_t i = 0; i < vec.size(); i++) {
        //first value of the row
        if (i % shapeFactor <= shapeFactor - 2) {
            curRow.push_back(vec.at(i));
        }
        //until e.g. i % 3 = 2, as i % 3 = 0 is first again
        else if (i % shapeFactor == shapeFactor - 1) {
            curRow.push_back(vec.at(i));
            vecXd.push_back(curRow);
            curRow.clear();
        }
    }
    return vecXd;
}

/**
 * @brief VectorHelper::reshapeXd1d reshapes a XD-Vector back to 1D
 * @param vec XD-vector
 * @return 1D-Vector
 */
std::vector<double> VectorHelper::reshapeXdTo1d(const std::vector<std::vector<double > > &vec) {
   std::vector<double> vec1d;
   for (unsigned int i = 0; i < vec.size(); i++) {
       std::vector<double> curRow = vec.at(i);
       for (unsigned int j = 0; j < curRow.size(); j++) {
           vec1d.push_back(curRow.at(j));
       }
   }
   return vec1d;
}

/**
 * @brief VectorHelper::getInfinityNorm gives back \f$||x||_{\infty}\f$ of a VectorHelper
 * @param vec vector as parameter
 * @return maximum
 */
double VectorHelper::getInfinityNorm(const std::vector<double> &vec) {
    double curValue = vec.at(0);
    for (const double& val : vec) {
        if (std::abs(val) > curValue) {
            curValue = std::abs(val);
        }
    }
    return curValue;
}

/**
 * @brief VectorHelper:: returns \f$\vec{v2} - \vec{v1}\f$
 * @param v1
 * @param v2
 * @return the result
 */
std::vector<double> VectorHelper::sub(const std::vector<double> &v1, const std::vector<double> &v2) {
    std::vector<double> result;
    Q_ASSERT_X(v1.size() == v2.size(), typeid(VectorHelper).name(), "size not equal");
    if (v1.size() == v2.size()) {
        for (unsigned int i = 0; i < v1.size(); i++) {
            result.emplace_back(v1.at(i) - v2.at(i));
        }
    }
    return result;
}

/**
 * @brief VectorHelper:: returns \f$\vec{v2} + \vec{v1}\f$
 * @param v1
 * @param v2
 * @return the result
 */
std::vector<double> VectorHelper::add(const std::vector<double> &v1, const std::vector<double> &v2) {
    std::vector<double> result;
    Q_ASSERT_X(v1.size() == v2.size(), typeid(VectorHelper).name(), "size not equal");
    if (v1.size() == v2.size()) {
        for (unsigned int i = 0; i < v1.size(); i++) {
            result.emplace_back(v1.at(i) + v2.at(i));
        }
    }
    return result;
}

/**
 * @brief VectorHelper::ceil rounds up all components of a vector
 * @param vec vector to be ceiled
 * @return vector with all ceiled components
 */
std::vector<double> VectorHelper::ceil(const std::vector<double> &vec) {
    std::vector<double> ceiled;
    for (auto& val : vec) {
        ceiled.emplace_back(std::ceil(val));
    }
    return ceiled;
}

/**
 * @brief VectorHelper::floor rounds down all components of a vector
 * @param vec vector to be floored
 * @return vector with all floored components
 */
std::vector<double> VectorHelper::floor(const std::vector<double> &vec) {
    std::vector<double> floored;
    for (auto& val : vec) {
        floored.emplace_back(std::floor(val));
    }
    return floored;
}

/**
 * @brief VectorHelper::norm2 gives back \f$||vec||\f$
 * @param vec vector to be normed
 * @return normed value
 */
double VectorHelper::norm2(const std::vector<double> &vec) {
    double norm = 0.0;
    for (const double &val : vec) {
        norm += std::pow(val, 2);
    }
    norm = std::sqrt(norm);
    return norm;
}

/**
 * @brief VectorHelper::getDerivOfAbsValue
 * @param vec
 * @return
 */
std::vector<double> VectorHelper::getDerivOfAbsValue(const std::vector<double> &vec) {
    std::vector<double> derivAbs;
    for (const double& val : vec) {
        if (val < 0.0) {
            derivAbs.emplace_back(-1.0);
        }
        else {
            derivAbs.emplace_back(1.0);
        }
    }
    return derivAbs;
}

/**
 * @brief VectorHelper::smaller evaluates, if vector vec1 is smaller then vec2
 * @param vec1
 * @param vec2
 * @return
 */
double VectorHelper::smaller(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    Q_ASSERT_X(vec1.size() == vec2.size(), typeid(VectorHelper).name(), "size not equal");
    double smallFac = 0.0;
    for (unsigned int i = 0; i < vec1.size(); i++) {
        smallFac += vec1.at(i) - vec2.at(i);
    }
    return smallFac;
}

/**
 * @brief VectorHelper::mult
 * @param v1
 * @param v2
 * @return
 */
std::vector<double> VectorHelper::mult(const std::vector<double> &vec1, const std::vector<double> &vec2) {
    Q_ASSERT_X(vec1.size() == vec2.size(), typeid(VectorHelper).name(), "size not equal");
   std::vector<double> result;
   for (unsigned int i = 0; i < vec1.size(); i++) {
       result.emplace_back(vec1.at(i) * vec2.at(i));
   }
   return result;
}

/**
 * @brief VectorHelper::mult
 * @param vec1
 * @param vec2
 * @return
 */
std::vector<double> VectorHelper::mult(const std::vector<double> &vec1, const double &scal) {
   std::vector<double> result;
   for (unsigned int i = 0; i < vec1.size(); i++) {
       result.emplace_back(vec1.at(i) * scal);
   }
   return result;
}


/**
 * @brief VectorHelper::direction gives the directions of the vector
 * if val < 0 - false, else true
 * @param vec
 * @return boolean vector with directions (v(i) < 0: false, else true)
 */
std::vector<bool> VectorHelper::direction(const std::vector<double> &vec) {
    std::vector<bool> directions;
    for (const double& val : vec) {
        if (val < 0.0) {
            directions.push_back(false);
        }
        else {
            directions.push_back(true);
        }
    }
    return directions;
}

/**
 * @brief VectorHelper::stepSizeInInterval
 * @param vec1 target State
 * @param vec2 new state with calculated step size
 * @param directions vector of boolean for evaluated directions
 * @return
 */
bool VectorHelper::stepSizeInInterval(const std::vector<double> &vec1, const std::vector<double> &vec2, const std::vector<bool>& directions) {
    bool isInStepSize = true;
    Q_ASSERT_X(vec1.size() == vec2.size(), typeid(VectorHelper).name(), "size not equal");
    for (unsigned int i = 0; i < vec1.size(); i++) {
        //negative directions, if new step is smaller than target step, skip
        if (directions.at(i) == false) {
            if (vec2.at(i) < vec1.at(i)) {
                isInStepSize = false;
                break;
            }
        }
        //positive direction, if new step is greater then target step, skip
        else {
            if (vec2.at(i) > vec1.at(i)) {
                isInStepSize = false;
                break;
            }
        }
    }
    return isInStepSize;
}

/**
 * @brief VectorHelper::getVectorValues returns a subvector from u with length N
 * @param vec vector from which subvector is taken
 * @param start start position of the taken subvector
 * @param length length of the subvector
 * @return
 */
std::vector<double> VectorHelper::getVectorValues(const std::vector<double>& vec, const unsigned int& startPos, const unsigned int& length) {
    unsigned int size = startPos + length;
    //qDebug() << "vector size: " << vec.size() << ", start + length: " << startPos + length;
    Q_ASSERT_X(vec.size() >= size, typeid(VectorHelper).name(), "startPos + length >= vec");
    std::vector<double> cutVector;
    cutVector.reserve(length);

    for (unsigned int i = startPos; i < size; i++) {
        cutVector.push_back(vec.at(i));
    }
    return cutVector;
}

/**
 * @brief VectorHelper::shiftStep removes the first steps entries and fill up with steps entries initialized with
 * @param vec vector to be shifted
 * @param steps
 * @param initVal
 * @return shifted vector
 */
std::vector<double> VectorHelper::shiftStep(const std::vector<double>& vec, const size_t &steps, const double& initVal) {
    std::vector<double> shiftVec = vec;
    auto itEnd = shiftVec.begin();
    itEnd += steps;
    shiftVec.erase(shiftVec.begin(), itEnd);
    for (size_t i = 0; i < steps; i++) {
        shiftVec.push_back(initVal);
    }
    return shiftVec;
}

/**
 * @brief VectorHelper::setSubVector sets the subvector values into
 * @param fullVector
 * @param subVector
 * @param startPos
 * @return
 */
std::vector<double> VectorHelper::setSubVector(const std::vector<double>& fullVector, const std::vector<double>& subVector, const size_t& startPos) {
    std::vector<double> retVector = fullVector;
    size_t fullVectorSize = fullVector.size();
    Q_ASSERT_X(subVector.size() + startPos <= fullVector.size(), typeid(VectorHelper).name(), "subvector size is out of range of fullvector");
    for (size_t i = 0; i < subVector.size(); i++) {
        retVector[i + startPos] = subVector.at(i);
    }
    return retVector;
}
