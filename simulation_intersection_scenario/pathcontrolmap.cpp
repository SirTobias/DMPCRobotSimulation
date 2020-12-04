#include "pathcontrolmap.h"

/**
 * @brief PathControlMap::PathControlMap convert the given path immediately to a skalar control vector
 * @param path given control path
 */
PathControlMap::PathControlMap(const Path& path, const PathControlMapOptimizer &opt)
{
    //size - 1, we have one value less, cause x has a start value
    if (opt == PathControlMapOptimizer::DLIB) {
    m_vec = column_vector(path.size() - 1);
        for (unsigned int i = 0; i < path.size() - 1; i++) {
            PathItem itemPre = path.at(i);
            PathItem itemSuc = path.at(i+1);
            //stays in same place 0 <= u < 1/9
            if (itemSuc.getX() == itemPre.getX() && itemSuc.getY() == itemPre.getY()) {
                m_vec(i) = 0;
            }
            //stays in same place 1/9 <= u < 2/9
            else if (itemSuc.getX() > itemPre.getX() && itemSuc.getY() == itemPre.getY()) {
                m_vec(i) = 1/m_countSteps;
            }
            else if (itemSuc.getX() > itemPre.getX() && itemSuc.getY() == itemPre.getY()) {
                m_vec(i) = 2/m_countSteps;
            }
            else if (itemSuc.getX() == itemPre.getX() && itemSuc.getY() > itemPre.getY()) {
                m_vec(i) = 3/m_countSteps;
            }
            else if (itemSuc.getX() < itemPre.getX() && itemSuc.getY() > itemPre.getY()) {
                m_vec(i) = 4/m_countSteps;
            }
            else if (itemSuc.getX() < itemPre.getX() && itemSuc.getY() == itemPre.getY()) {
                m_vec(i) = 5/m_countSteps;
            }
            else if (itemSuc.getX() < itemPre.getX() && itemSuc.getY() < itemPre.getY()) {
                m_vec(i) = 6/m_countSteps;
            }
            else if (itemSuc.getX() == itemPre.getX() && itemSuc.getY() < itemPre.getY()) {
                m_vec(i) = 7/m_countSteps;
            }
            else if (itemSuc.getX() > itemPre.getX() && itemSuc.getY() < itemPre.getY()) {
                m_vec(i) = 8/m_countSteps;
            }
        }
    }
    else if (opt == PathControlMapOptimizer::NOPT) {
        m_vecNlOpt.reserve(path.size());
        for (unsigned int i = 0; i < path.size() - 1; i++) {
            PathItem item = path.at(i);
            m_vecNlOpt.push_back(item.getX() + item.getY() * m_width);
        }
    }
}

/**
 * @brief PathControlMap::PathControlMap converts immediately the vector to a control path
 * @param vec
 */
PathControlMap::PathControlMap(const column_vector& vec)
{
    //m_path.push_back(PathItem(0, 0, 0));
    for (unsigned int i = 0; i < vec.size(); i++) {
        if (vec(i) < 1/m_countSteps) {
            m_path.push_back(PathItem(0, 0, i+1));
        }
        else if (vec(i) >= 1/m_countSteps && vec(i) < 2/m_countSteps) {
            m_path.push_back(PathItem(1, 0, i+1));
        }
        else if (vec(i) >= 2/m_countSteps && vec(i) < 3/m_countSteps) {
            m_path.push_back(PathItem(1, 1, i+1));
        }
        else if (vec(i) >= 3/m_countSteps && vec(i) < 4/m_countSteps) {
            m_path.push_back(PathItem(0, 1, i+1));
        }
        else if (vec(i) >= 4/m_countSteps && vec(i) < 5/m_countSteps) {
            m_path.push_back(PathItem(-1, 1, i+1));
        }
        else if (vec(i) >= 5/m_countSteps && vec(i) < 6/m_countSteps) {
            m_path.push_back(PathItem(-1, 0, i+1));
        }
        else if (vec(i) >= 6/m_countSteps && vec(i) < 7/m_countSteps) {
            m_path.push_back(PathItem(-1, -1, i+1));
        }
        else if (vec(i) >= 7/m_countSteps && vec(i) < 8/m_countSteps) {
            m_path.push_back(PathItem(0, -1, i+1));
        }
        else if (vec(i) >= 8/m_countSteps && vec(i) < 9/m_countSteps) {
            m_path.push_back(PathItem(1, -1, i+1));
        }
    }
}

/**
 * @brief PathControlMap constructor converts a std::vector<double> to a control path with absolute coordinates
 * the control values are always rounded up
 * at first this routine is used by NLOpt
 * @param x vector with absolute coordinates to be converted
 */
PathControlMap::PathControlMap(const std::vector<double>& x) {
    setVector(x);
}

/**
 * @brief PathControlMap::setVector converts a std::vector<double> to a control path with absolute coordinates
 * the control values are always rounded up
 * at first this routine is used by NLOpt
 * @param x vector with absolute coordinates to be converted
 */
void PathControlMap::setVector(const std::vector<double>& x) {
    m_path.clear();
    for (unsigned int i = 0; i < x.size(); i++) {
        //this gives us first the row, divided by the given width
        //for e.g. x(0) == 13.2 and x(1) == 13.0, than x(1) - 0.4 to get 13, 12, ...
        unsigned int rowY = std::floor(x.at(i) / m_width);
        unsigned colX = ( (unsigned int)std::ceil(x.at(i)) % m_width);
        //exception, at (0,0) no round up
        if (x.at(i) == 0.0) {
            colX = (unsigned int)x.at(i) % m_width;
        }
        //get the col for the row by modulo - 1 because it is 0-based-index
        //for optimization issues, round downwards, if they are under x.5
        /*unsigned int controlCol = 0;
        if (x.at(i) > 0.5) {
            controlCol = ((unsigned int)std::ceil(x.at(i)) % m_width);
        }
        else {
            controlCol = ((unsigned int)std::floor(x.at(i)) % m_width);
        }*/
        //i is taken as time
        //x is here col, because the numeration goes horizontal row-wise from 0...m_width (e.g. 3)
        //then 4...7 etc.
        m_path.push_back(PathItem(colX, rowY, 0));
    }
}

/**
 * @brief PathControlMap::getPath
 * @return
 */
Path PathControlMap::getPath() const {
   return m_path;
}

/**
 * @brief PathControlMap::getVector
 * @return
 */
column_vector PathControlMap::getVector() const {
    return m_vec;
}

std::vector<double> PathControlMap::getVectorNlOpt() const {
    return m_vecNlOpt;
}

/**
 * @brief PathControlMap::convertPathToVector maps directly a path to its vector values
 * @param path
 * @return
 */
std::vector<double> PathControlMap::convertPathToVector(const Path& path) {
    std::vector<double> vec;
    for (const PathItem& item : path) {
        vec.push_back(item.getX() + item.getY() * m_width);
    }
    return vec;
}
