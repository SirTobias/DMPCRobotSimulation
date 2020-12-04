#include "path.h"
#include "intersectionparameters.h"

/**
 * @brief Path::Path
 */
Path::Path()
{
}

Path::Path(const PathItem &start) {
    PathItem startCopy = start;
    this->push_back(startCopy);
}

/**
 * @brief Path::getPath
 * @return
 */
//QVector<PathItem> Path::getPath() {
//    return m_pathItems;
//}

/**
 * @brief Path::getPath
 * @return
 */
//const QVector<PathItem>& Path::getPath() const {
//    return m_pathItems;
//}

/**
 * @brief Path::addPathItem
 * @param item
 */
void Path::addPathItem(const PathItem &item) {
    this->push_back(item);
}

/**
 * @brief Path::push_back
 * @param path
 */
/*void Path::push_back(const Path& path) {
   for (const PathItem& item : path) {
       push_back(item);
   }
}*/

/*void Path::push_back(const PathItem& pathItem) {
    this->push_back(pathItem);
}*/

/**
 * @brief Path::isNeighboured tests, if the PathItems for each following differ by maximum of 1
 * @param violated
 * @return
 */
bool Path::isNeighboured(unsigned int* violated) const {
    bool valid = true;
    if (size() > 0) {
        for (unsigned int i = 0; i < size() - 1; i++) {
            PathItem cur = at(i);
            PathItem next = at(i+1);
            if (std::abs(next.getX() - cur.getX()) > 1 || std::abs(next.getY() - cur.getY()) > 1) {
                valid = false;
                if (violated) {
                    *violated += 1;
                }
            }
        }
    }
    return valid;
}

/**
 * @brief Path::normalizeForTime means, that each PathItem
 * @param start
 * @return
 */
Path Path::normalizeForTime(const PathItem &start) {
    double startTime = start.getTime() + 1.0;
    for (PathItem& item: *this) {
        item.setTime(startTime);
        startTime += 1.0;
    }
    return *this;
}

/**
 * @brief Path::prepend
 * @param item
 */
void Path::prependPathItem(const PathItem& item) {
    this->insert(this->begin(), item);
}

/**
 * @brief Path::isInvalid returns true, if path is invalid, which means
 * at first: contains PathItem out of grid
 * at second: Path not consistent
 * @return
 */
bool Path::isInvalid() const {
    bool invalid = false;
    for (const PathItem& item: *this) {
        if (item.getX() > InterSectionParameters::k || item.getY() > InterSectionParameters::m) {
            invalid = true;
            break;
        }
    }
    if (!invalid) {
        invalid = isNeighboured(nullptr);
    }
    return invalid;
}

/**
 * @brief Path::findSimilarities find, if the two paths share the same at the same time to obtain any conflicts
 * @param otherPath other path to compare
 * @return path, if there are similarities, else it is empty
 */
Path Path::findSimilarities(const Path& otherPath) const {
    Path commonCells;
    for (const PathItem& item : *this) {
        for (const PathItem& otherItem : otherPath) {
            if (item == otherItem) {
                commonCells.push_back(item);
            }
        }
    }
    return commonCells;
}

/**
 * @brief Path::getOutput for debug reasons to print the whole path
 * @return
 */
std::string Path::getOutput() const {
    std::string out;
    for (const PathItem& item : *this) {
        out.append("(" + std::to_string(item.getTime()) + ";" + std::to_string(item.getX()) + ";" + std::to_string(item.getY()) + ")");
    }
    return out;
}
