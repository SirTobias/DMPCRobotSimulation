#ifndef PATH_H
#define PATH_H
#include "pathitem.h"

#include <string>
#include <vector>
#include <memory>

/**
 * @brief The Path class contains the whole PathItems to form the path a car is taking through an intersection
 */
class Path : public std::vector<PathItem>
{
public:
    Path();
    Path(const PathItem& start);
    //QVector<PathItem> getPath();
    //const QVector<PathItem> &getPath() const;
    void addPathItem(const PathItem& item);
    void prependPathItem(const PathItem& item);
    bool isNeighboured(unsigned int *violated = nullptr) const;
    Path normalizeForTime(const PathItem& start);
    bool isInvalid() const;
    Path findSimilarities(const Path& otherPath) const;
    std::string getOutput() const;
private:
    ///the PathItems which describes a full path
    //QVector<PathItem> m_pathItems;
    //@TODO: Mechanismus schaffen, der berücksichtigt, dass sich ein Auto auch mehrere Zeiteinheiten auf einem Feld aufhalten kann
    //und Nachricht senden, wenn gridCell wieder frei ist oder generell t_r+1 annehmen, und dann Neuplanung
};

#endif // PATH_H
