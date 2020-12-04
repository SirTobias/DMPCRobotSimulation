#ifndef PATHCONTROLMAP_H
#define PATHCONTROLMAP_H

#include "path.h"
#include "intersectionparameters.h"
#include "optimize/dlib/optimization.h"


typedef dlib::matrix<double,0,1> column_vector;

enum class PathControlMapOptimizer : unsigned int __attribute__((visibility("default"))) {
    DLIB = 0,
    NOPT = 1
};

/**
 * @brief The PathControlMap class gives a conversion possibility
 * to convert the control value tupel (x, y, t) based on the grid to an scalar value interval between 0..1
 * Is is important, that the tupel values are not absolute to the coordinates but in difference to its
 * successors or predecessors.
 * According to the moore-neighbourhood http://en.wikipedia.org/wiki/Moore_neighborhood
 * it is assumed, that included the own position 9 different steps can be taken
 */
class PathControlMap
{
public:
    PathControlMap(const Path& path, const PathControlMapOptimizer& opt);
    PathControlMap(const column_vector& vec);
    PathControlMap(const std::vector<double>& x);
    Path getPath() const;
    column_vector getVector() const;
    std::vector<double> getVectorNlOpt() const;
    static std::vector<double> convertPathToVector(const Path& path);
    void setVector(const std::vector<double>& x);

private:
    ///Path whose PathItems could be converted to a control path
    Path m_path;
    ///Vector whose numbers are between 0 and 1 for direction (DLIB)
    column_vector m_vec;
    ///vector for NLOpt with integer value for each grid cell
    std::vector<double> m_vecNlOpt;
    ///interval steps, here 9 for moore-neighbourhood and own position
    static constexpr double m_countSteps = 9.0;
    ///width given here as constexpr
    /// TODO: BUG
    static constexpr int m_width = InterSectionParameters::k + 1;
};

#endif // PATHCONTROLMAP_H
