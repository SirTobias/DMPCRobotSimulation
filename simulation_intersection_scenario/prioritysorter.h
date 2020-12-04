#ifndef PRIORITYSORTER_H
#define PRIORITYSORTER_H

#include "cargroupqueue.h"
#include "../simulation-core/enumvalues.h"

#include <QtCore/QStringList>

#include <memory>
#include <map>

//TODO: Refactoring: put this in simulation core library

/**
 * @brief The PriorityCriteria enum classes specify the priority criteria for optimization,
 * here as FIXED order,
 * MINOPENLOOPCOSTS order sorting ascending after minimum open-loop costs
 * MINCLOSEDLOOPCOSTS order sorting ascending after minimum closed-loop costs
 * MAXOPENLOOPCOSTS order sorting ascending after maximum open-loop costs
 * MAXCLOSEDLOOPCOSTS order sorting ascending after maximum closed-loop costs
 * MINOPENLOOPCOSTSWITHMEMORY order sorting ascending after minimum open-loop costs with memory saving the dependency
 * MINCLOSEDLOOPCOSTSWITHMEMORY order sorting ascending after minimum closed-loop costs with memory saving the dependency
 * MINOPENLOOPCOSTSWITHMEMORYHIERARCHY order sorting ascending after minimum open-loop costs with memory saving the dependency in hierarchical order considering each row for executing
 * MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY order sorting ascending after minimum closed-loop costs with memory saving the dependency in hierarchical order considering each row for executing
 * MINOPENLOOPCOSTSWITHMEMORYTREE order sorting ascending after minimum open-loop costs with memory but only after which active constraints are violated
 * MINCLOSEDLOOPCOSTSWITHMEMORYTREE order sorting ascending after minimum closed-loop costs with memory but only after which active constraints are violated
 */
enum class PriorityCriteria {
    FIXED=0,
    MINOPENLOOPCOSTS=1,
    MINCLOSEDLOOPCOSTS=2,
    MAXOPENLOOPCOSTS=3,
    MAXCLOSEDLOOPCOSTS=4,
    MINOPENLOOPCOSTSWITHMEMORY=5,
    MINCLOSEDLOOPCOSTSWITHMEMORY=6,
    MINOPENLOOPCOSTSWITHMEMORYHIERARCHY = 7,
    MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY = 8,
    MINOPENLOOPCOSTSWITHMEMORYTREE = 9,
    MINCLOSEDLOOPCOSTSWITHMEMORYTREE = 10
};


/**
 * @brief The PriorityCriteriaStrings struct creates the basic structure to have a literal
 * description for GUI elements
 */
struct PriorityCriteriaStrings {
    PriorityCriteria criteria;
    char const* text;
};

/**
* defines the const map for Criteria and literal description
*/
constexpr PriorityCriteriaStrings priorityCriteriaMap[] = {
    {PriorityCriteria::FIXED, "FIXED"},
    {PriorityCriteria::MINCLOSEDLOOPCOSTS, "MINCLOSEDLOOPCOSTS"},
    {PriorityCriteria::MINOPENLOOPCOSTS, "MINOPENLOOPCOSTS"},
    {PriorityCriteria::MAXCLOSEDLOOPCOSTS, "MAXCLOSEDLOOPCOSTS"},
    {PriorityCriteria::MAXOPENLOOPCOSTS, "MAXOPENLOOPCOSTS"},
    {PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY, "MINOPENLOOPCOSTSWITHMEMORY"},
    {PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY, "MINCLOSEDLOOPCOSTSWITHMEMORY"},
    {PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY, "MINOPENLOOPCOSTSWITHMEMORYHIERARCHY"},
    {PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY, "MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY"},
    {PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE, "MINOPENLOOPCOSTSWITHMEMORYTREE"},
    {PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE, "MINCLOSEDLOOPCOSTSWITHMEMORYTREE"}
};

/**
 * @brief getPriorityCriteriaMapSize returns the size of the PriorityMap
 * @return
 */
constexpr int getPriorityCriteriaMapSize() {
    return sizeof(priorityCriteriaMap) / sizeof(priorityCriteriaMap[0]);
}

/**
 * @brief The PrioritySorter class provides to sort the cars after certain criteria
 */
class PrioritySorter
{
public:
    PrioritySorter(const PriorityCriteria& criteria);
    void sortAfterPriority(CarGroupQueue &cars, const std::map<QString, std::vector<std::vector<double> > > &contin,
                           const std::multimap<QString, Constraint> &constraints, const double &t0 = 0.0,
                           const double &T = 0.0, const size_t &N = InterSectionParameters::N, const double &radius = 0.5);
    std::vector<std::shared_ptr<Car> > findCarInDeorderPriorityMap(CarGroupQueue &deorderAndPriorityMap, const std::shared_ptr<Car>& car);
    PriorityCriteria getPriorityCriteria() const;
    static bool compareMinClosedLoopCostsLess(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2);
    static bool compareMinOpenLoopCostsLess(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2);
    static bool compareMaxClosedLoopCostsGreater(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2);
    static bool compareMaxOpenLoopCostsGreater(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2);
    static QStringList getTextForCriteria();
    static QString getTextForChosenCriteria(const PriorityCriteria& criteria);
    ///get optimisation unconstrained
    std::map<QString, std::vector<std::vector<double> > > getUnconstrainedSol(const CarGroupQueue &cars, const double &t0, const double &T) const;
    std::shared_ptr<Car> getCarWithHigherCosts(const std::shared_ptr<Car>& car1, const std::shared_ptr<Car>& car2) const;
    QStringList testCurrentCarsForCurrentRow(CarGroupQueue& cars, const std::map<QString, std::vector<std::vector<double> > > continSol,
                                      std::vector<std::shared_ptr<Car> > &carRow, const double &radius, const double &t0, const double &T, const size_t &N);
    std::vector<Constraint> getConstraintsFromMap(const std::multimap<QString, Constraint> &constraints, const QString& car) const;
private:
    ///chosen criteria
    PriorityCriteria m_criteria;
    //std::map<std::shared_ptr<Car>, std::vector<std::shared_ptr<Car> > > m_deorderAndPriorityMap;
};

#endif // PRIORITYSORTER_H
