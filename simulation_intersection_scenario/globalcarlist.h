#ifndef GLOBALCARLIST_H
#define GLOBALCARLIST_H

#include <list>
#include <memory>

class Car;

/**
 * @brief The GlobalCarList class represent a central
 * administration for handling all existing cars
 * This is a decoupling function so that either the simulation
 * thread nor the Car itself can look for other cars
 */
class GlobalCarList : public std::list<std::shared_ptr<Car> >
{
public:

    /**
     * @brief GlobalCarList::getInstance
     * @return the global instance of GlobalCarList
     */
    static GlobalCarList& getInstance() {
        static GlobalCarList m_globalCarList;
        return m_globalCarList;
    }
private:
    GlobalCarList();
    GlobalCarList(GlobalCarList const&) = delete;
    void operator=(GlobalCarList const&) = delete;

};
#endif // GLOBALCARLIST_H
