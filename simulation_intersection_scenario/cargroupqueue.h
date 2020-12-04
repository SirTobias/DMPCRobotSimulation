#ifndef CARGROUPQUEUE_H
#define CARGROUPQUEUE_H

#include "car.h"
#include "cargroup.h"


#include <memory>

#include <list>

/**
 * @brief The CargroupQueueType enum
 */
enum class CarGroupQueueType {
    FIXED = 0,
    SETBASED = 1,
    TREEBAESD = 2
};

/**
 * @brief The CarGroupQueue class
 */
class CarGroupQueue
{
public:

    std::vector<std::vector<std::shared_ptr<Car> > >::iterator begin() {return m_cars.begin();}
    std::vector<std::vector<std::shared_ptr<Car> > >::iterator end() {return m_cars.end();}
    std::vector<std::vector<std::shared_ptr<Car> > >::const_iterator cbegin() {return m_cars.cbegin();}
    std::vector<std::vector<std::shared_ptr<Car> > >::const_iterator cend() {return m_cars.cend();}
    CarGroupQueue();
    CarGroupQueue(const std::vector<std::vector<std::shared_ptr<Car> > >& cars);
    CarGroupQueue(const std::vector<std::shared_ptr<Car> >& cars);
    void setType(const CarGroupQueueType& type);
    void appendCar(const std::shared_ptr<Car>& car);
    void appendCar(const std::shared_ptr<Car>& car, const size_t& pos = 0);
    void storeOrder(const std::vector<std::shared_ptr<Car> >& cars, const size_t &pos = 0);
    std::vector<std::vector<std::shared_ptr<Car> > > getOrder() const;
    std::vector<std::shared_ptr<Car> > getOrderSeq() const;
    std::vector<std::shared_ptr<Car> >& findVector(const std::shared_ptr<Car>& car);
    void push_back(const std::shared_ptr<Car>& car);
    size_t size() const;
    size_t rowSize() const;
    std::vector<std::shared_ptr<Car> >::iterator removeCar(const std::shared_ptr<Car>& car);
    std::vector<std::shared_ptr<Car> >::iterator removeCar(std::vector<std::shared_ptr<Car> > &vec, const std::shared_ptr<Car>& car);
    std::vector<std::shared_ptr<Car> >& getRow(const size_t& pos);
    size_t getPosForRow(std::vector<std::shared_ptr<Car> > & row) const;
    void removeEmptyRows();
    void swap(const int &row1, const int &col1, const int &row2, const int &col2);
    void clear();
    std::vector<std::shared_ptr<Car> >::iterator moveCar(const std::shared_ptr<Car>& car, const size_t& newRow);
    std::vector<std::shared_ptr<Car> >::iterator moveCarToNextRow(const std::shared_ptr<Car>& car);
    void moveCarPreviousRow(const std::shared_ptr<Car>& car);
    void moveSuccessor(const std::shared_ptr<Car>& car);
    bool hasPreviousRow(std::vector<std::shared_ptr<Car> >& row) const;
    bool hasNextRow(std::vector<std::shared_ptr<Car> >& row) const;
    std::vector<std::shared_ptr<Car> >& getPreviousRow(std::vector<std::shared_ptr<Car> >& row);
    std::vector<std::shared_ptr<Car> >& getNextRow(std::vector<std::shared_ptr<Car> >& row);
    bool findCarInVector(const std::vector<std::shared_ptr<Car> >& vec, const std::shared_ptr<Car>& car) const;
    void addSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& succ);
    void removeSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& suc);
    std::list<std::shared_ptr<Car> > &getDirectSuccessors(const std::shared_ptr<Car>& car);
    std::list<std::shared_ptr<Car> >& getDirectPredecessors(const std::shared_ptr<Car>& car);
    bool isSuccessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& suc) const;
    bool isPredecessor(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& pred) const;
    bool isInRelationTo(const std::shared_ptr<Car>& car, const std::shared_ptr<Car>& itCar) const;
    /// get maximum number of level (predecessors) of car
    size_t getExecLevelAccordingPreds(const std::shared_ptr<Car> &car, const size_t& level = 0) const;
    bool hasSuccessor(const std::shared_ptr<Car>& car) const;
    bool hasPredecessors(const std::shared_ptr<Car>& car) const;
    std::list<std::shared_ptr<Car> >& computeRecursivePredecessors(const std::shared_ptr<Car>& car, std::list<std::shared_ptr<Car> > &predList) const;
    std::list<std::shared_ptr<Car> >& computeRecursiveSuccessors(const std::shared_ptr<Car>& car, std::list<std::shared_ptr<Car> >& succList) const;
    std::shared_ptr<Car> getRoot(const std::shared_ptr<Car>& car);
    void showCarGroupTree(const std::shared_ptr<Car>& car);
    bool contains(std::list<std::shared_ptr<Car> >& list, const std::shared_ptr<Car>& car) const;
    std::shared_ptr<Car> getCarById(const QString& id);
    void insertCar(const std::shared_ptr<Car>& car, const size_t& row, const size_t& pos);
private:
    ///CarGroupQueueType
    CarGroupQueueType m_queueType;
    ///vector of vector of cars (set-based priority order)
    std::vector<std::vector<std::shared_ptr<Car> > > m_cars;
    ///map of tree-based priority order (predecessors, successor)
    std::map<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > > m_carTreeSuccessors;
    std::map<std::shared_ptr<Car>, std::list<std::shared_ptr<Car> > > m_carTreePredecessors;

    ///helper methods for getting correct order (hierarchy)


};

#endif // CARGROUPQUEUE_H
