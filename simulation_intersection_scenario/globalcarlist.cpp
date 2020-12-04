#include "globalcarlist.h"


//std::list<std::shared_ptr<Car> > GlobalCarList::m_globalCarList = new GlobalCarList();

/**
 * @brief GlobalCarList::GlobalCarList
 */
GlobalCarList::GlobalCarList() :
    std::list<std::shared_ptr<Car> >()
{
}
