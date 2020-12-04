#ifndef INTERSECTION_H
#define INTERSECTION_H
#include <memory>
#include "../simulation-core/simulationresource.h"
#include "intersectioncell.h"
#include "pathcalculation.h"
#include "arrivalcar.h"

//@TODO: structure for modeling obstacles and real costs and if a cell is reserved
//for this the intersection has to give back a free time slot
using InterSectionGrid = QHash<unsigned int,QHash<unsigned int, std::shared_ptr<InterSectionCell> > >;
//typedef QHash<unsigned int,QHash<unsigned int, std::shared_ptr<InterSectionCell> > > InterSectionGrid;

static std::shared_ptr<InterSection> interSection;
/**
 * @brief The InterSection class holds the complete set for the intersection including a grid which represents each cell
 * Furthermore the class gives several methods to examine the grid and also reserve time slots or to check if time slots are
 * available.
 */
class InterSection : public SimulationResource
{
public:
    InterSection(const unsigned int &width, const unsigned int &height, const double cellSize = 1.0);
    void buildGrid(const unsigned int &width, const unsigned int &height, const double cellSize);
    PathCalculation* getPathCalculation();
    QList<std::weak_ptr<InterSectionCell> > getNeighbours(const std::weak_ptr<InterSectionCell> &cell, const unsigned int& radius) const;
    bool isNeighboured(const std::weak_ptr<InterSectionCell>& first, const std::weak_ptr<InterSectionCell>& second) const;
    std::shared_ptr<InterSectionGrid> getGrid() const;
    QList<std::weak_ptr<InterSectionCell> > getSuccessors(const std::weak_ptr<InterSectionCell>& start, const std::weak_ptr<InterSectionCell>& target) const;
    bool reserveTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t);
    double reserveNextFreeTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t);
    double getPossibleReserveTimeForCar(const unsigned int& k, const unsigned int& m, const double& t) const;
    double getPossiblePrelimReserveTimeForCar(const unsigned int& k, const unsigned int& m, const double& t) const;
    static std::shared_ptr<InterSection> getInstance();
    double tryReserveTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t);
    double tryReservePrelimTimeForCar(const QString &car, const unsigned int& k, const unsigned int& m, const double& t);
    std::shared_ptr<InterSectionCell> getInterSectionCell(const unsigned int& k, const unsigned int& m);
    std::shared_ptr<InterSectionCell> getInterSectionCell(const unsigned int& k, const unsigned int& m) const;
    bool isTimeAlreadyReserved(std::shared_ptr<InterSectionCell> cell, const QString& car, const double& time) const;
    void clearPrelimPathOfCar(const QString& car, const Path& path);
    void copyCurrentPositionsToPreliminaries();
    unsigned int getHeight() const;
    unsigned int getWidth() const;
    unsigned int getGridHeight() const;
    unsigned int getGridWidth() const;
    unsigned int getNumberOfReservations(const unsigned int k, const unsigned int m);
    void printReservations() const;
    std::shared_ptr<InterSectionCell> getCellFromCoordinates(const std::vector<double> &x);
    void setCellSize(const double& cellSize);
    double getCellSize() const;
    void setEntryPointsStandardLanes(const std::vector<DistParam> &distParam);
    std::shared_ptr<InterSectionCell> getEntryPoint(const unsigned int& index);
    unsigned int numberEntryPoints() const;
    unsigned int getAmountOfCarsForNextTime(const unsigned int& entryPoint);
private:
    ///Hash-Table which contains all Intersection-Cells in a 2D-grid
    std::shared_ptr<InterSectionGrid> m_gridMap;
    ///holds a map of enumerated entry points in the intersection, standardized, all beginning of lanes are entry points
    std::map<unsigned int, std::shared_ptr<InterSectionCell> > m_entryPoints;
    ///width of the gridMap
    unsigned int m_width;
    ///height of the gridMap
    unsigned int m_height;
    ///the grid is calculated according to the $\f m\_gridSizeWidth = \frac{m_width}{m_cellSize} \f$
    unsigned int m_gridSizeWidth;
    ///the grid is calculated according to the $\f m\_gridSizeHeight = \frac{m_height}{m_cellSize} \f$
    unsigned int m_gridSizeHeight;
    //@TODO: add additional parameter for cell size
    double m_cellSize;
    ///stochastic process for each entry point in the intersection
    std::vector<ArrivalCar> m_enterInterSectDist;

};

#endif // INTERSECTION_H
