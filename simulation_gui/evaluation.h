#ifndef EVALUATION_H
#define EVALUATION_H

#include "plot2d.h"

#include <QtCore/QString>

class Evaluation
{
public:
    Evaluation();
    void saveOpenLoopCosts(const double& t, const double &costs);
    void saveClosedLoopCosts(const double& t, const double &costs);
    //Plots
    void plotOpenLoopCosts();
    void plotClosedLoopCosts();

    //helper methods
    void showPlots();
    void exportPlots(const QString &type);
    unsigned int getNextValidColor(const unsigned int& index);
    void plot2DVector(const QString& title, const QString& subTitlePrefix, std::map<double, std::vector<double> > &map);
protected:

    ///open loop costs \f$V^N(x,u)\f$ over each agent over simulation time \f$n_{end}\f$
    std::map<double, std::vector<double> > m_openLoopCosts;
    ///closed loop costs \f$l(x,u)\f$ over each agent over simulation time \f$n_{end}\f$
    std::map<double, std::vector<double> > m_closedLoopCosts;
    ///choose colour
    unsigned int m_colourIndex;
    QList<QColor> m_colours;
};

#endif // EVALUATION_H
