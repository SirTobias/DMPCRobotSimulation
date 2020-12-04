#include "intersectionapplication.h"
#include "intersection.h"
#include "evaluation.h"
#include "car.h"
#include "intersectionparameters.h"
#include "simulationthread.h"

#include <QtCore/QFile>
#include <QtCore/QTime>
#include <QtTest/QTest>
#include <QtCore/QTextStream>
#include <QtCore/QStringBuilder>

#include <map>

InterSectionApplication::InterSectionApplication(int &argc, char **argv) :
    QApplication(argc, argv)
{
    std::cout << "start app..." << std::endl;
    //digraph for simulation
    //adevs::Digraph<SimulationObject*> simDiGraph;

    //evaluation instance
    //Evaluation eval;
    //intersection
    iw = new IntersectionWindow(InterSectionParameters::k, InterSectionParameters::m, InterSectionParameters::maxCars, InterSectionParameters::N, InterSectionParameters::T, InterSectionParameters::lambda, this);
    iw->show();
    std::cout << "create intersection..." << std::endl;

}
