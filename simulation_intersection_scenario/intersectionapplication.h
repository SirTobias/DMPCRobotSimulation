#ifndef INTERSECTIONAPPLICATION_H
#define INTERSECTIONAPPLICATION_H

#include <QtCore/QCoreApplication>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtCore/QMetaType>
#include "intersection.h"
#include "car.h"
#include "intersectionwindow.h"


class InterSectionApplication : public QApplication
{
    Q_OBJECT
public:
    explicit InterSectionApplication(int& argc, char** argv);


signals:

public slots:

private:
    QPointer<IntersectionWindow> iw;


};

#endif // INTERSECTIONAPPLICATION_H
