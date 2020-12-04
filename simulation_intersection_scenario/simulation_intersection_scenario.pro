#-------------------------------------------------
#
# Project created by QtCreator 2015-02-13T13:52:09
#
#-------------------------------------------------

QT       += core sql
QT       += widgets opengl
QT       += testlib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QMAKE_CXXFLAGS += -std=gnu++11 -Wall -Wextra -pedantic -g

INCLUDEPATH += $$PWD/../simulation_core/
INCLUDEPATH += $$PWD/../simulatoren_extern/
INCLUDEPATH += $$PWD/../simulatoren_extern/optimize/
INCLUDEPATH += $$PWD/../simulatoren_extern/google/src
INCLUDEPATH += $$PWD/../simulatoren_extern/optimize/nlopt/api
INCLUDEPATH += $$PWD/../simulatoren_extern/qwt/src/
#INCLUDEPATH += $$PWD/../simulatoren_extern/qwt3d/include
INCLUDEPATH += $$PWD/../simulation-core/

DEPENDPATH += ../simulation_core/

TARGET = simulation_intersection_scenario
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

win32 {
 CONFIG(debug, debug|release) {
  message(DEBUG selected)
  DESTDIR = $$OUT_PWD/debug
  QMAKE_CXXFLAGS += -g
 }
 else {
  message(RELEASE selected)
  DESTDIR = $$OUT_PWD/release
 }
}


SOURCES += main.cpp \
    intersection.cpp \
    intersectioncell.cpp \
    reservationcell.cpp \
    path.cpp \
    pathitem.cpp \
    car.cpp \
    mpccontroller.cpp \
    pathcalculation.cpp \
    astarpathcalculation.cpp \
    costfunction.cpp \
    systemfunction.cpp \
    intersectionapplication.cpp \
    pathcontrolmap.cpp \
    constraint.cpp \
    constraintfunction.cpp \
    evaluation.cpp \
    cargui.cpp \
    cellgui.cpp \
    intersectionwindow.cpp \
    simulationthread.cpp \
    startbutton.cpp \
    globalcarlist.cpp \
    carinformation.cpp \
    intersectionparameters.cpp \
    costcriteria.cpp \
    databasethread.cpp \
    constraintmin.cpp \
    constraintmax.cpp \
    contintraject.cpp \
    recordvideo.cpp \
    plot.cpp \
    plot2d.cpp \
    systemfunctiontest.cpp \
    qwtcustomscaledraw.cpp \
    prioritysorter.cpp \
    constraintdirectional.cpp \
    arrivalcar.cpp \
    enterintersectiondist.cpp \
    plot3d.cpp \
    distparam.cpp \
    floydwarshallpathcalculation.cpp \
    dstarlite.cpp \
    cargroupqueue.cpp \
    cargroup.cpp \
    extendeddata.cpp

HEADERS += \
    intersection.h \
    intersectioncell.h \
    reservationcell.h \
    path.h \
    pathitem.h \
    car.h \
    mpccontroller.h \
    pathcalculation.h \
    astarpathcalculation.h \
    costfunction.h \
    systemfunction.h \
    intersectionapplication.h \
    pathcontrolmap.h \
    constraint.h \
    constraintfunction.h \
    evaluation.h \
    cargui.h \
    cellgui.h \
    intersectionwindow.h \
    simulationthread.h \
    startbutton.h \
    intersectionparameters.h \
    globalcarlist.h \
    carinformation.h \
    costcriteria.h \
    databasethread.h \
    constraintmin.h \
    constraintmax.h \
    contintraject.h \
    recordvideo.h \
    plot.h \
    plot2d.h \
    systemfunctiontest.h \
    qwtcustomscaledraw.h \
    prioritysorter.h \
    constraintdirectional.h \
    arrivalcar.h \
    enterintersectiondist.h \
    plot3d.h \
    distparam.h \
    floydwarshallpathcalculation.h \
    dstarlite.h \
    cargroupqueue.h \
    cargroup.h \
    extendeddata.h


OTHER_FILES += \
    README.txt \
    README.txt \
    messages.proto

unix {
LIBS += -L"$$OUT_PWD/../out" -L"$$PWD/../simulatoren_extern/google/src/.libs" -lsimulation_core -lpthread -lnlopt#Core-Library for the basic simulation functions and NLOpt-package and NLOpt-package
LIBS += -L$$PWD/../simulatoren_extern/qwt/lib -lqwt -lqwtmathml#QWT for Plot
#LIBS += -L$$PWD/../simulatoren_extern/qwt3d/lib -lqwtplot3d -lGLU#Qwt3dPlot
}
win32 {
  LIBS += -L"$$OUT_PWD/../out" -L"$$PWD/../simulatoren_extern/google/src/.libs" -L"$$PWD/../simulatoren_extern/qwt/lib" -lsimulation_core -lnlopt-0 -lqwt -lqwtmathml #Core-Library for the basic simulation functions and NLOpt-package
}

#as there system calls in windows does not accept / in paths,
#encapsulation with double quotes is necessary
#but built-in functions of qmake like exists cannot handle double-quote paths
#therefore CUSTOUTPWDWIN with quotes and CUSTOUTPWD without quotes
CUSTOUTPWDWIN = \"$$OUT_PWD/../out\"
CUSTOUTPWD = $$OUT_PWD/../out
unix {
  mkdirs = $$CUSTOUTPWD
}
win32 {
mkdirs = $$CUSTOUTPWDWIN
}

unix {
  QMAKE_EXTRA_TARGETS += createDirs
  PRE_TARGETDEPS += createDirs
}
win32 {
 #workaround for mkdir -p
 !exists($$CUSTOUTPWD) {
  message($$CUSTOUTPWD)
  QMAKE_EXTRA_TARGETS += createDirs
  PRE_TARGETDEPS += createDirs
 }
 exists($$CUSTOUTPWD) {
  message(out-dir exists.)
 }
}

nlopt.target = libnlopt-0.*
nlopt.commands = $(COPY) $$shell_path($$PWD/../simulatoren_extern/optimize/nlopt/$$nlopt.target) $$shell_path($$OUT_PWD/../out/)
#message ($$shell_path($$PWD/../simulatoren_extern/optimize/nlopt/$$nlopt.target))


#message($$nlopt.target)

unix {
  QMAKE_POST_LINK += $(COPY) -P $$OUT_PWD/$$TARGET* $$CUSTOUTPWD
}
win32 {
  SOURCEWINPATH = $$shell_path($$DESTDIR/$${TARGET}.exe)
  TARGETWINPATH = $$shell_path($$CUSTOUTPWD)
  QMAKE_POST_LINK += $(COPY) $$SOURCEWINPATH $$TARGETWINPATH
  QMAKE_EXTRA_TARGETS += nlopt #here the internal qmake target has to be taken (nothing to do with Make!)
  PRE_TARGETDEPS += $$nlopt.target #here the filename has to be taken as in Makefile
}

DISTFILES += \
    pgsql.py
