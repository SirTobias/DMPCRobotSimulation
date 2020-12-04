QT += core
QT -= gui
QT += testlib


TARGET = simulation_gui_test
CONFIG += console
CONFIG -= app_bundle

CONFIG += testcase #run it via "make check"

TEMPLATE = app

SOURCES += main.cpp \
    systemfunctiontest.cpp

HEADERS += \
    systemfunctiontest.h

