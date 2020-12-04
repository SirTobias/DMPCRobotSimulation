QT += core
QT -= gui
QT += testlib

TARGET = simulation-core_test
CONFIG += console
CONFIG -= app_bundle


QMAKE_CXXFLAGS += -std=gnu++11 -Wall -Wextra -pedantic -fvisibility=hidden -openmp

LIBS += -L"$$OUT_PWD/../../out" -lsimulation_core #Core-Library for the basic simulation functions

TEMPLATE = app

SOURCES += main.cpp \
    vectorhelpertest.cpp

HEADERS += \
    vectorhelpertest.h

