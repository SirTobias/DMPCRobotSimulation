#-------------------------------------------------
#
# Project created by QtCreator 2014-09-23T09:54:43
#
#-------------------------------------------------

QT += core sql testlib
QT -= gui

#define release and debug directories
#RELEASE:DESTDIR = release
#DEBUG:DESTDIR = debug

win32 {
 CONFIG += dll
 CONFIG(debug, debug|release) {
  message(DEBUG selected)
  DESTDIR = $$OUT_PWD/debug
 }
 else {
  message(RELEASE selected)
  DESTDIR = $$OUT_PWD/release
 }
}

CONFIG += shared
CONFIG += C++11

DEFINES += SIM_CORE_SHARED_LIB #shared core library

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = simulation_core
TEMPLATE = lib
QMAKE_CXXFLAGS += -fvisibility=hidden
QMAKE_LFLAGS += -fopenmp
LIBS += -fopenmp


SOURCES += main.cpp\
    simulationresource.cpp \
    simulationactorlist.cpp \
    simulationactor.cpp \
    simulationobject.cpp \
    simulationresourceconfig.cpp \
    linearcongruentialgenerator.cpp \
    simulationobserver.cpp \
    simulationresourceobserver.cpp \
    simulationresourceobservernotifier.cpp \
    databasecore.cpp \
    enumvalues.cpp \
    vectorhelper.cpp

HEADERS  += \
    simulationresource.h \
    simulationactorlist.h \
    simulationactor.h \
    simulationobject.h \
    simdef.h \
    simulationresourceconfig.h \
    linearcongruentialgenerator.h \
    simulationobserver.h \
    simulationresourceobserver.h \
    simsharedlib.h \
    simulationresourceobservernotifier.h \
    databasecore.h \
    enumvalues.h \
    vectorhelper.h


OTHER_FILES += \
    README.txt

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

createDirs.commands = $(MKDIR) $$mkdirs


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

unix {
  QMAKE_POST_LINK += $(COPY) -P $$OUT_PWD/lib* $$CUSTOUTPWD
}
win32 {
  SOURCEWINPATH = $$shell_path($$DESTDIR/$${TARGET}.dll)
  TARGETWINPATH = $$shell_path($$CUSTOUTPWD)
  QMAKE_POST_LINK += $(COPY) $$SOURCEWINPATH $$TARGETWINPATH
}
