#-------------------------------------------------
#
# Project created by QtCreator 2015-01-05T11:14:52
#
#-------------------------------------------------

QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#CONFIG += qwt
TARGET = simulation_gui
#TEMPLATE = app
TEMPLATE = lib

QMAKE_CXXFLAGS += -Wall -Wextra -pedantic

INCLUDEPATH += $$PWD/../simulatoren_extern/qwt/src/
INCLUDEPATH += $$PWD/../simulatoren_extern/optimize/nlopt/
INCLUDEPATH += $$PWD/../simulation_core/

win32 {
 CONFIG(debug, debug|release) {
  message(DEBUG selected)
  DESTDIR = $$OUT_PWD/debug
 }
 else {
  message(RELEASE selected)
  DESTDIR = $$OUT_PWD/release
 }
}

DEPENDPATH += ../simulation_core/

SOURCES +=\
    plot.cpp \
    plot2d.cpp \
    evaluation.cpp \
    qwtcustomscaledraw.cpp
    #../simulation_core/databasecore.cpp

HEADERS  += mainwindow.h \
    plot.h \
    plot2d.h \
    evaluation.h \
    qwtcustomscaledraw.h

OTHER_FILES += \
    README.txt

LIBS += -L"$$OUT_PWD/../out" -lsimulation_core #Core-Library for the basic simulation functions
mkdirs = $$OUT_PWD/../out

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


#QMAKE_EXTRA_TARGETS += createDirs copyCoreLibs

#PRE_TARGETDEPS += createDirs copyCoreLibs

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
  #QMAKE_POST_LINK += $(COPY) -r $$OUT_PWD/*.so.* $$OUT_PWD/../out/
  QMAKE_POST_LINK += $(COPY) -r -P $$PWD/../simulatoren_extern/qwt/lib/*.so.* $$CUSTOUTPWD;
}
win32 {
  SOURCEWINPATH = $$shell_path($$DESTDIR/$${TARGET}.exe)
  TARGETWINPATH = $$shell_path($$CUSTOUTPWD)
  QMAKE_POST_LINK += $(COPY) $$SOURCEWINPATH $$TARGETWINPATH
}

DISTFILES += \
    Doxyfile
