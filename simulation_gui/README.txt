Simulation_GUI - a GUI for the simulator based on adevs / is both a shared library and executable

1. requirements for project:
- QwtPlot from http://qwt.sourceforge.net/ with standard compilation
- simulation_core as dependant project

2. prequesites for compiling:
- put compiled libsimulation_core.so in /usr/local/lib
- put compiled libqwt.so in /usr/local/lib
- call sudo ldconfig -v /usr/local/lib

3. compile:
- run qmake and make in console or run the build process in Qt Creator
- enjoy
