#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QPointer>

namespace Ui {
class MainWindow;
}

/**
 * @brief holds the main GUI window for the simulator
 *
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void activateSimulation();
    void initializeLinearCongruenceGenerator();
    void initProductionSimulation();
    void testDBMethods();
private:
    /// activate button for simulation starting
    QPushButton* m_pushActivateSimulation;
    ///activate button for initializing linear congruence generator
    QPushButton* m_pushActivateLinearCongruenceGenerator;
    /// description label
    QLabel* m_descrLabel;
    ///plot label
    QLabel* m_plotLabel;
    ///central widget to set the layout
    QWidget* m_centralWidget;
    ///activate button for testing database methods
    QPushButton* m_pushDBTest;
    ///SimulationThread for ProductionSzenario
    //QPointer<SimulationThread> m_prodSimThread;
    ///Production scenario
    QPushButton* m_pushProdSim;
};

#endif // MAINWINDOW_H
