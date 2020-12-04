#ifndef INTERSECTIONWINDOW_H
#define INTERSECTIONWINDOW_H

#include "recordvideo.h"
#include "cargui.h"
#include "car.h"
#include "cellgui.h"
#include "intersection.h"
#include "simulationthread.h"
#include "startbutton.h"
#include "databasethread.h"
#include "prioritysorter.h"

#include <QtWidgets/QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCommandLinkButton>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGraphicsItem>
#include <QtCore/QTextStream>
#include <QtWidgets/QGridLayout>
#include <QtCore/QPointer>

#include <iostream>
#include <vector>
#include <memory>

using IntersectionCellGui = QHash<unsigned int,QHash<unsigned int, std::shared_ptr<CellGui> > >;


/**
 * @brief The IntersectionWindow class graphically demonstrates Path algorithms of cars in an interseciton
 * contains a graphic scene, car objects, and intersection cell objects
 */
class IntersectionWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit IntersectionWindow(int k = InterSectionParameters::k, int m = InterSectionParameters::m,
                                int maxCars = InterSectionParameters::maxCars, int N = InterSectionParameters::N, double T = InterSectionParameters::T,
                                double lambda = InterSectionParameters::lambda, QObject *parent = 0);
    void start();
    void displayInfo(CellGui* cell);
    void displayCar(CarGui* car);
    ~IntersectionWindow();

private slots:

    void on_pauseButton_clicked();
    void on_resumeButton_clicked();
    void width_editing_finished(int newW);
    void height_editing_finished(int newH);
    void maxCars_editing_finished(int newMaxCars);
    void N_editing_finished(int newN);
    void samplingEditingFinished(double);
    void carIsSelected(QString carName);
    void setPriorityCriteria(const QString& priority);

public slots:
    void updateCarGUI(const QString& name, const double &newX, const double &newY);
    void updateCarGUIPrediction(const QString& name, const std::vector<std::vector<double> > &vec);
    void updateCarGUIDynamicReservations(const QString& name, const QMap<int, int>& occupiedCells);
    void updateCarGUIClearReservations(const QString &name);
    void updateCellGUI(const unsigned int& x, const unsigned int y, const unsigned int& reservations);
    void addCarGUI(const QString& name, const double& x, const double& y, const double& targetx, const double& targety);
    void addCellGUI(const unsigned int& x, const unsigned int& y, const unsigned int& reservations);
    void removeCarfromGUI(const QString& key, const int& row);
    void finished();
    void displayStep(int step);
    void getDBThread(QMap<QString,QVariantList> output);
    void reservationGUIChanged(int changed);
    void drawTitle();
    void drawCells(const unsigned int &rows, const unsigned int &cols);
    void carMinRadiusGUIChanged(int changed);
    void showPredictionGUI(int changed);
    void showConstraintMarginGUI(int changed);

private:
    unsigned int getNextValidColor(const unsigned int& index);

    QPointer<QGraphicsScene> scene;
    QObject* m_interSectionApp;
    QHash<QString, std::shared_ptr<CarGui>> carTable;
    IntersectionCellGui cellGrid;
    QPointer<SimulationThread> thread;
    QPointer<DatabaseThread> m_dbThread;

    int width, height, maxCars, N; //distances in simulation units
    const double lambda;
    ///sampling step
    double m_T;
    int v_max, h_max; //distances in graphics view units
    CarGui* clicked_car;  //based on mouse press
    QString selected_car;
    unsigned int target_x, target_y; //selected car's destination
    bool started;
    //division width/c for cell width in the GUI
    double m_divCol, m_divRow;
    QMap<QString, QMultiMap<int, int> > m_carOccupiedCells;
    bool m_showPrediction;
    bool m_showMinRadius;
    bool m_showConstraintMargin;
    PriorityCriteria m_priorityCriteria;
    ///serves for locking the occupied cell map. This should be read-locked, if a car is
    ///still removing its old prediction and tries to insert a new one
    QReadWriteLock m_rwLockOccupiedCellHash;

    //Window Widgets
    QWidget *centralwidget;
    QGraphicsView *graphicsView;
    QCommandLinkButton *pauseButton;
    QCommandLinkButton *resumeButton;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *widthLabel;
    QSpinBox *widthText;
    QLabel *heightLabel;
    QSpinBox *heightText;
    QLabel *maxCarsLabel;
    QSpinBox *maxCarsText;
    QLabel *nLabel;
    QSpinBox *nText;
    QLabel *samplingLabel;
    QDoubleSpinBox *samplingText;
    QLabel *cellLabel;
    QTextBrowser *cellText;
    QLabel *carsLabel;
    QTextBrowser *carsText;
    QLabel* dynamicReservLabel;
    QCheckBox* dynamicReservCheckBox;
    QCheckBox* carRadiusCheckBox;
    QLabel* radiusLabel;
    QLabel* showPredictionLabel;
    QCheckBox* showPredictionCheckBox;
    QLabel* showConstraintMarginLabel;
    QCheckBox* showConstraintMarginCheckBox;
    QMenuBar *menubar;
    QStatusBar *statusbar;
    QGraphicsTextItem* title;
    QLabel *selectCarLabel;
    QComboBox *carSelectBox;
    QLabel *carsAtTarget;
    QTextBrowser *carsAtTargetText;
    StartButton* screenstartbutton;
    unsigned int m_colorIndex;
    RecordVideo m_recVideo;
    CellGuiType m_reservGUIType;
    QList<QColor> m_colors;
    QLabel* priorityLabel;
    QComboBox* priorityComboBox;
    //set up functions
    void setup();
    void retranslateUi();

signals:
    void pause();
    void resume();

};

#endif // INTERSECTIONWINDOW_H
