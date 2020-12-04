#include "intersectionwindow.h"
#include <intersectionapplication.h>

#include "prioritysorter.h"

#include <QtCore/QTextStream>
#include <QtCore/QStandardPaths>
#include <QtCore/QDebug>
#include <QtCore/QTime>
#include <QtCore/QMetaType>
#include <QtTest/QTest>


#include <chrono>
#include <thread>

/** @brief constructs a window application containing an m-by-k intersection with cars
 * @param m width of intersection
 * @param k length of intersection
 * @param maxCars maximum number of cars in intersection
 * @param N
 * @param lambda
 * @param parent parent QObject
 */
IntersectionWindow::IntersectionWindow(int k, int m, int maxCars, int N, double T, double lambda, QObject *parent) :
    QMainWindow(nullptr),
    width(k+1),
    height(m+1),
    maxCars(maxCars),
    N(N),
    lambda(lambda),
    m_T(T),
    v_max((k+1)*50),
    h_max((m+1)*50),
    clicked_car(nullptr),
    selected_car(""),
    target_x(0),
    target_y(0),
    m_colorIndex(0),
    m_recVideo(this, QDir::currentPath() % QString("/simInterSec/")),
    m_reservGUIType(CellGuiType::CURRENTRESERVED),
    m_divCol(0.0),
    m_divRow(0.0),
    m_showPrediction(false),
    m_showMinRadius(false),
    m_showConstraintMargin(false),
    m_rwLockOccupiedCellHash(QReadWriteLock::NonRecursive),
    m_priorityCriteria(PriorityCriteria::FIXED)
{
    m_colors.append(QColor(174, 63, 63));
    m_colors.append(QColor(226, 192, 149));
    m_colors.append(QColor(136, 183, 184));
    m_colors.append(QColor(0,0,0));
    setup();
    //Set up form and dialog
    scene = new QGraphicsScene(this);
    graphicsView->setRenderHint(QPainter::Antialiasing);
    m_interSectionApp = parent;

    //Display parameters
    graphicsView->setScene(scene);
    screenstartbutton = new StartButton();
    scene->addItem(screenstartbutton);
    m_recVideo.setWidget(this);
}

/** @brief Display the cell location and number of reservations onto cell text window
 * @param cell pointer to cell graphic that has been clicked on
 */
void IntersectionWindow::displayInfo(CellGui *cell)
{
    int x = cell->x()/50;
    int y = cell->y()/50;

    unsigned int reservations = cell->getNumberOfReservations();

    QString text = "x: " + QString::number(x) + "\ny: " + QString::number(y) + "\nReservations: " + QString::number(reservations) +"\n";
    cellText->setText(text);
}

/** @brief Display the cars located inside cell
 * @param car pointer to car graphic that has been clicked on
 */
void IntersectionWindow::displayCar(CarGui *car)
{

    //if cars are located in different cell from previous display
    if (clicked_car == nullptr)
    {
        clicked_car = car;
    }
    else if (clicked_car == car)
    {
        carsText->clear();
    }
    else if (car->x() != clicked_car->x() || car->y() != clicked_car->y())
    {
       carsText->clear();
       clicked_car = car;
    }

    QString text = carsText->toPlainText();
    text = text+car->getName()+"\n";
    carsText->setText(text);

}

/** @brief Update the next position of specific car
 * @param name  name of car object
 * @param newX new x-coordinate of car
 * @param newY new y-coordinate of car
 */
void IntersectionWindow::updateCarGUI(const QString &name, const double& newX, const double& newY)
{
    //qDebug() << "Position of " << name << "(" << newX << "," << newY << ")";
    carTable[name]->updatePos(newX,newY);
}

/**
 * @brief IntersectionWindow::updateCarGUIPrediction
 * @param name
 * @param vec
 */
void IntersectionWindow::updateCarGUIPrediction(const QString& name, const std::vector<std::vector<double> >& vec) {
    if (!m_showMinRadius) {
        if (m_showPrediction) {
            carTable[name]->updatePrediction(vec);
        }
    }
}

/**
 * @brief IntersectionWindow::updateCarGUIDynamicReservations colors only for the current time stamps the cells according to the predictions
 * @param name name of car to identify color
 * @param occupiedCells predicted cells
 */
void IntersectionWindow::updateCarGUIDynamicReservations(const QString& name, const QMap<int, int>& occupiedCells) {
    if (!m_showMinRadius) {
        if (m_reservGUIType == CellGuiType::CURRENTRESERVED) {
            m_rwLockOccupiedCellHash.lockForRead();
            QColor color = carTable[name]->getColor();
            color.setAlpha(100);
            m_carOccupiedCells.remove(name);
            m_carOccupiedCells.insert(name, occupiedCells);
            //QColor white(Qt::white);
            for (auto it = occupiedCells.begin(); it != occupiedCells.end(); it++) {
                /*if (cellGrid[it.key()][it.value()]->getColor() != white) {
                    QColor colorSet = cellGrid[it.key()][it.value()]->getColor();
                    color.setRed(color.red()*0.5 + colorSet.red()*0.5);
                    color.setGreen(color.green()*0.5 + colorSet.green()*0.5);
                    color.setBlue(color.blue()*0.5 + colorSet.blue()*0.5);
                }*/
                cellGrid[it.key()][it.value()]->updateReservations(color);
            }
            m_rwLockOccupiedCellHash.unlock();
        }
    }
}

void IntersectionWindow::updateCarGUIClearReservations(const QString& name) {
    if (!m_showMinRadius) {
        if (m_reservGUIType == CellGuiType::CURRENTRESERVED) {
            //QColor white(Qt::white);
            if (m_carOccupiedCells.contains(name)) {
                m_rwLockOccupiedCellHash.lockForRead();
                QMultiMap<int, int> carMapToDelete =  m_carOccupiedCells.value(name);
                QColor colorCar = carTable[name]->getColor();
                colorCar.setAlpha(100);
                for (auto itMap = carMapToDelete.begin(); itMap != carMapToDelete.end(); itMap++) {
                    cellGrid[itMap.key()][itMap.value()]->removeReservation(colorCar);
                }
                /*for (QMap<QString, QMultiMap<int, int> >::const_iterator it = m_carOccupiedCells.begin(); it != m_carOccupiedCells.end(); it++) {
                    for (QMultiMap<int, int>::const_iterator itMap = it.value().begin(); itMap != it.value().end(); itMap++) {
                        cellGrid[itMap.key()][itMap.value()]->updateReservations(white);
                    }
                }*/
                m_rwLockOccupiedCellHash.unlock();
            }
        }
    }
}

/** @brief update the number of reservations in the cell
 * @param x x coordinate of cell
 * @param y y coordinate of cell
 * @param reservations: number of reservations in the cell
 */
void IntersectionWindow::updateCellGUI(const unsigned int &x, const unsigned int y, const unsigned int &reservations)
{
    if (m_reservGUIType == CellGuiType::ALLRESERVED) {
        cellGrid[x][y]->updateReservations(reservations);
    }
}

/** @brief Add new car onto the scene
 * also adds reference to the car into hash table of cars and WidgetList
 * @param name
 * @param x x-coordinate of car
 * @param y y-coordinate of car
 */
void IntersectionWindow::addCarGUI(const QString& name, const double& x, const double& y, const double& targetx, const double& targety)
{

    //cari to cari+1
    QString legendItem = name;
    /*legendItem.replace("car", "R ");
    int indexNumber = legendItem.indexOf(QRegExp("\\d?$"));
    int carNumber = QString(legendItem).remove(0, indexNumber).toInt();
    legendItem = legendItem.remove(indexNumber, legendItem.length() - indexNumber);
    legendItem += QChar::Space;
    carNumber++;
    */
    double minRadius = InterSectionParameters::robotDiameter;
    if (!m_showMinRadius) {
        minRadius = 0.0;
    }
    std::shared_ptr<CarGui> car_item = std::make_shared<CarGui>(legendItem,
                                                                x, y, targetx, targety,
                                                                m_colors.at(getNextValidColor(m_colorIndex)),
                                                                h_max / (double)width, scene, minRadius, m_showPrediction );
    m_colorIndex++;
    if (carTable.contains(name)) {
        scene->removeItem(carTable.value(name).get());
    }
    carTable[name] = car_item;
    //scene->addItem(car_item.get());
    //scene->addItem(car_item->getTrajectory().get());
    carSelectBox->addItem(name);
}

/** @brief Add new cell object onto scene
 * also add reference to the cell in hash table
 * @param x x-coordinate of the cell
 * @param y y-coordinate of the cell
 * @param reservations number of reservations cell contains
 */
void IntersectionWindow::addCellGUI(const unsigned int &x, const unsigned int &y, const unsigned int &reservations)
{
    double constraintMargin = 0.0;
    if (m_showConstraintMargin && thread) {
        constraintMargin = thread->getOverallConstraintMargin() - thread->getCurrentCellSize();
    }
    std::shared_ptr<CellGui> theCell = std::make_shared<CellGui>(x, y, reservations, m_divCol , m_reservGUIType, scene, constraintMargin, h_max / width);
    if (cellGrid[x].contains(y)) {
        std::shared_ptr<CellGui> oldCell = cellGrid[x].value(y);
        scene->removeItem(oldCell.get());
    }
    cellGrid[x].insert(y, theCell);
    scene->addItem(theCell.get());
}

/** @brief Remove car from GUI for when car reaches its destination
 * @param key: name of car to remove
 */
void IntersectionWindow::removeCarfromGUI(const QString &key, const int &row)
{
    std::this_thread::sleep_for(std::chrono::seconds(2));
    //Car has to be also removed out of scene
    //probably to remove first, then remove from scene, because the trajectories and the lines
    //have also to be removed
    //TODO: look, if this has to be removed or carTable.remove(...) is OK
    //showing prediction is a problem
    //scene->removeItem(carTable.value(key).get());
    QList<QGraphicsItem*> items = scene->items();
    carTable.remove(key);
    carSelectBox->removeItem(row); //remove from select list
    items = scene->items();

    //display names of cars that reached target
    QString text = carsAtTargetText->toPlainText();
    //text = text + key + ": (" + QString::number(x) + ", " + QString::number(y) + ")\n";
    text = text + key + "\n";
    carsAtTargetText->setText(text);
    if (carTable.size() == 0) {
        m_recVideo.stop();
    }

}

/** @brief Announces that simulation is finished
 */
void IntersectionWindow::finished()
{
    title->setPlainText(QString("FINISHED!"));
    carSelectBox->clear();
}

/** @brief Updates the title to be current step
 */
void IntersectionWindow::displayStep(int step)
{
    QString displayStep = "STEP: " + QString::number(step);
    title->setPlainText(displayStep);
}


/** @brief Pauses the simulation when button is clicked
 */
void IntersectionWindow::on_pauseButton_clicked()
{
    emit pause();
    QCoreApplication::processEvents();
}

/** @brief Resumes the simulation when button is clicked
 */
void IntersectionWindow::on_resumeButton_clicked()
{
    emit resume();
}

/** @brief Update new width for simulation
 */
void IntersectionWindow::width_editing_finished(int newW)
{
    width = newW;
    h_max = width*50;
}

/** @brief Update new height for simulation
 */
void IntersectionWindow::height_editing_finished(int newH)
{
    height = newH;
    v_max = height*50;
}

/** @brief Update new max number of cars for simulation
 */
void IntersectionWindow::maxCars_editing_finished(int newMaxCars)
{
    maxCars = newMaxCars;
}

/** @brief Update new N for simulation
 */
void IntersectionWindow::N_editing_finished(int newN)
{
    N = newN;
}

/**
 * @brief IntersectionWindow::samplingEditingFinished editing the sampling step size
 * @param newSampling
 */
void IntersectionWindow::samplingEditingFinished(double newSampling) {
    m_T = newSampling;
}

/** @brief Car is selected out of list to display its target
 * @param carName name of car
 */
void IntersectionWindow::carIsSelected(QString carName)
{
    if (carName == selected_car) {
        return;
    }

    if (carTable.find(selected_car) != carTable.end()) {
        carTable[selected_car]->unselected();
    }

    cellGrid[target_x][target_y]->setTargetStatus(false);

    target_x = carTable[carName]->targetX();
    target_y = carTable[carName]->targetY();
    carTable[carName]->selected();
    cellGrid[target_x][target_y]->setTargetStatus(true);
    selected_car = carName;
}

/**
 * @brief IntersectionWindow::reservationGUIChanged if checked == CURRENTRESERVED, it shows
 * the current occupied cells by each car
 * if unchecked, it will show the reservations over the whole simulation time
 * @param changed
 */
void IntersectionWindow::reservationGUIChanged(int changed) {
    if (changed == Qt::CheckState::Checked) {
        m_reservGUIType = CellGuiType::CURRENTRESERVED;
    }
    else if (changed == Qt::CheckState::Unchecked) {
        m_reservGUIType = CellGuiType::ALLRESERVED;
    }
}

/**
 * @brief IntersectionWindow::carMinRadiusGUIChanged if checked, the minimum radius for each car is shown,
 *  standard is d=0.5, otherwise none
 * @param changed
 */
void IntersectionWindow::carMinRadiusGUIChanged(int changed) {
    if (changed == Qt::CheckState::Checked) {
        m_showMinRadius = true;
        for (auto it = carTable.begin(); it != carTable.end(); it++) {
            it.value()->setRadius(InterSectionParameters::robotDiameter);
        }
    }
    else if (changed == Qt::CheckState::Unchecked) {
        m_showMinRadius = false;
        for (auto it = carTable.begin(); it != carTable.end(); it++) {
            it.value()->setRadius(0.0);
        }
    }
}

/**
 * @brief IntersectionWindow::showPredictionGUI if checked, it shows the prediction including the occupied cells
 * @param changed
 */
void IntersectionWindow::showPredictionGUI(int changed) {
    if (changed == Qt::CheckState::Checked) {
        m_showPrediction = true;
    }
    else if (changed == Qt::CheckState::Unchecked) {
        m_showPrediction = false;
    }
}

/**
 * @brief IntersectionWindow::showConstraintMarginGUI if checked, it shows the constraint margin around each
 * occupied cell
 * @param changed
 */
void IntersectionWindow::showConstraintMarginGUI(int changed) {
    if (changed == Qt::CheckState::Checked) {
        m_showConstraintMargin = true;
    }
    else if (changed == Qt::CheckState::Unchecked) {
        m_showConstraintMargin = false;
    }
}

/**
 * @brief IntersectionWindow::setPriorityCriteria changes the criteria to the chosen entry from the Combobox
 * @param priority
 */
void IntersectionWindow::setPriorityCriteria(const QString& priority) {
    for (int i = 0; i < getPriorityCriteriaMapSize(); i++) {
        if (QString(priorityCriteriaMap[i].text) == priority) {
            m_priorityCriteria = priorityCriteriaMap[i].criteria;
        }
    }
}

IntersectionWindow::~IntersectionWindow()
{
    /*scene->clear();
    int max = carTable.size();
    for (int i = 0; i < max; i++){
        carTable.clear();
    }

    max = cellGrid.size();
    for (int i = 0; i <= max; i++){
        cellGrid[i].clear();
    }
    cellGrid.clear();*/

}

/** @brief Private function to initialize widgets
 */
void IntersectionWindow::setup()
{
    if (this->objectName().isEmpty())
        this->setObjectName(QStringLiteral("IntersectionWindow"));
    this->resize(970, 900);
    centralwidget = new QWidget(this);
    centralwidget->setObjectName(QStringLiteral("centralwidget"));
    graphicsView = new QGraphicsView(centralwidget);
    graphicsView->setObjectName(QStringLiteral("graphicsView"));
    graphicsView->setGeometry(QRect(110, 40, 741, 741));
    pauseButton = new QCommandLinkButton(centralwidget);
    pauseButton->setObjectName(QStringLiteral("pauseButton"));
    pauseButton->setGeometry(QRect(860, 40, 93, 31));
    resumeButton = new QCommandLinkButton(centralwidget);
    resumeButton->setObjectName(QStringLiteral("resumeButton"));
    resumeButton->setGeometry(QRect(860, 73, 93, 31));
    /*startButton = new QCommandLinkButton(centralwidget);
    startButton->setObjectName(QStringLiteral("startButton"));
    startButton->setGeometry(QRect(210, 0, 93, 31));*/
    layoutWidget = new QWidget(centralwidget);
    layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
    layoutWidget->setGeometry(QRect(0, 40, 113, 311));
    verticalLayout = new QVBoxLayout(layoutWidget);
    verticalLayout->setSpacing(2);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
    widthLabel = new QLabel(layoutWidget);
    widthLabel->setObjectName(QStringLiteral("widthLabel"));

    verticalLayout->addWidget(widthLabel);

    widthText = new QSpinBox(layoutWidget);
    widthText->setObjectName(QStringLiteral("widthText"));
    widthText->setValue(width);

    verticalLayout->addWidget(widthText);

    heightLabel = new QLabel(QString::number(height), layoutWidget);
    heightLabel->setObjectName(QStringLiteral("heightLabel"));

    verticalLayout->addWidget(heightLabel);

    heightText = new QSpinBox(layoutWidget);
    heightText->setObjectName(QStringLiteral("heightText"));
    heightText->setValue(height);

    verticalLayout->addWidget(heightText);

    maxCarsLabel = new QLabel(layoutWidget);
    maxCarsLabel->setObjectName(QStringLiteral("maxCarsLabel"));

    verticalLayout->addWidget(maxCarsLabel);

    maxCarsText = new QSpinBox(layoutWidget);
    maxCarsText->setObjectName(QStringLiteral("maxCarsText"));
    maxCarsText->setValue(maxCars);

    verticalLayout->addWidget(maxCarsText);

    nLabel = new QLabel(layoutWidget);
    nLabel->setObjectName(QStringLiteral("nLabel"));

    verticalLayout->addWidget(nLabel);

    nText = new QSpinBox(layoutWidget);
    nText->setObjectName(QStringLiteral("nText"));
    nText->setValue(N);


    verticalLayout->addWidget(nText);

    samplingLabel = new QLabel(layoutWidget);
    samplingLabel->setObjectName(QStringLiteral("samplingLabel"));
    verticalLayout->addWidget(samplingLabel);

    samplingText = new QDoubleSpinBox(layoutWidget);
    samplingText->setObjectName(QStringLiteral("samplingText"));
    samplingText->setValue(m_T);
    samplingText->setSingleStep(0.25);
    verticalLayout->addWidget(samplingText);


    //show the reserved occupied cells in the color of the car
    dynamicReservLabel = new QLabel(layoutWidget);
    dynamicReservLabel->setObjectName("dynamicReservLabel");
    verticalLayout->addWidget(dynamicReservLabel);
    dynamicReservCheckBox = new QCheckBox(layoutWidget);
    if (m_reservGUIType == CellGuiType::CURRENTRESERVED) {
        dynamicReservCheckBox->setCheckState(Qt::CheckState::Checked);
    }
    else {
        dynamicReservCheckBox->setCheckState(Qt::CheckState::Unchecked);
    }
    verticalLayout->addWidget(dynamicReservCheckBox);

    //show minimum radius around each car
    radiusLabel = new QLabel(layoutWidget);
    radiusLabel->setObjectName("carRadiusLabel");
    verticalLayout->addWidget(radiusLabel);
    carRadiusCheckBox = new QCheckBox(layoutWidget);
    carRadiusCheckBox->setObjectName("carRadiusCheckBox");
    verticalLayout->addWidget(carRadiusCheckBox);
    carRadiusCheckBox->setCheckState(Qt::CheckState::Unchecked);

    //show predictions for cars (including occupied cells)
    showPredictionLabel = new QLabel(layoutWidget);
    showPredictionLabel->setObjectName("showPredictionLabel");
    verticalLayout->addWidget(showPredictionLabel);
    showPredictionCheckBox = new QCheckBox(layoutWidget);
    showPredictionCheckBox->setObjectName("showPrediction");
    verticalLayout->addWidget(showPredictionCheckBox);
    showPredictionCheckBox->setCheckState(Qt::CheckState::Unchecked);

    //show constraint margins for the occupied cells (psi)
    showConstraintMarginLabel = new QLabel(layoutWidget);
    showConstraintMarginLabel->setObjectName("showConstraintMarginLabel");
    verticalLayout->addWidget(showConstraintMarginLabel);
    showConstraintMarginCheckBox = new QCheckBox(layoutWidget);
    showConstraintMarginCheckBox->setObjectName("showConstraintMarginCheckBox");
    verticalLayout->addWidget(showConstraintMarginCheckBox);

    //priority criteria
    priorityLabel = new QLabel(layoutWidget);
    priorityLabel->setObjectName("priorityLabel");
    verticalLayout->addWidget(priorityLabel);
    priorityComboBox = new QComboBox(layoutWidget);
    priorityComboBox->setObjectName("priorityComboBox");
    priorityComboBox->addItems(PrioritySorter::getTextForCriteria());
    priorityComboBox->setCurrentText(priorityCriteriaMap[0].text);
    verticalLayout->addWidget(priorityComboBox);

    //label for chosen car
    cellLabel = new QLabel(layoutWidget);
    cellLabel->setObjectName(QStringLiteral("cellLabel"));
    cellLabel->setGeometry(QRect(11, 305, 91, 80));
    verticalLayout->addWidget(cellLabel);
    cellText = new QTextBrowser(layoutWidget);
    cellText->setObjectName(QStringLiteral("cellText"));
    cellText->setGeometry(QRect(11, 285, 91, 75));
    verticalLayout->addWidget(cellText);

    carsLabel = new QLabel(centralwidget);
    carsLabel->setObjectName(QStringLiteral("carsLabel"));
    carsLabel->setGeometry(QRect(11, 362, 91, 16));
    carsText = new QTextBrowser(centralwidget);
    carsText->setObjectName(QStringLiteral("carsText"));
    carsText->setGeometry(QRect(11, 380, 91, 91));
    this->setCentralWidget(centralwidget);
    menubar = new QMenuBar(this);
    menubar->setObjectName(QStringLiteral("menubar"));
    menubar->setGeometry(QRect(0, 0, 940, 21));
    this->setMenuBar(menubar);
    menubar = new QMenuBar(this);
    menubar->setObjectName(QStringLiteral("dataBaseFunctions"));
    statusbar = new QStatusBar(this);
    statusbar->setObjectName(QStringLiteral("statusbar"));
    this->setStatusBar(statusbar);

    selectCarLabel = new QLabel(centralwidget);
    selectCarLabel->setObjectName(QStringLiteral("selectCarLabel"));
    selectCarLabel->setGeometry(QRect(860, 130, 93, 16));
    carSelectBox = new QComboBox(centralwidget);
    carSelectBox->setObjectName(QStringLiteral("carSelect"));
    carSelectBox->setGeometry(860, 150, 93, 25);

    carsAtTarget = new QLabel(centralwidget);
    carsAtTarget->setObjectName(QStringLiteral("carArrivedLabel"));
    carsAtTarget->setGeometry(860, 265, 93, 16);
    carsAtTargetText = new QTextBrowser(centralwidget);
    carsAtTargetText->setObjectName(QStringLiteral("carArrivedText"));
    carsAtTargetText->setGeometry(860, 285, 93, 200);
    retranslateUi();

    //QMetaObject::connectSlotsByName(this);

    //connect widget signals and slots
    connect(pauseButton, SIGNAL(clicked()), this, SLOT(on_pauseButton_clicked()));
    connect(resumeButton, SIGNAL(clicked()), this, SLOT(on_resumeButton_clicked()));
    connect(widthText, SIGNAL(valueChanged(int)), this, SLOT(width_editing_finished(int)));
    connect(heightText, SIGNAL(valueChanged(int)), this, SLOT(height_editing_finished(int)));
    connect(maxCarsText, SIGNAL(valueChanged(int)), this, SLOT(maxCars_editing_finished(int)));
    connect(nText, SIGNAL(valueChanged(int)), this, SLOT(N_editing_finished(int)));
    connect(samplingText, SIGNAL(valueChanged(double)), this, SLOT(samplingEditingFinished(double)));
    connect(carSelectBox, SIGNAL(activated(QString)), this, SLOT(carIsSelected(QString)));
    connect(dynamicReservCheckBox, SIGNAL(stateChanged(int)), this, SLOT(reservationGUIChanged(int)));
    connect(carRadiusCheckBox, SIGNAL(stateChanged(int)), this, SLOT(carMinRadiusGUIChanged(int)));
    connect(showPredictionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(showPredictionGUI(int)));
    connect(showConstraintMarginCheckBox, SIGNAL(stateChanged(int)), this, SLOT(showConstraintMarginGUI(int)));
    connect(priorityComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(setPriorityCriteria(QString)));
}

/** @brief Private function to initialize widgets
 */
void IntersectionWindow::retranslateUi()
{
    this->setWindowTitle(QApplication::translate("IntersectionWindow", "Intersection Window", 0));
    pauseButton->setText(QApplication::translate("IntersectionWindow", "Pause", 0));
    resumeButton->setText(QApplication::translate("IntersectionWindow", "Resume", 0));
    //startButton->setText(QApplication::translate("IntersectionWindow", "Start", 0));
    widthLabel->setText(QApplication::translate("IntersectionWindow", "Width:", 0));
    heightLabel->setText(QApplication::translate("IntersectionWindow", "Height:", 0));
    maxCarsLabel->setText(QApplication::translate("IntersectionWindow", "Amount of Cars:", 0));
    nLabel->setText(QApplication::translate("IntersectionWindow", "N:", 0));
    samplingLabel->setText(QApplication::translate("IntersectionWindow", "T"));
    cellLabel->setText(QApplication::translate("IntersectionWindow", "Intersection Cell: ", 0));
    carsLabel->setText(QApplication::translate("IntersectionWindow", "Cars in Cell:",0));
    selectCarLabel->setText(QApplication::translate("IntersectionWindow", "Select Car:", 0));
    carsAtTarget->setText(QApplication::translate("IntersectionWindow", "Arrived Cars:", 0));
    dynamicReservLabel->setText(QApplication::translate("IntersectionWindow", "dynamic reservation", 0));
    radiusLabel->setText(QApplication::translate("IntersectionWindow", "car radius:", 0));
    showPredictionLabel->setText(QApplication::translate("IntersectionWindow", "predictions", 0));
    showConstraintMarginLabel->setText(QApplication::translate("IntersectionWindow", "constraint margins", 0));
}

/** @brief Draws the intersection and starts simulation thread
 */
void IntersectionWindow::start()
{
    //disable LineEdits
    widthText->setDisabled(true);
    heightText->setDisabled(true);
    maxCarsText->setDisabled(true);
    nText->setDisabled(true);

    scene->setSceneRect(0, 0, v_max, h_max);
    scene->clear();
    //Create grid
    /*QPen blackPen(Qt::black);
    blackPen.setWidth(1);
    blackPen.setStyle(Qt::DashLine);
    blackPen.setColor(Qt::black);

    scene->setSceneRect(0, 0, v_max, h_max);
    scene->clear();

    // Add the vertical lines
    for (int x=0; x<=h_max; x+=50){
        if (x == 0 || x == h_max)
            scene->addLine(x, -100, x, v_max+100, blackPen);
        else
            scene->addLine(x,0,x,v_max, blackPen);
    }
    // Add the horizontal lines

    for (int y=0; y<=v_max; y+=50){
        if (y == 0 || y == v_max)
            scene->addLine(-100, y, h_max+100, y, blackPen);
        else
            scene->addLine(0,y,h_max,y, blackPen);
    }

    //add rows to cellgrid hash table
    for (int i = 0; i < width; i++)
    {
        cellGrid.insert(i, QHash<unsigned int, std::shared_ptr<CellGui> >());
    }*/


    CommunicationScheme commScheme = CommunicationScheme::CONTINUOUS;
    thread = new SimulationThread(width, height, maxCars, N, m_T, lambda, {-1.0, 1.0}, {0.5, 0.5}, this, InterSectionParameters::robotDiameter, m_priorityCriteria, commScheme,PathAlgorithm::MPCCOBYLA);
    if (commScheme == CommunicationScheme::FULL || commScheme == CommunicationScheme::DIFFERENTIAL
            || commScheme == CommunicationScheme::MINMAXINTERVAL || commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        drawCells(thread->getGridHeight(), thread->getGridWidth());
    }
    drawTitle();



    //connect thread signals and slots
    connect(thread, SIGNAL(addCarGUI(const QString&, const double&, const double&, const double&, const double&)), this, SLOT(addCarGUI(const QString&, const double&, const double&, const double&, const double&)));
    if (commScheme == CommunicationScheme::FULL || commScheme == CommunicationScheme::DIFFERENTIAL
            || commScheme == CommunicationScheme::MINMAXINTERVAL || commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        connect(thread, SIGNAL(addCellGUI(const uint&,const uint&,const uint&)), this, SLOT(addCellGUI(const uint&, const uint&, const uint&)));
    }
    connect(thread, SIGNAL(updateCarGUI(const QString&, const double&, const double&)), this, SLOT(updateCarGUI(const QString&, const double&, const double&)));
    connect(thread, SIGNAL(updateCarGUIPrediction(const QString&, const std::vector<std::vector<double> >&)), this, SLOT(updateCarGUIPrediction(const QString&, const std::vector<std::vector<double> >&)));
    connect(thread, SIGNAL(updateCarGUIDynamicReservations(const QString&, const QMap<int, int>&)), this, SLOT(updateCarGUIDynamicReservations(const QString&, const QMap<int, int>&)));
    connect(thread, SIGNAL(updateCarGUIReservationsClear(const QString&)), this, SLOT(updateCarGUIClearReservations(const QString&)));
    connect(thread, SIGNAL(updateCellGUI(const uint&,const uint&,const uint&)), this, SLOT(updateCellGUI(const uint&, const uint&, const uint&)));
    connect(thread, SIGNAL(removeCarfromGUI(const QString&, const int&)), this, SLOT(removeCarfromGUI(const QString&, const int&)));
    connect(thread, SIGNAL(simFinished()), this, SLOT(finished()));
    connect(thread, SIGNAL(steps(int)), this, SLOT(displayStep(int)));
    if (commScheme == CommunicationScheme::FULL || commScheme == CommunicationScheme::DIFFERENTIAL
            || commScheme == CommunicationScheme::MINMAXINTERVAL || commScheme == CommunicationScheme::MINMAXINTERVALMOVING) {
        connect(thread, SIGNAL(drawCells(const unsigned int&, const unsigned int&)), this, SLOT(drawCells(const unsigned int&,const unsigned int&)));
    }

    connect(this, SIGNAL(pause()), thread, SLOT(pause()));
    connect(this, SIGNAL(resume()), thread, SLOT(resume()));

    thread->startSimulation();
    //m_recVideo.start(100);
}

void IntersectionWindow::drawTitle() {
    QString displayStep = "STEP: " + QString::number(0);
    title = scene->addText(displayStep, QFont("Helvetica", 12, QFont::Bold));
    title->setPos(70, -50);
}

/**
 * @brief InterSectionWindow::drawCells draw cells in dependence of cell width to obtain the right amount of cells
 * @param rows is height
 * @param cols is width
 */
void IntersectionWindow::drawCells(const unsigned int& rows, const unsigned int& cols) {
    QPen blackPen(Qt::black);
    blackPen.setWidth(1);
    blackPen.setStyle(Qt::DashLine);
    blackPen.setColor(Qt::black);

    QList<QGraphicsItem*> items = scene->items();
    for (const QGraphicsItem* item : items) {
        //qDebug() << item->type();
    }

    scene->clear();
    // Add the vertical lines
    m_divCol = (double)h_max / (double)cols;
    for (int x=0; x<=h_max; x+=m_divCol){
        if (x == 0 || x == h_max)
            scene->addLine(x, -100, x, v_max+100, blackPen);
        else
            scene->addLine(x,0,x,v_max, blackPen);
    }
    // Add the horizontal lines
    m_divRow = (double)v_max / (double)rows;
    for (int y=0; y<=v_max; y+=m_divRow){
        if (y == 0 || y == v_max)
            scene->addLine(-100, y, h_max+100, y, blackPen);
        else
            scene->addLine(0,y,h_max,y, blackPen);
    }

    //add rows to cellgrid hash table
    for (unsigned int i = 0; i < cols; i++)
    {
        cellGrid.insert(i, QHash<unsigned int, std::shared_ptr<CellGui> >());
    }


}

/** @brief setup and start database thread
 *  @param output
 */
void IntersectionWindow::getDBThread(QMap<QString,QVariantList> output) {

    qDebug() << "Start database thread ...";
    m_dbThread->StartDatabase(output);

}

/**
 * @brief IntersectionWindow::getNextValidColor returns the index for a color != white
 * @param index
 * @return
 */
unsigned int IntersectionWindow::getNextValidColor(const unsigned int& index)  {
    if (index >= m_colors.size()) {
        m_colorIndex = 0;
    }
    else {
        m_colorIndex = index;
    }
    return m_colorIndex;
}
