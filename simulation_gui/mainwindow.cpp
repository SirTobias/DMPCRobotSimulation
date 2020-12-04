#include "mainwindow.h"
#include <QGridLayout>
#include <QDir>
#include <QTextStream>
#include <QStringBuilder>
<<<<<<< HEAD
#include "../simulation-core/simdef.h"
#include "../simulation-core/simulationobject.h"
#include "../simulation-core/simulationactor.h"
#include "../simulation-core/simulationactorlist.h"
#include "../simulation-core/simulationresource.h"
#include "../simulation-core/simulationresourceconfig.h"
#include "../simulation-core/linearcongruentialgenerator.h"
=======
#include "../Simulation_Core/databasecore.h"
#include "../simulation_core/simdef.h"
#include "../simulation_core/simulationobject.h"
#include "../simulation_core/simulationactor.h"
#include "../simulation_core/simulationactorlist.h"
#include "../simulation_core/simulationresource.h"
#include "../simulation_core/simulationresourceconfig.h"
#include "../simulation_core/linearcongruentialgenerator.h"
>>>>>>> 730d80d44de4c0bcbdc8654d0001e8047331102f
#include "plot2d.h"
#include "productionobject.h"
#include <ctime>
#include <random>



enum class ProductionProblemType {
    CENTRALIZED = 0,
    DECENTRALIZED = 1
};

/**
 * @brief creates the main widget for the simulation
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    std::cout << "start" << std::endl;
    resize(400,400);
    //CentralWidget for MainWindow - every MainWindow has to contain at least one MainWidget
    m_centralWidget = new QWidget(this);
    setCentralWidget(m_centralWidget);
    m_descrLabel = new QLabel(this);
    m_plotLabel = new QLabel(this);
    m_plotLabel->resize(300,300);
    //simulation-activation-button
    m_descrLabel->setText("Start Example Simulation");

    m_pushActivateSimulation = new QPushButton("&Start Simulation", this);
    connect(m_pushActivateSimulation, SIGNAL(clicked()), this, SLOT(activateSimulation()));
    m_pushDBTest = new QPushButton("&Start DB Test");
    //linear-congruence-generator-button
    m_pushActivateLinearCongruenceGenerator = new QPushButton("Initialize linear congruence generator", this);
    connect(m_pushActivateLinearCongruenceGenerator, SIGNAL(clicked()), this, SLOT(initializeLinearCongruenceGenerator()));
    QGridLayout* gLayout = new QGridLayout;
    gLayout->addWidget(m_plotLabel,2,0);
    gLayout->addWidget(m_descrLabel,0,0 );
    gLayout->addWidget(m_pushActivateSimulation,1,0);
    gLayout->addWidget(m_pushActivateLinearCongruenceGenerator,1,1);
    gLayout->addWidget(m_pushDBTest);
    connect(m_pushDBTest, SIGNAL(clicked()), this, SLOT(testDBMethods()));
    m_centralWidget->setLayout(gLayout);
}

MainWindow::~MainWindow()
{
}

/**
 * @brief creates the simulation by creating the digraph slot-function for Qt to connect the execution button with the simulation start
 */
void MainWindow::activateSimulation() {
    adevs::Digraph<SimulationObject*> simDiGraph;
    //Produktkonfiguration hinterlegen (Fertigungszeiten)
    SimulationResourceConfig prodLineConfig;
    //define product configurations
    QString configOne("SimObjectOne");
    QString configTwo("SimObjectTwo");
    prodLineConfig.insertProductConfig(configOne, 4.0); //at first constant times, later different time values
    prodLineConfig.insertProductConfig(configTwo, 6.0); //at first constant times, later different time values
    //Simulationsaktoren definieren
    SimulationActor* actorOne = new SimulationActor(SimulationActorCreation::FROMFILE, QString("actorOne"), QDir::homePath() % QString("/l01.txt"), configOne);
    SimulationActor* actorTwo = new SimulationActor(SimulationActorCreation::FROMFILE, QString("actorTwo"), QDir::homePath() % QString("/l02.txt"), configTwo);
    SimulationActorList::registerSimulationActor(actorOne);
    SimulationActorList::registerSimulationActor(actorTwo);
    //Simulationsresource (hier Produktion definieren)
    SimulationResource* prodLine = new SimulationResource("ProdLineOne", prodLineConfig);
    //Akteure und Resourcen zum Graphen hinzufügen und die entsprechenden Ports verbinden
    simDiGraph.add(actorOne);
    simDiGraph.add(actorTwo);
    //connect the two actors with the product line
    simDiGraph.couple(actorOne, actorOne->created, prodLine, prodLine->arrive);
    simDiGraph.couple(actorTwo, actorTwo->created, prodLine, prodLine->arrive);
    //simulation starting
    adevs::Simulator<IOLogistic> sim(&simDiGraph);
    std::cout << "added" << std::endl;
    while (sim.nextEventTime() < DBL_MAX) {
        std::cout << "evaluate sim: " << std::endl;
        sim.execNextEvent();
    }
    prodLine->printEvaluation();
    //Simulation elements are deleted by the digraph
    return;
}

/**
 * @brief initializes the linear congruence generator and generates 10000 numbers and plots them in a 2D-plot
 */
void MainWindow::initializeLinearCongruenceGenerator() {
    //@TODO: write Interface for different random generators
    LinearCongruentialGenerator generator(0, 0, 100);
    QMultiMap<double, double> randomNumberMap;
    //randomNumberList.reserve(1000);
    unsigned int countStates = 0;
    unsigned int countNumbers = 0;
    //std::linear_congruential_engine<std::uint64_t, 48271, 0,2147483647> eng;
    //eng.seed(time(0));
    std::srand(time(0));
    //std::uint64_t rand = std::rand();
    //double uniRandom = (double)rand / (double)RAND_MAX;
    for (unsigned int i = 0; i < 10000; i++) {
        if (countNumbers > 0 && countNumbers % generator.getState() == 0) {
            countStates++;
        }
        //randomNumberMap.insert((double)countStates, generator.getRandom(false));
        //std::uint64_t rand = eng();
        //double uniRandom = (double)rand / (double)eng.modulus;
        //randomNumberMap.insert((double)countStates, uniRandom);
        std::uint64_t rand = std::rand();
        double uniRandom = (double)rand / (double)RAND_MAX;
        randomNumberMap.insert((double)countStates, uniRandom);
        countNumbers++;

    }
    QFile testOutputRandomFile("randomNumberFile.txt");
    if (testOutputRandomFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream outputRandomStream(&testOutputRandomFile);
        QString endLine ("\n");
        for (QMultiMap<double, double>::const_iterator itValues = randomNumberMap.begin(); itValues != randomNumberMap.end(); itValues++) {
            outputRandomStream << itValues.key() << ", " << itValues.value() << endLine;
        }
        testOutputRandomFile.close();
    }
    Plot2d* plot2d = new Plot2d(m_plotLabel, "general congruence generator");
    plot2d->addCurve(randomNumberMap, "curve", QwtPlotCurve::CurveStyle::Dots);
}

void MainWindow::initProductionSimulation() {
    QVector<ProductionObject> prodObjects;
    for (int i = 0; i < ProductionObjectAmount; i++) {
        prodObjects.push_back(ProductionObject());
    }

}

void MainWindow::testDBMethods(){
    /*DataBaseCore* db = new DataBaseCore();
    db->connect("localhost","simulation_intersection","Homyum","123456");
    db->Query(QString("car"));
    db->disConnect();*/
}
