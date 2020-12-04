#include "evaluation.h"

#include "prioritysorter.h"

#include <qwt_plot.h>

#include <QtCore/QLocale>
#include <QtCore/QDebug>

#include <QtCore/QFile>

Evaluation::Evaluation() :
    m_pathSteps(0),
    m_meanPathLength(0.0),
    m_minPathLength(0.0),
    m_maxPathLength(0.0),
    m_colorIndex(0),
    m_cellSize(0),
    m_commEffortWholeSimulation(0),
    m_skipTitle(false)
{

    m_colors.append(QColor(174, 63, 63)); //red
    m_colors.append(QColor(226, 192, 149)); //khaki
    m_colors.append(QColor(136, 183, 184)); //blue
    m_colors.append(QColor(0,0,0)); //black
    m_colors.append(QColor(0, 204, 0)); //green
    /*int i = 0;
    while (i < m_colors.size()) {
        if (m_colors.at(i).contains("lemonchiffon") || m_colors.at(i).contains("honeydew") || m_colors.at(i).contains("gainsboro")
                ||m_colors.at(i).contains("cornsilk") ||m_colors.at(i).contains("azure") ||m_colors.at(i).contains("white")
                || m_colors.at(i).contains("light") || m_colors.at(i).contains("gray") || m_colors.at(i).contains("beige")
                || m_colors.at(i).contains("snow") || m_colors.at(i).contains("linen") || m_colors.at(i).contains("mintcream")
                || m_colors.at(i).contains("transparent") || m_colors.at(i).contains("yellow") || m_colors.at(i).contains("bisque")
                || m_colors.at(i).contains("blanchedalmond") || m_colors.at(i).contains("silver") || m_colors.at(i).contains("wheat")
                || m_colors.at(i).contains("aliceblue") || m_colors.at(i).contains("grey") || m_colors.at(i).contains("ivory")
                || m_colors.at(i).contains("oldlace") || m_colors.at(i).contains("papayawhip")
                || m_colors.at(i).contains("seashell")) {
            m_colors.removeAt(i);
        }
        else {
            i++;
        }
    }*/
    qDebug() << m_colors.size();
    for (auto it = m_colors.cbegin(); it != m_colors.cend(); it++) {
        qDebug() << *it;
    }
    QLocale::setDefault(QLocale(QLocale::Language::English, QLocale::UnitedStates));
}

/**
 * @brief Evaluation::calculateMeanPathLength calculates the min/mean/max path length of the cars
 * @param amountCars amount of cars in the simulation
 */
void Evaluation::calculatePathValues(const unsigned int& amountCars) {
    unsigned int pathSteps = 0;
    std::map<QString, int>::const_iterator it = m_pathStepPerCar.begin();
    if (m_pathStepPerCar.size() > 0) {
        int minimalPathLength = m_pathStepPerCar[(*it).first];
        int maximalPathLength = minimalPathLength;
        for (std::map<QString, int>::const_iterator it = m_pathStepPerCar.begin(); it != m_pathStepPerCar.end(); it++) {
            pathSteps += (*it).second;
            //min. Pfadlänge
            if (minimalPathLength > (*it).second) {
                minimalPathLength = (*it).second;
            }
            //max. Pfadlänge
            if (maximalPathLength < (*it).second) {
                maximalPathLength = (*it).second;
            }

        }
        m_minPathLength = minimalPathLength;
        m_maxPathLength = maximalPathLength;
        m_meanPathLength = (double)pathSteps / (double)amountCars;
    }
}

/**
 * @brief Evaluation::savePathSizePerStepAndCar saves the path length for each car
 * @param car ID of the car
 * @param pathSteps amount of path steps taken by the car
 */
void Evaluation::setPathSizePerCar(const QString& car, const unsigned int& pathSteps) {
    m_pathStepPerCar[car] = pathSteps;
}

/**
 * @brief Evaluation::calculateCostsPerStep sums up the cumulated costs (distance without time) per step
 * @param cars
 */
void Evaluation::calculateDistanceCostsPerStep(const CarGroupQueue &cars, const unsigned int& step) {
    //sum up the costs for each car
    double costs = 0.0;
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        costs += car->getCurrentAbsDistance();
    }
    m_culmDistanceCostsPerStep[step] = costs;
}


/**
 * @brief Evaluation::getMeanPathLength
 * @return
 */
double Evaluation::getMeanPathLength() const {
    return m_meanPathLength;
}

/**
 * @brief Evaluation::getMinPathLength
 * @return
 */
unsigned int Evaluation::getMinPathLength() const {
    return m_minPathLength;
}

/**
 * @brief Evaluation::getMaxPathLength
 * @return
 */
unsigned int Evaluation::getMaxPathLength() const {
    return m_maxPathLength;
}

/**
 * @brief Evaluation::getCulmCostsPerStep
 * @return
 */
const std::map<int, double>& Evaluation::getCulmCostsPerStep() const {
    return m_culmDistanceCostsPerStep;
}


const std::vector<double>& Evaluation::getMeanFunctionValues() const {
    return m_meanAlphaValues;
}


/**
 * @brief Evaluation::setCostsForTimeStep stores for each car the costs for each time stamp
 * @param cars vector of cars
 * @param costs
 */
/*void Evaluation::setTimeDepCostsForTimeStep(const std::vector<std::shared_ptr<Car> >& cars) {
    for (const std::shared_ptr<Car>& car : cars) {
        //timeindependent
        //m_costsPerStep[car->getName()].push_back(car->getCurrentAbsDistance());
        //time dependent costs
        m_openLoopCosts[car->getName()].push_back(car->getMpcController()->getOpenLoopCosts());
        m_closedLoopCosts[car->getName()].push_back(car->getMpcController()->getClosedLoopCosts());
    }

}*/

void Evaluation::saveContAppliedControl(const CarGroupQueue &cars, const std::map<QString, std::vector<std::vector<double> > > &control) {
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        //m_controlContinuous.insert(std::make_pair(car->getName(), ))
        m_controlContinuous[car->getName()].push_back(control.at(car->getName()).at(0));
    }
}

/**
 * @brief Evaluation::saveContCosts save the open loop costs of the current timestep of each car
 * @param cars the continuous costs are obtained
 */
void Evaluation::saveContCostsPerCar(const CarGroupQueue &cars) {
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        m_closedLoopCosts[car->getName()].push_back(car->getClosedLoopCosts());
        m_openLoopCosts[car->getName()].push_back(car->getOpenLoopCosts());
    }
}

void Evaluation::saveCurrentDeltaForCar(CarGroupQueue& cars, const unsigned int& step) {
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        m_deltaOverTime[car->getName()].push_back(car->getDelta());
    }
}

/**
 * @brief Evaluation::plotAppliedContControl
 * @param accumulated
 */
void Evaluation::plotAppliedContControl(const size_t &N, const bool& accumulated) {
    //one colour for each curve
    Plot2d* p2d = nullptr;
    //over each car
    QString title = QString("Control ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N);
    for (auto it = m_controlContinuous.cbegin(); it != m_controlContinuous.cend(); it++) {
        title = QString("Control ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N);
        if (!accumulated) {
            title += ", " + it->first;
        }
        if (InterSectionParameters::varyCellSize == 1) {
            title += QString(", Cellsize ") + QString::number(m_cellSize);
        }
        if (!accumulated || p2d == nullptr) {
            if (m_skipTitle) {
                p2d = new Plot2d(nullptr, "");
            }
            else {
                p2d = new Plot2d(nullptr, title);
            }
        }
        QMultiMap<double, double> vals;
        //first component u(0)_0
        for (unsigned int i = 0; i < it->second.size(); i++) {
            vals.insert(i, it->second.at(i).at(0));
        }
        if (accumulated) {
            p2d->addCurve(vals, QString(it->first) + QString(": u(0)_0"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol, QColor(m_colors.at(getNextValidColor(m_colorIndex))));
        }
        else {
            p2d->addCurve(vals, QString(it->first) + QString(": u(0)_0"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol, QColor(m_colors.at(getNextValidColor(++m_colorIndex))));
        }
        vals.clear();
        //second component
        for (unsigned int i = 0; i < it->second.size(); i++) {
            vals.insert(i, it->second.at(i).at(1));
        }
        if (accumulated) {
            p2d->addCurve(vals, "u(0)_1", QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol, m_colors.at(getNextValidColor(m_colorIndex)));
            ++m_colorIndex;
        }
        else {
            p2d->addCurve(vals, "u(0)_1", QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol, m_colors.at(getNextValidColor(++m_colorIndex)));
        }
        if (!accumulated) {
            m_plots[title] = p2d;
        }
    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "u");
    p2d->enableGrid(true);
    if (accumulated) {
        m_plots[title] = p2d;
    }
}

/**
 * @brief Evaluation::plotContCosts
 * @param accumulated plots all costs in one graph if true, otherwise in seperated graphs
 */
void Evaluation::plotCosts(const size_t& N, const CostType& costType, const bool& accumulated, const bool& withCulmCosts) {
    Plot2d* p2d = nullptr;
    double sumCosts = 0.0;
    QMap<double, double> culmCostsOverTimeStep;
    auto costMap = std::map<QString, std::vector<double> >();
    QString preTitle;
    if (costType == CostType::CLOSEDLOOP) {
        costMap = m_closedLoopCosts;
        preTitle = QString("closedLoop ");
    }
    else if (costType == CostType::OPENLOOP) {
        costMap = m_openLoopCosts;
        preTitle = QString("openLoop ");
    }
    m_colorIndex = 4;

    QString title = preTitle + QString("Costs ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N);
    for (auto it = costMap.cbegin(); it != costMap.cend(); it++) {
        title = preTitle + QString("Costs ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N);
        if (!accumulated) {
            title += + ", " + it->first;
        }
        if (InterSectionParameters::varyCellSize == 1) {
            title += QString(", Cellsize ") + QString::number(m_cellSize);
        }
        if (!accumulated || p2d == nullptr) {
            if (m_skipTitle) {
                p2d = new Plot2d(nullptr, "");
            }
            else {
                p2d = new Plot2d(nullptr, title);
            }
        }
        QMultiMap<double, double> vals;
        for (unsigned int i = 0; i < it->second.size(); i++) {
            vals.insert(i, it->second.at(i));
            sumCosts += it->second.at(i);
            if (costType == CostType::CLOSEDLOOP) {
                m_culmClosedLoopCostsOverTimestepsOverCellsize[m_cellSize][i] += it->second.at(i);
                //DEBUG
                //qDebug() << "cellsize: " << m_cellSize << ", i: " << i << "val.: " << it->second.at(i) << endl;
                //--DEBUG
            }
            else if (costType == CostType::OPENLOOP) {
                m_culmOpenLoopCostsOverTimestepsOverCellsize[m_cellSize][i] += it->second.at(i);
            }
            culmCostsOverTimeStep[i] += it->second.at(i);
            std::cout << "SumCosts (Closed-Loop): " << sumCosts << "\n";
        }
        //cari to cari+1
        QString legendItem = it->first;
        legendItem.replace("car", "robot");
        int indexNumber = legendItem.indexOf(QRegExp("\\d?$"));
        int carNumber = QString(legendItem).remove(0, indexNumber).toInt();
        legendItem = legendItem.remove(indexNumber, legendItem.length() - indexNumber);
        legendItem += QChar::Space;
        carNumber++;
        p2d->addCurve(vals, legendItem + QString::number(carNumber), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol, m_colors.at(getNextValidColor(++m_colorIndex)));
        if (!accumulated) {
            p2d->enableGrid(true);
            m_plots[title] = p2d;
        }
    }
    //culm costs over one time step of all cars
    if (p2d && withCulmCosts && !culmCostsOverTimeStep.empty()) {
        if (costType == CostType::OPENLOOP) {
            p2d->addCurve(culmCostsOverTimeStep, QString(QChar(0x03A3)) + QString("V"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                          m_colors.at(getNextValidColor(++m_colorIndex)), true);
        }
        else if (costType == CostType::CLOSEDLOOP) {
            p2d->addCurve(culmCostsOverTimeStep, QString(QChar(0x03A3)) + QString("V"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                          m_colors.at(getNextValidColor(++m_colorIndex)), true);
        }
    }
    if (m_skipTitle) {
        p2d->setTitle("");
    }
    else {
        p2d->setTitle(preTitle + QString("Costs ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N));
    }
    if (costType == CostType::CLOSEDLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "l<sub>p</sub>");
    }
    else if (costType == CostType::OPENLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, getInlineSuperSubscriptStyle()
                          + QString("<span class='supsub'>V<sub class='subscript'>P</sub><sup class='superscript'>N</sup></span>(" + QString(QChar(0x00B7)) + QString(")")));
    }
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    p2d->setAxisFormat(QwtPlot::Axis::yLeft, QChar('f'));
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    if (accumulated) {
        m_plots[title] = p2d;
    }
}

void Evaluation::plotCulmCostsOverAllCellSizes(const size_t& N, const CostType &costType) {
    Plot2d* p2d = nullptr;
    QString title;
    auto culmCostsMapOverCellSize = std::map<double, std::map<double, double> >();
    if (costType == CostType::CLOSEDLOOP) {
        culmCostsMapOverCellSize = m_culmClosedLoopCostsOverTimestepsOverCellsize;
        title = QString("closedLoop ");
    }
    else if (costType == CostType::OPENLOOP) {
        culmCostsMapOverCellSize = m_culmOpenLoopCostsOverTimestepsOverCellsize;
        title = QString("openLoop ");
    }
    title += QString("AccumCosts over ") + QString::number(m_controlContinuous.size()) + QString(" cars, N eq ") + QString::number(N);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    std::pair<double, double> maxMinVal;
    m_colorIndex = 4;
    for (auto it = culmCostsMapOverCellSize.cbegin(); it != culmCostsMapOverCellSize.cend(); it++) {
        QMap<double, double> currentCulmCosts(it->second);
        //DEBUG
        /*if (costType == CostType::CLOSEDLOOP) {
            for (auto itDebug = currentCulmCosts.begin(); itDebug != currentCulmCosts.end(); itDebug++) {
                qDebug() << "extracted cellsize: " << itDebug.key() << "value: " << itDebug.value();
            }
        }*/
        //--DEBUG
        QMultiMap<double, double> multiCurrentCulmCosts(currentCulmCosts);
        maxMinVal = getMinAndMax(multiCurrentCulmCosts);
        p2d->addCurve(multiCurrentCulmCosts, QString("c=") + QString::number(it->first), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    if (costType == CostType::CLOSEDLOOP) {
        //p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "<msub><mi>M</mi><mn>P</mn></msub>", QwtText::MathMLText);
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "M<sub>P</sub>", QwtText::RichText);
    }
    else if (costType == CostType::OPENLOOP) {
        //p2d->setAxisTitle(QwtPlot::Axis::xBottom, "<mi>n</mi>", QwtPlot::Axis::yLeft, "<msub><mi>V</mi><mn>P</mn></msub>", QwtText::MathMLText);
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, getInlineSuperSubscriptStyle() + QString(QChar(0x03A3))
                          + QString("<span class='supsub'><sub class='subscript'>p=1</sub><sup class='superscript'>4</sup></span>")
                          + QString("<span class='supsub'>V<sub class='subscript'>P</sub><sup class='superscript'>N</sup></span>(" + QString(QChar(0x00B7)) + QString(",") + QString(QChar(0x00B7) + QString(")"))),
                          QwtText::RichText);
    }
    p2d->setScaleEngine(QwtPlotScaleType::LogScale);
    p2d->setLegendAlignment(Qt::AlignBottom|Qt::AlignLeft);
    p2d->setAxisFormat(QwtPlot::Axis::yLeft, QChar('e'));
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    //p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->setAutoScale(QwtPlot::Axis::yLeft, 10, yInterval.minValue(), yInterval.maxValue(), 100);
    p2d->getPlot()->setAxisMaxMinor(QwtPlot::Axis::yLeft, 0);
    //double stepSize = 0.0;
    //p2d->setAutoScale(QwtPlot::Axis::yLeft, std::floor(maxMinVal.second - maxMinVal.first), maxMinVal.first, maxMinVal.second, stepSize);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotCommEffortCostsInfinity plots the costs cumulated over all cars for each time step
 * against the communication effort, which was necessary
 */
void Evaluation::plotCommEffortCostsInfinity(const CostType& costType) {
    m_colorIndex = 3;
    //one colour for each curve
    Plot2d* p2d = nullptr;
    QString title;
    auto commEffortVsCosts = std::multimap<unsigned int, double>();
    if (costType == CostType::CLOSEDLOOP) {
        title = QString("closedLoop ");
        commEffortVsCosts = m_commEffortClosedLoopPerformance;
    }
    else if (costType == CostType::OPENLOOP) {
        title = QString("openLoop");
        commEffortVsCosts = m_commEffortOpenLoopPerformance;
    }
    title += QString("vs Communication,") + QString::number(m_controlContinuous.size()) + QString(" cars");
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    QMultiMap<double, double> vals;
    for (auto it = commEffortVsCosts.cbegin(); it != commEffortVsCosts.cend(); it++) {
        vals.insert(it->first, it->second);
    }
    p2d->addCurve(vals, QString(), QwtPlotCurve::CurveStyle::Dots, QwtSymbol::Style::NoSymbol, m_colors.at(getNextValidColor(++m_colorIndex)), false);
    if (costType == CostType::CLOSEDLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "K", QwtPlot::Axis::yLeft, "l");
    }
    else if (costType == CostType::OPENLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "K", QwtPlot::Axis::yLeft, "V");
    }
    p2d->setTitle("");
    p2d->setAxisFormat(QwtPlot::Axis::yLeft, QChar('e'));
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

void Evaluation::plotCommEffortToCellSize() {
    //one colour for each curve
    Plot2d* p2d = nullptr;
    QString title = QString("Full Communication vs cell size,") + QString::number(m_controlContinuous.size()) + QString(" cars");
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    m_colorIndex = 2;
    QMultiMap<double, double> vals;
    std::pair<double, double> firstValues;
    for (auto it = m_gridSizeCommEffort.begin(); it != m_gridSizeCommEffort.end(); it++) {
        vals.insert(it->first, it->second);
        if (it == m_gridSizeCommEffort.begin()) {
            firstValues.first = it->first;
        }
        else if (it != m_gridSizeCommEffort.begin()) {
            auto itback = it;
            --itback;
            if (itback == m_gridSizeCommEffort.begin()) {
                firstValues.second = it->first;
            }
        }
    }
    p2d->addCurve(vals, "", QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::Rect, m_colors.at(getNextValidColor(++m_colorIndex)), false);
    if (m_gridSizeCommEffort.size() > 1) {
        auto itLast = m_gridSizeCommEffort.end();
        itLast--;
        p2d->setAxisScale(QwtPlot::Axis::xBottom, m_gridSizeCommEffort.begin()->first, itLast->first, firstValues.second - firstValues.first);
        p2d->setAxisTicks(QwtPlot::Axis::xBottom, m_gridSizeCommEffort.begin()->first, itLast->first, m_gridSizeCommEffort.size() - 1, 0, 0, 0, true);
        std::pair<double, double> yMinMax = getMinAndMax(vals);
        p2d->setAxisTicks(QwtPlot::Axis::yLeft, yMinMax.first, yMinMax.second, 5.0, 0, 0, 0);

    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "c", QwtPlot::Axis::yLeft, "K");
    p2d->setAxisFormat(QwtPlot::Axis::yLeft, QChar('e'));
    p2d->setTitle("");
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

void Evaluation::plotCostToCellSize(const CostType& costType) {
    //one colour for each curve
    Plot2d* p2d = nullptr;
    QString title;
    m_colorIndex = 2;
    auto gridSizeCostsMap = std::map<double, double>();
    if (costType == CostType::CLOSEDLOOP) {
        title = QString("closedLoop ");
        gridSizeCostsMap = m_gridSizeClosedLoopCostsMap;
    }
    else if (costType == CostType::OPENLOOP) {
        title = QString("openLoop ");
        gridSizeCostsMap = m_gridSizeOpenLoopCostsMap;
    }
    gridSizeCostsMap.erase(0);

    title += QString("Costs vs cell size");
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    QMultiMap<double, double> vals;
    std::pair<double, double> firstValues;
    for (auto it = gridSizeCostsMap.begin(); it != gridSizeCostsMap.end(); it++) {
        vals.insert(it->first, it->second);
        if (it == gridSizeCostsMap.begin()) {
            firstValues.first = it->first;
        }
        else if (it != gridSizeCostsMap.begin()) {
            auto itback = it;
            --itback;
            if (itback == gridSizeCostsMap.begin()) {
                firstValues.second = it->first;
            }
        }
    }
    p2d->addCurve(vals, QString(), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::Rect, m_colors.at(getNextValidColor(++m_colorIndex)), false);
    if (gridSizeCostsMap.size() > 1) {
        auto itLast = gridSizeCostsMap.end();
        itLast--;
        p2d->setAxisScale(QwtPlot::Axis::xBottom, gridSizeCostsMap.begin()->first, itLast->first, firstValues.second - firstValues.first);
        p2d->setAxisTicks(QwtPlot::Axis::xBottom, gridSizeCostsMap.begin()->first, itLast->first, gridSizeCostsMap.size() - 1, 0, 0, 0, true);
    }
    p2d->disableLegend();
    if (costType == CostType::CLOSEDLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "c", QwtPlot::Axis::yLeft, "M", QwtText::PlainText);
    }
    else if (costType == CostType::OPENLOOP) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "c", QwtPlot::Axis::yLeft, "V", QwtText::PlainText);
    }
    p2d->setAxisFormat(QwtPlot::Axis::yLeft, QChar('e'), 0);
    std::pair<double, double> yMinMax = getMinAndMax(vals);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yMinMax.first, yMinMax.second, 5.0, 0, 0, 0);
    p2d->disableLegend();
    p2d->setTitle("");
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

void Evaluation::plotDifferentialCommunicationAndNormalCommunicationToCellSize(const bool& full, const bool& diff) {
    m_colorIndex = 0;
    //one colour for each curve
    Plot2d* p2d = nullptr;
    QString title = QString("Normal and Diff CommEffort vs cell size, ") + QString::number(m_controlContinuous.size()) + QString(" cars");
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    QMultiMap<double, double> vals;
    std::pair<double, double> firstValues;
    m_colorIndex = 0;
    //differential communication
    if (diff) {
        double summedValue = 0.0;
        for (auto it = m_gridSizeDifferentialCommEffort.begin(); it != m_gridSizeDifferentialCommEffort.end(); it++) {
            //TODO: entweder gemittlelter Wert oder absolute Zahlen
            vals.insert(it->first, it->second / m_culmClosedLoopCostsOverTimestepsOverCellsize.at(it->first).size());
            qDebug() << "diffComm:" << it->first << ", " << it->second;
            if (it == m_gridSizeDifferentialCommEffort.begin()) {
                firstValues.first = it->first;
            }
            else if (it != m_gridSizeDifferentialCommEffort.begin()) {
                auto itback = it;
                --itback;
                if (itback == m_gridSizeDifferentialCommEffort.begin()) {
                    firstValues.second = it->first;
                }
            }
            summedValue += it->second;
        }
        qDebug() << "diff comm: first values: " << firstValues.first << "," << firstValues.second;
        p2d->addCurve(vals, "With Differential Communication", QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::Rect, m_colors.at(getNextValidColor(++m_colorIndex)), false);
        p2d->addExtData("sumDiffCommEffort: ", QVariant(summedValue));
    }
    vals.clear();
    //normal communication
    if (full) {
        double summedValue = 0.0;
        for (auto it = m_gridSizeCommEffort.begin(); it != m_gridSizeCommEffort.end(); it++) {
            vals.insert(it->first, it->second / m_culmClosedLoopCostsOverTimestepsOverCellsize.at(it->first).size());
            summedValue += it->second;
            qDebug() << "fullComm:" << it->first << ", " << it->second;
        }
        for (auto it = m_gridSizeCommEffort.begin(); it != m_gridSizeCommEffort.end(); it++) {
            vals.insert(it->first, it->second / m_culmClosedLoopCostsOverTimestepsOverCellsize.at(it->first).size());
            qDebug() << "diffComm:" << it->first << ", " << it->second;
            if (it == m_gridSizeCommEffort.begin()) {
                firstValues.first = it->first;
            }
            else if (it != m_gridSizeCommEffort.begin()) {
                auto itback = it;
                --itback;
                if (itback == m_gridSizeCommEffort.begin()) {
                    firstValues.second = it->first;
                }
            }
        }
        p2d->addExtData("sumFullCommEffort: ", QVariant(summedValue));
        p2d->addCurve(vals, "Without Differential Communication", QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::Diamond, m_colors.at(getNextValidColor(++m_colorIndex)), false);
    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "c", QwtPlot::Axis::yLeft, "K");
    if (m_gridSizeCommEffort.size() > 1) {
        auto itLast = m_gridSizeCommEffort.end();
        itLast--;
        p2d->setAxisScale(QwtPlot::Axis::xBottom, m_gridSizeCommEffort.begin()->first, itLast->first, firstValues.second - firstValues.first);
        p2d->setAxisTicks(QwtPlot::Axis::xBottom, m_gridSizeCommEffort.begin()->first, itLast->first, m_gridSizeCommEffort.size() - 1, 0, 0, 0, true);
        //std::pair<double, double> yMinMax = getMinAndMax(vals);
        //p2d->setAxisTicks(QwtPlot::Axis::yLeft, yMinMax.first, yMinMax.second, 5.0, 0, 0, 0);

    }
    else if (m_gridSizeDifferentialCommEffort.size() > 1) {
        auto itLast = m_gridSizeDifferentialCommEffort.end();
        itLast--;
        p2d->setAxisScale(QwtPlot::Axis::xBottom, m_gridSizeDifferentialCommEffort.begin()->first, itLast->first, firstValues.second - firstValues.first);
        p2d->setAxisTicks(QwtPlot::Axis::xBottom, m_gridSizeDifferentialCommEffort.begin()->first, itLast->first, m_gridSizeDifferentialCommEffort.size() - 1, 0, 0, 0, true);
    }
    /*if (p2d->curves().size() < 2) {
        p2d->disableLegend();
    }
    else {
        p2d->enableLegend();
    }*/
    p2d->setTitle("");
    p2d->getPlot()->setAxisMaxMinor(QwtPlot::Axis::yLeft, 0);
    p2d->setLegendAlignment(Qt::AlignBottom|Qt::AlignLeft);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotPredictionDifference plots \f$ \sum_{k=0}^{N-1} plots the difference of the prediction for each car and optionally
 * culmulative
 * @param N prediction horizon length
 * @param culmulative if true, the summed up curve will also be drawn
 */
void Evaluation::plotPredictionDifference(const size_t &N, const bool& culmulative) {
    Plot2d* p2d = nullptr;
    QString title = QString("Prediction Difference,") + QString::number(m_controlContinuous.size()) + QString("cars") + QString(",Neq") + QString::number(N) +
            QString("cellsize") + QString::number(m_cellSize);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    unsigned int maxValue = 0;
    //go over each car and add a curve for prediction difference in each time step
    for (auto itCarPrediction = m_diffPredictions.begin(); itCarPrediction != m_diffPredictions.end(); itCarPrediction++) {
        //add elements for each time step
        QMap<double, double> vals;
        unsigned int time = 0;
        for (auto vecElement : itCarPrediction->second) {
            vals.insert(time, vecElement);
            time++;
        }
        if (time > maxValue) {
            maxValue = time;
        }
        //cari to cari+1
        QString legendItem = itCarPrediction->first;
        legendItem.replace("car", "p=");
        int indexNumber = legendItem.indexOf(QRegExp("\\d?$"));
        int carNumber = QString(legendItem).remove(0, indexNumber).toInt();
        legendItem = legendItem.remove(indexNumber, legendItem.length() - indexNumber);
        //legendItem += QChar::Space;
        carNumber++;
        p2d->addCurve(vals, /*itCarPrediction->first*/ legendItem + QString::number(carNumber), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    if (culmulative) {
        QMap<double, double> culmVals;
        //iterate over time
        for (unsigned int i = 0; i < maxValue; i++) {
            //iterate over cards
            for (auto itCarPrediction = m_diffPredictions.begin(); itCarPrediction != m_diffPredictions.end(); itCarPrediction++) {
                if (itCarPrediction->second.size() > i) {
                    culmVals[i] += itCarPrediction->second.at(i);
                }
            }
        }
        p2d->addCurve(culmVals, QString("r<sub>c</sub>"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    if (culmulative) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString(QChar(0x03A3)) + QString("r<sub>c</sub>"), QwtText::RichText);
    }
    else {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "r<sub>c</sub>", QwtText::RichText);
    }
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotPredictionDifferenceOverCellSizes plots the prediction difference accumulated over all cars for each cell size
 * @param N
 */
void Evaluation::plotPredictionDifferenceOverCellSizes(const size_t &N) {
    Plot2d* p2d = nullptr;
    QString title = QString("Prediction Difference,") + QString::number(m_controlContinuous.size()) + QString("cars") + QString(",Neq") + QString::number(N);
    QStringList cols;
    cols << "time";
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    //iterate over cell sizes
    for (auto itSim = m_diffPredictionsOverCellSize.begin(); itSim != m_diffPredictionsOverCellSize.end(); itSim++) {
        auto carPredictions = itSim->second;
        size_t maxValue = 0;
        for (auto itMaxValue = carPredictions.begin(); itMaxValue !=  carPredictions.end(); itMaxValue++) {
            if (maxValue < itMaxValue->second.size()) {
                maxValue = itMaxValue->second.size();
            }
        }
        QMap<double, double> summedValues;
        //iterate over time
        for (size_t i = 0; i < maxValue; i++) {
            //iterate over car predictions
            for (auto itCarPrediction = carPredictions.begin(); itCarPrediction != carPredictions.end(); itCarPrediction++) {
                if (itCarPrediction->second.size() > i) {
                    summedValues[i] += itCarPrediction->second.at(i);
                }
            }
        }
        p2d->addCurve(summedValues, QString("c=") + QString::number(itSim->first, 'f', 1), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                                   m_colors.at(getNextValidColor(++m_colorIndex)), true);
        cols << QString("c=") + QString::number(itSim->first);
    }
    p2d->exportToTextFile(title + QString(".txt"));
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString(QChar(0x03A3)) + "r<sub>c</sub>", QwtText::RichText);
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotOccupancyGridDifferenceOverCellSizes plots the cumulated sum of differences of two successive occupancy grids over the whole prediction horizon
 * (open-loop)
 * @param N
 */
void Evaluation::plotOccupancyGridDifferenceOverCellSizes(const size_t &N) {
    Plot2d* p2d = nullptr;
    QString title = QString("Occupancy Grid Difference,") + QString::number(m_controlContinuous.size()) + QString("cars") + QString(",Neq") + QString::number(N);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    //iterate over cell sizes
    for (auto itSim = m_diffOccupancyGridsOverCellSize.begin(); itSim != m_diffOccupancyGridsOverCellSize.end(); itSim++) {
        auto carPredictions = itSim->second;
        size_t maxValue = 0;
        for (auto itMaxValue = carPredictions.begin(); itMaxValue !=  carPredictions.end(); itMaxValue++) {
            if (maxValue < itMaxValue->second.size()) {
                maxValue = itMaxValue->second.size();
            }
        }
        QMap<double, double> summedValues;
        //iterate over time
        for (unsigned int i = 0; i < maxValue; i++) {
            //iterate over car predictions
            for (auto itCarPrediction = carPredictions.begin(); itCarPrediction != carPredictions.end(); itCarPrediction++) {
                if (itCarPrediction->second.size() > i) {
                    summedValues[i] += itCarPrediction->second.at(i);
                }
            }
        }
        p2d->addCurve(summedValues, QString("c=") + QString::number(itSim->first, 'f', 1), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                                   m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "r<sub>q</sub>", QwtText::RichText);
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotGridPredictionDifference plots the differences of occupancy grid predictions
 * @param N prediction horizon length
 * @param culmulative
 */
void Evaluation::plotGridPredictionDifference(const size_t &N, const bool &culmulative) {
    Plot2d* p2d = nullptr;
    QString title = QString("Occupancy Grid Difference, ") + QString::number(m_controlContinuous.size()) + QString("cars") + QString(",Neq") + QString::number(N)
            + QString("cellsize") + QString::number(m_cellSize);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    unsigned int maxValue = 0;
    for (auto itDiffOccupancyGrid = m_diffOccupancyGrid.begin(); itDiffOccupancyGrid != m_diffOccupancyGrid.end(); itDiffOccupancyGrid++) {
        QMap<double, double> vals;
        unsigned int time = 0;
        for (auto vecElem : itDiffOccupancyGrid->second) {
            vals.insert(time, vecElem);
            time++;
        }
        if (time > maxValue) {
            maxValue = time;
        }
        //cari to cari+1
        QString legendItem = itDiffOccupancyGrid->first;
        legendItem.replace("car", "p=");
        int indexNumber = legendItem.indexOf(QRegExp("\\d?$"));
        int carNumber = QString(legendItem).remove(0, indexNumber).toInt();
        legendItem = legendItem.remove(indexNumber, legendItem.length() - indexNumber);
        //legendItem += QChar::Space;
        carNumber++;
        p2d->addCurve(vals, /*itDiffOccupancyGrid->first*/legendItem + QString::number(carNumber), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    if (culmulative) {
        QMap<double, double> culmVals;
        //iterate over time
        for (unsigned int i = 0; i < maxValue; i++) {
            //iterate over cards
            for (auto itCarPrediction = m_diffOccupancyGrid.begin(); itCarPrediction != m_diffOccupancyGrid.end(); itCarPrediction++) {
                if (itCarPrediction->second.size() > i) {
                    culmVals[i] += itCarPrediction->second.at(i);
                }
            }
        }
        p2d->addCurve(culmVals, QString("r<sub>c</sub>(n)"), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, "r<sub>q</sub>", QwtText::RichText);
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 10.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotOccupancyCellsHeatMap
 * @param time
 * @param numberVehicles
 */
void Evaluation::plotOccupancyCellsHeatMap(const unsigned int& time, const unsigned int &numberVehiclesArrived,
                                           const unsigned int &numberVehiclesWaiting, const double& cellSize, const std::vector<DistParam>& distParams) {
    Plot2d* p2d = nullptr;
    QString title = QString("Occupancy Grid Heat Map,time,") + QString::number(time)
            + QString("numberVehiclesArr,") + QString::number(numberVehiclesArrived)
            + QString("numberVehiclesWaiting,") + QString::number(numberVehiclesWaiting)
            + QString("cellSize,") + QString::number(cellSize);
    QString subTitle = title;
    for (size_t i = 0; i < distParams.size(); i++) {
        subTitle += "en" + QString::number(i) + QString(",");
        if (distParams.at(i).distribFunc == DistributionFunction::POISSON) {
            subTitle += "poisson,";
        }
        else if (distParams.at(i).distribFunc == DistributionFunction::EXP) {
            subTitle += "exponential,";
        }
        subTitle += "ma," + QString::number(distParams.at(i).meanArrivalTime) + QString(",");

    }
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    //evaluate maximum x and maximum y
    //unsigned int x = 0, y = 0;
    /*for (auto it = m_occupiedCells.begin(); it != m_occupiedCells.end(); it++) {
        if (it->first->getX() > (double)x) {
            x = it->first->getX();
        }
        if (it->first->getY() > (double)y) {
            y = it->first->getY();
        }
    }*/
    //raster matrix to depict the whole intersection
    QVector<QVector<unsigned int> > valueMatrix;
    valueMatrix.reserve(m_occupiedCells.size());
    for (QVector<unsigned int> valsY : valueMatrix) {
        valsY.reserve(m_occupiedCells.at(0).size());
    }
    for (auto row = 0; valueMatrix.size() < m_occupiedCells.size(); row++) {
        valueMatrix.push_back(QVector<unsigned int>());
        for (auto col = 0; valueMatrix.at(row).size() < m_occupiedCells.at(row).size(); col++) {
            valueMatrix[row].push_back(0);
        }
    }
    //fill the raster now with the intersection values
    for (unsigned int row = 0; row < m_occupiedCells.size(); row++) {
        for (unsigned int col = 0; col < m_occupiedCells.at(row).size(); col++) {
            valueMatrix[row][col] = m_occupiedCells.at(row).at(col);
        }
    }
    p2d->addSpectogram(valueMatrix, title);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotMaxPriorityQueueLength plots the maximum length of priority queues for each time instant
 * @param numberCars number of cars
 * @param N horizon length
 * @param prioritySort priority criteria
 */
void Evaluation::plotMaxPriorityQueueLength(const size_t& numberCars, const size_t& N, const QString &prioritySort) {
    Plot2d* p2d = nullptr;
    QString title = QString("MaxPriorityQueues,") + prioritySort + QString(",") + QString::number(numberCars)
            + QString("cars,") + QString("Neq,") + QString::number(N);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    m_colorIndex = 4;
    QMap<double, double> vals;
    for (auto itCell = m_maxPriorityQueueLength.begin(); itCell != m_maxPriorityQueueLength.end(); itCell++) {
        vals.clear();
        for (auto itQueue = itCell->second.begin(); itQueue != itCell->second.end(); itQueue++) {
            vals[itQueue->first] = itQueue->second;
        }
        p2d->addCurve(vals, QString("c=") + QString::number(itCell->first, 'f', 1), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                                   m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    if (prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORY)
        || prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORY)
            ) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString("max<sub>|") + QString(QChar(0x03A0)) + QString("(n)|</sub>"), QwtText::RichText);
    }
    else if (prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYHIERARCHY)
             || prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYHIERARCHY)) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString("#") + QString("M(n)"), QwtText::RichText);
    }
    else if (prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINCLOSEDLOOPCOSTSWITHMEMORYTREE)
             || prioritySort == PrioritySorter::getTextForChosenCriteria(PriorityCriteria::MINOPENLOOPCOSTSWITHMEMORYTREE)) {
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString("max|") + QString("G(n)|"), QwtText::RichText);
    }

    std::pair<double, double> minMax = getMinAndMax(vals);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, minMax.first, minMax.second + 1.0, minMax.second - 1.0, 0.0, 0.0, 0.0, true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotNumberOfPriorityQueues plots the number of priority queues in each time instant
 * @param numberCars number of cars
 * @param N horizon length
 * @param prioritySort criteria for priority sort
 */
void Evaluation::plotNumberOfPriorityQueues(const size_t &numberCars, const size_t &N, const QString &prioritySort) {
    Plot2d* p2d = nullptr;
    QString title = QString("NumberOfPriorityQueues,") + prioritySort + QString(",") + QString::number(numberCars)
            + QString("cars,") + QString("Neq") + QString::number(N);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    m_colorIndex = 4;
    QMap<double, double> vals;
    for (auto itCell = m_numberPriorityQueues.begin(); itCell != m_numberPriorityQueues.end(); itCell++) {
        vals.clear();
        for (auto itQueue = itCell->second.begin(); itQueue != itCell->second.end(); itQueue++) {
            vals[itQueue->first] = itQueue->second;
        }
        p2d->addCurve(vals, QString("c=") + QString::number(itCell->first, 'f', 2), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                                   m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString(QChar(0x03A0) + QString("<sub>n#</sub>")), QwtText::RichText);
    std::pair<double, double> minMax = getMinAndMax(vals);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, minMax.first, minMax.second, minMax.second - 1.0, 0.0, 0.0, 0.0, true);
    m_plots[title] = p2d;
}


void Evaluation::showPlots() {
    for (auto it = m_plots.begin(); it != m_plots.end(); it++) {
        it->second->show();
    }
}

/**
 * @brief plotDeltaOfCellSize plots the delta time, how much time is left for one car to cross the interval
 * @param cellSize
 */
void Evaluation::plotDeltaOfCellSize(const size_t &numberCars, const size_t& N, const double& cellSize) {
    m_colorIndex = 4;
    Plot2d* p2d = nullptr;
    QString title = QString("Delta,") + QString::number(numberCars)
            + QString("cars,") + QString("Neq") + QString::number(N) + QString(",cellsize") + QString::number(cellSize);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    unsigned int maxValue = 0;
    for (auto itDeltaCarVec = m_deltaOverTime.begin(); itDeltaCarVec != m_deltaOverTime.end(); itDeltaCarVec++) {
        QMap<double, double> vals;
        unsigned int time = 0;
        for (auto vecElem : itDeltaCarVec->second) {
            vals.insert(time, vecElem);
            time++;
        }
        if (time > maxValue) {
            maxValue = time;
        }
        //cari to cari+1
        QString legendItem = itDeltaCarVec->first;
        legendItem.replace("car", "p=");
        int indexNumber = legendItem.indexOf(QRegExp("\\d?$"));
        int carNumber = QString(legendItem).remove(0, indexNumber).toInt();
        legendItem = legendItem.remove(indexNumber, legendItem.length() - indexNumber);
        //legendItem += QChar::Space;
        carNumber++;
        p2d->addCurve(vals, /*itDiffOccupancyGrid->first*/legendItem + QString::number(carNumber), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                      m_colors.at(getNextValidColor(++m_colorIndex)), true);
    }
    p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString("k") + QString("<sub>l</sub>"), QwtText::RichText);
    p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
    QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
    //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
    p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
    QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
    p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 4.0), 0.0, 0.0, 0.0, true);
    p2d->enableGrid(true);
    m_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotConsumedCellDifference
 * @param numberCars
 * @param N
 */
void Evaluation::plotConsumedCellDifference(const size_t &numberCars, const size_t &N) {
    m_colorIndex = 4;
    Plot2d* p2d = nullptr;
    QString title = QString("CellDifference,") + QString::number(numberCars)
            + QString("cars,") + QString("Neq") + QString::number(N);
    if (m_skipTitle) {
        p2d = new Plot2d(nullptr, "");
    }
    else {
        p2d = new Plot2d(nullptr, title);
    }
    //iterate over cell sizes
    if (m_intervalTypeConsumedCellSizes.find(CommunicationScheme::MINMAXINTERVAL) != m_intervalTypeConsumedCellSizes.end()) {
        for (auto itCell = m_intervalTypeConsumedCellSizes.at(CommunicationScheme::MINMAXINTERVAL).begin(); itCell != m_intervalTypeConsumedCellSizes.at(CommunicationScheme::MINMAXINTERVAL).end(); itCell++) {
            QMap<double, double> vals;
            //iterate over time instants
            for (size_t i = 0; i < itCell->second.size(); i++) {
                if (m_intervalTypeConsumedCellSizes.at(CommunicationScheme::MINMAXINTERVALMOVING).size() > 0) {
                    std::vector<int> movingVector = m_intervalTypeConsumedCellSizes.at(CommunicationScheme::MINMAXINTERVALMOVING).at(itCell->first);
                    if (i < movingVector.size()) {
                        vals.insert(i, itCell->second.at(i) - movingVector.at(i));
                    }
                }
            }
            p2d->addCurve(vals, QString("c=") + QString::number(itCell->first, 'f', 1), QwtPlotCurve::CurveStyle::Lines, QwtSymbol::Style::NoSymbol,
                          m_colors.at(getNextValidColor(++m_colorIndex)), true);
            vals.clear();
        }
        p2d->setAxisTitle(QwtPlot::Axis::xBottom, "n", QwtPlot::Axis::yLeft, QString("c") + QString("<sub>l</sub>"), QwtText::RichText);
        p2d->setLegendAlignment(Qt::AlignTop|Qt::AlignRight);
        QwtInterval xInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::xBottom);
        //QwtAxisScaleDraw = p2d->getPlot()->axisScaleDraw(QwtPlot::Axis::xBottom);
        p2d->setAxisTicks(QwtPlot::Axis::xBottom, xInterval.minValue(), xInterval.maxValue(),xInterval.maxValue() / 5.0, 0.0, 0.0, 0.0, true);
        QwtInterval yInterval = p2d->getPlot()->axisInterval(QwtPlot::Axis::yLeft);
        p2d->setAxisTicks(QwtPlot::Axis::yLeft, yInterval.minValue(), yInterval.maxValue(), yInterval.maxValue() / (yInterval.maxValue() / 4.0), 0.0, 0.0, 0.0, true);
        p2d->enableGrid(true);
        m_plots[title] = p2d;
    }
}

void Evaluation::exportPlots(const QString &type) {
    for (auto it = m_plots.begin(); it != m_plots.end(); it++) {
        it->second->exportToFile(it->first, type);
    }
}

/**
 * @brief Evaluation::saveCurrentCommunicatedConstraints save for each timestep n the effort of communication
 * @param cars vector of all cars
 * @param step current time step
 */
void Evaluation::saveCurrentCommunicatedConstraints(const CarGroupQueue &cars, unsigned int& step) {
    unsigned int commEffort = 0;
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        commEffort += car->getCountCommunicatedConstraints();
    }
    m_commConstraintsPerStep.insert(std::pair<unsigned int, unsigned int>(step, commEffort));
}

/**
 * @brief Evaluation::sumCostsOfCarToInfinity calculates the whole costs for each car over the full simulation time
 * @param costsContinuous
 * @return a map for each car with culmulated costs
 */
std::map<QString, double> Evaluation::sumCostsOfCarToInfinity(const std::map<QString, std::vector<double> >& costsContinuous) const {
    std::map<QString, double> carCumulatedCosts;
    for (auto it = costsContinuous.begin(); it != costsContinuous.end(); it++) {
        double culmCosts = 0.0;
        for (const double& val : it->second) {
            culmCosts += val;
        }
        carCumulatedCosts.insert(std::pair<QString, double>(it->first, culmCosts));
    }
    return carCumulatedCosts;
}

/**
 * @brief Evaluation::communicationEffortClosedLoopPerformance  calculates the whole costs over all cars over the whole simulation and sums up the communication effort
 * @param commEffort
 * @param culmCosts
 * @return
 */
std::pair<unsigned int, double> Evaluation::communicationEffortCostsPerformance(std::map<unsigned int, unsigned int>& commEffort, std::map<QString, double> culmCosts) {
    unsigned int commEffortWhole = 0;
    for (auto it = commEffort.cbegin(); it != commEffort.cend(); it++) {
        commEffortWhole += it->second;
    }
    double culmCostsWholeCars = 0.0;
    for (auto it = culmCosts.cbegin(); it != culmCosts.end(); it++) {
        culmCostsWholeCars += it->second;
    }
    return std::pair<unsigned int, double> (commEffortWhole, culmCostsWholeCars);
}

/**
 * @brief Evaluation::getCostsContinuous
 * @param costType Open-loop or closedLoop costs
 * @return
 */
std::map<QString, std::vector<double> > Evaluation::getCostsContinuous(const CostType& costType) const {
    if (costType == CostType::OPENLOOP) {
        return m_openLoopCosts;
    }
    else if (costType == CostType::CLOSEDLOOP) {
        return m_closedLoopCosts;
    }
}

/**
 * @brief Evaluation::setCostsContinuous sets the map with the cars with costs for each time step of the whole simulation
 * @param costsContinuous
 */
void Evaluation::setCostsContinuous(std::map<QString, std::vector<double> >& costsContinuous, const CostType& costType) {
    if (costType == CostType::OPENLOOP) {
        m_openLoopCosts = costsContinuous;
    }
    else if (costType == CostType::CLOSEDLOOP) {
        m_closedLoopCosts = costsContinuous;
    }
}

/**
 * @brief Evaluation::setCostsContinuousInfinity for each car over one simulation run
 * @param costsInf
 */
void Evaluation::setCostsContinuousInfinity(const std::map<QString, double>& costsInf, const CostType& costType) {
    if (costType == CostType::OPENLOOP) {
        m_openLoopCostsContinuousInfinity = costsInf;
    }
    else if (costType == CostType::CLOSEDLOOP) {
        m_closedLoopCostsContinuousInfinity = costsInf;
    }
}

/**
 * @brief Evaluation::addCommEffortClosedLoopPerformance inserts in the communication effort/closed loop performance map
 * the pair (comm effort / closed loop performance)
 * @param pair of comm effort / closed loop performance
 */
void Evaluation::addCommEffortClosedLoopPerformance(const std::pair<unsigned int, double>& pair) {
    m_commEffortClosedLoopPerformance.insert(pair);
}

/**
 * @brief Evaluation::addCommEffortOpenLoopPerformance inserts in the communication effort/open loop performance map
 * the pair (comm effort / open loop performance)
 * @param pair of comm effort / open loop performance
 */
void Evaluation::addCommEffortOpenLoopPerformance(const std::pair<unsigned int, double>& pair) {
    m_commEffortOpenLoopPerformance.insert(pair);
}

/**
* @brief Evaluation::getCommConstraintsPerStep returns the communication effort for current time step in the simulation
* @return
*
*/
std::map<unsigned int, unsigned int> Evaluation::getCommConstraintsPerStep() const {
    return m_commConstraintsPerStep;
}

/**
 * @brief Evaluation::getCostsContinuousInfinity returns a map containing the costs for each car over the whole simulation
 * @return
 */
std::map<QString, double> Evaluation::getCostsContinuousInfinity(const CostType& costType) const {
    if (costType == CostType::OPENLOOP) {
        return m_openLoopCostsContinuousInfinity;
    }
    else if (costType == CostType::CLOSEDLOOP) {
        return m_closedLoopCostsContinuousInfinity;
    }
}

/**
 * @brief Evaluation::setCellSize sets the cell size from the intersection
 * @param cellSize
 */
void Evaluation::setCellSize(const double& cellSize){
    m_cellSize = cellSize;
}

/**
 * @brief Evaluation::clearStatisticsAfterOneRun clears the statistics after one simulation run to have a clean setup
 * for the next run - only statistics over many runs should be preserved
 */
void Evaluation::clearStatisticsAfterOneRun() {
    //current continuous costs
    m_openLoopCosts.clear();
    m_closedLoopCosts.clear();
    m_controlContinuous.clear();
    m_openLoopCostsContinuousInfinity.clear();
    m_closedLoopCostsContinuousInfinity.clear();
    m_commConstraintsPerStep.clear();
    m_commEffortWholeSimulation = 0;
    m_predictions.clear();
    m_occupancyGrid.clear();
    m_diffOccupancyGrid.clear();
    m_diffPredictions.clear();
    m_countDiffOccupancyGrid.clear();
    m_countDiffPredictions.clear();
    m_deltaOverTime.clear();
}

void Evaluation::clearStatisticsAfterIteratedCellSizes() {
    m_closedLoopCostsContinuousInfinity.clear();
    m_openLoopCostsContinuousInfinity.clear();
    m_culmClosedLoopCostsOverTimestepsOverCellsize.clear();
    m_culmOpenLoopCostsOverTimestepsOverCellsize.clear();
    m_culmDistanceCostsPerStep.clear();
    m_gridSizeClosedLoopCostsMap.clear();
    m_gridSizeOpenLoopCostsMap.clear();
}

/**
 * @brief Evaluation::getNextValidColor returns the index for a color != white
 * @param index
 * @return
 */
unsigned int Evaluation::getNextValidColor(const unsigned int& index)  {
    if (index >= m_colors.size()) {
        m_colorIndex = 0;
    }
    else {
        m_colorIndex = index;
    }
    return m_colorIndex;
}

/**
 * @brief Evaluation::getCommConstraintsForWholeSim sums up the whole communication effort of one simulation run
 * important for evaluating, for which cell size which communication effort is necessary
 * @param commEffort
 * @return
 */
std::pair<double, unsigned int> Evaluation::getCommConstraintsForWholeSim(const std::map<unsigned int, unsigned int> &commEffort) const {
    unsigned int commEffortSum = 0;
    for (auto it = commEffort.begin(); it != commEffort.end(); it++) {
        commEffortSum += it->second;
    }
    return std::pair<double, unsigned int>(m_cellSize, commEffortSum);
}

/**
 * @brief Evaluation::setCommEfortWholeSimulationSum inserts a pair of current grid size related to the communication effort over the whole simulation in the map
 * @param commSum gridsize / communication effort
 */
void Evaluation::setCommEffortWholeSimulationSum(const std::pair<double, unsigned int>& pair) {
    m_gridSizeCommEffort.insert(pair);
    m_commEffortWholeSimulation += pair.second;
}

/**
 * @brief Evaluation::setDiffCommEffortWholeSimulationSum inserts a pair of current grid size related to the differential communication effort over the whole simulation in the map
 * @param pair
 */
void Evaluation::setDiffCommEffortWholeSimulationSum(const std::pair<double, unsigned int> &pair) {
    m_gridSizeDifferentialCommEffort.insert(pair);
    m_commEffortWholeSimulation += pair.second;
}

/**
 * @brief Evaluation::getCostsToCellSize evaluates the costs over the whole simulation to the given cell grid size $\f c\f$
 * @return
 */
std::pair<double, double> Evaluation::getCostsToCellSize(const CostType& costType) const {
    double costs = 0.0;
    for (auto it = m_closedLoopCostsContinuousInfinity.begin(); it != m_closedLoopCostsContinuousInfinity.end(); it++) {
         costs += it->second;
    }
    return std::pair<double, double>(m_cellSize, costs);
}

/**
 * @brief Evaluation::setGridSizeToCosts insert a new pair of grid size/costs to the map
 * @param pair of (grid size, costs)
 */
void Evaluation::setGridSizeToCosts(const std::pair<double, double>& pair, const CostType& costType) {
    if (costType == CostType::CLOSEDLOOP) {
        m_gridSizeClosedLoopCostsMap.insert(pair);
    }
    else if (costType == CostType::OPENLOOP) {
        m_gridSizeOpenLoopCostsMap.insert(pair);
    }
}

/**
 * @brief Evaluation::skipTitle skips of enable title option
 * @param skipTitle
 */
void Evaluation::disableTitle(const bool& skipTitle) {
    m_skipTitle = skipTitle;
}

/**
 * @brief Evaluation::getMinAndMax value of a MultiMap
 * @param vals
 * @return
 */
std::pair<double, double> Evaluation::getMinAndMax(const QMultiMap<double, double> &map) const {
    QList<double> values = map.values();
    qSort(values.begin(), values.end());
    return std::pair<double, double>(values.first(), values.last());
}

QString Evaluation::getInlineSuperSubscriptStyle() {
    QString css = "<style type=\"text/css\">";
    css += ".supsub {position: absolute}";
    css += ".subscript {color: black; display:block; position:relative; left:2px; top: -5px}";
    css += ".superscript {color: black; display:block; position:relative; left:2px; top: -5px}";
    css+="</style>";
    return css;
}

/**
 * @brief Evaluation::calculatePredictionDifference measures the difference of two predictions (metric: +1 for difference in one coordinate)
 * @param newPred
 * @param oldPred
 * @return
 */
unsigned int Evaluation::countPredictionDifference(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const {
    Q_ASSERT_X(newPred.size() == oldPred.size(), typeid(this).name(), "prediction size not equal");
    unsigned int difference = 0;
    for (unsigned int i = 1; i < oldPred.size(); i++) {
        //t1 for old prediction is t0 for the new one
        const std::vector<double>& pred1Row = newPred.at(i-1);
        for (unsigned int j = 0; j < pred1Row.size(); j++) {
            if (pred1Row.at(j) != oldPred.at(i).at(j)) {
                difference++;
                break;
            }
        }
    }
    return difference;
}

/**
 * @brief Evaluation::calculatePredictionDifference calculates the difference between two predictions component-wise
 * \f$\sum_{k=0}^{N-1} \|z_i(k) - z_j(k-1)\|_2\f$ excluding time
 * @param pred1
 * @param pred2
 * @return
 */
double Evaluation::calculatePredictionDifference(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const {
    Q_ASSERT_X(newPred.size() == oldPred.size(), typeid(this).name(), "prediction size not equal");
    double difference = 0.0;
    for (unsigned int i = 1; i < oldPred.size(); i++) {
        //t1 for old prediction is t0 for new one
        const std::vector<double>& pred1Row = newPred.at(i-1);
        difference += VectorHelper::norm2(VectorHelper::sub(pred1Row, oldPred.at(i)));
    }
    return difference;
}

/**
 * @brief Evaluation::calculatePredictionEqualityLength calculates the equality length as two predictions are equal over the finite time horizon
 * @param pred1
 * @param pred2
 * @return
 */
unsigned int Evaluation::calculatePredictionEqualityLength(const std::vector<std::vector<double> >& newPred, const std::vector<std::vector<double> >& oldPred) const {
    Q_ASSERT_X(newPred.size() == oldPred.size(), typeid(this).name(), "prediction size not equal");
    unsigned int equality = 0;
    bool stillEqual = true;
    for (unsigned int i = 1; i < oldPred.size(); i++) {
        if (!stillEqual) {
            break;
        }
        const std::vector<double>& pred1Row = newPred.at(i-1);
        for (unsigned int j = 0; j < pred1Row.size(); j++) {
            if (pred1Row.at(j) == oldPred.at(i).at(j)) {
                equality++;
            }
            else {
                stillEqual = false;
            }
            break;
        }
    }
    return equality;
}

/**
 * @brief Evaluation::getPrediction returns the curent predictions for each car to current time step
 * @return
 */
const std::map<QString, std::vector<std::vector<double> > >& Evaluation::getPrediction() const {
    return m_predictions;
}

/**
 * @brief Evaluation::countDiffInPredictions takes the maps of predictions and counts the difference
 * but only vector-wise (so if more than one vector element is invalid, count only once)
 * @param pred1
 * @param pred2
 * @return difference for each car
 */
std::map<QString, unsigned int> Evaluation::countDiffInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                  const std::map<QString, std::vector<std::vector<double> > >& pred2) const {
    std::map<QString, unsigned int> diffPred;
    if (!pred1.empty() && !pred2.empty()) {
        for (auto it = pred1.begin(); it != pred1.end(); it++) {
            auto pred1Vec = it->second;
            if (pred2.find(it->first) != pred2.end()) {
                auto pred2Vec = pred2.at(it->first); //get vector
                diffPred[it->first] = countPredictionDifference(pred1Vec, pred2Vec);
            }
        }
    }
    return diffPred;
}

/**
 * @brief Evaluation::calculateDiffInPredictions calculates the difference of two predictions \f$ \sum_{k=0}^{N-1} z_i(k) - z_j(k)\f$
 * @param pred1
 * @param pred2
 * @return
 */
std::map<QString, double> Evaluation::calculateDiffInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                  const std::map<QString, std::vector<std::vector<double> > >& pred2) const {
    std::map<QString, double> diffMap;
    if (!pred1.empty() && !pred2.empty()) {
        for (auto itPred1 = pred1.begin(); itPred1 != pred1.end(); itPred1++) {
            if (pred2.find(itPred1->first) != pred2.end()) {
                diffMap[itPred1->first] = calculatePredictionDifference(itPred1->second, pred2.at(itPred1->first));
            }
        }
    }
    return diffMap;
}

/**
 * @brief Evaluation::calculateEqualityInPredictions counts the equality of the predictions of the cars in comparision to last time step predictions
 * @param pred1
 * @param pred2
 * @return
 */
std::map<QString, unsigned int> Evaluation::calculateEqualityInPredictions(const std::map<QString, std::vector<std::vector<double> > >& pred1,
                                                  const std::map<QString, std::vector<std::vector<double> > >& pred2) const {
    std::map<QString, unsigned int> equalPredLengths;
    if (!pred1.empty() && !pred2.empty()) {
        for (auto it = pred1.begin(); it != pred1.end(); it++) {
            auto pred1Vec = it->second;
            auto pred2Vec = pred2.at(it->first); //get vector
            equalPredLengths[it->first] = calculatePredictionEqualityLength(pred1Vec, pred2Vec);
        }
    }
    return equalPredLengths;
}



/**
 * @brief Evaluation::saveCurrentPrediction
 * @param pred
 */
void Evaluation::saveCurrentPrediction(const std::map<QString, std::vector<std::vector<double> > >& pred) {
    m_predictions = pred;
}

/**
 * @brief Evaluation::addCountDiffPredictions add the current difference predictions for each car for current time step
 * @param pred
 */
void Evaluation::addCountDiffPredictions(const std::map<QString, unsigned int>& pred) {
    for (auto it = pred.begin(); it != pred.end(); it++) {
        m_countDiffPredictions[it->first].push_back(it->second);
    }
}

/**
 * @brief Evaluation::addDiffPredictions adds for the current time step the difference in predictions
 * @param pred
 */
void Evaluation::addDiffPredictions(const std::map<QString, double >& pred) {
    for (auto it = pred.begin(); it != pred.end(); it++) {
        m_diffPredictions[it->first].push_back(it->second);
    }
}


/**
 * @brief Evaluation::setCountDiffOccupancyGrid sets the count map of difference of the occupancy grid predictions
 * @param diffOccupancyGrid
 */
void Evaluation::addCountDiffOccupancyGrid(const std::map<QString, unsigned int>& diffOccupancyGrid) {
    for (auto it = diffOccupancyGrid.begin(); it != diffOccupancyGrid.end(); it++) {
        if (!diffOccupancyGrid.empty()) {
            m_countDiffOccupancyGrid[it->first].push_back(it->second);
        }
    }
}

/**
 * @brief Evaluation::addDiffOccupancyGrid adds for the current time step the difference in occupancy grid predictions
 * @param diffOccupancyGrid
 */
void Evaluation::addDiffOccupancyGrid(const std::map<QString, unsigned int>& diffOccupancyGrid) {
    for (auto it = diffOccupancyGrid.begin(); it != diffOccupancyGrid.end(); it++) {
        m_diffOccupancyGrid[it->first].push_back(it->second);
    }
}

/**
 * @brief Evaluation::calculateOccupancyEqual counts the equality of two occupancy grids (\f$I_p(n-1) == I_p(n)\f$
 * @param grid1
 * @param grid2
 * @return
 */
unsigned int Evaluation::calculateOccupancyEqual(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const {
    unsigned int equality = 0;
    bool stillEqual = true;
    auto itOldGrid = oldGrid.begin();
    //t1 from the old prediction is the t0 of new one
    itOldGrid++;
    auto itNewGrid = newGrid.begin();
    for (; itOldGrid != oldGrid.end(); itOldGrid++) {
        if (!stillEqual) {
            break;
        }
        if (itNewGrid.key() == itOldGrid.key() && itNewGrid.value() == itOldGrid.value()) {
            equality++;
        }
        else {
            stillEqual = false;
        }
        itNewGrid++;
    }
    return equality;
}

/**
 * @brief Evaluation::calculateEqualityInOccupancyGrid counts for each car the equality
 * @param cars
 * @param occupiedCells
 * @return
 */
std::map<QString, unsigned int> Evaluation::calculateEqualityInOccupancyGrid(const CarGroupQueue &cars,
                                                                 const std::map<QString, QMap<int, int> >& occupiedCells) const {
    std::map<QString, unsigned int> equalOccupancyGrid;
    if (occupiedCells.empty()) {
        for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
            equalOccupancyGrid[car->getName()] = calculateOccupancyEqual(car->getOccupiedCells(), occupiedCells.at(car->getName()));
        }
    }
    return equalOccupancyGrid;
}


/**
 * @brief Evaluation::calculateOccupancyDiff calculates the difference between old and new prediction grid
 * @param newGrid
 * @param oldGrid
 * @return
 */
unsigned int Evaluation::calculateOccupancyDiff(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const {
    unsigned int difference = 0;
    if (!newGrid.isEmpty() && !oldGrid.isEmpty()) {
        auto itOldGrid = oldGrid.begin();
        auto itNewGrid = newGrid.begin();
        itOldGrid++;
        for (; itOldGrid != oldGrid.end(); itOldGrid++) {
            std::vector<double> oldGridVec = {(double)itOldGrid.key(), (double)itOldGrid.value()};
            std::vector<double> newGridVec = {(double)itNewGrid.key(), (double)itNewGrid.value()};
            difference += VectorHelper::norm2(VectorHelper::sub(oldGridVec, newGridVec));
            itNewGrid++;
        }
    }
    return difference;
}

std::map<QString, unsigned int> Evaluation::calculateDiffInOccupancyGrids(const CarGroupQueue &cars,
                                                             const std::map<QString, QMap<int, int> >& occupiedCells) const {
    std::map<QString, unsigned int> diffOccupancyGrids;
    if (!occupiedCells.empty()) {
        for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
            if (occupiedCells.find(car->getName()) != occupiedCells.end()) {
                diffOccupancyGrids[car->getName()] = calculateOccupancyDiff(car->getOccupiedCells(), occupiedCells.at(car->getName()));
            }
        }
    }
    return diffOccupancyGrids;
}
/**
 * @brief Evaluation::countOccupancyDifference counts the difference between two occupancy grids
 * @param grid1
 * @param grid2
 * @return
 */
unsigned int Evaluation::countOccupancyDifference(const QMap<int, int>& newGrid, const QMap<int, int>& oldGrid) const {
    unsigned int difference = 0;
    auto itOldGrid = oldGrid.begin();
    //t1 from the old prediction is the t0 of new one
    itOldGrid++;
    auto itNewGrid = newGrid.begin();
    for (; itOldGrid != oldGrid.end(); itOldGrid++) {
        if (itNewGrid.key() != itOldGrid.key() || itNewGrid.value() != itOldGrid.value()) {
            difference++;
        }
        itNewGrid++;
    }
    return difference;
}

/**
 * @brief Evaluation::countDiffInOccupancyGrids counts the difference of all occupancy grids
 * @param cars
 * @param occupiedCells
 * @return
 */
std::map<QString, unsigned int> Evaluation::countDiffInOccupancyGrids(const CarGroupQueue &cars ,
                                                                const std::map<QString, QMap<int, int> >& occupiedCells) const {
    std::map<QString, unsigned int> diffOccupancyGrid;
    if (!occupiedCells.empty()) {
        for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
            QMap<int, int> currentOccupancy = car->getOccupiedCells();
            if (!currentOccupancy.isEmpty() && occupiedCells.find(car->getName()) != occupiedCells.end()) {
                diffOccupancyGrid[car->getName()] = countOccupancyDifference(car->getOccupiedCells(), occupiedCells.at(car->getName()));
            }
        }
    }
    return diffOccupancyGrid;
}

/**
 * @brief Evaluation::saveCurrentOccupiedCells for each car which is either based on current positions (closed-loop reservations)
 * or the reserved predictions (open-loop reservations)
 * @param cars
 */
void Evaluation::saveCurrentOccupiedCells(const CarGroupQueue &cars, const bool &openLoop) {
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        m_occupancyGrid[car->getName()] = car->getOccupiedCells();
        QMultiMap<int, int> currentCells;
        if (!openLoop) {
             currentCells = car->getOccupiedCells(false);
        }
        else {
            currentCells = car->getOccupiedCells();
        }
        int x = currentCells.size(), y = 1;
        for (auto row = currentCells.begin(); row != currentCells.end(); row++) {
            if (row.key() > x) {
                x = row.key();
            }
            if (row.value() > y) {
                y = row.value();
            }
        }
        while (m_occupiedCells.size() < x + 1) {
            m_occupiedCells.push_back(std::vector<int>());
        }
        for (auto row = 0; row < m_occupiedCells.size(); row++) {
            while (m_occupiedCells.at(row).size() < y + 1) {
                m_occupiedCells[row].push_back(0);
            }
        }
        //count the reservations of cells
        for (auto it = currentCells.begin(); it != currentCells.end(); it++) {
            m_occupiedCells[it.key()][it.value()] += 1;
        }
    }
}

/**
 * @brief Evaluation::getOccupancyGrid
 * @return
 */
const std::map<QString, QMap<int, int> >& Evaluation::getOccupancyGrid() const {
    return m_occupancyGrid;
}

std::map<QString, std::vector<std::vector<double> > > Evaluation::determineCurrentPredictionStates(
        const CarGroupQueue& cars, const double& t0, const double& T, const int& N) const {
    std::map<QString, std::vector<std::vector<double> > > predictionMap;
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        predictionMap[car->getName()] = car->getPredictedTrajectory(car->getCurrentStateContinuous(), car->getCurrentPrediction(), t0, T, N);
    }
    return predictionMap;
}

/**
 * @brief Evaluation::saveCurrentDiffPredictionsforCellSize
 * @param cellSize
 * @param diffPredictions
 */
void Evaluation::saveCurrentDiffPredictionsforCellSize(const double& cellSize, const std::map<QString, std::vector<double> >& diffPredictions) {
    m_diffPredictionsOverCellSize[cellSize] = diffPredictions;
}

/**
 * @brief Evaluation::getDiffPredictions
 * @return
 */
const std::map<QString, std::vector<double> >& Evaluation::getDiffPredictions() const {
    return m_diffPredictions;
}

/**
 * @brief Evaluation::getDiffOccupancyGrids
 * @return
 */
const std::map<QString, std::vector<unsigned int> >& Evaluation::getDiffOccupancyGrids() const {
    return m_diffOccupancyGrid;
}

void Evaluation::saveCurrentDiffOccupancyGridsForCellSize(const double& cellSize, const std::map<QString, std::vector<unsigned int> >& diffOccupancyGrids) {
    m_diffOccupancyGridsOverCellSize[cellSize] = diffOccupancyGrids;
}

/**
 * @brief Evaluation::saveOccupiedCells saves the number of occupancy for each intersection cell
 * @param intersect
 * @param cars
 */
/*void Evaluation::saveOccupiedCells(const std::shared_ptr<InterSection>& intersect, std::vector<std::shared_ptr<Car> >& cars) {
    for (const std::shared_ptr<Car>& car : cars) {
        std::shared_ptr<InterSectionCell> cell = intersect->getCellFromCoordinates(car->getCurrentStateContinuous());
        m_occupiedCellsPerStep[cell] += 1;
    }
}*/

/**
 * @brief Evaluation::saveMaxPriorityQueueLength save the current maximum length of the priority queues in each time instant
 * @param cars priority queue
 * @param step time instant
 */
void Evaluation::saveMaxPriorityQueueLength(CarGroupQueue& cars, const unsigned int& step) {
    size_t maxSize = 0;
    for (auto itQueue = cars.cbegin(); itQueue != cars.cend(); itQueue++) {
        if (maxSize < (*itQueue).size()) {
            maxSize = (*itQueue).size();
        }
        m_maxPriorityQueueLength[m_cellSize][step] = maxSize;
    }
}

/**
 * @brief Evaluation::saveNumberOfPriorityQueues saves the number of priority queues in every time instant
 * @param cars priority queue
 * @param step time instant
 */
void Evaluation::saveNumberOfPriorityQueues(CarGroupQueue& cars, const unsigned int& step) {
    for (auto itQueue = cars.cbegin(); itQueue != cars.cend(); itQueue++) {
        m_numberPriorityQueues[m_cellSize][step] = cars.rowSize();
    }
}

/**
 * @brief Evaluation::saveCurrentCellReservations save the current number of cell reservations dependent on the communication type (Interval-based)
 * @param cars
 * @param intervalType chosen type of interval
 * @param cellsize
 * @param step
 */
void Evaluation::saveCurrentCellReservations(CarGroupQueue& cars, const CommunicationScheme& intervalType, const double cellsize, const unsigned int& step) {
    for (const std::shared_ptr<Car>& car : cars.getOrderSeq()) {
        if (m_intervalTypeConsumedCellSizes[intervalType][cellsize].size() <= step) {
            m_intervalTypeConsumedCellSizes[intervalType][cellsize].push_back(car->getNumberCellsReserved());
        }
        else {
            m_intervalTypeConsumedCellSizes[intervalType][cellsize].at(step) += car->getNumberCellsReserved();
        }
    }
}

std::map<QString, Plot2d*> Evaluation::plots() const {
    return m_plots;
}

/**
 * @brief Evaluation::exportPlotInText exports the plots to a text file if they are needed e.g. in gnuplot
 * @param fileName
 * @param cols
 */
void Evaluation::exportPlotToText(const QString& fileName, const Plot2d *plot) {
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly| QIODevice::Text)) {
         qDebug() << "cannot write to " << fileName;
    }
    else {
        QTextStream outStream(&file);
        if (plot) {
            outStream << "Plot: " << plot->getTitle() << endl;
            //write names
            for (auto itCurves = plot->curves().begin(); itCurves != plot->curves().end(); itCurves++) {
                outStream << itCurves.key() << QString("     ");
            }
            outStream << QChar::CarriageReturn;
            //write data
            for (auto itCurves = plot->curves().begin(); itCurves != plot->curves().end(); itCurves++) {
                outStream << "curve: " << itCurves.key() << QChar::CarriageReturn;
                outStream << itCurves.value()->data();
            }
        }
    }
}

/**
 * @brief Evaluation::setPriorityCriteria
 * @param prio
 */
void Evaluation::setPriorityCriteria(const PriorityCriteria& prio) {
   m_prio = prio;
}

QDataStream& operator<<(QDataStream& os, Evaluation& eval) {
    os << "communication effort:";
    os << "priority: " << priorityCriteriaMap[(int)eval.m_prio].text;
    os << "cellsize, sumMessages";
    for (auto itComm = eval.m_gridSizeCommEffort.begin(); itComm != eval.m_gridSizeCommEffort.end(); itComm++) {
        os << itComm->first << "," << itComm->second;
    }
}
