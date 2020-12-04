 #include "evaluation.h"

Evaluation::Evaluation() :
    m_colourIndex(0)
{
    QStringList m_colourNames = QColor::colorNames();
    int i = 0;
    while (i < m_colourNames.size()) {
        if (m_colourNames.at(i).contains("lemonchiffon") || m_colourNames.at(i).contains("honeydew") || m_colourNames.at(i).contains("gainsboro")
                ||m_colourNames.at(i).contains("cornsilk") ||m_colourNames.at(i).contains("azure") ||m_colourNames.at(i).contains("white")
                || m_colourNames.at(i).contains("light") || m_colourNames.at(i).contains("gray") || m_colourNames.at(i).contains("beige")
                || m_colourNames.at(i).contains("snow") || m_colourNames.at(i).contains("linen") || m_colourNames.at(i).contains("mintcream")
                || m_colourNames.at(i).contains("transparent") || m_colourNames.at(i).contains("yellow") || m_colourNames.at(i).contains("bisque")
                || m_colourNames.at(i).contains("blanchedalmond") || m_colourNames.at(i).contains("silver") || m_colourNames.at(i).contains("wheat")
                || m_colourNames.at(i).contains("aliceblue") || m_colourNames.at(i).contains("grey") || m_colourNames.at(i).contains("ivory")
                || m_colourNames.at(i).contains("oldlace") || m_colourNames.at(i).contains("papayawhip")
                || m_colourNames.at(i).contains("seashell")) {
            m_colourNames.removeAt(i);
        }
        else {
            i++;
        }
    }
    for (QString& colourName : m_colourNames) {
        m_colours.append(QColor(colourName));
    }
    qDebug() << m_colours.size();
    for (auto it = m_colours.cbegin(); it != m_colours.cend(); it++) {
        qDebug() << *it;
    }
    QLocale::setDefault(QLocale(QLocale::Language::English, QLocale::UnitedStates));
}

/**
 * @brief Evaluation::saveOpenLoopCosts
 * @param t time step to save
 * @param costs costs are pushed back
 */
void Evaluation::saveOpenLoopCosts(const double& t, const double& costs) {
    m_openLoopCosts[t].push_back(costs);
}

/**
 * @brief Evaluation::saveClosedLoopCosts
 * @param t
 * @param costs
 */
void Evaluation::saveClosedLoopCosts(const double& t, const double &costs) {
    m_closedLoopCosts[t].push_back(costs);
}

/**
 * @brief Evaluation::plotOpenLoopCosts
 */
void Evaluation::plotOpenLoopCosts() {
    QString plotTitle = QString("Open-loop Costs");
    plot2DVector(plotTitle, QString::null, m_openLoopCosts);
}

/**
 * @brief Evaluation::plotClosedLoopCosts
 */
void Evaluation::plotClosedLoopCosts() {
    QString plotTitle = QString("Closed-loop Costs");
    plot2DVector(plotTitle, QString::null, m_closedLoopCosts);
}

/**
 * @brief Evaluation::showPlots
 */
void Evaluation::showPlots() {
    /*for (auto it = m_plots.begin(); it != m_plots.end(); it++) {
        it->second->show();
    }*/
}

/**
 * @brief Evaluation::plot2DVector
 * @param title
 * @param subTitlePrefix
 * @param vec expecting time : (x_1(0),...,x_P(0)
 */
void Evaluation::plot2DVector(const QString& title, const QString& subTitlePrefix, std::map<double, std::vector<double> > &map) {
    Plot2d* p2d = new Plot2d(nullptr, title);
    //for each machine, make one time row
    QMap<int, QMultiMap<double, double> > valsOverTime;
    //iterate over time
    for (auto itTime = map.begin(); itTime != map.end(); itTime++) {
        auto timeRow = itTime->second;
        //iterate over each time row, where value is x_1(0), x_2(0), ...
        for (int i = 0; i < timeRow.size(); i++) {
            //insert for machine i, at the Map at time t the value
            valsOverTime[i].insert(itTime->first, timeRow.at(i));
        }
    }
    int i = 0;
    for (auto itValsOverTime = valsOverTime.begin(); itValsOverTime != valsOverTime.end(); itValsOverTime++) {
        QMultiMap<double, double> vals = itValsOverTime.value();
        QString subTitle = subTitlePrefix;
        if (!subTitle.isNull()) {
            subTitle += QString("<sub>") + QString::number(i) + QString("</sub>");
        }
        p2d->addCurve(vals, subTitle, QwtPlotCurve::CurveStyle::Lines,
                      QwtSymbol::Style::NoSymbol, QColor(m_colours.at(getNextValidColor(m_colourIndex))), true);
        m_colourIndex++;
    }
    //TODOm_plots[title] = p2d;
}

/**
 * @brief Evaluation::plotMachineValues takes a map distinguished for every machine
 * @param title
 * @param subTitlePrefix
 * @param map time slot over every machine
 */

void Evaluation::exportPlots(const QString &type) {
    /*for (auto it = m_plots.begin(); it != m_plots.end(); it++) {
        it->second->exportToFile(it->first, type);
    }*/
}

/**
 * @brief Evaluation::getNextValidColor returns the index for a color != white
 * @param index
 * @return
 */
unsigned int Evaluation::getNextValidColor(const unsigned int& index)  {
    if (index >= m_colours.size()) {
        m_colourIndex = 0;
    }
    else {
        m_colourIndex = index;
    }
    return m_colourIndex;
}

