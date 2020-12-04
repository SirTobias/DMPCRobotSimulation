
#include "plot2d.h"

#include <qwt_plot.h>

#include <qwt_plot_curve.h>
#include <../textengines/mathml/qwt_mathml_text_engine.h>
#include <qwt_plot_renderer.h>
#include <qwt_legend.h>
#include <qwt_scale_engine.h>
#include <qwt_abstract_scale.h>
#include <qwt_text.h>
#include <qwtcustomscaledraw.h>

#include <QPointF>
#include <QPoint>
#include <typeinfo>

/**
 * @brief Plot2d::Plot2d
 * @param parent
 * @param title
 */
Plot2d::Plot2d(QWidget* parent, const QString &title) :
    m_title(title),
    m_plot2d(nullptr),
    m_curves(QMap<QString, QwtPlotCurve*>()),
    m_symb(nullptr),
    m_legendItem(nullptr),
    m_grid(nullptr),
    m_xArray(nullptr),
    m_yArray(nullptr)
{
    m_plot2d = new QwtPlot(QwtText(title), parent);
    m_plot2d->setCanvasBackground(QColor::fromRgb(255,255,255));
    m_plot2d->setPalette(Qt::white);
    m_legendItem = new QwtPlotLegendItem();
    m_legendItem->attach(m_plot2d);
    //increase font size
}

/**
 * @brief Plot2d::~Plot2d
 */
Plot2d::~Plot2d() {
    delete[] m_xArray;
    m_xArray = nullptr;
    delete[] m_yArray;
    m_yArray = nullptr;
    while (!m_curves.isEmpty()) {
        delete m_curves.take(m_curves.firstKey());
    }
    delete m_plot2d;
    m_plot2d = nullptr;
}

/**
 * @brief Plot2d::addCurve add a curve from a multimap with title, style and color
 * @param values Multimap as <double, double>
 * @param title title of the curfe
 * @param style style of the curve
 * @param color color of the curve
 */
void Plot2d::addCurve(const QMultiMap<double, double> &values, const QString &title, const QwtPlotCurve::CurveStyle &style, const QwtSymbol& symbol,
                      const QColor& color, const bool& showLegend) {
    QwtPlotCurve* curve = new QwtPlotCurve(title);
    int valuesSize = values.size();
    m_xArray = new double[valuesSize];
    m_yArray = new double[valuesSize];

    QVector<QPointF> pointsconverted;
    pointsconverted.resize(values.size());
    int countArray = 0;
    for (QMultiMap<double, double>::const_iterator itValues = values.begin(); itValues != values.end(); itValues++) {
        m_xArray[countArray] = itValues.key();
        m_yArray[countArray] = itValues.value();
        countArray++;
    }
    //QwtPointSeriesData<QPointF> points;
    curve->setRawSamples(m_xArray, m_yArray, valuesSize);
    if (showLegend) {
        curve->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
        //framebox around legend
        m_legendItem->setBorderPen(QPen(Qt::SolidLine));
        //avoid 2-col
        m_legendItem->setMaxColumns(1);
        //grid should not be visible behind legend
        m_legendItem->setBackgroundBrush(Qt::white);
        if (m_plot2d->legend()) {
            m_plot2d->legend()->setFrameStyle(QFrame::Box);
        }
    }
    else {
        curve->setLegendAttribute(QwtPlotCurve::LegendNoAttribute, false);
        curve->setLegendAttribute(QwtPlotCurve::LegendShowLine, false);
        curve->setItemAttribute(QwtPlotItem::Legend, false);

    }
    curve->setStyle(style);
    curve->setPen(color, 1.0, Qt::PenStyle::SolidLine);
    if (symbol.style() != QwtSymbol::NoSymbol) {
        m_symb = new QwtSymbol( symbol.style(), QBrush(curve->pen().color()), QPen(curve->pen().color()), QSize(10,10));
        curve->setSymbol(m_symb);
    }
    curve->attach(m_plot2d);
    //increase font size
    setFontSize(10);
    m_curves.insert(title, curve);
    m_plot2d->resize(400, 400);
    m_plot2d->updateAxes();
    m_plot2d->updateLegend();
    m_plot2d->replot();

}

/**
 * @brief Plot2d::addToLegend
 * @param data
 */
void Plot2d::addToLegend(const QString& data) {
    m_legendItem->setTitle(data);
    m_legendItem->legendChanged();
    m_legendItem->show();
    /*QList<QwtLegendData> legendDataList = m_legendItem->legendData();

    QwtLegendData legendItem;
    legendItem.setValue(QwtLegendData::UserRole, data);
    legendDataList.append(legendItem);
    m_legendItem->updateLegend(m_legendItem, legendDataList);
    m_legendItem->setTitle(data);
    for (QwtLegendData& leg : legendDataList) {
        QMap<int, QVariant> map = leg.values();
        for (auto it = map.begin(); it != map.end(); it++) {
            qDebug () << "Map Item: " << it->toString();
        }
    }*/
}

/**
 * @brief Plot2d::setAxisScale
 * @param axisId
 * @param min
 * @param max
 * @param step
 */
void Plot2d::setAxisScale(const int& axisId, const double& min, const double& max, const double& step) {
    m_plot2d->setAxisScale(axisId, min, max, step);
}

/**
 * @brief Plot2d::setAxisTicks set manually the axis ticks (major and minor ticks)
 * @param axisID id for axis
 * @param majorMin
 * @param majorMax
 * @param majorSteps
 * @param minorMin
 * @param minorMax
 * @param minorSteps
 */
void Plot2d::setAxisTicks(const int& axisID, const double& majorMin, const double& majorMax, const double& countMajorTicks,
                          const double& minorMin, const double& minorMax, const double& countMinorTicks, const bool& override) {
    QwtScaleDraw* scaleDraw = m_plot2d->axisScaleDraw(axisID);
    QwtInterval intervalMajor, intervalMinor;
    intervalMajor.setInterval(majorMin, majorMax);
    intervalMinor.setInterval(minorMin, minorMax);
    if (scaleDraw && !override) {
        const QwtScaleDiv& scaleDiv = scaleDraw->scaleDiv();
        intervalMajor = scaleDiv.interval();
        intervalMinor = scaleDiv.interval();
    }

    double majorDelta = 0.0, minorDelta = 0.0;
    QList<double> majorTicks, minorTicks;
    if (countMajorTicks != 0.0) {
        majorDelta = (intervalMajor.maxValue() - intervalMajor.minValue()) / countMajorTicks;
        for (double i = intervalMajor.minValue(); i <= intervalMajor.maxValue(); i += majorDelta) {
            majorTicks.append(i);
        }
    }
    if (countMinorTicks != 0.0) {
        minorDelta = (intervalMinor.maxValue() - intervalMinor.minValue()) / countMinorTicks;
        for (double i = minorMin; i <= minorMax; i += minorDelta) {
            minorTicks.append(i);
        }
    }
    //it is not appropriate to use QwtScaleDraw and to assign it to QwtPlot, as the changes get lost with replot!
    m_plot2d->setAxisScaleDiv(axisID, QwtScaleDiv(majorMin, majorMax, minorTicks, QList<double>(), majorTicks));
}
/**
 * @brief Plot2d::setAutoScale
 * @param axisId
 * @param maxNumSteps
 * @param x1
 * @param x2
 * @param stepSize
 */
void Plot2d::setAutoScale(const int& axisId, const int& maxNumSteps, const double& x1, const double& x2, const double& stepSize) {
    QwtScaleEngine* currentEngine = m_plot2d->axisScaleEngine(axisId);
    double calcX1 = x1, calcX2 = x2, calcStepSize = stepSize;
    if (currentEngine) {
        currentEngine->autoScale(maxNumSteps, calcX1, calcX2, calcStepSize);
    }
}

void Plot2d::setAxisTitle(const int& xAxisId, const QString& xAxisTitle, const int& yAxisId, const QString& yAxisTitle, const QwtText::TextFormat &textFormat) {
    if (textFormat == QwtText::MathMLText) {
        QwtText::setTextEngine(QwtText::MathMLText, new QwtMathMLTextEngine());
    }
    m_plot2d->setAxisTitle(xAxisId, QwtText(xAxisTitle, textFormat));
    m_plot2d->setAxisTitle(yAxisId, QwtText(yAxisTitle, textFormat));
}

void Plot2d::setLegendAlignment(const Qt::Alignment& align) {
    m_legendItem->setAlignment(align);
}

/**
 * @brief Plot2d::show
 */
void Plot2d::show() {
    m_plot2d->updateLegend();
    m_plot2d->replot();
    m_plot2d->show();
}

/**
 * @brief Plot2d::exportToFile
 * @param filename
 */
void Plot2d::exportToFile(const QString& filename, const QString& type) {
    m_plot2d->replot();
    QwtPlotRenderer* renderer = new QwtPlotRenderer(this->m_plot2d);
    renderer->setDiscardFlag(QwtPlotRenderer::DiscardNone);
    renderer->setLayoutFlag(QwtPlotRenderer::LayoutFlag::DefaultLayout);
    QString simplifiedSpace = filename;
    simplifiedSpace.replace(".", ",");
    simplifiedSpace = simplifiedSpace.remove(QChar(QChar::Space));
    renderer->renderDocument(m_plot2d, simplifiedSpace + QString(".") + type.toLower(), type, QSizeF(135, 135));
}

/**
 * @brief Plot2d::getTitle
 * @return
 */
QString Plot2d::getTitle() const {
    return m_title;
}

void Plot2d::setTitle(const QString& title) {
    m_plot2d->setTitle(title);
}

/**
 * @brief Plot2d::setScaleEngine either linear or logarithmic
 * @param type
 */
void Plot2d::setScaleEngine(const QwtPlotScaleType& type) {
    if (type == QwtPlotScaleType::LinearScale) {
        QwtScaleEngine* currentEngine = m_plot2d->axisScaleEngine(QwtPlot::Axis::yLeft);
        if (strncmp(typeid(currentEngine).name(), "QwtLogScaleEngine", sizeof("QwtLogScaleEngine")) != 0) {
            //setAxisScaleEngine deletes the old scale engine already
            m_plot2d->setAxisScaleEngine(QwtPlot::Axis::yLeft, new QwtLinearScaleEngine);
        }
    }
    else if (type == QwtPlotScaleType::LogScale) {
        QwtScaleEngine* currentEngine = m_plot2d->axisScaleEngine(QwtPlot::Axis::yLeft);
        if (strncmp(typeid(currentEngine).name(), "QwtLinearScaleEngine", sizeof("QwtLinearScaleEngine")) != 0) {
            //setAxisScaleEngine deletes the old scale engine already
            QwtLogScaleEngine* logEngine = new QwtLogScaleEngine(10);
            //QwtAbstractScale* logMajorScale = new QwtAbstractScale();
            //logMajorScale->setScaleEngine(logEngine);
            //logMajorScale->setScaleMaxMinor(1);
            //logMajorScale->setScaleMaxMajor(3);
            if (m_grid) {
                m_grid->enableYMin(false);
            }
            m_plot2d->setAxisScaleEngine(QwtPlot::Axis::yLeft, logEngine);
        }
    }
}

/**
 * @brief Plot2d::enableGrid
 * @param enable
 */
void Plot2d::enableGrid(const bool& enable) {
    if (enable) {
        if (!m_grid) {
            m_grid = new QwtPlotGrid();
        }
        m_grid->enableXMin(true);
        m_grid->enableX(true);
        m_grid->enableYMin(true);
        m_grid->enableY(true);
        m_grid->setMajorPen(QPen(Qt::black, 0.1, Qt::DashLine));
        m_grid->setMinorPen(QPen(Qt::gray, 0.0, Qt::DotLine));
        m_grid->attach(m_plot2d);
    }
    else {
        if (m_grid) {
            m_grid->detach();
        }
    }
}

/**
 * @brief Plot2d::setAxisFormat set label format for the axis
 * @param axisId id of axis in the format QwtPlot::Axis::yLeft
 * @param format e for exponential form or f for floating form
 * @param precision numbers after decimal point
 */
void Plot2d::setAxisFormat(const int& axisId, const QChar& format, const int& precision) {
    m_plot2d->setAxisScaleDraw(axisId, new QwtCustomScaleDraw(format, precision));
}


/**
 * @brief Plot2d::curves returns the curves in the current plot
 */
QMap<QString, QwtPlotCurve *> Plot2d::curves() const {
    return m_curves;
}

/**
 * @brief Plot2d::disableLegend removes legend
 */
void Plot2d::disableLegend() {
    m_plot2d->detachItems(QwtPlotItem::Legend, true);
    m_plot2d->detachItems(QwtPlotItem::LegendInterest, true);
}

/**
 * @brief Plot2d
 */
QwtPlot* Plot2d::getPlot() {
    return m_plot2d;
}

/**
 * @brief Plot2d::setFontSize set fontsize for x/y-axis to fontsize
 * @param fontsize
 */
void Plot2d::setFontSize(const unsigned int& fontsize) {
    QFont xAxisFont = m_plot2d->axisFont(QwtPlot::Axis::xBottom);
    xAxisFont.setPointSize(fontsize);
    m_plot2d->setAxisFont(QwtPlot::Axis::xBottom, xAxisFont);
    QFont yAxisFont = m_plot2d->axisFont(QwtPlot::Axis::yLeft);
    yAxisFont.setPointSize(fontsize);
    m_plot2d->setAxisFont(QwtPlot::Axis::yLeft, yAxisFont);
}
