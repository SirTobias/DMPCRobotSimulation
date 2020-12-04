#ifndef PLOT2D_H
#define PLOT2D_H

#include "plot.h"

#include <qwt_plot_grid.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_legenditem.h>
#include <qwt_symbol.h>

#include <QtCore/QString>
#include <QtCore/QMultiMap>
#include <QtWidgets/QWidget>

class QwtPlot;
class QwtPlotCurve;
class QwtText;

enum class QwtPlotScaleType {
    LinearScale=0,
    LogScale=1
};

/**
 * @brief The Plot2d class based on QwtPlot
 */
class Plot2d : public Plot
{
public:
    Plot2d(QWidget *parent, const QString &title);
    ~Plot2d();

    void addCurve(const QMultiMap<double, double> &values, const QString &title = QString(), const QwtPlotCurve::CurveStyle &style = QwtPlotCurve::CurveStyle::Lines,
                  const QwtSymbol &symbol = QwtSymbol::Style::NoSymbol,
                  const QColor &color = QColor::fromRgb(0,0,0), const bool &showLegend = true);
    void show();
    void exportToFile(const QString& filename, const QString &type);
    QString getTitle() const;
    void addToLegend(const QString& data);
    void setAxisScale(const int& axisId, const double& min, const double& max, const double &step = 0.0);
    void setAxisTitle(const int& xAxisId, const QString& xAxisTitle, const int& yAxisId, const QString& yAxisTitle, const QwtText::TextFormat &textFormat = QwtText::AutoText);
    void setLegendAlignment(const Qt::Alignment& align);
    void setTitle(const QString &title);
    void setScaleEngine(const QwtPlotScaleType& type);
    void enableGrid(const bool& enable);
    void setAutoScale(const int& axisId, const int &maxNumSteps, const double &x1, const double &x2, const double &stepSize);
    void setAxisTicks(const int& axisID, const double& majorMin, const double& majorMax, const double& countMajorTicks,
                              const double& minorMin, const double& minorMax, const double& countMinorTicks, const bool &override = false);
    QwtPlot* getPlot();
    void setAxisFormat(const int& axisId, const QChar& format, const int& precision = 0);
    QMap<QString, QwtPlotCurve*> curves() const;
    void disableLegend();
    void setFontSize(const unsigned int& fontsize);
private:
    ///title of the plot
    QString m_title;
    ///handle for 2D-Qwt-Plot-Window
    QwtPlot* m_plot2d;
    ///Curves or points which should be plotted
    QMap<QString, QwtPlotCurve*> m_curves;
    ///symbol for data points
    QwtSymbol* m_symb;
    QwtPlotLegendItem* m_legendItem;
    ///Grid
    QwtPlotGrid* m_grid;
    ///x-points
    double* m_xArray;
    ///y-points
    double* m_yArray;
};

#endif // PLOT2D_H
