#ifndef PLOT2D_H
#define PLOT2D_H

#include "plot.h"
#include "extendeddata.h"

#include <qwt_plot_grid.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_legenditem.h>
#include <qwt_plot_spectrogram.h>
#include <qwt_symbol.h>
#include <qwt_plot.h>

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
    void exportToTextFile(const QString& fileName);
    QString getTitle() const;
    void addToLegend(const QString& data);
    void setAxisScale(const int& axisId, const double& min, const double& max, const double &step);
    void setAxisTitle(const int& xAxisId, const QString& xAxisTitle, const int& yAxisId, const QString& yAxisTitle, const QwtText::TextFormat &textFormat = QwtText::AutoText);
    void setLegendAlignment(const Qt::Alignment& align);
    void setTitle(const QString &title);
    void setScaleEngine(const QwtPlotScaleType& type);
    void enableGrid(const bool& enable);
    void setAutoScale(const int& axisId, const int &maxNumSteps, const double &x1, const double &x2, const double &stepSize);
    void setAxisTicks(const int& axisID, const double& majorMin, const double& majorMax, const double& countMajorTicks,
                              const double& minorMin, const double& minorMax, const double& countMinorTicks, const bool &override = false);
    void addSpectogram(const QVector<QVector<unsigned int> > &values, const QString &title = QString());
    QwtPlot* getPlot();
    void setAxisFormat(const QwtPlot::Axis &axisId, const QChar& format, const int& precision = 0);
    QMap<QString, QwtPlotCurve*> curves() const;
    void disableLegend();
    void enableLegend();
    void setFontSize(const unsigned int& fontsize);
    void addExtData(const QString& key, const QVariant& value);
    QVariant extData(const QString& key) const;
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
    ///Spectogram for raster plots
    QwtPlotSpectrogram* m_spectogram;
    ///x-points
    double* m_xArray;
    ///y-points
    double* m_yArray;
    ExtendedData m_extData;
};

#endif // PLOT2D_H
