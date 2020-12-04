#ifndef QWTCUSTOMSCALEDRAW_H
#define QWTCUSTOMSCALEDRAW_H

#include <qwt_scale_draw.h>

#include <qwt_plot.h>

/**
 * @brief The QwtCustomScaleDraw class is mainly implemented to format the styles of numbers
 * and also the position of the ticks to get this more to 'MATLAB' style
 */
class QwtCustomScaleDraw : public QwtScaleDraw
{
public:
    explicit QwtCustomScaleDraw(const QChar& format = 'f', const int &precision = 6, const QwtPlot::Axis &axisId = QwtPlot::Axis::xBottom);

signals:

public slots:
public:
    virtual QwtText label(double value) const override;
    //virtual void drawTick(QPainter *painter, double val, double len) const;
private:
    QChar m_format;
    int m_precision;
    QwtPlot::Axis m_axisId;

};

#endif // QWTCUSTOMSCALEDRAW_H
