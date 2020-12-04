#include "qwtcustomscaledraw.h"

/**
 * @brief QwtCustomScaleDraw::QwtCustomScaleDraw
 * @param format
 * @param precision
 */
QwtCustomScaleDraw::QwtCustomScaleDraw(const QChar &format, const int &precision, const QwtPlot::Axis &axisId ) :
    QwtScaleDraw(),
    m_format(format),
    m_precision(precision),
    m_axisId(axisId)
{
}

/**
 * @brief QwtCustomScaleDraw::label
 * @param value
 * @return
 */
QwtText QwtCustomScaleDraw::label(double value) const {

    QString num = QString::number(value, m_format.cell(), m_precision);
    return QwtText(num, QwtText::PlainText);
}

/**
 * @brief QwtCustomScaleDraw::drawTick
 * @param val
 * @param len
 */
/*void QwtCustomScaleDraw::drawTick(QPainter *painter, double val, double len) const {
    if (painter) {
        if (m_axisId == QwtPlot::Axis::xBottom) {
            QwtScaleDraw::drawTick(painter, val-= 10.0, len);
        }
        else if (m_axisId == QwtPlot::Axis::yLeft) {
            QwtScaleDraw::drawTick(painter, val+= 10.0, len);
        }
    }
}*/
