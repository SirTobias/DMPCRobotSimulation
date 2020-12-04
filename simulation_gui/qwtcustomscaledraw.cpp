#include "qwtcustomscaledraw.h"

QwtCustomScaleDraw::QwtCustomScaleDraw(const QChar &format, const int precision) :
    QwtScaleDraw(),
    m_format(format),
    m_precision(precision)
{
}

QwtText QwtCustomScaleDraw::label(double value) const {

    QString num = QString::number(value, m_format.cell(), m_precision);
    return QwtText(num, QwtText::PlainText);
}
