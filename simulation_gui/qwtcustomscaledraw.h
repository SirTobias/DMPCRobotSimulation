#ifndef QWTCUSTOMSCALEDRAW_H
#define QWTCUSTOMSCALEDRAW_H

#include <qwt_scale_draw.h>

class QwtCustomScaleDraw : public QwtScaleDraw
{
public:
    explicit QwtCustomScaleDraw(const QChar& format = 'f', const int precision = 6);

signals:

public slots:
public:
    virtual QwtText label(double value) const override;
private:
    QChar m_format;
    int m_precision;

};

#endif // QWTCUSTOMSCALEDRAW_H
