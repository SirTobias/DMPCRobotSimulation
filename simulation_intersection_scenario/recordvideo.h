#ifndef RECORDVIDEO_H
#define RECORDVIDEO_H

#include <QtCore/QString>
#include <QtGui/QScreen>
class RecordVideo : public QObject
{
public:
    RecordVideo(const QWidget *widget, const QString& path);
    void setWidget(const QWidget* widget);
    void start(const unsigned int& interval = 67);
    void timerEvent(QTimerEvent* event);
    void stop();
private:
    QString m_path;
    WId m_windowId;
    QScreen* m_screen;
    int64_t m_capIndex;
    int m_timerID;
    QString m_supportedFormat;
};

#endif // RECORDVIDEO_H
