#include "recordvideo.h"


#include <QtCore/QFile>
#include <QtCore/QStringBuilder>
#include <QtCore/QObject>
#include <QtGui/QGuiApplication>
#include <QtGui/QPixmap>
#include <QtWidgets/QWidget>
#include <QtGui/QImageWriter>

RecordVideo::RecordVideo(const QWidget* widget, const QString &path) :
    m_path(path),
    m_windowId(0),
    m_capIndex(0),
    m_timerID(0)
{
    m_screen = QGuiApplication::screens().at(0);
    m_windowId = widget->winId();
    QList<QByteArray> supportFormats = QImageWriter::supportedImageFormats();
    if (supportFormats.contains(QString("png").toLatin1())) {
        m_supportedFormat = "png";
    }
    else {
        m_supportedFormat = QString(supportFormats.at(0));
    }
}

void RecordVideo::setWidget(const QWidget* widget) {
    m_windowId = widget->winId();
}

void RecordVideo::start(const unsigned int& interval) {
    m_timerID = QObject::startTimer(interval);
}

void RecordVideo::timerEvent(QTimerEvent* event) {
    if (event->timerId() == m_timerID) {
        QPixmap capture = m_screen->grabWindow(m_windowId, 0, 0, -1, -1);
        //QDialog* dia = new QDialog;
        //QHBoxLayout hbox;
        //QLabel* lab = new QLabel;
        //lab->setPixmap(capture);
        //hbox.addWidget(lab);
        //dia->setLayout(&hbox);
        //dia->exec();
        QString title = m_path % QString("image") % QString::number(m_capIndex) % QString(".") % m_supportedFormat;
        //QFile testFile(title);
        //bool opened = testFile.open(QIODevice::WriteOnly);
        //testFile.close();
        bool captured = capture.save(title);
        m_capIndex++;
        //delete dia;
    }
}

void RecordVideo::stop() {
    QObject::killTimer(m_timerID);
}
