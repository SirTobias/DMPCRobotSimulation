#ifndef CARGUI_H
#define CARGUI_H

#include "contintraject.h"
#include <QtGui/QPainter>
#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsScene>
#include <QtCore/QTimer>
#include <QtCore/QObject>
#include <QtCore/QPointer>
#include "memory"

/**
 * @brief The CarGui class represents a car object in the GUI window.
 * Inherits QGraphicsItem
 */
class CarGui : public QObject, public QGraphicsItem
{

public:
    CarGui(QString name = "car 0", const double& x = 0, const double& y = 0,
           const double& targetx = 0, const double& targety = 0, const QColor &trajecColor = QColor::fromRgb(0,0,255),
           const double& scalingFactor = 50.0, QPointer<QGraphicsScene> parent = nullptr, const double& minRadius = 0.0, const bool& showPrediction = false);
    virtual ~CarGui();
    QRectF boundingRect() const;
    QString getName() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void updatePos(const double &x, const double &y);
    void updatePrediction(const std::vector<std::vector<double> >& vec);
    void selected();
    void unselected();
    bool arrivedAtNewSpot();
    void timerEvent(QTimerEvent *event);
    const double& targetX();
    const double& targetY();
    std::shared_ptr<continTraject> getTrajectory();
    const QColor& getColor() const;
    void setRadius(const double& radius);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    QPointF getMiddlePoint(const QPointF &point) const;

private:
    QString m_name;
    bool m_isSelected;
    double m_finalX, m_finalY;
    double m_distanceX, m_distanceY;
    double m_TargetX, m_targetY;
    double m_scalingFactor;
    int m_timerId;
    std::shared_ptr<continTraject> m_continTrajectory;
    //style
    QBrush m_brush;
    QPointer<QGraphicsScene> m_scene;
    double m_minRadius;
    std::shared_ptr<QGraphicsEllipseItem> m_minRadiusItem;
    bool m_showPrediction;
};

#endif // CARGUI_H
