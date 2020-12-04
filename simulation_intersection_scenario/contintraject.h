#ifndef CONTINTRAJECT_H
#define CONTINTRAJECT_H

#include <QtWidgets/QGraphicsLineItem>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsScene>
#include <QtGui/QBrush>
#include <QtGui/QPainter>
#include <QtCore/QPointer>
#include <QtCore/QVector>
#include <QtCore/QObject>

#include <memory>

/**
 * @brief The continTraject class draws the continuous trajectory from the parent car linewise from point to point
 * inherits QGraphicsItem
 *
 */
class continTraject : public QObject, public QGraphicsItem
{
public:
    continTraject(const QPointer<QGraphicsScene> scene, QGraphicsItem* parent, const QPointF &startPoint, const QBrush& brush);
    virtual ~continTraject();
    void appendPoint(const QPointF &point);
    void setPrediction(const QVector<QPointF>& prediction);
    QRectF boundingRect() const;
    QVector<QGraphicsLineItem*> getLines();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
protected:
    static bool compareX(const QPointF& p1, const QPointF& p2);
    static bool compareY(const QPointF& p1, const QPointF& p2);
private:
    void deletePredictions();
    ///scene which takes the trajectory as item
    QPointer<QGraphicsScene> m_scene;
    ///points of the trajectory
    QVector<QPointF> m_points;
    ///lines which connects the points of the trajectory
    QVector<QGraphicsLineItem*> m_lines;
    ///defines color for cars and trajectory
    QBrush m_brush;
    QVector<QPointF> m_prediction;
    QVector<QGraphicsEllipseItem*> m_predictionItems;
    QVector<QGraphicsLineItem*> m_linePredictionItems;
    std::pair<QPointF, QPointF> m_rectBounds;
};

#endif // CONTINTRAJECT_H
