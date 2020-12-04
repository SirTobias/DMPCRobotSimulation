#ifndef STARTBUTTON_H
#define STARTBUTTON_H

#include <QtCore/QObject>
#include <QtWidgets/QGraphicsItem>
#include <QtGui/QPainter>

/**
 * @brief The StartButton class allows window to display the start button right onto the scene
 */
class StartButton : public QGraphicsItem
{
public:
    StartButton(QGraphicsItem* parent = 0);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
};

#endif // STARTBUTTON_H
