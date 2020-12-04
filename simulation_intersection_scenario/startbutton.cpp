#include "startbutton.h"
#include "intersectionwindow.h"

StartButton::StartButton(QGraphicsItem *parent):QGraphicsItem(parent)
{

}

/** @brief overridden to provide bounds for the button
 */
QRectF StartButton::boundingRect() const
{
    return QRectF(0,0,200,100);
}

/** @brief tell how to draw button object when added to scene
 */
void StartButton::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rec = boundingRect();
    painter->setFont(QFont("Helvetica", 20, QFont::Bold));
    QBrush brush(Qt::gray);
    painter->fillRect(rec, brush);
    painter->setPen(Qt::white);
    painter->drawText(rec,Qt::AlignCenter, QString("START\nSIMULATION"));

}

/** @brief calls the parent scene to start the parent window simulation when button is pressed
 * @param event mouse pressed
 */
void StartButton::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene* parentO = this->scene();
    IntersectionWindow* iw = ((IntersectionWindow*) parentO->parent());
    QGraphicsItem::mousePressEvent(event);
    iw->start();
}

