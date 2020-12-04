#ifndef CELLGUI_H
#define CELLGUI_H

#include <QtCore/QObject>
#include <QtCore/QPointer>
#include <QtGui/QPainter>
#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsScene>
#include <iostream>
#include <memory>


/**
 * @brief The CellGuiType enum distinguish between
 * ALLRESERVED: displays the cell reservations over the whole simulation time
 * CURRENTRESERVED: displays only the current reserved cells
 */
enum class CellGuiType {
    ALLRESERVED = 0,
    CURRENTRESERVED = 1
};

/**
 * @brief The CellGui class represents an intersection cell in the GUI window
 * Inherits QGraphicsItem
 */
class CellGui : public QGraphicsItem, public QObject
{

public:
    CellGui(unsigned int x, unsigned int y, unsigned int reservations = 0, const unsigned int& width = 50,
            const CellGuiType guiType = CellGuiType::ALLRESERVED, const QPointer<QGraphicsScene> parent = NULL,
            const double &constraintMargin = 0.0, const double& scalingFactor = 0.0);
    virtual ~CellGui();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void createConstraintLines();
    void deleteConstraintLines();
    void updateReservations(unsigned int reservations);
    void updateReservations(const QColor& color);
    void removeReservation(const QColor& color);
    unsigned int getNumberOfReservations();
    void setTargetStatus(bool status);
    QColor getColor() const;

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);


private:
    unsigned int m_reservations;
    bool isTarget;
    CellGuiType m_guiType;
    QBrush m_brush;
    unsigned int m_width;
    ///handle for graphics scene
    QPointer<QGraphicsScene> m_scene;
    /// amount of constraint margin
    double m_constraintMargin;
    /// store current colors of the cars, which occupies the cell
    QList<QColor> m_colors;
    QList<QGraphicsLineItem*> m_constraintSquare;

};

#endif // CELLGUI_H
