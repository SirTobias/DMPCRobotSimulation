#include "cellgui.h"
#include "intersectionwindow.h"

constexpr unsigned int scalingFactor = 50;
constexpr unsigned int greenZone = 1;
constexpr unsigned int yellowZone = 10;
constexpr unsigned int redZone = 20;

/** @brief constructor to initialize cell at position x, y
 * @param x x-coordinate of cell
 * @param y y-coordinate of cell
 * @param reservations number of reservations in cell
 * @param width width related to amount of cols
 * @param guiType which CellGuiType reservation (all or current)
 * @param parent QGraphicsScene
 * @param constraintMargin overall margin for constraint size
 * @param scaling factor scales from simulation size (intersection width/height) to GUI size
 */
CellGui::CellGui(unsigned int x, unsigned int y, unsigned int reservations, const unsigned int &width, const CellGuiType guiType, const QPointer<QGraphicsScene> parent,
                 const double &constraintMargin, const double &scalingFactor):
    QGraphicsItem(nullptr),
    m_reservations(reservations),
    isTarget(false),
    m_guiType(guiType),
    m_brush(Qt::white),
    m_width(width),
    m_scene(parent),
    m_constraintMargin(constraintMargin * scalingFactor)
{
    //initialize private inherited members
    setPos(x*m_width, y*m_width);
    this->setZValue(-1);
}

CellGui::~CellGui() {
    //qDebug() << "CellGui: deleted";
}

/**
 * @brief CellGui::createConstraintLines create the rectangle around the occupied cells according to the constraints
 */
void CellGui::createConstraintLines() {
    if (m_constraintMargin > 0.0 && m_constraintSquare.size() == 0) {
        double cLeft = std::sqrt(std::pow(m_width - m_constraintMargin, 2)
                                 + std::pow(m_width - m_constraintMargin, 2));
        QGraphicsLineItem* item = new QGraphicsLineItem(nullptr);
        //left
        item->setLine(this->pos().x() - cLeft, this->pos().y() - cLeft,  this->pos().x() - cLeft, this->pos().y() + m_width + cLeft);
        m_scene->addItem(item);
        m_constraintSquare.append(item);
        //up
        item = new QGraphicsLineItem(this->parentItem());
        item->setLine(this->pos().x() - cLeft, this->pos().y() + m_width + cLeft, this->pos().x() + m_width + cLeft, this->pos().y() + m_width + cLeft);
        m_scene->addItem(item);
        m_constraintSquare.append(item);
        //right
        item = new QGraphicsLineItem(this->parentItem());
        item->setLine(this->pos().x() + m_width + cLeft, this->pos().y() + m_width + cLeft, this->pos().x() + m_width + cLeft, this->pos().y() - cLeft);
        m_scene->addItem(item);
        m_constraintSquare.append(item);
        //bottom
        item = new QGraphicsLineItem(this->parentItem());
        item->setLine(this->pos().x() + m_width + cLeft, this->pos().y() - cLeft, this->pos().x() - cLeft, this->pos().y() - cLeft);
        m_scene->addItem(item);
        m_constraintSquare.append(item);
    }
}

/**
 * @brief CellGui::deleteConstraintLines delete the lines around the now unoccupied cells
 */
void CellGui::deleteConstraintLines() {
    while (m_constraintSquare.size() > 0) {
        QGraphicsLineItem* lineItem = m_constraintSquare.takeFirst();
        m_scene->removeItem(lineItem);
        delete lineItem;
    }
}

/** @brief overridden to provide bounds for the cell
 */
QRectF CellGui::boundingRect() const
{
    return QRectF(0,0,m_width,m_width);

}

/** @brief tell how to draw cell object when added to scene
 */
void CellGui::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rec = boundingRect();
    QRectF textBound = QRectF(35, 35, 50, 50);

    if (m_guiType == CellGuiType::ALLRESERVED) {
        if (m_reservations >= redZone)
            m_brush.setColor(Qt::red);

        else if (m_reservations >= yellowZone)
            m_brush.setColor(Qt::yellow);

        else if (m_reservations >= greenZone)
            m_brush.setColor(Qt::green);
    }
    painter->fillRect(rec, m_brush);
    painter->drawRect(rec);

    if (m_guiType == CellGuiType::ALLRESERVED) {
        QString text = QString::number(m_reservations);
        painter->drawText(textBound, text);

        if (isTarget) {
            QRectF textBound2 = QRect(0, 0, 50, 50);
            painter->setFont(QFont("Helvetica", 25, QFont::Bold));
            painter->drawText(textBound2, Qt::AlignCenter, QStringLiteral("X"));
        }
    }
    //draws the constraint margin lines
    if (m_constraintSquare.size() > 0) {
        for (QGraphicsLineItem* line : m_constraintSquare) {
            line->paint(painter, option, widget);
        }
    }
}

/** @brief prompts GUI window to display informaiton regarding the cell
 * @param event mouse click
 */
void CellGui::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene* parentO = this->scene();
    IntersectionWindow* iw = ((IntersectionWindow*) parentO->parent());
    iw->displayInfo(this);
    QGraphicsItem::mousePressEvent(event);
}

/** @brief update the number of reservations per cell
 * @param reservations new number of reservations
 */
void CellGui::updateReservations(unsigned int reservations)
{
    m_reservations = reservations;
    update();

}

/**
 * @brief CellGui::updateReservations add the color of the car to the cell
 * if there are more than one color, the colors are added as
 * \f$ \frac{1}{#colors + 1} \sum_\limits{k=1}{n}color\f$
 * and color is added to the list
 * @param color color which should be set or merged to a set of colors
 */
void CellGui::updateReservations(const QColor& color) {

    double factor = 1.0/(m_colors.size() + 1.0);
    QColor colorSum;
    colorSum.setAlpha(color.alpha());
    colorSum.setRed(color.red() * factor);
    colorSum.setGreen(color.green() * factor);
    colorSum.setBlue(color.blue() * factor);
    for (const QColor currentColor : m_colors) {
        if (colorSum.red() + currentColor.red() * factor > 254) {
            colorSum.setRed(254);
        }
        else {
            colorSum.setRed(colorSum.red() + currentColor.red() * factor);
        }
        colorSum.setGreen(colorSum.green() + currentColor.green() * factor);
        colorSum.setBlue(colorSum.blue() + currentColor.blue() * factor);
    }
    m_brush.setColor(colorSum);
    m_colors.append(color);
    createConstraintLines();
    if (m_constraintSquare.size() > 0) {
        for (QGraphicsLineItem* line : m_constraintSquare) {
            QBrush constraintBrush = m_brush;
            QColor constraintColor = color;
            constraintColor.setAlpha(150);
            constraintBrush.setColor(constraintColor);
            constraintBrush.setStyle(Qt::NoBrush);
            line->setPen(constraintBrush.color());
        }
    }
    update();
}

/**
 * @brief CellGui::removeReservation removes the color from a set via
 * \f$ c_{new} = c_{all} - \frac{1\{#colors + 1} c_{remove}
 * @param color
 */
void CellGui::removeReservation(const QColor& color) {
    double factor = 1.0/(m_colors.size() + 1.0);
    QColor removeColor = m_colors.removeOne(color);
    if (m_colors.size() > 0) {
        QColor newColor = m_brush.color();
        newColor.setRed(newColor.red() - removeColor.red() * factor);
        newColor.setGreen(newColor.green() - removeColor.green() * factor);
        newColor.setBlue(newColor.blue() - removeColor.blue() * factor);
        for (const QColor currentColor : m_colors) {
            //newColor.setRed(newColor.red() + currentColor.red() * factor);
        }
        m_brush.setColor(newColor);
    }
    else {
        m_brush.setColor(Qt::white);
        deleteConstraintLines();
    }
    if (m_constraintSquare.size() > 0) {
        QBrush constraintBrush = m_brush;
        QColor constraintColor = constraintBrush.color();
        constraintColor.setAlpha(50);
        constraintBrush.setColor(constraintColor);
        QPen constraintPen;
        constraintPen.setColor(constraintColor);
        deleteConstraintLines();
    }
    update();
}

/** @brief get the current number of reservations the cell has
 * @return reservations
 */
unsigned int CellGui::getNumberOfReservations()
{
    return m_reservations;
}

/**
 * @brief CellGui::setTargetStatus
 * @param status
 */
void CellGui::setTargetStatus(bool status)
{
    isTarget = status;
    update();
}

/**
 * @brief CellGui::getColor
 * @return
 */
QColor CellGui::getColor() const {
    return m_brush.color();
}
