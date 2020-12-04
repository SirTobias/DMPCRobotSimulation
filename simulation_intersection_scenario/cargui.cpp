#include "cargui.h"
#include "intersectionwindow.h"
#include <cmath>

//constexpr unsigned int scalingFactor = 50; //coordinates to pixels

/** @brief constructor, default is car 0 at 0,0
 * converts coordinates to screen coordinates before moving car over
 */
CarGui::CarGui(QString name, const double &x, const double &y, const double &targetx, const double &targety,
               const QColor& trajecColor, const double& scalingFactor, QPointer<QGraphicsScene> parent,
               const double& minRadius, const bool &showPrediction):
    QGraphicsItem(nullptr),
    m_name(name),
    m_isSelected(false),
    m_finalX(x),
    m_finalY(y),
    m_distanceX(0.0),
    m_distanceY(0.0),
    m_TargetX(targetx),
    m_targetY(targety),
    m_scalingFactor(56.0),//TODO: scaling factor correct
    //m_scalingFactor(scalingFactor),
    m_timerId(0),
    m_brush(QColor(trajecColor)),
    m_scene(parent),
    m_minRadius(minRadius * m_scalingFactor),
    m_showPrediction(showPrediction)
{
    //initialize inherited private members
    setPos(x*(double)m_scalingFactor, y*(double)m_scalingFactor);
    QPointF middlePos = getMiddlePoint(pos());
    setPos(middlePos.x(), middlePos.y());
    m_scene->addItem(this);
    //TODO: Debug crashes sometimes after finished simulations, some parent objects are not there, where they should be?!
    //m_continTrajectory = std::make_shared<continTraject>(parent, this->parentItem(), middlePos, QBrush(QColor(trajecColor)));
    m_brush.setColor(QColor::fromRgb(m_brush.color().red(), m_brush.color().green(), m_brush.color().blue(), 150));
    setZValue(zValue() + 1);
    if (m_minRadius > 0.0) {
        if (m_minRadiusItem == nullptr) {
            m_minRadiusItem = std::make_shared<QGraphicsEllipseItem>(this->parentItem());
            m_scene->addItem(m_minRadiusItem.get());
            m_minRadiusItem->setBrush(m_brush.color());
            QBrush radiusBrush = m_minRadiusItem->brush();
            radiusBrush.setColor(QColor(radiusBrush.color().red(), radiusBrush.color().green(), radiusBrush.color().blue(), 100));
            m_minRadiusItem->setBrush(radiusBrush);
            QPen radiusPen = m_minRadiusItem->pen();
            radiusPen.setWidth(1.0);
            m_minRadiusItem->setPen(radiusPen);
        }
        m_minRadiusItem->setRect(pos().x(), pos().y(), m_minRadius, m_minRadius);
    }
    else {
        m_minRadiusItem.reset();
    }
}

CarGui::~CarGui() {
    if (m_minRadiusItem) {
        m_scene->removeItem(m_minRadiusItem.get());
    }
    //qDebug() << "CarGUI: deleted";
}

/** @brief bounds of car graphic
 * @return QRectF
 */
QRectF CarGui::boundingRect() const
{
    if (m_minRadius > 0.0) {
        return QRectF(0,0,m_minRadius, m_minRadius);
    }
    return QRectF(0,0,30,30);
}

/** @brief return car name
 * @return m_name
 */
QString CarGui::getName() const
{
    return m_name;
}

/** @brief tell how to draw car object when added to scene
 */
void CarGui::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rec = boundingRect();


    QRectF textBound = QRectF(2, 0, 30, 30);

    if(m_isSelected)
        m_brush.setColor(Qt::cyan);

    if (m_minRadius == 0.0) {
        painter->fillRect(rec,m_brush);
        painter->drawRect(rec);
    }
    else {
        painter->setBrush(m_brush);
        painter->drawEllipse(rec.x(), rec.y(), m_minRadius, m_minRadius);
    }
    painter->drawText(textBound,m_name);
    /*if (m_minRadiusItem) {
        m_minRadiusItem->setRect(rec.x(), rec.y(), m_minRadius, m_minRadius);
        m_minRadiusItem->paint(painter, option, widget);
    }*/
}

/** @brief will move the car to new x and y position
 * @param x new x-coordinate of cell
 * @param y new y-coordinate of cell
 */
void CarGui::updatePos(const double& x, const double& y)
{
    QPointF finalPos(x*(double)m_scalingFactor, y*(double)m_scalingFactor);
    finalPos = getMiddlePoint(finalPos);
    m_finalX = finalPos.x();
    m_finalY = finalPos.y();

    m_distanceX = m_finalX - this->x();
    m_distanceY = m_finalY - this->y();

    if (m_timerId > 0) {
        killTimer(m_timerId);
        //qDebug() << "timer of car" << m_name << "killed";
    }
    //DEBUG
    /*if (m_name == "car6" || m_name == "car7") {
        qDebug() << "car " << m_name << ": " << "got pos(" << m_continTrajectory->getLines().size() <<  "): " << x << "," << y << ";"
                 << "curPos: " << this->pos().x() << "," << this->pos().y()
                    << ", finalPos: " << m_finalX << "," << m_finalY;
    }*/
    //--DEBUG
    //to see car overlayed on other car
    //setZValue(zValue()+1);
    //append position to new trajectory
    if (m_continTrajectory) {
        m_continTrajectory->appendPoint(finalPos);
    }
    //timer interval steps amount should be at least diff(dx/dy) and the absolute value
    m_timerId = startTimer(10);
    /*if (m_minRadius > 0.0 && m_minRadiusItem) {
        m_minRadiusItem->setRect(pos().x(), pos().y(), m_minRadius, m_minRadius);
    }
    else {
        m_scene->removeItem(m_minRadiusItem.get());
        m_minRadiusItem.reset();
    }*/
}

/**
 * @brief CarGui::updatePrediction updates the points for the prediction
 * @param vec
 */
void CarGui::updatePrediction(const std::vector<std::vector<double> >& vec) {
    if (m_showPrediction) {
        QVector<QPointF> prediction;
        for (const std::vector<double> pos : vec) {
            QPointF finalPos(pos.at(0)*(double)m_scalingFactor, pos.at(1)*(double)m_scalingFactor);
            finalPos = getMiddlePoint(finalPos);
            prediction.append(finalPos);
        }
        if (m_continTrajectory) {
            //m_continTrajectory->setPrediction(prediction); TODO: problem with delete of predictions
        }
    }
}


/** @brief car is selected to display path on simulation gui
 */
void CarGui::selected()
{
    m_isSelected = true;
    setZValue(zValue()+10);
}

/** @brief when different car is selected on simulation gui
 */
void CarGui::unselected()
{
    m_isSelected = false;
    setZValue(zValue()-10); //reset z value to before select
}

/** @brief check to see if car arrived at destination
 * @return boolean
 */
bool CarGui::arrivedAtNewSpot()
{
    if (std::abs(this->x() - m_finalX) < 0.01 && std::abs(this->y() - m_finalY) < 0.01)
        return true;

    return false;
}

/** @brief moves car every timer event towards destination
 */
void CarGui::timerEvent(QTimerEvent *event)
{
    if (m_name == "car0") {
        QString str = "";
    }
    m_timerId = event->timerId();
    double direction;
    if (m_distanceX == 0) {
        //direction = .01;
    }
    else {
        direction = m_distanceX / 100;
    }

    if (this->x() != m_finalX) {
        setPos(this->x()+direction, this->y());
        /*if (m_minRadiusItem) {
            m_minRadiusItem->setPos(this->pos().x() + direction, this->pos().y());
        }*/
    }

    if (m_distanceY == 0) {
        //direction = .01;
    }
    else {
        direction = m_distanceY / 100;
    }

    if (this->y() != m_finalY) {
        setPos(this->x(), this->y()+direction);
        /*if (m_minRadiusItem) {
            m_minRadiusItem->setPos(this->pos().x(), this->pos().y() + direction);
        }*/
    }
    if (arrivedAtNewSpot()) {
        killTimer(m_timerId);
        //qDebug() << m_name << " reached at position: (" << this->pos().x()<< "," << this->pos().y() << ")";
        m_timerId = 0;
    }
    //DEBUG
    //qDebug() << m_name << " Pos: " << pos().x() << "," << pos().y();
    //--DEBUG
}

/** @brief returns x-coordinate of target
 * @return target_x
 */
const double &CarGui::targetX()
{
   return m_TargetX;
}

/** @brief returns y-coordinate of target
 * @return target_y
 */
const double &CarGui::targetY()
{
   return m_targetY;
}

/** @brief will tell GUI window to display car name onto text box
 * @param event QGraphicsScene*
 */
void CarGui::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene* parentO = this->scene();
    IntersectionWindow* iw = ((IntersectionWindow*) parentO->parent());
    iw->displayCar(this);
    QGraphicsItem::mousePressEvent(event);
}

/**
 * @brief CarGui::getMiddlePoint returns the middle point of the rectangle as we have to take
 * the actual given coordinates as middle point and not as start point
 * @param rect
 * @return
 */
QPointF CarGui::getMiddlePoint(const QPointF& point) const {
    double length = std::sqrt(std::pow(point.x() + 30 - point.x(),2) + std::pow(point.y() + 30 - point.y(), 2) );
    return QPointF(point.x() - length / 2.0, point.y() - length / 2.0);
}

/**
 * @brief CarGui::getTrajectory
 * @return
 */
std::shared_ptr<continTraject> CarGui::getTrajectory() {
    return m_continTrajectory;
}

/**
 * @brief CarGui::getColor returns the color of the car
 * @return
 */
const QColor& CarGui::getColor() const {
    return m_brush.color();
}

/**
 * @brief CarGui::setRadius sets the radius for the car, if 0.0, nothing will be drawn
 * @param radius radius of drawn circle, if 0.0, nothing will be drawn
 */
void CarGui::setRadius(const double& radius) {
    m_minRadius = radius;
}
