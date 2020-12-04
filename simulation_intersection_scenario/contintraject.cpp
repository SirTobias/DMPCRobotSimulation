#include "contintraject.h"
#include <QtCore/QDebug>

/**
 * @brief continTraject::continTraject creates an instance with a startpoint, style (brush) and interval (steps) for a beginning
 * continuous trajectory
 * @param startPoint startpoint of the trajectory
 * @param brush style and colour
 */
continTraject::continTraject(const QPointer<QGraphicsScene> scene, QGraphicsItem* parent, const QPointF &startPoint, const QBrush& brush) :
    QGraphicsItem(nullptr),
    m_scene(scene),
    m_brush(brush),
    m_prediction(QVector<QPointF>()),
    m_predictionItems(QVector<QGraphicsEllipseItem*>()),
    m_linePredictionItems(QVector<QGraphicsLineItem*>())
{
    m_scene->addItem(this);
    m_points.push_back(startPoint);
    //setZValue(zValue() + 2);
}

/**
 * @brief continTraject::appendPoint appends a new point to the trajectory and paints them from the last point
 * to the current point
 * @param point next point to be painted
 */
void continTraject::appendPoint(const QPointF &point) {
    m_points.push_back(point);
    QGraphicsLineItem* lineItem = new QGraphicsLineItem(m_points.at(m_points.size() - 2).x(), m_points.at(m_points.size() - 2).y(),
                                                m_points.at(m_points.size() - 1).x(), m_points.at(m_points.size() - 1).y(), this->parentItem());
    if (lineItem) {
        lineItem->setPen(QPen(m_brush, lineItem->pen().width() + 3.0));
        //lineItem->setZValue(zValue()+1);
        m_lines.push_back(lineItem);
        m_scene->addItem(lineItem);
    }
    QVector<QPointF> sortPoints = m_points;
    std::sort(sortPoints.begin(), sortPoints.end(), compareX);
    //set min x and max x
    m_rectBounds.first.setX(sortPoints.first().x());
    m_rectBounds.second.setX(sortPoints.last().x());
    std::sort(sortPoints.begin(), sortPoints.end(), compareY);
    m_rectBounds.first.setY(sortPoints.first().y());
    m_rectBounds.second.setY(sortPoints.last().y());

    prepareGeometryChange();
    //DEBUG
    //qDebug() << "line: (" << m_lines.back()->line().x1() << "," << m_lines.back()->line().y1() << ") - ("
    //         << m_lines.back()->line().x2() << "," << m_lines.back()->line().y2() << ")";
    //--DEBUG
}

/**
 * @brief continTraject::compareX find smallest x
 * @param p1
 * @param p2
 * @return
 */
bool continTraject::compareX(const QPointF& p1, const QPointF& p2) {
    return p1.x() < p2.x();
}

/**
 * @brief continTraject::compareY find smallest y
 * @param p1
 * @param p2
 * @return
 */
bool continTraject::compareY(const QPointF& p1, const QPointF& p2) {
    return p1.y() < p2.y();
}

/**
 * @brief continTraject::boundingRect the outer bound of the whole trajectory
 * @return
 */
QRectF continTraject::boundingRect() const {
    /*if (m_points.size() < 2) {
        return QRectF(m_points.first(), m_points.last());
    }
    else {
        return QRectF(m_points.at(m_points.size() - 2), m_points.last());
    }*/
    return QRectF(m_rectBounds.first, m_rectBounds.second);
}

/**
 * @brief continTraject::paint overrides the paint-method
 * @param painter given painter instance
 * @param option not used here
 * @param widget not used here
 */
void continTraject::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    for (QGraphicsLineItem* line : m_lines) {
        line->paint(painter, option, widget);
        //qDebug() << "painted line: (" << line->line().x1() << "," << line->line().y1() << ")-(" << line->line().x2() << "," << line->line().y2() << ")";
    }
    for (QGraphicsEllipseItem* item : m_predictionItems) {
        item->paint(painter, option, widget);
    }
    for (QGraphicsLineItem* item : m_linePredictionItems) {
        item->paint(painter, option, widget);
    }
}

continTraject::~continTraject() {
    deletePredictions();
    for (QGraphicsLineItem* lines : m_lines) {
        if (m_scene) {
            m_scene->removeItem(lines);
        }
        delete lines;
        lines = nullptr;
    }
    m_lines.clear();
    m_scene->removeItem(this);
}


QVector<QGraphicsLineItem*> continTraject::getLines() {
    return m_lines;
}

/**
 * @brief continTraject::setPrediction sets the prediction points for the current stage
 * @param prediction
 */
void continTraject::setPrediction(const QVector<QPointF>& prediction) {
    //first remove old predictions
    deletePredictions();
    m_prediction = prediction;
    bool m_firstPredictionItem = true;
    for (const QPointF& pointPrediction : prediction) {
        QGraphicsEllipseItem* item = new QGraphicsEllipseItem(pointPrediction.x(), pointPrediction.y(), 4.0, 4.0, this->parentItem());
        item->setPen(QPen(m_brush, item->pen().width() + 2.0));
        m_scene->addItem(item);
        //qDebug() << "Prediction: " << pointPrediction.x() << ", " << pointPrediction.y();
        item->setRect(pointPrediction.x(), pointPrediction.y(), 4.0, 4.0);
        if (!m_firstPredictionItem) {
            //QGraphicsLineItem* linePredictionItem = new QGraphicsLineItem(m_predictionItems.last()->x(), m_predictionItems.last()->y(), item->x(), item->y());
            QGraphicsLineItem* linePredictionItem = new QGraphicsLineItem(item);
            linePredictionItem->setLine(m_predictionItems.last()->rect().x(), m_predictionItems.last()->rect().y(), item->rect().x(), item->rect().y());
            QPen parentPen = item->pen();
            parentPen.setStyle(Qt::DashLine);
            parentPen.setWidth(1.0);
            linePredictionItem->setPen(parentPen);
            //m_scene->addItem(linePredictionItem); //is done by parent
            m_linePredictionItems.append(linePredictionItem);
        }
        //qDebug() << "PredictionItem: " << item->x() << ", " << item->y();
        m_predictionItems.append(item);
        m_firstPredictionItem = false;
    }
}

/**
 * @brief continTraject::deletePredictions deletes the prediction points and lines objects
 */
void continTraject::deletePredictions() {
    for (QGraphicsLineItem* lineItem : m_linePredictionItems) {
        if (m_scene) {
            m_scene->removeItem(lineItem);
        }
        delete lineItem;
        lineItem = nullptr;
    }
    m_linePredictionItems.clear();
    for (QGraphicsEllipseItem* item : m_predictionItems) {
        if (m_scene) {
            m_scene->removeItem(item);
        }
        delete item;
        item = nullptr;
    }
    m_predictionItems.clear();
}
