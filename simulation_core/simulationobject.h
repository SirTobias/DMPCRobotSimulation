#ifndef SIMULATIONOBJECT_H
#define SIMULATIONOBJECT_H
#include "simsharedlib.h"

#include <QtCore/QObject>
#include <QString>

#include <memory>

/**
 * @brief this class provides base class function for a simulation object
 * like a car or a product which should be processed by a simulation resource
 *
 */
class SIM_CORE_EXPORT SimulationObject : public QObject
{
public:
    SimulationObject();
    SimulationObject(const QString &name, const QString &productConfig);
    virtual ~SimulationObject();
    //get-set
    void setWaitTime(const double &t);
    void setEnterTime(const double &t);
    void setLeaveTime(const double &t);
    double getWaitTime() const;
    double getEnterTime() const;
    double getLeaveTime() const;
    QString getName() const;
    void setName(const QString &name);
    QString getProductConfig() const;
    void setProductConfig(const QString &config);
private:
    ///optional Object name
    QString m_name;
    ///wait time for being processed
    double m_twait;
    ///time when object enter the resource
    double m_tenter;
    ///time when object leave the resource
    double m_tleave;
    ///which product config has to be used - defined in SimulationResourceConfig
    QString m_productConfig;
};

#endif // SIMULATIONOBJECT_H
