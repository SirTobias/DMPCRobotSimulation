#ifndef SIMULATIONRESOURCECONFIG_H
#define SIMULATIONRESOURCECONFIG_H
#include "simsharedlib.h"

#include <QMap>
#include <QString>

/**
 * @brief holds an internal map about the product and its processing time
 * Key is a String as a unique name and processing time is a double as value
 */
class SIM_CORE_EXPORT SimulationResourceConfig
{
public:
    SimulationResourceConfig();
    virtual ~SimulationResourceConfig();
    void insertProductConfig(const QString& config, const double &time);
    double getProductConfig(const QString& config) const;
private:
    ///internal map for product config
    QMap<QString, double> m_productConfig;
};

#endif // SIMULATIONRESOURCECONFIG_H
