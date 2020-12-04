#include "simulationresourceconfig.h"

/**
 * @brief initialize the map
 */
SimulationResourceConfig::SimulationResourceConfig()
{
    m_productConfig = QMap<QString, double>();
}

/**
 * @brief Destructor
 */
SimulationResourceConfig::~SimulationResourceConfig() {

}

/**
 * @brief insert a Key-Value-Pair as a product config
 * @param config name of product - key
 * @param time value of product processing - value
 */
void SimulationResourceConfig::insertProductConfig(const QString& config, const double &time) {
    m_productConfig.insert(config, time);
}

/**
 * @brief returns a time for a given product
 * @param config name of product
 * @return time for processing the product
 */
double SimulationResourceConfig::getProductConfig(const QString& config) const {
    return m_productConfig.value(config);
}
