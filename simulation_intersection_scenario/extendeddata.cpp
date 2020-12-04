#include "extendeddata.h"

/**
 * @brief ExtendedData::ExtendedData
 */
ExtendedData::ExtendedData()
{

}

void ExtendedData::setData(const QString& key, const QVariant& value) {
    m_data[key] = value;
}
QVariant ExtendedData::data(const QString& key) const {
    return m_data.at(key);
}
