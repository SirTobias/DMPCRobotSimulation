#ifndef EXTENDEDDATA_H
#define EXTENDEDDATA_H

#include <QtCore/QString>
#include <QtCore/QVariant>

#include <map>

/**
 * @brief The ExtendedData class is to generalise the serial extended output beyond curves for databases or text files.
 * This could be values, which are summing up the data or mean values
 */
class ExtendedData
{
public:
    ExtendedData();
    void setData(const QString& key, const QVariant& value);
    QVariant data(const QString& key) const;
private:
    std::map<QString, QVariant> m_data;
};

#endif // EXTENDEDDATA_H
