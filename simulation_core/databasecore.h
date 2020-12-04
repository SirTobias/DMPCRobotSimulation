#ifndef DATABASECORE_H
#define DATABASECORE_H

#include "simsharedlib.h"

#include <QtSql>
#include <QString>
#include <QDebug>
#include <QPointer>
#include <QList>
#include <QMap>

/**
 * @brief The DataBaseCore class
 */
class SIM_CORE_EXPORT DataBaseCore
{
public:
    DataBaseCore(const QString driver);
    // the default driver is QPSQL
    DataBaseCore();
    ~DataBaseCore();
    bool connect( const QString& server,
                           const QString& databaseName,
                           const QString& userName,
                           const QString& password );
    void disConnect();

    // delete all data in a table
    int Delete(const QString& table);
    // referring to giving exact values, deleting data
    int Delete(const QString& table, const QStringList& columns, const QVariantList& values);
    // referring to range of the giving values, deleting data
    int Delete(const QString& table, const QStringList& columns, const QVariantList& values, const QStringList& relations);
    // delete a batch of data
    int Delete_Batch(const QString& table, const QString& column, const QVariantList& values);

    //update data in a table
    int Update(const QString& table, const QStringList& columns, const QVariantList& newValues, const QString& conditionColumn = QString(), const QVariant& conditionValue = QVariant());

    //select data from a table
    int Query(const QString& table, const QString& column = NULL, const QVariant& value = QVariant(), const QString& relation = QString("="));
    // referring to the giving exact values, selecting data from multiple tables
    int Query(const QStringList& tables, const QStringList& columns, const QVariantList& values);
    // referring to range of the giving values, selecting data from multiple tables
    int Query(const QStringList& tables, const QStringList& columns, const QVariantList& values, const QStringList& relations);

    // insert data into a table
    int Insert(const QString& table, const QStringList& columns, const QVariantList& values);
    //insert data into multiple tables
    int Insert(const QStringList& tables, const QStringList& columns, const QVariantList& values);
    // insert a batch of data into a table
    int Insert_Batch(const QString& table, const QStringList& columns, const QList<QVariantList>& listValues);
    // insert a batch of data into multiple tables
    int Insert(QStringList& tables, const QMap<QString,QVariantList>& lists);

    // basic function to call internal exec()
    bool Execute();

    // basic function to call internal excBatch()
    bool Execute_Batch();


    /// Every variable as below represents the name of a table in database
    const QString t_Car = "car";
    const QString t_Path = "path";
    const QString t_PathItem = "pathitem";

    /// Every variable as below represents the property of tables in database
    const QString p_CarId = "carid";
    const QString p_CarName = "carname";
    const QString p_PathId = "pathid";
    const QString p_PathItemId = "id";
    const QString p_State = "state";
    const QString p_X = "x";
    const QString p_Y = "y";
    const QString p_Time = "time";


signals:

public slots:

private:
    /// define a variable m_db to set up connection
    QSqlDatabase* m_db;
    /// define a variable m_query to execute operations
    QSqlQuery* m_query;

    // output data with a form of like a table
    void output();
    // define a method to create SQL statement for some query operations
    QString CreateQuery(const QStringList& tables, const QStringList& columns);
    //define a method to check variables
    bool CheckParams(const QStringList& key, const QVariantList& value);
};

#endif // DATABASECORE_H
