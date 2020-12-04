#ifndef DATABASETHREAD_H
#define DATABASETHREAD_H

#include <QtCore/QThread>
#include "../simulation-core/databasecore.h"
#include <QtCore/QMutex>



/**
 * @brief create a new thread of database
 *
 *
 */
class DatabaseThread : public QThread
{

public:
    DatabaseThread();
    DatabaseThread(QMap<QString, QVariantList> input);
    ~DatabaseThread();
    QMap<QString, QVariantList> db_input;

signals:


protected:
    void run();

    QStringList db_carNames;
    void getCarNames();

public slots:
    void StartDatabase(QMap<QString,QVariantList> input);


private:
    DataBaseCore* m_dbc;

    QMutex mutex;
};

#endif // DATABASETHREAD_H
