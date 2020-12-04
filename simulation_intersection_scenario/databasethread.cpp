#include "databasethread.h"
#include <QtCore/QDebug>
#include <iostream>

DatabaseThread::DatabaseThread()
{
    m_dbc = new DataBaseCore();
    m_dbc->connect("localhost","simulation_intersection","Homyum","123456");

}

DatabaseThread::DatabaseThread(QMap<QString, QVariantList> input)
{
    DatabaseThread();
//    db_input = input;
}


DatabaseThread::~DatabaseThread(){

}

/** @brief start database thread
 */
void DatabaseThread::StartDatabase(QMap<QString,QVariantList> input)
{
    db_input = input;
    start(LowestPriority);

}

/** @brief execute database thread
 */
void DatabaseThread::run()
{
    mutex.lock();

    QStringList tables;
    tables.push_back(m_dbc->t_Car);
    tables.push_back(m_dbc->t_Path);
    tables.push_back(m_dbc->t_PathItem);
    qDebug() << "Executing insertion ...";
    m_dbc->Insert(tables, db_input);
    qDebug() << "Insertion is Finished!";
    m_dbc->disConnect();

    mutex.unlock();

}
