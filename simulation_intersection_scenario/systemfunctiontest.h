#ifndef SYSTEMFUNCTIONTEST_H
#define SYSTEMFUNCTIONTEST_H

#include <QtTest/QtTest>

/**
 * @brief The SystemFunctionTest class tests the systemfunction class for calculating trajectory
 */
class SystemFunctionTest : public QObject
{
    Q_OBJECT
public:
    SystemFunctionTest();
private slots:
    void calculateTrajectory();
private:

};

#endif // SYSTEMFUNCTIONTEST_H
