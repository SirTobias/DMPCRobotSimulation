#ifndef VECTORHELPERTEST_H
#define VECTORHELPERTEST_H

#include <QtTest/QtTest>

#include "../vectorhelper.h"

class VectorHelperTest : public QObject
{
    Q_OBJECT
public:
    VectorHelperTest();
private slots:
    void initTestCase();
    void testShiftVector();
private:
};

#endif // VECTORHELPERTEST_H
