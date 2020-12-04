#include "vectorhelpertest.h"

/**
 * @brief VectorHelperTest::VectorHelperTest
 */
VectorHelperTest::VectorHelperTest()
{
}

void VectorHelperTest::initTestCase() {
    //
}

/**
 * @brief VectorHelperTest::testShiftVector
 */
void VectorHelperTest::testShiftVector() {
   std::vector<double> testVec = {1.0, 2.0, 3.0, 4.0};
   std::vector<double> expected = {2.0, 3.0, 4.0, 0.0};
   std::vector<double> result = VectorHelper::shiftStep(testVec, 1, 0.0);
   QCOMPARE(expected, result);
}
