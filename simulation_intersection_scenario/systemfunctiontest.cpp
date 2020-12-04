#include "systemfunctiontest.h"
#include "systemfunction.h"

SystemFunctionTest::SystemFunctionTest()
{
}

void SystemFunctionTest::calculateTrajectory() {
    std::vector<double> startPos({0.0, 0.0});
    SystemFunction sysFunc("car0", startPos);
    std::vector<std::vector<double> > u;
    double t0 = 0;
    unsigned int T = 1;
    unsigned N = 3;
    for (unsigned int i = 0; i < N; i++) {
        u.push_back({1.0, 0.5});
    }

    std::vector<std::vector<double> > x = sysFunc.getHolonomicSystemTrajectory(sysFunc.getCurrentContinuousState(), u, t0, T, N);
    std::vector<std::vector<double> > refTrajectory;
    refTrajectory.push_back(startPos);
    for (unsigned int i = 0; i < N; i++) {
        refTrajectory.push_back({refTrajectory.back().at(0) + 1.0, refTrajectory.back().at(1) + 0.5});
    }
    QCOMPARE(x, refTrajectory);
}
