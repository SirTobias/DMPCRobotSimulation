#include "linearcongruentialgenerator.h"
#include <iostream>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDir>
#include <QStringBuilder>

#include <cmath>

const int LinearCongruentialGenerator::primesFileCount = 1000000;
const int LinearCongruentialGenerator::primesFileTabs = 8;
const int LinearCongruentialGenerator::primesFileStart = 3;

/**
 * @brief LinearCongruentialGenerator::LinearCongruentialGenerator
 */
LinearCongruentialGenerator::LinearCongruentialGenerator(const uint64_t &seed, const uint64_t &startInterval, const uint64_t &endInterval) :
    state(0),
    increment(0),
    startInterval(startInterval),
    endInterval(endInterval),
    diffInterval(endInterval - startInterval),
    curResultPosition(0)
{
    init(seed, startInterval, endInterval);
}

int LinearCongruentialGenerator::getState() {
    return this->state;
}

/**
 * @brief LinearCongruentialGenerator::~LinearCongruentialGenerator
 */
LinearCongruentialGenerator::~LinearCongruentialGenerator() {

}

/**
 * @brief LinearCongruentialGenerator::init
 * @param seed
 * @param startInterval
 * @param endInterval
 */
void LinearCongruentialGenerator::init(const uint64_t &seed, const uint64_t &startInterval, const uint64_t &endInterval) {
    state = 1;
    modul = new uint64_t[state];
    factor = new double[state];
    increment = 0;
    //increment = new double[state];
    result = new double[state];
    for (unsigned int i = 0; i < state; i++) {
        modul[i] = 0;
        factor[i] = 0;
        result[i] = 0;
    }
    //prime numbers for mod
    createArrayWithPrim(modul, state);
    //TODO: test suite: test with state == 1
    if (state == 1) {
        modul[0] = 2147483647;
    }

    //factors randomly with cpu load
    //for initializing a as factor according to Knuth it should be 0.01 * m < a < 0.99 * m
    for (unsigned int i = 0; i < state; i++) {
        double fac = (double)modul[i] * getCpuLoad();
        factor[i] = fac;
    }
    //TODO: test suite: test with state == 1
    if (state == 1) {
        factor[0] = 16807;
    }
    for (unsigned int i = 0; i < state; i++) {
        result[i] = getSeededStartValue();
    }
    //TODO: test suite: test with state == 1
    if (state == 1) {
        result[0] = 1;
    }

    increment = smallestValue(modul, state);
    //TODO: test suite: test with state == 1
    if (state == 1) {
        increment = 0;
    }
    if (startInterval != 0) {
        this->startInterval = startInterval;

    }
    if (endInterval != 0) {
        this->endInterval = endInterval;
    }
    if (endInterval == 0) {
        this->endInterval = INT_MAX;
    }
    getRandom(true);
}

double LinearCongruentialGenerator::getSeededStartValue() {
    QProcess seedStartProc;
    //we took the ctext-kernel pages which are increasing randomly as basis for the seed
    //@TODO: equivalent system call function for windows
    seedStartProc.start("sh -c \"cat /proc/stat | grep -i ctxt\"", QIODevice::ReadOnly);
    double startValue = 0;
    if (seedStartProc.waitForFinished(1000)) {
        int procOutputSize = 100;
        char* procOutput = new char[procOutputSize];
        procOutput[0] = '0';
        seedStartProc.read(procOutput, procOutputSize);
        QString procOutputStr(procOutput);
        delete[] procOutput;
        procOutput = nullptr;
        procOutputStr.remove("ctxt ");
        procOutputStr.remove("\n");
        procOutputStr.remove("\177");
        bool converted = false;
        startValue = procOutputStr.toDouble(&converted);
        if (!converted) {
            std::cout << "Kernelfunction /proc/stat could not be readed for seed" << std::endl;
        }
    }
    seedStartProc.close();
    return startValue;
}

double LinearCongruentialGenerator::getCpuLoad() {
    double cpuLoad = 0;
    QProcess cpuLoadProcess;
    cpuLoadProcess.setProcessChannelMode(QProcess::MergedChannels);
    cpuLoadProcess.start("sh -c \"cat /proc/loadavg | cut -d ' ' -f 1\"", QIODevice::ReadOnly);
    QStringList arg = cpuLoadProcess.arguments();
    if (cpuLoadProcess.waitForFinished(1000)) {
        //unsigned int cpuOutputSize = 4;
        //char* cpuOutput = new char[cpuOutputSize];
        //cpuOutput[0] = '\0';
        QString cpuLoadStr = QString(cpuLoadProcess.readAllStandardOutput());
        cpuLoadStr.remove("\n");
        cpuLoad = cpuLoadStr.toDouble();
        //if cpuLoad > 1, decrement, because we need values between 0 and 1
        if (cpuLoad > 1) {
            double ceiled = std::ceil(cpuLoad);
            cpuLoad = cpuLoad / ceiled;
        }
    }
    cpuLoadProcess.close();
    return cpuLoad;
}

/**
 * @brief creates prim with the sieve of eratosthenes
 * @param mod array to be filled
 * @param length
 */
void LinearCongruentialGenerator::createArrayWithPrim(uint64_t *&mod, const int &length) {
    QDir primesPath = QDir::current();
    primesPath.cdUp();
    primesPath.cd("prime");
    QFile primFile(primesPath.path() % QString(QDir::separator()) % QString("primes1.txt"));
    int lineLength = 82;
    int countArrayLength = 0;
    if (primFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        uint64_t primFileSize = primFile.size();
        uint64_t startPoint = uint64_t((double)getCpuLoad() * (double)primFileSize) / primesFileTabs;
        QTextStream primeIn(&primFile);
        //header must be overjumped
        if (startPoint < 69) {
            startPoint = 69;
        }
        if (startPoint > primFileSize - length * lineLength) {
            startPoint -= primFileSize - length * lineLength;
        }
        primFile.seek(startPoint);
        //uint64_t curPos = primFile.pos();
        //read until next line
        //bool alignedToLineBegin = false;
        QString strChar = "0";
        while (!primeIn.atEnd() && strChar.at(0) != QChar::LineFeed) {
            strChar = primFile.read(1);
        }
        //curPos = primFile.pos();
        //int countReadedLines = 0;
        QChar whiteSpace(' ');
        while (!primeIn.atEnd() && countArrayLength < length ) {
            QString primeLine = primeIn.readLine();
            QStringList curPrimeNumberList = primeLine.split(whiteSpace, QString::SkipEmptyParts);
            for (int i = 0; i < curPrimeNumberList.size(); i++) {
                curPrimeNumberList[i].trimmed();
                bool converted = false;
                if (countArrayLength < length) {
                    mod[countArrayLength] = curPrimeNumberList.at(i).toInt(&converted);
                }
                else {
                    break;
                }
                countArrayLength++;
            }
        }

    }
    else {
        Q_ASSERT_X(false, "Prime file", "cannot read prime file");
    }
}

/**
 * @brief LinearCongruentialGenerator::getRandom
 * @return
 */
double LinearCongruentialGenerator::getRandom(const bool &forceSumUp) {
    double ret = 0;
    if (!forceSumUp && curResultPosition < state) {
        ret = result[curResultPosition];
        //return (int)ret % (endInterval - startInterval);
    }
    else {
        curResultPosition = 0;
        for (unsigned int i = 0; i < state; i++) {
            //double curResult = result[i];
            //double fac = factor[i];
            //double resultAdd = result[state - i - 1];
            //uint64_t mod = modul[i];
            double res = 0;
            for (int j = state - i -1; j >= 0; j--) {
                res += (uint64_t)(factor[i] * result[j]);
            }
            res += (uint64_t)increment;
            res = (uint64_t)res % modul[i];

            result[i] = res;
        }
        ret = result[curResultPosition];
        //ret = result[curResultPosition++] % (endInterval - startInterval);
        //return ret;
    }
    //scale this to the interval generally to (a,b) with x_i = a + (x_i - min) * (b - a) / max - min
    //ret = startInterval + (ret - startInterval) * (endInterval - startInterval) / (endInterval - startInterval);
    //scale this to the current mod
    ret = ret / (double)modul[curResultPosition++];
    if (ret > 1) {
        unsigned int outCurResultPosition = 0;
        if (curResultPosition > 0) {
            outCurResultPosition = curResultPosition - 1;
        }
        std::cout << "ret: " << ret << ", modul: " << modul[outCurResultPosition] << ", curResultPosition: " << outCurResultPosition << std::endl;
    }
    return ret;
}

uint64_t LinearCongruentialGenerator::smallestValue(uint64_t *&arr, unsigned int &length) {
    QList<uint64_t> tmpList;
    for (unsigned int i = 0; i < length; i++) {
        tmpList.append(arr[i]);
    }
    qSort(tmpList);
    return tmpList.first();
}
