
#include "intersectionapplication.h"
#include <QCoreApplication>
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "start..." << std::endl;
    InterSectionApplication a(argc, argv);
    return a.exec();
}
