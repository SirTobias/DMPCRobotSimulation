//Mark these lines modifed by Chao
Files                                     line                       action                time
---------------------------------------------------------------------------------------------------------------
simulation_intersection_scenario.pro        17th                    add the line            01/06/2016
car.cpp                                     149th          cancel the function of the line  01/06/2016

compiling protobuf on windows:

1. Install cygwin
install autoconf, libtool, gcc, g++

2. Execute ./configure

3. make -j4

4. Compile Qwt (with mathml support), look into qwtinstall

5. Compile intersection scenario


using MinGW and MSYS
mount "C:\MinGW /mingw" in /msys/1.0/etc/fstab

taking videos:

ffmpeg -start_number 0 -f image2 -r 60 -i 'image%01d.png' -c:v libx264 output_lib264.mp4
