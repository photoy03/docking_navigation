@ECHO OFF

REM gcc version 8.1.0 (x86_64-posix-seh-rev0, Built by MinGW-W64 project)

set path=D:\mingw64\bin;%path%

g++ --version

echo "MinGW compile"

g++ -std=c++11 -DCSERIALPORT_DEBUG CSerialPortDemoNoGui.cpp ../../src/SerialPortInfo.cpp ../../src/SerialPortInfoBase.cpp ../../src/SerialPortInfoWinBase.cpp ../../src/SerialPort.cpp ../../src/SerialPortBase.cpp ../../src/SerialPortWinBase.cpp -lpthread -luser32 -ladvapi32 -lsetupapi -I../../include -o CSerialPortDemoNoGui-MinGW

pause