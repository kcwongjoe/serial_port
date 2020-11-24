#include <serial_port.h>
#include <iostream>

using namespace SerialPortUtils;

int main(int argc, char *argv[])
{
    // Get all COM port
    std::vector<SerialPortInfo> comPorts = SerialPort::getSerialPortList();

    // Print result
    if (comPorts.size() > 0)
    {
        std::cout << "There is " + std::to_string(comPorts.size()) + " port found:" << std::endl;
        for (int i=0;i<comPorts.size();i++) {
            std::cout << comPorts[i].friendlyName << std::endl;
        }        
    } 
    else
    {
        std::cout << "There is no any serial port." << std::endl;
    }

}