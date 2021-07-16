#include "eg2_read.h"

#include <serial_port_looper.h>
#include <iostream>
#include <chrono>
#include <thread>

using namespace SerialPortUtils;

void eg2_read()
{
    // Get all COM port
    std::vector<SerialPortInfo> comPorts = SerialPort::getSerialPortList();

    // Print result
    if (comPorts.size() > 0)
    {
        std::string comName = comPorts[0].friendlyName;

        std::cout << std::to_string(comPorts.size()) + " serial port found. \"" + comName + "\" will be openned." << std::endl;
        std::cout << "Initialing looper..." << std::endl;

        // Create looper with a smart pointer. 
        // It is suggested to use smart pointer which can help to prevent memory leaks.
        std::unique_ptr<SerialPortLooper> serialPortLooper(new SerialPortLooper());

        // Use getSerialPort() to set the serial port settings
        serialPortLooper->getSerialPort()->setBaudRate(9600); 

        // Set the delimiter of the data stream
        serialPortLooper->setEndOfChar('\n');

        // Add read processes
        serialPortLooper->setReadStringLineProcess([](std::vector<std::string> buffer) {
            for (int i=0;i< buffer.size();i++){
                std::cout << buffer[i] << std::endl;
            }            
        });

        // Add stop process
        serialPortLooper->setStopProcess([comName](std::unique_ptr<SerialPort> &serialPort) {
            serialPort->close();
            std::cout << "\"" + comName + "\" was closed." << std::endl;
        });

        // Open the first serial port
        std::cout << "Openning port..." << std::endl;
        bool success = serialPortLooper->open(comPorts[0]);

        if (success)
        {
            std::cout << "Success to open the \"" + comName + "\". Start looper." << std::endl;

            // Start
            serialPortLooper->start();

            // Main thread will be stop after 10 second.
            std::this_thread::sleep_for(std::chrono::seconds(10));

            // Stop
            std::cout << "Start to disconnect the \"" + comName + "\" after 10 seconds." << std::endl;
            serialPortLooper->stop();
        }
        else
        {
            std::cout << "Fail to open the \"" + comName + "\"." << std::endl;
        }

    } 
    else
    {
        std::cout << "There is no any serial port." << std::endl;
    }

}