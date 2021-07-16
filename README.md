![C++](https://img.shields.io/badge/c++->=11-blue)
![Platform](https://img.shields.io/badge/platform-Window-lightgrey)
[![MIT license](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

# SerialPortUtils

A window based serial port library in c++.

### Features
* Get COM port list in firendly names
* A looper to handle the serial port life cycle

# Table of content

- [Example](#Example)
- [Requirements](#Requirements)
- [Installation](#Installation)
    - [CMake](#CMake)
    - [For Dummy](#For-Dummy)  
- [Looper](#Looper)
    - [Example](#looperexample)
    - [Process types](#Process-types)
    - [Flow chart](#Flow-chart)  
- [Default settings](#Default-settings)
- [Deploy example codes](#Deploy-example-codes)


# Example

```cpp
#include <serial_port.h>

using namespace SerialPortUtils; // Using namespace is a bad practice. Don't use in your code.

// Get all COM port
std::vector<SerialPortInfo> comPorts = SerialPort::getSerialPortList();
std::cout << comPorts[0].friendlyName << std::endl;

// Open Serial Port
SerialPort serialPort;
serialPort.setBaudRate(9600);
serialPort.open(comPorts[0].port);

// Send ASCII
serialPort.sendASCII("Do your job!!");

// Read ASCII
std::string receivedASCII = serialPort.readASCII();

// Close Serial Port
serialPort.close();
```
 
*For more details, please refer to the [Documentation](https://www.kcwongjoe.com/serial_port/index.html) and [Diagram](Diagram.md)*

# Requirements
Minimum C++ 14

# Installation

## Cmake

1. Create **libs** folder in you project
2. Clone this repository into the **libs** folder
   ```
   git clone --recurse-submodules https://github.com/kcwongjoe/serial_port.git
   ```
3. Add the code in your project *CMakeLists.txt*
   ```
   # Serial port library
   add_subdirectory(./libs/serial_port)
   target_link_libraries(${PROJECT_NAME}
       PRIVATE
           serial_port
   )
   ```
## For dummy
1. Copy all files inside **src** folder and **include/serial_port** folder to your project.

# Looper

Looper is a looping thread worked in detach mode which makes serial port life cycle easy to handle.

## <a name="looperexample" id="looperexample"></a>Example

```cpp
#include <serial_port_looper.h>

using namespace SerialPortUtils; // Using namespace is a bad practice. Don't use in your code.

// ****** Create looper ******
SerialPortLooper serialPortLooper;
serialPortLooper.getSerialPort().setBaudRate(9600);
serialPortLooper.setEndOfChar('\n');

// ****** Add processes ******
//  --- Add a start process ---
serialPortLooper.setStartProcess([](std::unique_ptr<SerialPortUtils::SerialPort> &serialport) {
    // Try to connect serial port
    if (!serialport->isOpened())
        serialport->open(1); // Try to open COM1
});

//  --- Add a send process ---
bool sendString = false; // Everytime you set as true, message will be sent. After sent, it will return to false.
serialPortLooper.setSendStringPreProcess([&sendString]() {
    std::string sendAscii;

    // If user set sendString as true, send "Do your job!" to serial port.
    if (sendString) 
    {
        sendAscii = "Do your job!";
        sendString = false;
    }

    return sendAscii;
});

//  --- Add a read string line process --- 
serialPortLooper.setReadStringLineProcess([](std::vector<std::string> buffer) {
    // count the number of line received.
    std::cout << "Number of line read: " + std::to_string(buffer.size()) << std::endl;
});

// ****** Start ******
serialPortLooper.start();

// Send message
sendSring = true;

// ****** Stop ******
serialPortLooper.stop(); // The stop() is work in sync mode. To stop in async, use stop(true)
```

## Flow chart
![Looper](docs/looper.png)

## Process types

```cpp
void SerialProcess(std::unique_ptr<SerialPort> &serialPort);
std::pair<unsigned char*, int> SendBytePreProcess();
std::string SendStringPreProcess();
void SendPostProcess(int n);
void ReadByteProcess(unsigned char* buffer, int bufferSize);
void ReadStringProcess(std::string buffer);
void ReadStringLineProcess(std::vector<std::string> buffer);
```

# Default settings
* BaudRate = 9600
* Byte Size = 8
* Stop Bits = 1
* Parity = No parity
* Flow Control = None
* End Of Char = 0
* Timeout = 50ms

# Deploy example codes

1. Clone this repository
   ```
   git clone --recurse-submodules https://github.com/kcwongjoe/serial_port.git
   ```

2. Run **build.bat** in Solution folder

   Type `build x86` or `build x64`

3. Go to *build* folder and open visual studio solution. Set serial_port_examples project as Startup project.

