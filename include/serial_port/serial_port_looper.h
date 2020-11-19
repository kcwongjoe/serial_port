#pragma once
#ifndef SERIAL_PORT_CONTROLLER_H
#define SERIAL_PORT_CONTROLLER_H

//************Content************

#include <thread>
#include <vector>
#include "serial_port.h"
#include <regex>

namespace SerialPortUtils
{
    /**
     * @brief A looper for serial port. You can set any lambda function to handle the sens or read process.
     * 
     */
    class SerialPortLooper {
    public:

        // Define function type
        typedef std::function<std::pair<unsigned char*, int>()> SendBytePreProcess;
        typedef std::function<std::string()> SendStringPreProcess;
        typedef std::function<void(int n)> SendPostProcess;

        typedef std::function<void(unsigned char* buffer, int bufferSize)> ReadByteProcess;
        typedef std::function<void(std::string buffer)> ReadStringProcess;
        typedef std::function<void(std::vector<std::string> buffer)> ReadStringLineProcess;

        typedef std::function<void(std::unique_ptr<SerialPort> &serialPort)> SerialProcess;

        // Constructor and Destructor
        SerialPortLooper();
        SerialPortLooper(std::unique_ptr<SerialPort> serialPort);

        ~SerialPortLooper();

        // Function in thread loop
        void setStartProcess(SerialProcess func);

        void setSendBytePreProcess(SendBytePreProcess func);
        void setSendStringPreProcess(SendStringPreProcess func);        
        void setSendPostProcess(SendPostProcess func);
        void clearBufferAfterSent(bool clearIt);

        void setReadPreProcess(SerialProcess func);

        void setReadByteProcess(ReadByteProcess func);
        void setReadStringProcess(ReadStringProcess func);
        void setReadStringLineProcess(ReadStringLineProcess func);

        void setStopProcess(SerialProcess func);

        // Thread control
        void start();
        bool stop(bool async = false);

        // Connection
        bool open(int port);
        void close();

        // Transmission
        bool sendASCII(std::string ascii);
        int sendBytes(unsigned char* buffer, int bufferSize);

        std::string readASCII(int bufferSize);
        int readBytes(unsigned char* buffer, int bufferSize);

        // Getter and Setter
        void setEndOfChar(char endOfChar);
        char getEndOfChar();

        void setBufferSize(int bufferSize);
        int getBuffereSize();

        void setDelayTimeAfterSend(int delayTime);
        int getDelayTimeAfterSend();

        void setIterationDelayTime(int delayTime);
        int getIterationDelayTime();

        void setWaitForStopTimeout(int timeout);
        int getWaitForStopTimeout();

        SerialPort* getSerialPort();

        bool isRunning();
        bool isOpened();

    private:
        // Status
        bool m_stopThread = false;
        bool m_stoppedThread = true;
        bool m_isRunning = false;

        // Setting
        std::thread m_thread;
        std::unique_ptr<SerialPort> m_serialPort;

        std::unique_ptr<unsigned char[]> m_buffer = NULL;
        int m_bufferSize = 1024;
        std::string m_stringLineBuffer;

        char m_endOfChar = 0;
        int m_DelayTimeAfterSend = 50;
        int m_iterationDelayTime = 50;
        int m_waitForStopTimeout = 3000;

        // Function in thread loop
        SerialProcess m_startProcess = NULL;

        bool m_clearSendByteBuffer = false;
        SendBytePreProcess m_sendBytePreProcess = NULL;
        SendStringPreProcess m_sendStringPreProcess = NULL;
        SendPostProcess m_sendPostProcess = NULL;

        SerialProcess m_readPreProcess = NULL;

        ReadByteProcess m_readByteProcess = NULL;
        ReadStringProcess m_readStringProcess = NULL;
        ReadStringLineProcess m_readStringLineProcess = NULL;

        SerialProcess m_stopProcess = NULL;

    private:
        void run();
        void reset();

        std::vector<std::string> splitStr(std::string str, std::string delimiter);
    };
}

//*******************************

#endif