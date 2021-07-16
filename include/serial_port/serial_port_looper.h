#pragma once
#ifndef SERIAL_PORT_CONTROLLER_H
#define SERIAL_PORT_CONTROLLER_H

//************Content************

#include <thread>
#include <vector>
#include <serial_port.h>
#include <regex>

namespace SerialPortUtils
{
    /**
     * @brief A looper for serial port. You can set any lambda function to handle the send or read process.
     * 
     * @startuml {looper_state.svg} "Looper State Chart"
     *  !theme sketchy
     *  skinparam linetype polyline
     *  skinparam linetype ortho
     *  (*) -d-> "Main Loop"
     *  note left
     *    Call stop() to set the m_stopThread to true
     *  end note
     *  if m_stopThread then
     *    -l-> [true] "Stop Process"
     *    -d-> (*)
     *  else
     *    -d-> [false] "Start Process"
     *    note right
     *        **setStartProcess()**
     *        //void StartProcess()//
     *        A pre-process to do something before send and read
     *    end note
     *    partition "Send Processes" {
     *        -d-> "Send Byte Process"
     *        partition "Send Byte Processs Detail" {
     *            "Send Byte Process" -[hidden]-> "Send Byte Pre-Process"
     *            note right
     *                **setSendBytePreProcess()**
     *               //std::pair<unsigned char*, int> SendBytePreProcess()//
     *                Prepare your byte to be sent. Return (Bytes, lenght of bytes)
     *            end note
     *            -d-> "Send to serial" #b7285c
     *            -d-> "Send Post-Process"
     *            note right
     *                **setSendPostProcess()**
     *                //void SendPostProcess(int n)//
     *                A post-process after sent.
     *            end note
     *            -d-> "Clear buffer if necessary"
     *            note right
     *                Enable this function by setting clearBUfferAfterSent() as true
     *                if unsigned char* should be released after sent.
     *            end note
     *        }
     *        "Send Byte Process" -d-> "Send String Process"
     *        partition "Send String Processs Detail" {
     *            "Send String Process" -[hidden]-> "Send String Pre-Process"
     *            note right
     *                **setSendStringPreProcess()**
     *                //std::string SendStringPreProcess()//
     *                Prepare your string to be sent.
     *            end note
     *            -d-> "Send String to serial" #b7285c
     *            -d-> "Send String Post-Process"
     *            note right
     *                **setSendPostProcess()**
     *                //void SendPostProcess(int n)//
     *                A post-process after sent.
     *            end note
     *        }
     *    }
     *    "Send String Process" -d-> "Delay After send" #b7285c
     *    note right
     *        **setDelayTimeAfterSend(int)**
     *        A delay after sent
     *    end note
     *    -d-> "Read Pre-Process"
     *    note right
     *        **setReadPreProcess()**
     *        //void ReadPreProcess()//
     *        A pre-process before reading bytes
     *    end note
     *    partition "Read Processes" {
     *    -d-> "Read Bytes" #b7285c
     *    -d-> "Read Byte Process"
     *    note right
     *        **setReadByteProcess()**
     *        //void ReadByteProcess(unsigned char* buffer, int bufferSize)//
     *        User can process the received bytes in here.
     *    end note
     *   -d-> "Read String Process"
     *    note right
     *        **setReadStringProcess()**
     *        //void ReadStringProcess(std::string buffer)//
     *        Received bytes will be converted to string in here.
     *        User can handle the received bytes in the string format here.
     *    end note
     *    -d-> "Read String Line Process"
     *    note right
     *        **setReadStringLineProcess()**
     *       //void ReadStringLineProcess(std::vector<std::string> buffer)//
     *        Received string will be split by delimiter.
     *        User can set the delimieter by **setEndOfChar()**
     *    end note
     *    }
     *    -d-> "Delay between iteration" #b7285c
     *   -u-> "Main Loop"
     *  endif
     * 
     * @enduml
     */
    class SerialPortLooper {
    public:

        // Define function type

        /**
         * @ref setSendBytePreProcess()
         */
        typedef std::function<std::pair<unsigned char*, int>()> SendBytePreProcess;

        /**
         * @ref setSendStringPreProcess()
         */
        typedef std::function<std::string()> SendStringPreProcess;

        /**
         * @ref setSendPostProcess()
         */
        typedef std::function<void(int n)> SendPostProcess;

        /**
         * @ref setReadByteProcess()
         */
        typedef std::function<void(unsigned char* buffer, int bufferSize)> ReadByteProcess;

        /**
         * @ref setReadStringProcess()
         */
        typedef std::function<void(std::string buffer)> ReadStringProcess;

        /**
         * @ref setReadStringLineProcess()
         * 
         */
        typedef std::function<void(std::vector<std::string> buffer)> ReadStringLineProcess;

        /**
         * @ref setStartProcess(), setStopProcess(), setReadPreProcess()
         */
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
        bool open(SerialPortInfo port);
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