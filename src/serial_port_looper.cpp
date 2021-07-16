#include <serial_port_looper.h>

namespace SerialPortUtils
{

#pragma region Constructor and Destructor

    /**
     * @brief Construct a new Serial Port Controller:: Serial Port Controller object
     * 
     */
    SerialPortLooper::SerialPortLooper()
    {
        m_serialPort = std::unique_ptr<SerialPort>(new SerialPort());

        reset();
    }

    /**
     * @brief Construct a new Serial Port Thread:: Serial Port Thread object.
     * The smart point will be moved into this handler by using std::move() function.
     * You can use getSerialPort() to use the serialport.
     * 
     * @code{.cpp}
     * using namespace SerialPortUtils;
     * 
     * serialPort = std::unique_ptr<SerialPort>(new SerialPort());
     * serialPort->setBaudRate(115200);
     * auto SerialPortLooper = std::unique_ptr<SerialPortLooper>(new SerialPortLooper(std::move(serialPort)));
     * @endcode
     * 
     * @param[in] serialPort 
     */
    SerialPortLooper::SerialPortLooper(std::unique_ptr<SerialPort> serialPort)
    {
        m_serialPort = std::move(serialPort);

        reset();
    }

    /**
     * @brief Destroy the Serial Port Thread:: Serial Port Thread object
     * 
     */
    SerialPortLooper::~SerialPortLooper()
    {
    }

    /**
     * @brief Reset the status and buffer
     */
    void SerialPortLooper::reset()
    {
        // initialize buffer
        setBufferSize(m_bufferSize);

        // Reset serial rxtx buffer
        m_serialPort->resetBuffer();
    }

#pragma endregion Constructor and Destructor

#pragma region Thread

    /**
     * @brief Start the thread
     */
    void SerialPortLooper::start()
    {
        reset();
        m_thread = std::thread(&SerialPortLooper::run, this);
        m_thread.detach();
    }

    /**
     * @brief Stop the thread. This stop function default operating in sync mode which will wait for the thread stopped.
     * 
     * @param async Default as false. Set it as true to stop in async mode.
     * @return Return true if success to close. Return false if timeout.
     */
    bool SerialPortLooper::stop(bool async)
    {
        m_stopThread = true;

        if (async == false)
        {
            // Sync mode

            // Wait
            int waitingTime = 0;
            int delayTime = 100;
            while (!m_stoppedThread && (waitingTime <= m_waitForStopTimeout || m_waitForStopTimeout == 0))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(delayTime));
                waitingTime += delayTime;
            }

            // Return
            if (m_waitForStopTimeout > 0 && waitingTime > m_waitForStopTimeout)
            {
                // Time out
                return false;
            }
            else
            {
                // Success
                return true;
            }
        }
        else
        {
            // Async mode
            return true;
        }

    }

    /**
     * @brief The main loop
     * 
     */
    void SerialPortLooper::run()
    {
        // Thread start to run.
        m_isRunning = true;

        try
        {
            m_stopThread = false;
            m_stoppedThread = false;
            while (!m_stopThread)
            {
                // Start Process
                if (m_startProcess != nullptr)
                    m_startProcess(m_serialPort);

                // serial port is opened
                if (m_serialPort->isOpened())
                {
                    // *** Send data ***

                    // Send Byte Process
                    if (m_sendBytePreProcess != nullptr)
                    {
                        // Send Byte Pre-Process
                        std::pair<unsigned char*, int> sendBuffer = m_sendBytePreProcess();

                        int n = 0;
                        if (sendBuffer.second != 0) 
                            {
                            // Send Bytes
                            n = m_serialPort->sendBytes(sendBuffer.first, sendBuffer.second);
                        }
                        // Send Byte Post-Process
                            if (m_sendPostProcess != nullptr)
                            {
                                m_sendPostProcess(n);
                            }

                        // Clear byte buffer
                        if (m_clearSendByteBuffer)
                            delete sendBuffer.first;
                    }

                    // Send String Process
                    if (m_sendStringPreProcess != nullptr)
                    {
                        // Send String Pre-Process
                        std::string sendBuffer = m_sendStringPreProcess();

                        bool sent = false;
                        if (!sendBuffer.empty())
                        {
                            // Send String
                            sent = m_serialPort->sendASCII(sendBuffer);
                        }

                        // Send String Post-Process
                        if (m_sendPostProcess != nullptr)
                        {
                            int n = 0;
                            if (sent) n = (int)sendBuffer.length();

                            m_sendPostProcess(n);
                        }
                    }

                    // Delay after send
                    if (m_DelayTimeAfterSend > 0 &&
                        (m_sendBytePreProcess != nullptr || m_sendStringPreProcess != nullptr) &&   // Send process exist
                        (m_readByteProcess != nullptr || m_readStringProcess != nullptr || m_readStringLineProcess != nullptr))    // Rend process exist
                        std::this_thread::sleep_for(std::chrono::milliseconds(m_DelayTimeAfterSend));

                    // Read Pre-process
                    if (m_readPreProcess != nullptr)
                        m_readPreProcess(m_serialPort);

                    // *** Get data ***
                    int n = m_serialPort->readBytes(m_buffer.get(), m_bufferSize);

                    // *** Process for received data ***
                    if (n > 0) {
                        // Read Byte Process
                        if (m_readByteProcess != nullptr)
                        {
                            m_readByteProcess(m_buffer.get(), n);
                        }

                        // Read String Process
                        if (m_readStringProcess != nullptr)
                        {
                            std::string bufferStr(reinterpret_cast<char*>(m_buffer.get()), n);
                            m_readStringProcess(bufferStr);
                        }

                        // Read String Line Process
                        if (m_readStringLineProcess != nullptr)
                        {
                            std::string bufferStr(reinterpret_cast<char*>(m_buffer.get()), n);

                            // Split
                            std::string delimiterStr(1, m_endOfChar);
                            std::vector<std::string> result = splitStr(bufferStr, delimiterStr);

                            // Append previous string
                            if (!m_stringLineBuffer.empty())
                            {
                                result[0] = m_stringLineBuffer + result[0];
                                m_stringLineBuffer.clear();
                            }

                            // Take out last element if delimiter is not at the end
                            if (m_buffer.get()[n - 1] != m_endOfChar)
                            {
                                m_stringLineBuffer = result.back();
                                result.pop_back();
                            }

                            m_readStringLineProcess(result);
                        }
                    }
                }

                // iteration delay time
                if (m_iterationDelayTime > 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(m_iterationDelayTime));
            }

            // Stop process
            if (m_stopProcess != nullptr)
                m_stopProcess(m_serialPort);

            // Flag to record the thread was stopped.
            m_stoppedThread = true;
        }
        catch (...)
        {
            // Update status
            m_isRunning = false;
            m_stoppedThread = true;

            // Rethrow error
            throw;
        }

        // Thread is no more running.
        m_isRunning = false;
    }


#pragma endregion Thread

#pragma region Connection

    /**
     * @brief Open Serial Port
     * 
     * @param[in] port The Port to be openned.
     * @return Return true if connection success, otherwise return false
     */
    bool SerialPortLooper::open(SerialPortInfo port)
    {
        return m_serialPort->open(port);
    }

    /**
     * @brief Open Serial Port
     *
     * @param[in] port The Port to be openned. If you want to open COM3, port = 3
     * @return Return true if connection success, otherwise return false
    */
    bool SerialPortLooper::open(int port)
    {
        return m_serialPort->open(port);
    }

    /**
     * @brief Close the serial port.
     *
     */
    void SerialPortLooper::close()
    {
        m_serialPort->close();
    }

#pragma endregion Connection

#pragma region Tranmission

    /**
     * @brief Send ASCII to serial port
     *
     * @param[in] ascii
     * @return Return true if success.
     */
    bool SerialPortLooper::sendASCII(std::string ascii)
    {
        return m_serialPort->sendASCII(ascii);
    }

    /**
     * @brief Send Bytes to serial port
     *
     * @param[in] buffer Bytes to be sent
     * @param[in] bufferSize Number of bytes to be sent
     * @return Return the number of bytes sent. If serial port was not connected, return -1.
     */
    int SerialPortLooper::sendBytes(unsigned char* buffer, int bufferSize)
    {
        return m_serialPort->sendBytes(buffer, bufferSize);
    }

    /**
     * @brief Read ASCII from serial port
     *
     * @param[in] bufferSize Buffer size to be read
     * @return Return the ASCII string. If connection fail or no bytes collected, return a empty string.
     */
    std::string SerialPortLooper::readASCII(int bufferSize)
    {
        return m_serialPort->readASCII(bufferSize);
    }

    /**
     * @brief Read Bytes from serial port
     *
     * @param[out] buffer Bytes buffer
     * @param[in] bufferSize Buffer size
     * @return Return number of bytes collected. If serial port was not connected, return -1.
     */
    int SerialPortLooper::readBytes(unsigned char* buffer, int bufferSize)
    {
        return m_serialPort->readBytes(buffer, bufferSize);
    }

#pragma endregion Tranmission

#pragma region Function in Thread Loop


    /**
     * @brief Set the start Process
     *
     * @details
     * The StartProcess will be process at the beginning of each iteration in the thread loop.
     *
     * @code{.cpp}
     * void func(SerialPort* serialPort)
     * {
     *     if (!serialPort->isOpened())
     *         serialPort->open(1);
     * }
     *
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setStartProcess(func);
     * }
     * @endcode
     *
     * @param func void func(SerialPort* serialPort)
     */
    void SerialPortLooper::setStartProcess(SerialProcess func)
    {
        m_startProcess = func;
    }

    void SerialPortLooper::clearBufferAfterSent(bool clearIt)
    {
        m_clearSendByteBuffer = clearIt;
    }

    /**
     * @brief Set Send Byte Pre-Process function.
     * 
     * @details
     * The SendBytePreProcess will be process before send bytes.
     * 
     * The SendBytePreProcess must return a std::pair<unsigned char*, int>
     * where "unsigned char*" is the bytes to be sent and "int" is the length of the bytes.
     * If the "unsigned char*" is not used after sent, set clearBufferAfterSent(true);
     * @code{.cpp}
     * std::pair<unsigned char*, int> func()
     * {
     *    unsigned char* sendBytes = "SendIt";
     *    return std::make_pair(sendBytes ,6);
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setSendBytePreProcess(func);
     *    serial.clearBufferAfterSent(true); // Clear memory after sent
     * }
     * @endcode
     * 
     * @param func std::pair<unsigned char*, int> func()
     * @see SerialPortLooper::clearBufferAfterSent(bool)
     */
    void SerialPortLooper::setSendBytePreProcess(SendBytePreProcess func)
    {
        m_sendBytePreProcess = func;
    }

    /**
     * @brief Set Send String Pre-Process function.
     * 
     * @details
     * The SendStringPreProcess will be process before send ASCII string.
     * 
     * The SendStringPreProcess must return a std::string which is the string to be sent.
     * 
     * @code{.cpp}
     * std::string func()
     * {
     *    std::string sendBytes = "SendIt";
     *    return sendBytes;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setSendStringPreProcess(func);
     * }
     * @endcode
     * 
     * @param func std::string func()
     */
    void SerialPortLooper::setSendStringPreProcess(SendStringPreProcess func)
    {
        m_sendStringPreProcess = func;
    }

    /**
     * @brief Set the Send Post-Process fucntion
     * 
     * @details
     * The SendPostProcess will be process after send bytes or ASCII string. 
     *  
     * The "int" input variable is the number of bytes to be sent.
     * 
     * @code{.cpp}
     * void func(int n)
     * {
     *    if (n > 0)
     *       std::cout << "Yeah" << std::endl;
     *    else
     *       std::cout << "So sad." << std::endl;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setSendPostProcess(func);
     * }
     * @endcode
     * 
     * @param func void func(int numOfByteSent)
     */
    void SerialPortLooper::setSendPostProcess(SendPostProcess func)
    {
        m_sendPostProcess = func;
    }

    /**
     * @brief Set the Read Pre-Process
     *
     * @details
     * The ReadPreProcess will be process before reading and after sending in the thread loop.
     *
     * @code{.cpp}
     * void func(SerialPort* serialPort)
     * {
     *     std::cout << "Bytes sent. Waiting for read..."
     * }
     *
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setReadPreProcess(func);
     * }
     * @endcode
     *
     * @param func void func(SerialPort* serialPort)
     */
    void SerialPortLooper::setReadPreProcess(SerialProcess func)
    {
        m_readPreProcess = func;
    }

    /**
     * @brief Set the Read Byte Process function
     * 
     * @details
     * The ReadByteProcess will be process after receive bytes.
     * 
     * Do not clear the "char*". The "char*" will be reuse on next iteration.
     * 
     * @code{.cpp}
     * void func(unsigned char* buffer, int bufferSize)
     * {
     *    if (bufferSize > 0)
     *       std::cout << "Yeah" << std::endl;
     *    else
     *       std::cout << "So sad." << std::endl;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setReadByteProcess(func);
     * }
     * @endcode
     * 
     * @param func void func(unsigned char* buffer, int bufferSize)
     */
    void SerialPortLooper::setReadByteProcess(ReadByteProcess func)
    {
        m_readByteProcess = func;
    }

    /**
     * @brief Set the Read String Process function
     * 
     * @details
     * The ReadStringProcess will be process after receive ASCII string.
     * 
     * @code{.cpp}
     * void func(std::string buffer)
     * {
     *     std::cout << buffer << std::endl;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setReadStringProcess(func);
     * }
     * @endcode
     * 
     * @param func void func(std::string buffer)
     */
    void SerialPortLooper::setReadStringProcess(ReadStringProcess func)
    {
        m_readStringProcess = func;
    }

    /**
     * @brief Set the Read String Line Process function
     * 
     * @details
     * The ReadStringLineProcess will be process after receive ASCII string.
     * 
     * It will split the received ASCII string by the endOfChar.
     * You can set endOfChar by setEndOfChar(endOfChar)
     * 
     * The end of the string line will save in buffer and add at the beginning on the next cycle.
     * 
     * e.g.
     * iter1: aaa;bbb;c       -> "aaa","bbb"
     * iter2: c + cc;ddd;eee; -> "ccc", "ddd", "eee" 
     * iter3: fff;ggg         -> "fff"
     * 
     * @code{.cpp}
     * void func(std::vector<std::string> buffer)
     * {
     *     for each(auto ascii in buffer)
     *         std::cout << ascii << std::endl;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setEndOfChar('\n'); // Set delimiter
     *    serial.setReadStringLineProcess(func);
     * }
     * @endcode
     * 
     * @param func void func(std::vector<std::string> buffer)
     * 
     * @see SerialPortLooper::setEndOfChar(char)
     */
    void SerialPortLooper::setReadStringLineProcess(ReadStringLineProcess func)
    {
        m_readStringLineProcess = func;
    }

    /**
     * @brief Set the Stop Process
     * 
     * @details
     * The StopProcess will be process after the thread loop finished.
     * You can use this process to close the serial conenction.
     * 
     * @code{.cpp}
     * void func(SerialPort* serialPort)
     * {
     *     serialPort->close();
     *     delete serialPort;
     * }
     * 
     * int main()
     * {
     *    ......
     *    ......
     *    serial.setStopProcess(func);
     * }
     * @endcode
     * 
     * @param func void func(SerialPort* serialPort)
     */
    void SerialPortLooper::setStopProcess(SerialProcess func)
    {
        m_stopProcess = func;
    }

#pragma endregion Function in Thread Loop

#pragma region Getter and Setter

    /**
     * @brief Set the delimiter of the ASCII string. It is used for ReadStringLineProcess.
     * 
     * @param[in] endOfChar delimiter
     * 
     * @see SerialPortLooper::setReadStringLineProcess(ReadStringLineProcess)
     */
    void SerialPortLooper::setEndOfChar(char endOfChar)
    {
        m_endOfChar = endOfChar;
        m_serialPort->setEndOfChar(endOfChar);
    }

    /**
     * @brief Get the delimiter
     * 
     * @return Return the delimiter 
     */
    char SerialPortLooper::getEndOfChar()
    {
        return m_endOfChar;
    }

    /**
     * @brief Set the buffer size for the sending and receiving.
     * 
     * @param[in] bufferSize Buffer size
     */
    void SerialPortLooper::setBufferSize(int bufferSize)
    {
        // Exception
        if (bufferSize <= 0)
            throw std::invalid_argument("bufferSize(" + std::to_string(bufferSize) + ") must be > 0.");

        // initialize buffer
        m_buffer.reset(new unsigned char[bufferSize]());
        m_bufferSize = bufferSize;
    }

    /**
     * @brief Get the buffer size. Default is 1024 bytes.
     * 
     * @return int 
     */
    int SerialPortLooper::getBuffereSize()
    {
        return m_bufferSize;
    }

    /**
     * @brief Set the delay time between send and read bytes. This delay will not use if send and read process are not existed.
     * 
     * @param delayTime Delay time in millisecond.
     */
    void SerialPortLooper::setDelayTimeAfterSend(int delayTime)
    {
        // Exception
        if (delayTime < 0)
            throw std::invalid_argument("delayTime(" + std::to_string(delayTime) + ") must be >= 0.");

        m_DelayTimeAfterSend = delayTime;
    }

    /**
     * @brief Get delay time between send and read bytes. Default is 50ms.
     * 
     * @return Delay time in millisecond.
     */
    int SerialPortLooper::getDelayTimeAfterSend()
    {
        return m_DelayTimeAfterSend;
    }

    /**
     * @brief Set the delay time between each iteration.
     * 
     * @param delayTime Delay time in millisecond.
     */
    void SerialPortLooper::setIterationDelayTime(int delayTime)
    {
        // Exception
        if (delayTime < 0)
            throw std::invalid_argument("delayTime(" + std::to_string(delayTime) + ") must be >= 0.");

        m_iterationDelayTime = delayTime;
    }

    /**
     * @brief Get the delay time between each iteration. Default is 50ms.
     * 
     * @return Delay time in millisecond.
     */
    int SerialPortLooper::getIterationDelayTime()
    {
        return m_iterationDelayTime;
    }

    /**
     * @brief Set the wait for stopping timeout 
     * 
     * @param timeout in millisecond. Set as 0 to disable the timeout. Default as 3 seconds.
     */
    void SerialPortLooper::setWaitForStopTimeout(int timeout)
    {
        // Exception
        if (timeout < 0)
            throw std::invalid_argument("timeout(" + std::to_string(timeout) + ") must be >= 0.");

        m_waitForStopTimeout = timeout;
    }

    /**
     * @brief Get the wait for stopping timeout 
     * 
     * @return Timeout in millisecond.
     */
    int SerialPortLooper::getWaitForStopTimeout()
    {
        return m_waitForStopTimeout;
    }

    /**
     * @brief Return true if thread is running.
     * 
     * @return Return true if thread is running.
     */
    bool SerialPortLooper::isRunning()
    {
        return m_isRunning;
    }

    /**
     * @brief Return true if serial port is openned. 
     * 
     * @return Return true if serial port is openned.
    */
    bool SerialPortLooper::isOpened()
    {
        return m_serialPort->isOpened();
    }

    /**
     * @brief Get the serial port.
     * 
     * @return Return the serial port pointer.
     */
    SerialPort* SerialPortLooper::getSerialPort()
    {
        return m_serialPort.get();
    }

#pragma endregion Getter and Setter

#pragma region Utils

    /**
     * @brief Split string by delimiter
     * 
     * @param str String to be splitted
     * @param delimiter Delimiter
     * @return std::vector<std::string> Splitted strings
     */
    std::vector<std::string> SerialPortLooper::splitStr(std::string str, std::string delimiter)
    {
        std::regex re("[" + std::string(delimiter) + "]");

        std::sregex_token_iterator it{ str.begin(), str.end(), re, -1 };
        std::vector<std::string> tokenized{ it, {} };

        // Additional check to remove empty strings 
        tokenized.erase(
            std::remove_if(tokenized.begin(), tokenized.end(),
                [](std::string const& s) {
                    return s.size() == 0;
                }
            ),
            tokenized.end()
                    );

        return tokenized;
    }

#pragma endregion 

}