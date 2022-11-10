#include "serial_port/serial_port.h"
#include <iostream>

namespace SerialPortUtils
{
    /**
     * @brief Collect serial ports information by QueryDosDevice
     *
     * @details
     * 1. Use QueryDosDevice() to collect all hardware names.
     * 2. Search serial port on hardware names which start with "COM"
     * 3. Use QueryDosDevice() to get the serial port name
     * 4. Collect all registry value which value names are "FriendlyName".
     * 5. Get the friendly name based on looking up the collected registry values.
     *
     * Modified from http://www.naughter.com/enumser.html
     *
     * @return Return the list of serial port information.
     */
    std::vector<SerialPortInfo> SerialPort::getSerialPortList()
    {
        std::vector<SerialPortInfo> result;

        // Collect all device name
        int deviceBufferSize = 32767;
        bool stopTosearch = false;
        bool success = true;
        std::vector<TCHAR> devices;
        while (!stopTosearch) {
            // Allocate device buffer
            devices.resize(deviceBufferSize);

            // Collect device name
            const DWORD collectDevicesStatus = QueryDosDevice(nullptr, &(devices[0]), deviceBufferSize);
            if (collectDevicesStatus == 0) {
                const DWORD collectDevicesError = GetLastError();
                if (collectDevicesError == ERROR_INSUFFICIENT_BUFFER) {
                    // Expand the buffer if insufficient buffer
                    deviceBufferSize *= 2;
                }
                else {
                    // Encounter error
                    success = false;
                }
            }
            else {
                stopTosearch = true;
            }
        }

        // Look up device name
        size_t deviceStartCharLoc = 0;
        while (devices[deviceStartCharLoc] != _T('\0')) {
            // Get the current device name
            LPTSTR currentDevice = &(devices[deviceStartCharLoc]);
            const size_t currentDeviceNameLen = _tcslen(currentDevice);

            // Search for device name start from "COM"
            if (currentDeviceNameLen > 3 && _tcsnicmp(currentDevice, _T("COM"), 3) == 0) {
                SerialPortInfo currentSerialPortInfo;
                // Get the numerical char after "COM"
                std::string portNumber = "";
                bool isPort = true;
                for (size_t i = 3; i < currentDeviceNameLen && isPort; i++) {
                    isPort = (iswdigit(currentDevice[i]) != 0);
                    if (isPort)
                        portNumber += (char)currentDevice[i];
                }

                // If numerical existed
                if (!portNumber.empty()) {
                    // Get port number
                    currentSerialPortInfo.port = std::stoi(portNumber);

                    // Get Device name
                    currentSerialPortInfo.queryDosDeviceName = new TCHAR[currentDeviceNameLen];
#pragma warning(suppress : 4996)
                    std::_tcscpy(currentSerialPortInfo.queryDosDeviceName, currentDevice);

                    // Get serial port location
                    LPTSTR currentDeviceName = new TCHAR[300];
                    const DWORD charCount = QueryDosDevice(currentDevice, currentDeviceName, 300);

#ifdef _UNICODE
                    std::wstring currentDeviceNameWStr(currentDeviceName);
                    currentSerialPortInfo.deviceName = std::string(currentDeviceNameWStr.begin(), currentDeviceNameWStr.end());
#else
                    currentSerialPortInfo.deviceName = currentDeviceName;
#endif

                    result.push_back(currentSerialPortInfo);
                }
            }

            // Move to next device name
            deviceStartCharLoc += (currentDeviceNameLen + 1);
        }

        // Get friendly name
        if (!result.empty()) {
            // Collect all friendlyNames from registry
            std::vector<std::string> friendNames;
            processRegistryValue(HKEY_LOCAL_MACHINE,
                                 "SYSTEM\\CurrentControlSet\\Enum",
                                 [&friendNames](std::string valueName, DWORD dataType, unsigned char* data, int dataLen) {
                                     if (valueName.compare("FriendlyName") == 0 && dataType == REG_SZ) {
                                         // Found and convert to string, add to result
                                         std::string dataString(reinterpret_cast<char*>(data), dataLen);
                                         friendNames.push_back(dataString);
                                     }
                                 });

            // Update friendly name
            for (int i = 0; i < result.size(); i++) {
                std::string COMName = "COM" + std::to_string(result[i].port);

                // Search on friendly name
                for (std::string friendlyName : friendNames) {
                    if (friendlyName.find(COMName) != std::string::npos) {
                        // Found
                        result[i].friendlyName = friendlyName;
                        break;
                    }
                }
            }
        }

        return result;
    }

#pragma region Constructor and Destructor

    /**
     * @brief Construct a new Serial Port object
     *
     */
    SerialPort::SerialPort()
    {
        // Initialize variable
        m_connected = false;

        m_baudRate = 9600;
        m_byteSize = 8;
        m_stopBits = ONESTOPBIT;
        m_parity = NOPARITY;
        m_flowControl = SERIAL_PORT_FCTL_NONE;
        m_endOfChar = 0;
        m_timeout = 50;
    }

    /**
     * @brief Destroy the Serial Port object
     *
     */
    SerialPort::~SerialPort()
    {
        close();
    }

#pragma endregion Constructor and Destructor

#pragma region Connection

    /**
     * @brief Open Serial Port
     *
     * @param[in] port The Port to be openned.
     * @return Return true if connection success, otherwise return false
     */
    bool SerialPort::open(SerialPortInfo port)
    {
        return open(port.port);
    }

    /**
     * @brief Open Serial Port
     *
     * @param[in] port The Port to be openned. If you want to open COM3, port = 3
     * @return Return true if connection success, otherwise return false
     */
    bool SerialPort::open(int port)
    {
        // Exception
        if (port < 0)
            throw std::invalid_argument("port(" + std::to_string(port) + ") must be >= 0.");

        return open("\\\\.\\COM" + std::to_string(port));
    }

    /**
     * @brief Open Serial Port
     *
     * @param[in] port The Port name to be openned. If you want to open COM3, port = "COM3" or "\\\\.\\COM3"
     * @return Return true if connection success, otherwise return false
     */
    bool SerialPort::open(std::string port)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_connected) {
            throw std::runtime_error("opening an open serial port");
        }
        // Connect
#ifdef _UNICODE
        LPCWSTR portLPCSTR = std::wstring(port.begin(), port.end()).c_str();
#else
        LPCSTR portLPCSTR = port.c_str();
#endif
        m_serialHandle = CreateFile(portLPCSTR,
                                    GENERIC_READ | GENERIC_WRITE,
                                    0, // Share mode = no share
                                    NULL, // Security attribute = no security
                                    OPEN_EXISTING,
                                    0, // FILE_ATTRIBUTE_NORMAL,
                                    NULL // no templates
        );

        if (m_serialHandle != INVALID_HANDLE_VALUE) {
            // Set parameters
            bool setSerialPara = setAllSerialState();
            bool setTimeout = setTimeoutSetting(m_timeout);
            bool setBuffer = SetupComm(m_serialHandle, m_rxtxBufferSize, m_rxtxBufferSize); // Set the input and output buffer
            PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR); // Reset buffer

            if (!setSerialPara || !setTimeout || !setBuffer) {
                // Fail to set parameters
                CloseHandle(m_serialHandle);
                m_connected = false;
            }
            else {
                m_connected = true;
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Close the serial port.
     *
     */
    void SerialPort::close()
    {
        // Lock mutex
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_connected) {
            CloseHandle(m_serialHandle);
            m_connected = false;
        }
    }

#pragma endregion Connection

#pragma region Setting

    /**
     * @brief Set all serial port state
     *
     * @return Return true if success.
     */
    bool SerialPort::setAllSerialState()
    {
        return setSerialStateDecorator(
            [this](DCB& serialParams) {
                serialParams.BaudRate = m_baudRate;
                serialParams.ByteSize = m_byteSize;
                serialParams.StopBits = m_stopBits;
                serialParams.Parity = m_parity;
                if (m_endOfChar != 0)
                    serialParams.EofChar = m_endOfChar;

                // Set flow control
                this->setFlowControlSubFunc(serialParams, m_flowControl);
            },
            false // do not reset buffer
        );
    }

    /**
     * @brief  A subfunction to set flow contorl parameter. It must be used with setSerialStateDecorator().
     *
     * @param[in] thisObj this
     * @param[in, out] serialParams
     */
    void SerialPort::setFlowControlSubFunc(DCB& serialParams, int flowControl)
    {
        serialParams.fOutX = flowControl == SERIAL_PORT_FCTL_XON_XOFF;
        serialParams.fInX = flowControl == SERIAL_PORT_FCTL_XON_XOFF;
        serialParams.fOutxCtsFlow = flowControl == SERIAL_PORT_FCTL_HARDWARE;
        serialParams.fOutxDsrFlow = flowControl == SERIAL_PORT_FCTL_HARDWARE;
        serialParams.fDsrSensitivity = flowControl == SERIAL_PORT_FCTL_HARDWARE;
        serialParams.fRtsControl = flowControl == SERIAL_PORT_FCTL_HARDWARE ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_DISABLE;
        serialParams.fDtrControl = flowControl == SERIAL_PORT_FCTL_HARDWARE ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_DISABLE;
    }

    /**
     * @brief Set timeout setting.
     *
     * @return Return true if success
     */
    bool SerialPort::setTimeoutSetting(int timeout)
    {
        // Get timeouts
        COMMTIMEOUTS commTimeout = {0};
        if (!GetCommTimeouts(m_serialHandle, &commTimeout)) {
            return false;
        }
        commTimeout.ReadIntervalTimeout = MAXDWORD;
        commTimeout.ReadTotalTimeoutConstant = timeout;
        commTimeout.ReadTotalTimeoutMultiplier = 0;
        commTimeout.WriteTotalTimeoutConstant = timeout;
        commTimeout.WriteTotalTimeoutMultiplier = 0;

        return SetCommTimeouts(m_serialHandle, &commTimeout);
    }

    /**
     * @brief Reset rxtx buffer
     *
     */
    void SerialPort::resetBuffer()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR);
        }
    }

#pragma endregion Setting

#pragma region Getter and Setter

    /**
     * @brief Set baudrate
     *
     * @param[in] baudRate Baudrate: 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000
     * @return Return true if success
     */
    bool SerialPort::setBaudRate(int baudRate)
    {
        if (baudRate != CBR_110 && baudRate != CBR_300 && baudRate != CBR_600 && baudRate != CBR_1200 && baudRate != CBR_2400
            && baudRate != CBR_4800 && baudRate != CBR_9600 && baudRate != CBR_14400 && baudRate != CBR_19200
            && baudRate != CBR_38400 && baudRate != CBR_57600 && baudRate != CBR_115200 && baudRate != CBR_128000
            && baudRate != CBR_256000) {
            throw std::invalid_argument("baudRate(" + std::to_string(baudRate) + ") is invalid.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setSerialStateDecorator([baudRate](DCB& serialParams) { serialParams.BaudRate = baudRate; })) {
                m_baudRate = baudRate;
                return true;
            }
            return false;
        }
        else {
            m_baudRate = baudRate;
            return true;
        }
    }

    /**
     * @brief Get baud rate
     *
     * @return Return the Baud rate
     */
    int SerialPort::getBaudRate()
    {
        return m_baudRate;
    }

    /**
     * @brief Set byte size
     *
     * @param byteSize The number of bits in the bytes transmitted and received. (4-8)
     * @return Return true if success.
     */
    bool SerialPort::setByteSize(int byteSize)
    {
        if (byteSize < 4 || byteSize > 8) {
            throw std::invalid_argument("byteSize(" + std::to_string(byteSize) + ") must be 4 - 8.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setSerialStateDecorator([byteSize](DCB& serialParams) { serialParams.ByteSize = byteSize; })) {
                m_byteSize = byteSize;
                return true;
            }
            return false;
        }
        else {
            // Serial not connect
            m_byteSize = byteSize;
            return true;
        }
    }

    /**
     * @brief Get byte size
     *
     * @return int
     */
    int SerialPort::getByteSize()
    {
        return m_byteSize;
    }

    /**
     * @brief Set stop bits
     *
     * @param[in] stopBits It must be ONESTOPBIT, ONE5STOPBITS or TWOSTOPBITS
     * @return Return true if success
     */
    bool SerialPort::setStopBits(int stopBits)
    {
        if (stopBits != ONESTOPBIT && stopBits != ONE5STOPBITS && stopBits != TWOSTOPBITS) {
            throw std::invalid_argument("stopBits(" + std::to_string(stopBits) + ") must be 1, 1.5 or 2.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setSerialStateDecorator([stopBits](DCB& serialParams) { serialParams.StopBits = stopBits; })) {
                m_stopBits = stopBits;
                return true;
            }
            return false;
        }
        else {
            // Serial not connect
            m_stopBits = stopBits;
            return true;
        }
    }

    /**
     * @brief Get stop bits
     *
     * @return Return stopbits as ONESTOPBIT, ONE5STOPBITS or TWOSTOPBITS
     */
    int SerialPort::getStopBits()
    {
        return m_stopBits;
    }

    /**
     * @brief Set parity
     *
     * @param[in] parity It must be NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY or SPACEPARITY
     * @return Return true if success
     */
    bool SerialPort::setParity(int parity)
    {
        // Check
        if (parity != NOPARITY && parity != ODDPARITY && parity != EVENPARITY && parity != MARKPARITY && parity != SPACEPARITY) {
            throw std::invalid_argument("stopBits(" + std::to_string(parity)
                                        + ") must be NOPARITY, EVENPARITY, MARKPARITY or SPACEPARITY.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setSerialStateDecorator([parity](DCB& serialParams) { serialParams.Parity = parity; })) {
                m_parity = parity;
                return true;
            }
            return false;
        }
        else {
            // Serial not connect
            m_parity = parity;
            return true;
        }
    }

    /**
     * @brief Get Parity
     *
     * @return Return parity as NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY or SPACEPARITY
     */
    int SerialPort::getParity()
    {
        return m_parity;
    }

    /**
     * @brief Set flow contorl
     *
     * @param flowControl SERIAL_PORT_FCTL_NONE, SERIAL_PORT_FCTL_XON_XOFF or SERIAL_PORT_FCTL_HARDWARE
     * @return Return true if success
     */
    bool SerialPort::setFlowControl(int flowControl)
    {
        if (flowControl != SERIAL_PORT_FCTL_NONE && flowControl != SERIAL_PORT_FCTL_XON_XOFF
            && flowControl != SERIAL_PORT_FCTL_HARDWARE) {
            throw std::invalid_argument(
                "flowControl(" + std::to_string(flowControl)
                + ") must be SERIAL_PORT_FCTL_NONE, SERIAL_PORT_FCTL_XON_XOFF or SERIAL_PORT_FCTL_HARDWARE.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_connected) {
            if (setSerialStateDecorator(
                    [this, flowControl](DCB& serialParams) { setFlowControlSubFunc(serialParams, flowControl); })) {
                m_flowControl = flowControl;
                return true;
            }
            return false;
        }
        else {
            // Serial not connect
            m_flowControl = flowControl;
            return true;
        }
    }

    /**
     * @brief Get flow control.
     *
     * @return SERIAL_PORT_FCTL_NONE, SERIAL_PORT_FCTL_XON_XOFF or SERIAL_PORT_FCTL_HARDWARE
     */
    int SerialPort::getFlowControl()
    {
        return m_flowControl;
    }

    /**
     * @brief Set end of char
     *
     * @param endOfChar
     * @return Return true if success
     */
    bool SerialPort::setEndOfChar(char endOfChar)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setSerialStateDecorator([endOfChar](DCB& serialParams) { serialParams.EofChar = endOfChar; })) {
                m_endOfChar = endOfChar;
                return true;
            }
            return false;
        }
        else {
            m_endOfChar = endOfChar;
            return true;
        }
    }

    /**
     * @brief Get end of char
     *
     * @return char
     */
    char SerialPort::getEndOfChar()
    {
        return m_endOfChar;
    }

    /**
     * @brief Set timeout in millisecond
     *
     * @param timeout timeout in millisecond
     * @return Return true if success
     */
    bool SerialPort::setTimeout(int timeout)
    {
        if (timeout < 0 || timeout > MAXDWORD) {
            throw std::invalid_argument("timeout(" + std::to_string(timeout) + ") must be >=0 and < MAXDWORD.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (setTimeoutSetting(timeout)) {
                m_timeout = timeout;
                return true;
            }
            return false;
        }
        else {
            m_timeout = timeout;
            return true;
        }
    }

    /**
     * @brief Get timeout in millisecond
     *
     * @return int
     */
    int SerialPort::getTimeout()
    {
        return m_timeout;
    }

    /**
     * @brief Set up the Rxtx hardware buffer size. Default is 1024. If the protocol packets is larger, please make it bigger.
     *
     * @param bufferSize Buffer size in bytes.
     * @return Return true if success.
     */
    bool SerialPort::setRxTxBufferSize(int bufferSize)
    {
        // Exception
        if (bufferSize <= 0 || bufferSize > MAXDWORD) {
            throw std::invalid_argument("bufferSize(" + std::to_string(bufferSize) + ") must be >0 and < MAXDWORD.");
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_connected) {
            if (SetupComm(m_serialHandle, bufferSize, bufferSize)) {
                m_rxtxBufferSize = bufferSize;
                PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR); // Reset buffer
                return true;
            }
            return false;
        }
        else {
            m_rxtxBufferSize = bufferSize;
            return true;
        }
    }

    /**
     * @brief Get the Rxtx hardware buffer size.
     *
     * @return
     */
    int SerialPort::getRxTxBufferSize()
    {
        return m_rxtxBufferSize;
    }

    /**
     * @brief Return true if serial port is openned.
     *
     * @return Return true if serial port is openned.
     */
    bool SerialPort::isOpened()
    {
        return m_connected;
    }

#pragma endregion Getter and Setter

#pragma region Transmission

    /**
     * @brief Send ASCII to serial port
     *
     * @param[in] ascii
     * @return Return true if success.
     */
    bool SerialPort::sendASCII(std::string ascii)
    {
        int n = sendBytes(reinterpret_cast<std::byte const*>(ascii.data()), ascii.size());
        return n == ascii.size();
    }

    /**
     * @brief Send Bytes to serial port
     *
     * @param[in] buffer Bytes to be sent
     * @param[in] bufferSize Number of bytes to be sent
     * @return Return the number of bytes sent. If serial port was not connected, return -1.
     */
    std::size_t SerialPort::sendBytes(std::byte const* buffer, std::size_t bufferSize)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_connected) {
            throw std::runtime_error("write to closed serial port");
        }
        int n;
        if (WriteFile(m_serialHandle, buffer, bufferSize, (LPDWORD)((void*)&n), NULL)) {
            return n;
        }
        throw std::runtime_error("write to serial port failed");
    }

    /**
     * @brief Read ASCII from serial port. Buffer size = 1024 byte
     *
     * @return Return the ASCII string. If connection fail or no bytes collected, return a empty string.
     */
    std::string SerialPort::readASCII()
    {
        return readASCII(1024);
    }

    /**
     * @brief Read ASCII from serial port
     *
     * @param[in] strBufferSize Buffer size to be read in byte
     * @return Return the ASCII string. If connection fail or no bytes collected, return a empty string.
     */
    std::string SerialPort::readASCII(std::size_t strBufferSize)
    {
        std::string result;
        result.resize(strBufferSize + 1);
        auto n = readBytes(reinterpret_cast<std::byte*>(result.data()), strBufferSize);
        result.resize(n);
        return result;
    }

    /**
     * @brief Read Bytes from serial port
     *
     * @param[out] buffer Bytes buffer
     * @param[in] bufferSize Buffer size
     * @return Return number of bytes collected. If serial port was not connected, return -1.
     */
    std::size_t SerialPort::readBytes(std::byte* buffer, std::size_t bufferSize)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_connected) {
            throw std::runtime_error("write to closed serial port");
        }
        int n;
        if (ReadFile(m_serialHandle, buffer, bufferSize, (LPDWORD)((void*)&n), NULL)) {
            return n;
        }
        throw std::runtime_error("failed to read from serial port");
    }

#pragma endregion Transmission

} // namespace SerialPortUtils
