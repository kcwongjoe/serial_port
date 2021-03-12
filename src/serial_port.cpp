#include <serial_port.h>
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
		while (!stopTosearch)
		{
			// Allocate device buffer
			devices.resize(deviceBufferSize);

			// Collect device name
			const DWORD collectDevicesStatus = QueryDosDevice(nullptr, &(devices[0]), deviceBufferSize);
			if (collectDevicesStatus == 0)
			{
				const DWORD collectDevicesError = GetLastError();
				if (collectDevicesError == ERROR_INSUFFICIENT_BUFFER)
				{
					//Expand the buffer if insufficient buffer
					deviceBufferSize *= 2;
				}
				else
				{
					// Encounter error
					success = false;
				}
			}
			else
			{
				stopTosearch = true;
			}
		}

		// Look up device name
		size_t deviceStartCharLoc = 0;
		while (devices[deviceStartCharLoc] != _T('\0'))
		{
			//Get the current device name
			LPTSTR currentDevice = &(devices[deviceStartCharLoc]);
			const size_t currentDeviceNameLen = _tcslen(currentDevice);

			//Search for device name start from "COM"
			if (currentDeviceNameLen > 3 && _tcsnicmp(currentDevice, _T("COM"), 3) == 0)
			{
				SerialPortInfo currentSerialPortInfo;
				// Get the numerical char after "COM"
				std::string portNumber = "";
				bool isPort = true;
				for (size_t i = 3; i < currentDeviceNameLen && isPort; i++)
				{
					isPort = (iswdigit(currentDevice[i]) != 0);
					if (isPort) portNumber += (char)currentDevice[i];
				}

				// If numerical existed
				if (!portNumber.empty())
				{
					//Get port number
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

			//Move to next device name
			deviceStartCharLoc += (currentDeviceNameLen + 1);
		}

		// Get friendly name
		if (!result.empty())
		{
			// Collect all friendlyNames from registry
			std::vector<std::string> friendNames;
			processRegistryValue(HKEY_LOCAL_MACHINE,
				"SYSTEM\\CurrentControlSet\\Enum",
				[&friendNames](std::string valueName, DWORD dataType, unsigned char* data, int dataLen)
				{
					if (valueName.compare("FriendlyName") == 0 && dataType == REG_SZ)
					{
						// Found and convert to string, add to result
						std::string dataString(reinterpret_cast<char*>(data), dataLen);
						friendNames.push_back(dataString);
					}
				}
			);

			// Update friendly name
			for (int i = 0; i < result.size(); i++)
			{
				std::string COMName = "COM" + std::to_string(result[i].port);

				// Search on friendly name
				for (std::string friendlyName : friendNames)
				{
					if (friendlyName.find(COMName) != std::string::npos)
					{
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
	SerialPort::SerialPort() {

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
	SerialPort::~SerialPort() {
		close();
	}

#pragma endregion Constructor and Destructor

#pragma region Connection

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
		bool result = false;
		if (!m_connected)
		{
			// Lock mutex
			std::lock_guard<std::mutex> lock(m_mutex);

			// Connect
#ifdef _UNICODE
			LPCWSTR portLPCSTR = std::wstring(port.begin(), port.end()).c_str();
#else
			LPCSTR portLPCSTR = port.c_str();			
#endif
			m_serialHandle = CreateFile(portLPCSTR,
				GENERIC_READ | GENERIC_WRITE,
				0,  // Share mode = no share
				NULL,  // Security attribute = no security
				OPEN_EXISTING,
				0, //FILE_ATTRIBUTE_NORMAL,
				NULL  // no templates
			);

			if (m_serialHandle != INVALID_HANDLE_VALUE)
			{
				// Set parameters 
				bool setSerialPara = setAllSerialState();
				bool setTimeout = setTimeoutSetting(m_timeout);
				bool setBuffer = SetupComm(m_serialHandle, m_rxtxBufferSize, m_rxtxBufferSize); // Set the input and output buffer
				PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR);   //Reset buffer

				if (!setSerialPara || !setTimeout || !setBuffer)
				{
					// Fail to set parameters
					CloseHandle(m_serialHandle);
					m_connected = false;
					result = false;
				}
				else
				{
					m_connected = true;
					result = true;
				}
			}
		}

		return result;

	}

	/**
	 * @brief Close the serial port.
	 *
	 */
	void SerialPort::close()
	{
		if (this)
		{
			// Lock mutex
			std::lock_guard<std::mutex> lock(m_mutex);

			if (m_connected)
			{
				CloseHandle(m_serialHandle);
				m_connected = false;
			}
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
		return setSerialStateDecorator([this](DCB& serialParams)
			{
				serialParams.BaudRate = m_baudRate;
				serialParams.ByteSize = m_byteSize;
				serialParams.StopBits = m_stopBits;
				serialParams.Parity = m_parity;
				if (m_endOfChar != 0) serialParams.EofChar = m_endOfChar;

				// Set flow control
				this->SetFlowControlSubFunc(serialParams, m_flowControl);
			},
			false   // do not reset buffer
				);
	}

	/**
	 * @brief  A subfunction to set flow contorl parameter. It must be used with setSerialStateDecorator().
	 *
	 * @param[in] thisObj this
	 * @param[in, out] serialParams
	 */
	void SerialPort::SetFlowControlSubFunc(DCB& serialParams, int flowControl)
	{
		// Set Flow control
		if (flowControl == SERIAL_PORT_FCTL_NONE)
		{
			// No flow control
			serialParams.fOutX = false;
			serialParams.fInX = false;
			serialParams.fOutxCtsFlow = false;
			serialParams.fOutxDsrFlow = false;
			serialParams.fDsrSensitivity = false;
			serialParams.fRtsControl = RTS_CONTROL_DISABLE;
			serialParams.fDtrControl = DTR_CONTROL_DISABLE;
		}
		else if (flowControl == SERIAL_PORT_FCTL_XON_XOFF)
		{
			// Xon/Xoff flow control
			serialParams.fOutX = true;
			serialParams.fInX = true;
			serialParams.fOutxCtsFlow = false;
			serialParams.fOutxDsrFlow = false;
			serialParams.fDsrSensitivity = false;
			serialParams.fRtsControl = RTS_CONTROL_DISABLE;
			serialParams.fDtrControl = DTR_CONTROL_DISABLE;
		}
		else if (flowControl == SERIAL_PORT_FCTL_HARDWARE)
		{
			// Hardware flow control
			serialParams.fOutX = false;
			serialParams.fInX = false;
			serialParams.fOutxCtsFlow = true;
			serialParams.fOutxDsrFlow = true;
			serialParams.fDsrSensitivity = true;
			serialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
			serialParams.fDtrControl = DTR_CONTROL_HANDSHAKE;
		}
	}

	/**
	 * @brief Set timeout setting.
	 *
	 * @return Return true if success
	 */
	bool SerialPort::setTimeoutSetting(int timeout)
	{
		bool result = false;

		// Get timeouts
		COMMTIMEOUTS commTimeout = { 0 };
		result = GetCommTimeouts(m_serialHandle, &commTimeout);
		if (result)
		{
			commTimeout.ReadIntervalTimeout = MAXDWORD;
			commTimeout.ReadTotalTimeoutConstant = timeout;
			commTimeout.ReadTotalTimeoutMultiplier = 0;
			commTimeout.WriteTotalTimeoutConstant = timeout;
			commTimeout.WriteTotalTimeoutMultiplier = 0;

			result = SetCommTimeouts(m_serialHandle, &commTimeout);
		}

		return result;
	}

	/**
	 * @brief Reset rxtx buffer
	 *
	 */
	void SerialPort::resetBuffer()
	{
		if (m_connected)
		{
			// Lock mutex
			std::lock_guard<std::mutex> lock(m_mutex);

			// Reset
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
		if (baudRate != CBR_110 &&
			baudRate != CBR_300 &&
			baudRate != CBR_600 &&
			baudRate != CBR_1200 &&
			baudRate != CBR_2400 &&
			baudRate != CBR_4800 &&
			baudRate != CBR_9600 &&
			baudRate != CBR_14400 &&
			baudRate != CBR_19200 &&
			baudRate != CBR_38400 &&
			baudRate != CBR_57600 &&
			baudRate != CBR_115200 &&
			baudRate != CBR_128000 &&
			baudRate != CBR_256000)
			throw std::invalid_argument("baudRate(" + std::to_string(baudRate) + ") is invalid.");

		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		// Set
		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator([baudRate](DCB& serialParams)
				{
					serialParams.BaudRate = baudRate;
				}
			);

			// If success
			if (result) m_baudRate = baudRate;
		}
		else
		{
			// Serial not connect
			m_baudRate = baudRate;
			result = true;
		}

		return result;

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
		// Check
		if (byteSize < 4 || byteSize > 8)
			throw std::invalid_argument("byteSize(" + std::to_string(byteSize) + ") must be 4 - 8.");

		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		// Set
		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator([byteSize](DCB& serialParams)
				{
					serialParams.ByteSize = byteSize;
				}
			);

			// If success
			if (result) m_byteSize = byteSize;
		}
		else
		{
			// Serial not connect
			m_byteSize = byteSize;
			result = true;
		}

		return result;
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
		// Check
		if (stopBits != ONESTOPBIT && stopBits != ONE5STOPBITS && stopBits != TWOSTOPBITS)
			throw std::invalid_argument("stopBits(" + std::to_string(stopBits) + ") must be 1, 1.5 or 2.");

		// Initlialize variable
		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		// Set
		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator([stopBits](DCB& serialParams)
				{
					serialParams.StopBits = stopBits;
				}
			);

			// If success
			if (result) m_stopBits = stopBits;
		}
		else
		{
			// Serial not connect
			m_stopBits = stopBits;
			result = true;
		}

		return result;
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
		if (parity != NOPARITY &&
			parity != ODDPARITY &&
			parity != EVENPARITY &&
			parity != MARKPARITY &&
			parity != SPACEPARITY)
			throw std::invalid_argument("stopBits(" + std::to_string(parity) + ") must be NOPARITY, EVENPARITY, MARKPARITY or SPACEPARITY.");

		// Initlialize variable
		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		// Set
		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator([parity](DCB& serialParams)
				{
					serialParams.Parity = parity;
				}
			);

			// If success
			if (result) m_parity = parity;
		}
		else
		{
			// Serial not connect
			m_parity = parity;
			result = true;
		}

		return result;
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
		// Check
		if (flowControl != SERIAL_PORT_FCTL_NONE && flowControl != SERIAL_PORT_FCTL_XON_XOFF && flowControl != SERIAL_PORT_FCTL_HARDWARE)
			throw std::invalid_argument("flowControl(" + std::to_string(flowControl) + ") must be SERIAL_PORT_FCTL_NONE, SERIAL_PORT_FCTL_XON_XOFF or SERIAL_PORT_FCTL_HARDWARE.");

		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator([this, flowControl](DCB& serialParams)
				{
					SetFlowControlSubFunc(serialParams, flowControl);
				}
			);

			// If success
			if (result) m_flowControl = flowControl;
		}
		else
		{
			// Serial not connect
			m_flowControl = flowControl;
			result = true;
		}

		return result;
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
		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			// Serial connected

			// Set
			result = setSerialStateDecorator(
				[endOfChar](DCB& serialParams)
				{
					serialParams.EofChar = endOfChar;
				}
			);

			// If success
			if (result) m_endOfChar = endOfChar;
		}
		else
		{
			// Serial not connect
			m_endOfChar = endOfChar;
			result = true;
		}

		return result;
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
		// Exception
		if (timeout < 0 || timeout > MAXDWORD)
			throw std::invalid_argument("timeout(" + std::to_string(timeout) + ") must be >=0 and < MAXDWORD.");

		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			// Serial connected

			// Set
			result = setTimeoutSetting(timeout);

			// If success
			if (result) m_timeout = timeout;
		}
		else
		{
			// Serial not connect
			m_timeout = timeout;
			result = true;
		}

		return result;
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
		if (bufferSize <= 0 || bufferSize > MAXDWORD)
			throw std::invalid_argument("bufferSize(" + std::to_string(bufferSize) + ") must be >0 and < MAXDWORD.");

		bool result = false;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			// Serial connected

			// Set
			result = SetupComm(m_serialHandle, bufferSize, bufferSize); // Set the input and output buffer

			// If success
			if (result)
			{
				m_rxtxBufferSize = bufferSize;
				PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR);   //Reset buffer
			}
		}
		else
		{
			// Serial not connect
			m_rxtxBufferSize = bufferSize;
			result = true;
		}

		return result;
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
		bool result = false;

		// Parse string to unsigned char
		const int bufferSize = (int)ascii.length() + 1;
		std::unique_ptr<unsigned char[]> buffer = std::unique_ptr<unsigned char[]>(new unsigned char[bufferSize]);
		strcpy_s((char*)buffer.get(), bufferSize, ascii.c_str());

		// Send byte
		int n = sendBytes(buffer.get(), (int)ascii.length());
		if (n == ascii.length()) result = true;

		return result;
	}

	/**
	 * @brief Send Bytes to serial port
	 *
	 * @param[in] buffer Bytes to be sent
	 * @param[in] bufferSize Number of bytes to be sent
	 * @return Return the number of bytes sent. If serial port was not connected, return -1.
	 */
	int SerialPort::sendBytes(unsigned char* buffer, int bufferSize)
	{
		int result = -1;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			int n;
			if (WriteFile(m_serialHandle, buffer, bufferSize, (LPDWORD)((void*)&n), NULL))
				result = n;
			else
				result = 0;
		}

		return result;
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
	std::string SerialPort::readASCII(int strBufferSize)
	{
		// Read
		std::unique_ptr<unsigned char[]> buffer = std::unique_ptr<unsigned char[]>(new unsigned char[strBufferSize]);
		//unsigned char* buffer = new unsigned char[strBufferSize]();
		int n = readBytes(buffer.get(), strBufferSize);

		//Parse to string
		std::string result = std::string();
		if (n > 0)
		{
			result = std::string(reinterpret_cast<char*>(buffer.get()), n);
		}
		//delete[] buffer;

		return result;
	}

	/**
	 * @brief Read Bytes from serial port
	 *
	 * @param[out] buffer Bytes buffer
	 * @param[in] bufferSize Buffer size
	 * @return Return number of bytes collected. If serial port was not connected, return -1.
	 */
	int SerialPort::readBytes(unsigned char* buffer, int bufferSize)
	{
		int result = -1;

		// Lock mutex
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_connected)
		{
			// Reset buffer
			buffer[0] = '\0';

			// Read
			int n;
			if (ReadFile(m_serialHandle, buffer, bufferSize - 1, (LPDWORD)((void*)&n), NULL)) // The nNumberOfBytesToRead is bufferSize-1 instead of bufferSize because a '\0' may be added at Ipbuffer[n-1]
			{
				// Add end of string
				buffer[n] = '\0';
				result = n;
			}
			else
			{
				// Reset buffer
				buffer[0] = '\0';

				result = 0;
			}
		}

		return result;
	}

#pragma endregion Transmission

}
