#pragma once
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

//************Content************

#include <windows.h>
#include <string>
#include <mutex>
#include <functional>
#include <vector>
#include <atlbase.h>
#include <winreg.h>
#include <tchar.h>

/**
 * @example eg1_print_ports.cpp
 * @example eg2_read.cpp
 * 
 */
namespace SerialPortUtils
{
    /**
     * @brief Flow Control: None
     */
    const int SERIAL_PORT_FCTL_NONE = 0;

    /**
     * @brief Flow Control: Xon/Xoff
     */
    const int SERIAL_PORT_FCTL_XON_XOFF = 1;

    /**
     * @brief Flow Control: Hardware
     */
    const int SERIAL_PORT_FCTL_HARDWARE = 2;

    /**
     * @brief A structure to store the serial port information.
    */
    struct SerialPortInfo
    {
        int port = -1;                      /* Port. Default as -1.*/
        LPTSTR queryDosDeviceName = _T(""); /* Device Name use for QueryDosDevice()*/
        std::string deviceName = "";        /* Device Names */
        std::string friendlyName = "";      /* Friendly Name */
    };

    /**
     * @brief A class to control the serial port.
     * 
     * @details
     * Default setting: 
     * BaudRate = 9600
     * Byte Size = 8
     * Stop Bits = 1
     * Parity = No parity
     * Flow Control = None
     * End Of Char = 0
     * Timeout = 50ms
     * 
     */
    class SerialPort
    {
    public:
        static std::vector<SerialPortInfo> getSerialPortList();

    public:

        // Constructor and Destructor
        SerialPort();

        ~SerialPort();

        // Connection
        bool open(int port);
        bool open(std::string port);

        void close();

        void resetBuffer();

        // Getter and Setter
        bool setBaudRate(int baudrate);
        int getBaudRate();

        bool setByteSize(int byteSize);
        int getByteSize();

        bool setStopBits(int stopBits);
        int getStopBits();

        bool setParity(int parity);
        int getParity();

        bool setFlowControl(int flowControl);
        int getFlowControl();

        bool setEndOfChar(char endOfChar);
        char getEndOfChar();

        bool setTimeout(int timeout);
        int getTimeout();

        bool setRxTxBufferSize(int bufferSize);
        int getRxTxBufferSize();

        bool isOpened();

        // Transmission
        bool sendASCII(std::string ascii);
        int sendBytes(unsigned char* buffer, int bufferSize);

        std::string readASCII();
        std::string readASCII(int bufferSize);
        int readBytes(unsigned char* buffer, int bufferSize);

    private:
        HANDLE m_serialHandle = NULL;
        bool m_connected = false;
        std::mutex m_mutex;

        // Setting
        DWORD m_baudRate;
        BYTE m_byteSize;
        BYTE m_stopBits;
        BYTE m_parity;
        int m_flowControl;
        char m_endOfChar;
        int m_timeout;
        int m_rxtxBufferSize = 1024;
    
    private:
        bool setAllSerialState();

        void SetFlowControlSubFunc(DCB &serialParams, int flowControl);
        bool setTimeoutSetting(int timeout);

        /**
         * @brief A decorator to set serial setting
         *
         * @tparam SetSerialParaFunc void(DCB &serialParams)
         * 
         * @param setFunc Lambda function to set parameter.
         * @param[in] resetBuffer Set it as true to reset rxtx buffer after set state. Default as true.
         * @return bool Return true if success.
         */
        template <typename SetSerialParaFunc> bool setSerialStateDecorator(SetSerialParaFunc setFunc, bool resetBuffer = true)
        {
            bool result = false;

            // Initialize DCB structure.
            DCB serialParams;
            SecureZeroMemory(&serialParams, sizeof(DCB));
            serialParams.DCBlength = sizeof(DCB);

            // Get Parameters 
            result = GetCommState(m_serialHandle, &serialParams);

            // Set Parameters
            if (result)
            {
                // Set function
                setFunc(serialParams);

                // Set
                result = SetCommState(m_serialHandle, &serialParams);
            }

            // Reset buffer
            if (resetBuffer)
                PurgeComm(m_serialHandle, PURGE_TXCLEAR | PURGE_RXCLEAR);

            return result;
        }

    private:

/**
 * @brief The maximum length of the registry key.
 */
#define REGISTRY_MAX_KEY_LENGTH 255 

        /**
         * @brief Look up values under the key and subkey in registry and process.
         * 
         * @code{.cpp}
         * // Print all hardware FriendlyName values
         * int numOfValues = Utils::processRegistryValue(HKEY_LOCAL_MACHINE,
         *            "SYSTEM\\CurrentControlSet\\Enum",
        *            [](std::string valueName, DWORD dataType, unsigned char* data, int dataLen)
        *            {
        *                if (valueName.compare("FriendlyName") == 0 && dataType == REG_SZ)
        *                {
        *                    // Found and convert to string, add to result
        *                    std::string dataString(reinterpret_cast<char*>(data), dataLen);
        *                    std::cout << dataString;
        *                }
        *            }
        * );
        * @endcode
        * 
        * @tparam ValueProcess void(std::string valueName, DWORD dataType, unsigned char[] data,DWORD sizeOfData)
        * 
        * @param rootKey The root key. @c HKEY_CLASSES_ROOT, @c HKEY_CURRENT_USER, @c HKEY_LOCAL_MACHINE, @c HKEY_USERS, @c HKEY_CURRENT_CONFIG, @c HKEY_CURRENT_USER_LOCAL_SETTINGS, @c HKEY_PERFORMANCE_DATA, @c HKEY_PERFORMANCE_NLSTEXT or @c HKEY_PERFORMANCE_TEXT.
        * @param searchKey The key to be search. e.g. @c "HARDWARE\\DEVICEMAP\\SERIALCOMM"
        * @param func The function to be processed on value.
        * @param searchSubKey [Option] Default as true. If true,all of subkey under the key will also be processed. Set it as false if you only want to process the current key.
        * @return int Return the number of value to be processed.
        */
        template <typename ValueProcess> 
        static int processRegistryValue(HKEY rootKey, std::string searchKey, ValueProcess func, bool searchSubKey = true)
        {
            // Initilaize values
            int result = 0;

            // Create registry key
            HKEY hKey;
            if (RegOpenKeyEx(rootKey, searchKey.c_str(), 0, KEY_READ, &hKey) == ERROR_SUCCESS)
            {
                // Get information of subkey
                DWORD numOfSubKey = 0;
                DWORD maxSubKeySize;
                DWORD numOfValue;
                DWORD maxValueNameSize = 0;
                DWORD maxValueSize = 0;
                DWORD retCode = RegQueryInfoKey(hKey, nullptr, nullptr, nullptr, &numOfSubKey, &maxSubKeySize, nullptr, &numOfValue, &maxValueNameSize, &maxValueSize, nullptr,    nullptr);

                // Get value in current key
                if (numOfValue > 0)
                {
                    // Look up all value
#pragma warning(suppress : 26451)
                    std::unique_ptr<char[]> valueNameBuffer(new char[maxValueNameSize + 1]);
                    std::unique_ptr<unsigned char[]> dataByteBuffer(new unsigned char[maxValueSize]);
                    for (int i = 0; i < (int)numOfValue; i++)
                    {
                        //TCHAR valueName[REGISTRY_MAX_VALUE_NAME_LENGTH];
                        DWORD dataType;
                        DWORD valueNameBufferLen = maxValueNameSize + 1;
                        DWORD sizeOfDataByteBuffer = maxValueSize;

                        // Get the subkey
                        if (RegEnumValue(hKey, i, valueNameBuffer.get(), &valueNameBufferLen, NULL, &dataType, dataByteBuffer.get(), &sizeOfDataByteBuffer) == ERROR_SUCCESS)
                        {
                            // Convert value name to string
                            std::string valueName = std::string(valueNameBuffer.get(), (int)valueNameBufferLen);

                            // Process
                            try
                            {
                                func(valueName, dataType, dataByteBuffer.get(), sizeOfDataByteBuffer);
                            }
                            catch (...)
                            {
                                // Release buffer
                                valueNameBuffer.release();
                                dataByteBuffer.release();

                                // Close
                                RegCloseKey(hKey);

                                // Rethrow
                                throw;
                            }

                            // Update counter
                            result++;                
                        }
                    }
                }

                // Search value in subkey
                if (searchSubKey && numOfSubKey > 0)
                {
                    // Look up all subkey
                    for (int i = 0; i < (int)numOfSubKey; i++)
                    {
                        TCHAR subKey[REGISTRY_MAX_KEY_LENGTH];
                        DWORD subKeyBufferLen = sizeof(subKey);

                        // Get the subkey
                        if (RegEnumKeyEx(hKey, i, subKey, &subKeyBufferLen, NULL, NULL, NULL, NULL) == ERROR_SUCCESS)
                        {
                            // Search under the subkey
                            try
                            {
                                result += processRegistryValue(rootKey, searchKey + "\\" + subKey, func, searchSubKey);
                            }
                            catch (...)
                            {
                                // Close
                                RegCloseKey(hKey);

                                //Rethrow
                                throw;
                            }
                            
                        }
                    }
                }            
            }

            // Close
            RegCloseKey(hKey);

            return result;
        }

    };


}

//*******************************

#endif