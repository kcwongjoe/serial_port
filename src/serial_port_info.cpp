#include "serial_port/serial_port_info.h"

namespace SerialPortUtils
{
    /**
     * @brief Convert SerialPortInfo List to Friendly Name list.
     *
     * @param serialPorts SerialPortInfo List
     * @return Return the Friendly Name list.
     */
    std::vector<std::string> SerialPortInfo::toFriendlyNameList(std::vector<SerialPortInfo> serialPorts)
    {
        std::vector<std::string> result;
        for (int i = 0; i < serialPorts.size(); i++) {
            result.push_back(serialPorts[i].friendlyName);
        }

        return result;
    }

    /**
     * @brief Convert SerialPortInfo List to Port list.
     *
     * @param serialPorts SerialPortInfo List
     * @return Return the Port list.
     */
    std::vector<int> SerialPortInfo::toPortList(std::vector<SerialPortInfo> serialPorts)
    {
        std::vector<int> result;
        for (int i = 0; i < serialPorts.size(); i++) {
            result.push_back(serialPorts[i].port);
        }

        return result;
    }
} // namespace SerialPortUtils