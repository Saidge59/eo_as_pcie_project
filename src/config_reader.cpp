// config_reader.cpp
#include "config_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * @brief Reads TCP configuration data from a file
 *
 * This function opens a configuration file specified by file_path and reads IP addresses
 * and ports for TCP channels. It expects the file to list an IP address followed by its
 * corresponding port number for each channel, alternately. The function populates the
 * config_tcp struct with server IPs and ports.
 *
 * @param file_path Path to the configuration file
 * @param out_config Reference to config_tcp struct where configuration will be stored
 * @return True if configuration is successfully read and valid, false otherwise
 */
bool config_reader::read_config_file(const std::string &file_path, config_tcp &out_config)
{
    std::ifstream config_file(file_path);
    if (!config_file.is_open())
    {
        std::cerr << "Failed to open config file: " << file_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(config_file, line))
    {
        if (!line.empty() && std::isdigit(line[0]))
        { // Assuming IP addresses won't start with a digit
            int port = std::stoi(line);
            out_config.ports.push_back(port);
        }
        else
        {
            out_config.server_ips.push_back(line); // Read server IP
        }
    }

    if (out_config.server_ips.size() != 8 || out_config.ports.size() != 8)
    {
        std::cerr << "Incorrect number of IPs or ports in config file." << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief Reads register data from a file
 *
 * Parses a file containing register addresses and values, storing them in a vector
 * of register_data structs. The file format expects hex values separated by '#'.
 *
 * @param file_path Path to the file containing register data
 * @return Vector of register_data structs with parsed address and value pairs
 */
std::vector<register_data> config_reader::read_register_data(const std::string &file_path)
{
    std::vector<register_data> data;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string address_str, value_str;
        if (std::getline(iss, address_str, '#') && std::getline(iss, value_str, '#'))
        {
            register_data reg_data;
            reg_data.address = std::stoul(address_str, nullptr, 16); // Convert hex string to uint32_t
            reg_data.value = std::stoul(value_str, nullptr, 16);
            data.push_back(reg_data);
        }
    }
    return data;
}