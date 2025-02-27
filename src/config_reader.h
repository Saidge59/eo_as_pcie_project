// config_reader.h
#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <vector>
#include <string>
#include <utility>
#include <stdint.h>

/**
 * @struct register_data
 * @brief Represents a register address and value pair
 */
struct register_data
{
    uint32_t address; ///< Register address
    uint32_t value;   ///< Value to be written to the register
};

/**
 * @struct config_tcp
 * @brief Configuration structure holding list of servers IP and ports
 */
struct config_tcp
{
    std::vector<std::string> server_ips; ///< List of server IP addresses, one for each TCP channel
    std::vector<int> ports;              ///< List of ports, one for each TCP channel
};

/**
 * @class config_reader
 * @brief Utility class for reading configuration data from files
 */
class config_reader
{
public:
    /**
     * @brief Reads TCP configuration from a file
     * @param file_path Path to the configuration file
     * @param out_config Output configuration structure to populate
     * @return True if reading was successful, false otherwise
     */
    static bool read_config_file(const std::string &file_path, config_tcp &out_config);

    /**
     * @brief Reads register data from a file
     *
     * Parses a file containing register addresses and values,
     * and stores them in a vector of register_data structs.
     *
     * @param file_path Path to the file containing register data
     * @return Vector of register_data structs with parsed address and value pairs
     */
    static std::vector<register_data> read_register_data(const std::string &file_path);
};

#endif // CONFIG_READER_H