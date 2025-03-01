#ifndef TCP_COMMAND_SERVER_H
#define TCP_COMMAND_SERVER_H

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <array>
#include <functional>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "eo_as_interface_lib.h"
#include "shared_state.h"
// #include "tcp_server.h"
#include "config_reader.h"

/// @defgroup PortDefinitions Port Definitions
/// @{
#define COMMAND_SERVER_PORT 3030       ///< Port number for the command server
#define DMA_COMMAND_SERVER_PORT 3031   ///< Port number for DMA command server
#define TCP_COMMAND_SERVER_PORT 3032   ///< Port number for TCP command server
#define LOCAL_COMMAND_SERVER_PORT 3033 ///< Port number for local command server
/// @}

/// @defgroup ChannelDefinitions Channel and Size Definitions
/// @{
#define LOCAL_RX_STREAMING_START_CHANNEL 0  ///< Starting channel for local RX streaming
#define LOCAL_TX_STREAMING_START_CHANNEL 8 ///< Starting channel for local TX streaming
#define TCP_STREAMING_START_CHANNEL 0       ///< Starting channel for TCP streaming
#define MAX_TCP_STREAMING_CHANNELS 8        ///< Maximum number of TCP streaming channels
#define MAX_LOCAL_STREAMING_TX_CHANNELS 8   ///< Maximum number of local streaming TX channels
#define MAX_LOCAL_STREAMING_RX_CHANNELS 8  ///< Maximum number of local streaming RX channels
#define MAX_LOCAL_STREAMING_CHANNELS 8     ///< Maximum number of local streaming channels
#define MAX_DESCRIPTORS_IN_CHANNEL 6        ///< Maximum number of descriptors in a channel
#define SIZE_2MB 2 * 1024 * 1024            ///< Size definition for 2MB
/// @}

#pragma pack(push, 1)

/**
 * @enum sampler_error_code
 * @brief Defines error codes used throughout the system.
 */
enum sampler_error_code
{
    SAMPLER_SUCCESS = 0,                    ///< Operation completed successfully
    SAMPLER_ERROR_INVALID_COMMAND_SIZE = 1, ///< Error due to invalid command size
    SAMPLER_ERROR_GENERAL_FAILED = 2,       ///< General operation failure
    SAMPLER_ERROR_NETWORK_FAILED = 3,       ///< Network-related failure
};

/**
 * @enum command_opcodes
 * @brief Defines operation codes for commands handled by the server.
 */
enum command_opcodes
{
    SAMPLER_CONNECT_REQ = 800,                         ///< Request to connect to the server
    SAMPLER_CONNECT_RES = 801,                         ///< Response to connect request
    SAMPLER_SYSTEM_INIT_DMA_REQ = 802,                 ///< Request to initialize DMA system
    SAMPLER_SYSTEM_INIT_DMA_RES = 803,                 ///< Response to DMA system initialization
    SAMPLER_SYSTEM_START_DMA_CONFIGURATION_REQ = 804,  ///< Request to start DMA configuration
    SAMPLER_SYSTEM_START_DMA_CONFIGURATION_RES = 805,  ///< Response to start DMA configuration
    SAMPLER_SYSTEM_START_DMA_GLOBAL_RX_REQ = 806,      ///< Request to start global RX DMA
    SAMPLER_SYSTEM_START_DMA_GLOBAL_RX_RES = 807,      ///< Response to start global RX DMA
    SAMPLER_SYSTEM_STOP_DMA_GLOBAL_RX_REQ = 808,       ///< Request to stop global RX DMA
    SAMPLER_SYSTEM_STOP_DMA_GLOBAL_RX_RES = 809,       ///< Response to stop global RX DMA
    SAMPLER_SYSTEM_START_DMA_LIST_CHANNELS_REQ = 810,  ///< Request to start DMA for a list of channels
    SAMPLER_SYSTEM_START_DMA_LIST_CHANNELS_RES = 811,  ///< Response to start DMA for channels
    SAMPLER_SYSTEM_STOP_DMA_LIST_CHANNELS_REQ = 812,   ///< Request to stop DMA for a list of channels
    SAMPLER_SYSTEM_STOP_DMA_LIST_CHANNELS_RES = 813,   ///< Response to stop DMA for channels
    SAMPLER_CONFIG_START_TCP_STREAMING_REQ = 814,      ///< Request to start TCP streaming
    SAMPLER_CONFIG_START_TCP_STREAMING_RES = 815,      ///< Response to start TCP streaming
    SAMPLER_CONFIG_STOP_TCP_STREAMING_REQ = 816,       ///< Request to stop TCP streaming
    SAMPLER_CONFIG_STOP_TCP_STREAMING_RES = 817,       ///< Response to stop TCP streaming
    SAMPLER_CONFIG_START_LOCAL_STREAMING_REQ = 818,    ///< Request to start local streaming
    SAMPLER_CONFIG_START_LOCAL_STREAMING_RX_RES = 819, ///< Response to start local streaming RX
    SAMPLER_CONFIG_STOP_LOCAL_STREAMING_REQ = 820,     ///< Request to stop local streaming
    SAMPLER_CONFIG_STOP_LOCAL_STREAMING_RX_RES = 821,  ///< Response to stop local streaming RX
    SAMPLER_CONFIG_WRITE_REG_BAR1_REQ = 822,           ///< Request to write to BAR1 register
    SAMPLER_CONFIG_WRITE_REG_BAR1_RES = 823,           ///< Response to write to BAR1 register
    SAMPLER_CONFIG_READ_REG_BAR1_REQ = 824,            ///< Request to read from BAR1 register
    SAMPLER_CONFIG_READ_REG_BAR1_RES = 825,            ///< Response to read from BAR1 register
    SAMPLER_SYSTEM_START_DMA_GLOBAL_TX_REQ = 826,      ///< Request to start global TX DMA
    SAMPLER_SYSTEM_START_DMA_GLOBAL_TX_RES = 827,      ///< Response to start global TX DMA
    SAMPLER_SYSTEM_STOP_DMA_GLOBAL_TX_REQ = 828,       ///< Request to stop global TX DMA
    SAMPLER_SYSTEM_STOP_DMA_GLOBAL_TX_RES = 829,       ///< Response to stop global TX DMA
    SAMPLER_UNKNOW                                     ///< Unknown or invalid command
};

/**
 * @enum test_opcodes
 * @brief Defines operation codes for test commands.
 */
enum test_opcodes
{
    SAMPLER_TEST_NO = 1,                    ///< No test operation
    SAMPLER_TEST_TX_CP_CH_8_TO_CH_12 = 10,  ///< Test TX copy from channel 8 to 12
    SAMPLER_TEST_TX_CP_CH_8_TO_CH_16 = 11,  ///< Test TX copy from channel 8 to 16
    SAMPLER_TEST_TX_CP_CH_9_TO_CH_13 = 12,  ///< Test TX copy from channel 9 to 13
    SAMPLER_TEST_TX_CP_CH_9_TO_CH_17 = 13,  ///< Test TX copy from channel 9 to 17
    SAMPLER_TEST_TX_CP_CH_10_TO_CH_14 = 15, ///< Test TX copy from channel 10 to 14
    SAMPLER_TEST_TX_CP_CH_10_TO_CH_18 = 16, ///< Test TX copy from channel 10 to 18
    SAMPLER_TEST_TX_CP_CH_11_TO_CH_15 = 17, ///< Test TX copy from channel 11 to 15
    SAMPLER_TEST_TX_CP_CH_11_TO_CH_19 = 18, ///< Test TX copy from channel 11 to 19
    SAMPLER_TEST_UNKNOW                     ///< Unknown or invalid test command
};

/**
 * @struct sampler_connect_command
 * @brief Represents a command to connect to the server.
 */
struct sampler_connect_command
{
    uint32_t opcode;     ///< Command opcode (SAMPLER_CONNECT_REQ)
    uint32_t ip_address; ///< Target IP address for connection
    uint16_t port;       ///< Target port for connection
};

/**
 * @struct sampler_system_dma_init_command
 * @brief Represents a command to initialize the DMA system.
 */
struct sampler_system_dma_init_command
{
    uint32_t opcode;           ///< Command opcode (SAMPLER_SYSTEM_INIT_DMA_REQ)
    uint32_t token_id;         ///< Unique token identifier for the request
    uint32_t dma_channels;     ///< Number of DMA channels to initialize
    uint32_t descriptors_num;  ///< Number of descriptors per channel
    uint32_t descriptors_size; ///< Size of each descriptor
};

/**
 * @struct sampler_generic_command
 * @brief Generic command structure for simple operations.
 */
struct sampler_generic_command
{
    uint32_t opcode;   ///< Command opcode
    uint32_t token_id; ///< Unique token identifier for the request
};

/**
 * @struct sampler_start_tcp_streaming_command
 * @brief Represents a command to start TCP streaming on specified channels.
 */
struct sampler_start_tcp_streaming_command
{
    uint32_t opcode;                                   ///< Command opcode (SAMPLER_CONFIG_START_TCP_STREAMING_REQ)
    uint32_t token_id;                                 ///< Unique token identifier for the request
    bool channels[MAX_TCP_STREAMING_CHANNELS];         ///< Flags indicating which channels to enable
    char ip_addresses[MAX_TCP_STREAMING_CHANNELS][16]; ///< IP addresses for each channel (max 15 chars + null)
    uint16_t ports[MAX_TCP_STREAMING_CHANNELS];        ///< Port numbers for each channel
    uint32_t cpu_id;                                   ///< CPU identifier for the operation
};

/**
 * @struct sampler_start_local_streaming_command
 * @brief Represents a command to start local streaming on specified channels.
 */
struct sampler_start_local_streaming_command
{
    uint32_t opcode;                             ///< Command opcode (SAMPLER_CONFIG_START_LOCAL_STREAMING_REQ)
    uint32_t token_id;                           ///< Unique token identifier for the request
    uint32_t test_id;                            ///< Identifier for the test operation
    bool channels[MAX_LOCAL_STREAMING_CHANNELS]; ///< Flags indicating which channels to enable
    uint32_t start_cpu_id;                       ///< Starting CPU identifier for the operation
};

/**
 * @struct sampler_stop_tcp_streaming_command
 * @brief Represents a command to stop TCP streaming on specified channels.
 */
struct sampler_stop_tcp_streaming_command
{
    uint32_t opcode;                           ///< Command opcode (SAMPLER_CONFIG_STOP_TCP_STREAMING_REQ)
    uint32_t token_id;                         ///< Unique token identifier for the request
    bool channels[MAX_TCP_STREAMING_CHANNELS]; ///< Flags indicating which channels to disable
};

/**
 * @struct sampler_stop_local_streaming_command
 * @brief Represents a command to stop local streaming on specified channels.
 */
struct sampler_stop_local_streaming_command
{
    uint32_t opcode;                             ///< Command opcode (SAMPLER_CONFIG_STOP_LOCAL_STREAMING_REQ)
    uint32_t token_id;                           ///< Unique token identifier for the request
    bool channels[MAX_LOCAL_STREAMING_CHANNELS]; ///< Flags indicating which channels to disable
};

/**
 * @struct sampler_start_list_dma_channels_command
 * @brief Represents a command to start DMA on a list of channels.
 */
struct sampler_start_list_dma_channels_command
{
    uint32_t opcode;                            ///< Command opcode (SAMPLER_SYSTEM_START_DMA_LIST_CHANNELS_REQ)
    uint32_t token_id;                          ///< Unique token identifier for the request
    bool channels[MAX_NUM_CHANNELS];            ///< Flags indicating which channels to enable
    bool is_counter_channels[MAX_NUM_CHANNELS]; ///< Flags indicating counter channels
};

/**
 * @struct sampler_stop_list_dma_channels_command
 * @brief Represents a command to stop DMA on a list of channels.
 */
struct sampler_stop_list_dma_channels_command
{
    uint32_t opcode;                 ///< Command opcode (SAMPLER_SYSTEM_STOP_DMA_LIST_CHANNELS_REQ)
    uint32_t token_id;               ///< Unique token identifier for the request
    bool channels[MAX_NUM_CHANNELS]; ///< Flags indicating which channels to disable
};

/**
 * @struct sampler_write_register_command
 * @brief Represents a command to write to a register.
 */
struct sampler_write_register_command
{
    uint32_t opcode;   ///< Command opcode (SAMPLER_CONFIG_WRITE_REG_BAR1_REQ)
    uint32_t token_id; ///< Unique token identifier for the request
    uint64_t address;  ///< Memory address of the register
    uint32_t value;    ///< Value to write to the register
};

/**
 * @struct sampler_read_register_command
 * @brief Represents a command to read from a register.
 */
struct sampler_read_register_command
{
    uint32_t opcode;   ///< Command opcode (SAMPLER_CONFIG_READ_REG_BAR1_REQ)
    uint32_t token_id; ///< Unique token identifier for the request
    uint64_t address;  ///< Memory address of the register
};

/**
 * @struct sampler_system_general_res
 * @brief Generic response structure for system operations.
 */
struct sampler_system_general_res
{
    uint32_t opcode;           ///< Response opcode
    uint32_t token_id;         ///< Token identifier matching the request
    sampler_error_code status; ///< Status code indicating the result
};

/**
 * @struct sampler_read_register_command_res
 * @brief Response structure for a register read operation.
 */
struct sampler_read_register_command_res
{
    struct sampler_system_general_res gen_res; ///< General response data
    uint64_t address;                          ///< Address of the register read
    uint32_t value;                            ///< Value read from the register
};

/**
 * @brief Structure representing the message header.
 *
 * This structure is used for synchronizing and describing the details of a message
 * in a communication protocol. It includes information about the message source, destination,
 * type, sequence, and other specific parameters related to FPGA and processing server.
 */
typedef struct message_header {
    uint32_t prefix1;                ///< Message prefix 1. Used for synchronization. Should be 0xAAAA.
    uint8_t src_id;                  ///< Message source process ID.
    uint8_t dst_id;                  ///< Message destination process ID.
    uint32_t size;                   ///< Message length in bytes including this header.
    uint16_t type;                   ///< Message type.
    uint32_t msg_id;                 ///< Message sequence counter for this source on this interface.
    uint8_t spare1[48];              ///< Spare bytes/reserved.

    // FPGA Part 128 bytes
    // Includes; timestamp, status, etc....
    uint32_t prefix2;                ///< Message prefix 2. Used for synchronization. Should be 0xBBBB.
    int64_t dwell_id;
    int32_t port_id;
    int64_t sampling_rate_hz;        ///< Port sampling frequency in Hz.
    int64_t center_frequency_hz;     ///< Port RF center frequency in Hz.
    int32_t number_of_samples_dwell;
    uint8_t is_counter_mode;         ///< 0 for ADCs, 1 for Counter.
    uint8_t snapshots_rate;          ///< Number of snapshots per second.
    uint8_t msg_type;                ///< 0 for Snapshots, 1 for Sustain.
    uint8_t DMA_id;                  ///< DMA index (0 for Snapshots, 1-4 for Sustain).
    uint8_t fpga_reserved[88];       ///< Reserved for FPGA.

    // Reserved for processing server.
    uint8_t reserved[128];           ///< Reserved for processing server.
} message_header_t;

#pragma pack(pop)

/**
 * @brief Pointer to message_header structure.
 */
typedef message_header_t* pmessage_header;

/// @brief Callback function type for handling commands.
/// @param client_socket The client socket descriptor.
/// @param command_data The command data as a string.
using command_callback = std::function<void(int, const std::string &)>;

/**
 * @class tcp_command_server
 * @brief Implements a TCP server for handling sampler commands and streaming operations.
 */
class tcp_command_server
{
public:
    /**
     * @brief Constructs a TCP command server instance.
     * @param state Reference to the shared state object.
     * @param port Port number on which the server will listen.
     * @param devicePath Path to the device for driver interaction.
     */
    tcp_command_server(shared_state &state, uint16_t port, const char *devicePath);

    /**
     * @brief Destructor, ensures proper cleanup of resources.
     */
    ~tcp_command_server();

    /// @name Command Handlers
    /// @brief Functions to handle specific sampler commands.
    /// @{
    void handle_sampler_connect(int client_socket, const std::string &command_data);
    void handle_sampler_dma_system_init(int client_socket, const std::string &command_data);
    void handle_sampler_start_tcp_streaming(int client_socket, const std::string &command_data);
    void handle_sampler_stop_tcp_streaming(int client_socket, const std::string &command_data);
    void handle_sampler_start_local_streaming(int client_socket, const std::string &command_data);
    void handle_sampler_stop_local_streaming(int client_socket, const std::string &command_data);
    void handle_sampler_start_dma_configuration(int client_socket, const std::string &command_data);
    void handle_sampler_stop_rx_dma_global(int client_socket, const std::string &command_data);
    void handle_sampler_start_rx_dma_global(int client_socket, const std::string &command_data);
    void handle_sampler_stop_tx_dma_global(int client_socket, const std::string &command_data);
    void handle_sampler_start_tx_dma_global(int client_socket, const std::string &command_data);
    void handle_sampler_stop_list_dma_channels(int client_socket, const std::string &command_data);
    void handle_sampler_start_list_dma_channels(int client_socket, const std::string &command_data);
    void handle_sampler_read_register(int client_socket, const std::string &command_data);
    void handle_sampler_write_register(int client_socket, const std::string &command_data);
    /// @}

    /**
     * @brief Starts the TCP command server.
     * @return True if the server starts successfully, false otherwise.
     */
    bool start();

    /**
     * @brief Stops the TCP command server and cleans up resources.
     */
    void stop();

    /**
     * @brief Registers a callback function for a specific command opcode.
     * @param opcode The command opcode to associate with the callback.
     * @param callback The callback function to execute when the opcode is received.
     */
    void register_callback(uint32_t opcode, command_callback callback);

private:
    /// @brief Accepts incoming client connections in a separate thread.
    void accept_connections();

    /// @brief Handles communication with a connected client.
    /// @param client_socket The socket descriptor for the client.
    void handle_client(int client_socket);

    /**
     * @brief Sends data to a client.
     * @param token_id Token identifier for the client.
     * @param data The data to send.
     * @return True if data is sent successfully, false otherwise.
     */
    bool send_data(uint32_t token_id, const std::string &data);

    /**
     * @brief Generates a unique token ID for a client connection.
     * @param ip_address The client's IP address.
     * @param port The client's port number.
     * @param client_socket The client's socket descriptor.
     * @return A unique token ID.
     */
    uint32_t generate_token_id(uint32_t ip_address, uint16_t port, int client_socket);

    /**
     * @brief Retrieves IP address and port from a token ID.
     * @param token_id The token identifier to query.
     * @param ip_address Pointer to store the retrieved IP address.
     * @param port Pointer to store the retrieved port.
     * @param client_socket The client's socket descriptor.
     */
    void retrieve_ip_and_port_from_token(uint32_t token_id, uint32_t *ip_address, uint16_t *port, int client_socket);

    /**
     * @brief Initializes the DMA system with provided register data and command.
     * @param register_data Vector of register address-value pairs.
     * @param init_dma_command Pointer to the DMA initialization command.
     */
    void initialize_dma(const std::vector<register_data> &register_data, const sampler_system_dma_init_command *init_dma_command);

    /**
     * @brief Sends data to a client over a specific channel.
     * @param token_id Token identifier for the client.
     * @param channel_index Index of the channel to send data over.
     * @param data Pointer to the data buffer.
     * @param data_size Size of the data to send.
     */
    void send_data_to_client(uint32_t token_id, int channel_index, const char *data, size_t data_size);

    /**
     * @brief Sends an error code response to a client.
     * @param token_id Token identifier for the client.
     * @param opcode The opcode associated with the error.
     * @param error_code The error code to send.
     */
    void send_error_code(uint32_t token_id, uint32_t opcode, sampler_error_code error_code);

    /**
     * @brief Builds a TCP configuration from a streaming command.
     * @param command The TCP streaming command to parse.
     * @param config Reference to the config_tcp structure to populate.
     */
    void build_config_from_command(const sampler_start_tcp_streaming_command &command, config_tcp &config);

    /**
     * @brief Verifies the integrity of received data.
     * @param data_buffer Pointer to the data buffer.
     * @param data_size Size of the data buffer.
     * @param offset Starting offset in the buffer.
     * @return True if data integrity is verified, false otherwise.
     */
    bool verify_data_integrity(char *data_buffer, size_t data_size, int offset);

    /**
     * @brief Prints the message header from a data buffer for debugging.
     * @param data_buffer Pointer to the buffer containing the message header.
     */
    void print_message_header_from_buffer(const char *data_buffer);

    uint16_t port_;                                         ///< Port number the server listens on
    int listen_socket_;                                     ///< Socket descriptor for listening
    std::thread accept_thread_;                             ///< Thread for accepting connections
    std::atomic<bool> running_;                             ///< Flag indicating if the server is running
    std::mutex connections_mutex_;                          ///< Mutex for synchronizing connection access
    std::map<uint32_t, std::string> connections_;           ///< Map of token IDs to connection data
    std::unordered_map<uint32_t, int> token_to_socket_map_; ///< Mapping of tokens to socket descriptors
    // std::unordered_map<uint32_t, std::unique_ptr<tcp_server>> streaming_servers_; ///< Map of streaming servers (commented out)
    std::map<uint32_t, std::vector<bool>> streaming_local_channels_; ///< Local streaming channel states
    std::map<uint32_t, std::vector<bool>> streaming_tcp_channels_;   ///< TCP streaming channel states
    global_start_dma_configuration start_dma_configuration_;         ///< DMA configuration data
    driver_interface driver_interface_;                              ///< Interface to the driver
    shared_state &shared_state_;                                     ///< Reference to shared state
    std::map<uint32_t, command_callback> command_callbacks_;         ///< Registered command callbacks

    // DMA configuration members
    eo_as_mem_map memory_data_;   ///< Memory mapping data
    eo_as_event_data event_data_; ///< Event data
    eo_as_dma_params dma_param_;  ///< DMA parameters
    void *va_mmap_;

    std::array<std::array<std::thread, 8>, MAX_TCP_STREAMING_CHANNELS> tcp_worker_threads_; ///< Worker threads for TCP streaming
    std::array<std::atomic<bool>, MAX_TCP_STREAMING_CHANNELS> tcp_streaming_running_;       ///< TCP streaming running flags
    std::array<int, MAX_TCP_STREAMING_CHANNELS> tcp_streaming_stop_event_;                  ///< TCP streaming stop event flags

    std::array<std::array<std::thread, MAX_DESCRIPTORS_IN_CHANNEL>, MAX_LOCAL_STREAMING_RX_CHANNELS> local_rx_worker_threads_; ///< RX worker threads
    std::array<std::atomic<bool>, MAX_LOCAL_STREAMING_RX_CHANNELS> local_rx_streaming_running_;                                ///< RX streaming running flags
    std::array<int, MAX_LOCAL_STREAMING_RX_CHANNELS> local_rx_streaming_stop_event_;                                           ///< RX streaming stop event flags

    std::array<std::array<std::thread, MAX_DESCRIPTORS_IN_CHANNEL>, MAX_LOCAL_STREAMING_TX_CHANNELS> local_tx_worker_threads_; ///< TX worker threads
    std::array<std::atomic<bool>, MAX_LOCAL_STREAMING_TX_CHANNELS> local_tx_streaming_running_;                                ///< TX streaming running flags
    std::array<int, MAX_LOCAL_STREAMING_TX_CHANNELS> local_tx_streaming_stop_event_;                                           ///< TX streaming stop event flags
};

#endif // TCP_COMMAND_SERVER_H