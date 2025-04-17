#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <cstdint>
#include <string>
#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#pragma pack(1)
/**
 * @brief Structure representing the message header.
 *
 * This structure is used for synchronizing and describing the details of a message
 * in a communication protocol. It includes information about the message source, destination,
 * type, sequence, and other specific parameters related to FPGA and processing server.
 */
typedef struct MessageHeader {
    uint32_t prefix1;                ///< Message prefix 1. Used for synchronization. Should be 0xAAAA.
    uint8_t src_id;                  ///< Message source process ID.
    uint8_t dst_id;                  ///< Message destination process ID.
    uint32_t size;                   ///< Message length in bytes including this header.
    uint16_t type;                   ///< Message type.
    uint32_t msg_id;                 ///< Message sequence counter for this source on this interface.
    uint8_t spare1[48];              ///< Spare bytes/reserved.

    // FPGA Part 128 bytes
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
} MessageHeader;

/**
 * @brief Pointer to MessageHeader structure.
 */
typedef MessageHeader* PMessageHeader;

/**
 * @struct tcp_channel
 * @brief Structure representing TCP channel information.
 */
struct tcp_channel {
    int socket; ///< Socket descriptor.
    bool is_connected; ///< Flag indicating whether the channel is connected.
};
#pragma pack()

/**
 * @class tcp_client
 * @brief Manages a single TCP connection to a server.
 *
 * tcp_client class is responsible for handling TCP connection operations such as connect, send, and close.
 * This class is designed to handle a single connection per instance, allowing flexible usage in multi-threaded environments.
 */
class tcp_client {
public:
    /**
     * @brief Constructor for tcp_client.
     * @param server_ip IP address of the server.
     * @param port Port number to connect on.
     */
    tcp_client(const std::string& server_ip, int port);

    /**
     * @brief Destructor for tcp_client.
     * Cleans up the connection.
     */
    ~tcp_client();

    /**
     * @brief Establishes connection with the server.
     * @return True if connection is successful, false otherwise.
     */
    bool connect();

    /**
     * @brief Establishes re-connection with the server.
     * @return None
     */
    void reconnect_client();

    /**
     * @brief Sends data over the TCP connection.
     * @param data Pointer to the data buffer.
     * @param data_size Size of the data to send.
     * @return Number of bytes sent or -1 on failure.
     */
    int send_data(const char* data, size_t data_size);

    /**
     * @brief Configure socket options after connect with the server.
     * @return None
     */
    void configure_socket_options();

    /**
     * @brief Closes the TCP connection.
     */
    void close();

    /**
     * @brief Checks if the client is currently connected.
     * @return True if connected, false otherwise.
     */
    bool is_connected() const;

private:
    int socket_;                 ///< Socket descriptor for the connection.
    std::string server_ip_;      ///< Server IP address.
    int port_;                   ///< Port number.
    mutable std::mutex mutex_;   ///< Mutex for thread-safe operations.
};

#endif // TCP_CLIENT_H