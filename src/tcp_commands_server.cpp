#include "tcp_commands_server.h"

#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <limits.h>
#include <semaphore.h>
#include "tcp_client.h"
#include <poll.h>

#define REG_SRV_VER 0x1C
#define SRV_VERSION 0x202
#define INVALID_SOCKET -1

std::string get_executable_path()
{
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    if (count == -1)
    {
        return "";
    }
    path[count] = '\0';
    return std::string(path);
}

std::string get_executable_directory()
{
    std::string full_path = get_executable_path();
    size_t last_slash_idx = full_path.find_last_of("/");
    if (last_slash_idx != std::string::npos)
    {
        return full_path.substr(0, last_slash_idx);
    }
    return "";
}

tcp_command_server::tcp_command_server(shared_state &state, uint16_t port, const char *devicePath)
    : shared_state_(state),
      port_(port),
      listen_socket_(INVALID_SOCKET),
      running_(false),
      driver_interface_(devicePath)
{
    memset(&start_dma_configuration_, 0, sizeof(start_dma_configuration_));

    for (size_t i = 0; i < MAX_LOCAL_STREAMING_CHANNELS; i++)
    {
        if (i < MAX_TCP_STREAMING_CHANNELS)
        {
            tcp_streaming_running_[i] = false;
            tcp_streaming_stop_event_[i] = 0;
        }
        if (i < MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            local_rx_streaming_running_[i] = false;
            local_rx_streaming_stop_event_[i] = 0;
        }
        if (i >= MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            local_tx_streaming_running_[i - MAX_LOCAL_STREAMING_RX_CHANNELS] = false;
            local_tx_streaming_stop_event_[i - MAX_LOCAL_STREAMING_RX_CHANNELS] = 0;
        }
    }
    driver_interface_.write_register(1, REG_SRV_VER, SRV_VERSION);
}

tcp_command_server::~tcp_command_server()
{
    stop();
}

bool tcp_command_server::start()
{
    listen_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_socket_ == INVALID_SOCKET)
    {
        std::cerr << "Failed to create listen socket: " << strerror(errno) << std::endl;
        return false;
    }

    int flags = fcntl(listen_socket_, F_GETFL, 0);
    fcntl(listen_socket_, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "Failed to bind listen socket: " << strerror(errno) << std::endl;
        close(listen_socket_);
        return false;
    }

    if (listen(listen_socket_, SOMAXCONN) < 0)
    {
        std::cerr << "Failed to listen on socket: " << strerror(errno) << std::endl;
        close(listen_socket_);
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&tcp_command_server::accept_connections, this);
    return true;
}

void tcp_command_server::stop()
{
    running_ = false;

    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto &pair : token_to_socket_map_)
    {
        if (pair.second != INVALID_SOCKET)
        {
            close(pair.second);
        }
    }
    token_to_socket_map_.clear();

    if (listen_socket_ != INVALID_SOCKET)
    {
        close(listen_socket_);
        listen_socket_ = INVALID_SOCKET;
    }

    if (accept_thread_.joinable())
    {
        accept_thread_.join();
    }

    for (size_t i = 0; i < MAX_LOCAL_STREAMING_CHANNELS; ++i)
    {
        if (i < MAX_TCP_STREAMING_CHANNELS)
        {
            tcp_streaming_stop_event_[i] = 1;
            tcp_streaming_running_[i] = false;
            for (auto &thread : tcp_worker_threads_[i])
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }
        }
        if (i < MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            local_rx_streaming_stop_event_[i] = 1;
            local_rx_streaming_running_[i] = false;

            for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i + LOCAL_RX_STREAMING_START_CHANNEL].dma_descriptors_count; ++d)
            {
                int event_fd = event_data_.chans[i].events[d].event_handle;
                if (event_fd != -1)
                    close(event_fd);

                if (local_rx_worker_threads_[i][d].joinable())
                {
                    local_rx_worker_threads_[i][d].join();
                }
            }
        }
        if (i >= MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            size_t tx_idx = i - MAX_LOCAL_STREAMING_RX_CHANNELS;
            local_tx_streaming_stop_event_[tx_idx] = 1;
            local_tx_streaming_running_[tx_idx] = false;
            for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i].dma_descriptors_count; ++d)
            {
                if (local_tx_worker_threads_[tx_idx][d].joinable())
                {
                    local_tx_worker_threads_[tx_idx][d].join();
                }
            }
        }
    }

    driver_interface_.clear_mmap();
    driver_interface_.start_stop_DMA_global(false, true);
    driver_interface_.start_stop_DMA_global(false, false);
}

void tcp_command_server::register_callback(uint32_t opcode, command_callback callback)
{
    command_callbacks_[opcode] = std::move(callback);
}

void tcp_command_server::accept_connections()
{
    while (running_)
    {
        int client_socket = accept(listen_socket_, nullptr, nullptr);
        if (client_socket == INVALID_SOCKET)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                std::cerr << "Accept failed: " << strerror(errno) << std::endl;
            }
            continue;
        }
        std::thread client_thread(&tcp_command_server::handle_client, this, client_socket);
        client_thread.detach();
    }
}

void tcp_command_server::handle_client(int client_socket)
{
    while (running_)
    {
        char buffer[1024];
        ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer), 0);
        if (bytes_read > 0)
        {
            std::string command_data(buffer, bytes_read);
            uint32_t opcode = *reinterpret_cast<const uint32_t *>(command_data.data());

            auto callback_it = command_callbacks_.find(opcode);
            if (callback_it != command_callbacks_.end())
            {
                callback_it->second(client_socket, command_data);
            }
            else
            {
                std::cerr << "Unknown command opcode: " << opcode << std::endl;
                uint32_t token = *reinterpret_cast<const uint32_t *>(command_data.data() + sizeof(uint32_t));
                send_error_code(token, opcode, SAMPLER_ERROR_GENERAL_FAILED);
            }
        }
        else if (bytes_read == 0)
        {
            break;
        }
        else
        {
            std::cerr << "Error receiving data from client: " << strerror(errno) << std::endl;
            break;
        }
    }
    close(client_socket);
}

uint32_t tcp_command_server::generate_token_id(uint32_t ip_address, uint16_t port, int client_socket)
{
    uint32_t token_id = (ip_address << 16) | port;
    return token_id + static_cast<uint32_t>(client_socket);
}

void tcp_command_server::retrieve_ip_and_port_from_token(uint32_t token_id, uint32_t *ip_address, uint16_t *port, int client_socket)
{
    uint32_t adjusted_token = token_id - static_cast<uint32_t>(client_socket);
    *ip_address = adjusted_token >> 16;
    *port = static_cast<uint16_t>(adjusted_token & 0xFFFF);
}

bool tcp_command_server::send_data(uint32_t token_id, const std::string &data)
{
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = token_to_socket_map_.find(token_id);
    if (it == token_to_socket_map_.end())
    {
        std::cerr << "No client associated with token ID " << token_id << std::endl;
        return false;
    }
    int client_socket = it->second;
    ssize_t bytes_sent = send(client_socket, data.c_str(), data.length(), 0);
    if (bytes_sent < 0)
    {
        std::cerr << "Failed to send data to client with token ID " << token_id << ": " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void tcp_command_server::send_error_code(uint32_t token_id, uint32_t opcode, sampler_error_code error_code)
{
    sampler_system_general_res error_response = {};
    error_response.opcode = opcode;
    error_response.status = error_code;
    std::string response(reinterpret_cast<char *>(&error_response), sizeof(error_response));
    send_data(token_id, response);
}

void tcp_command_server::handle_sampler_connect(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_connect_command))
    {
        std::cerr << "Invalid command data size for SAMPLER_Connect" << std::endl;
        return;
    }

    const sampler_connect_command *command = reinterpret_cast<const sampler_connect_command *>(command_data.data());
    uint32_t token_id = generate_token_id(command->ip_address, command->port, client_socket);
    std::string ip_address =
        std::to_string(command->ip_address & 0xFF) + '.' +
        std::to_string((command->ip_address >> 8) & 0xFF) + '.' +
        std::to_string((command->ip_address >> 16) & 0xFF) + '.' +
        std::to_string((command->ip_address >> 24) & 0xFF);

    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        connections_[token_id] = ip_address + ":" + std::to_string(ntohs(command->port));
        token_to_socket_map_[token_id] = client_socket;
        driver_interface_.read_DMA_memory_map_and_event_handles(dma_param_, memory_data_, event_data_);
        driver_interface_.init_mmap(memory_data_);
    }

    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_CONNECT_RES;
    command_response.token_id = token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(token_id, response);
    std::cout << "Received SAMPLER_Connect command: IP = " << ip_address << ", port = " << ntohs(command->port) << std::endl;
}

void tcp_command_server::handle_sampler_dma_system_init(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_system_dma_init_command))
    {
        std::cerr << "Invalid command data size for SAMPLER_SystemInit" << std::endl;
        return;
    }

    const sampler_system_dma_init_command *command = reinterpret_cast<const sampler_system_dma_init_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLERSystemDmaInitCommand" << std::endl;
            return;
        }
    }
    std::string config_registers_file_path = get_executable_directory() + "/configRegisters.txt";
    std::vector<register_data> register_data = config_reader::read_register_data(config_registers_file_path);

    initialize_dma(register_data, command);

    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_INIT_DMA_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);

    std::cout << "Received SAMPLERSystemDmaInitCommand command: Token ID = " << command->token_id
              << ", DMA Channels = " << command->dma_channels
              << ", Descriptors Num = " << command->descriptors_num
              << ", Descriptors Size = " << command->descriptors_size << std::endl;
}

void tcp_command_server::handle_sampler_start_tcp_streaming(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_start_tcp_streaming_command))
    {
        std::cerr << "Invalid command data size for SAMPLER_StartTcpStreaming" << std::endl;
        return;
    }

    const sampler_start_tcp_streaming_command *command = reinterpret_cast<const sampler_start_tcp_streaming_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLER_StartTcpStreaming" << std::endl;
            return;
        }
    }

    config_tcp config;
    build_config_from_command(*command, config);

    for (size_t i = 0; i < MAX_TCP_STREAMING_CHANNELS; ++i)
    {
        tcp_streaming_running_[i] = command->channels[i];
        if (command->channels[i])
        {
            tcp_streaming_stop_event_[i] = 0;
        }
    }

    for (uint32_t tcp_channel_idx = 0; tcp_channel_idx < config.server_ips.size(); ++tcp_channel_idx)
    {
        if (!command->channels[tcp_channel_idx])
            continue;
        uint32_t dma_channel_idx = TCP_STREAMING_START_CHANNEL + tcp_channel_idx;
        for (uint32_t descriptor_idx = 0; descriptor_idx < MAX_NUM_DESCRIPTORS; ++descriptor_idx)
        {
            tcp_worker_threads_[tcp_channel_idx][descriptor_idx] = std::thread([this, tcp_channel_idx, dma_channel_idx, descriptor_idx, config]()
                                                                               {
                uint64_t virtual_address = memory_data_.chans[dma_channel_idx].descs[descriptor_idx].buf_va;
                tcp_client tcp_client(config.server_ips[tcp_channel_idx], config.ports[tcp_channel_idx]);
                tcp_client.connect();
                while (tcp_streaming_running_[tcp_channel_idx]) {
                    if (tcp_streaming_stop_event_[tcp_channel_idx]) {
                        break;
                    }

                    char* data_buffer = reinterpret_cast<char*>(virtual_address);

                    pmessage_header header = reinterpret_cast<pmessage_header>(virtual_address);
    
                    if (header->prefix1 == 0xaaaaaaaa) {
                        std::cout << "TCP: DMA event signaled for channel " << dma_channel_idx 
                                    << ", descriptor " << descriptor_idx 
                                    << ", header->msg_id = " << header->msg_id 
                                    << ", virtualAddress = 0x" << std::hex << virtual_address << std::endl;
                        
                        size_t data_size = static_cast<size_t>(header->size);
                        if (tcp_client.is_connected()) {
                            int res = tcp_client.send_data(data_buffer, data_size);
                            if (res < 0) {
                                tcp_client.connect();
                                if (tcp_client.is_connected()) {
                                    res = tcp_client.send_data(data_buffer, data_size);
                                }
                            }
                            if (res < 0) {
                                std::cerr << "Failed to send data on tcp channel " << tcp_channel_idx << std::endl;
                            } else {
                                std::cout << "Data sent successfully on tcp channel " << tcp_channel_idx << std::endl;
                            }
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                } });

            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(command->cpu_id % std::thread::hardware_concurrency(), &cpuset);
            pthread_setaffinity_np(tcp_worker_threads_[tcp_channel_idx][descriptor_idx].native_handle(),
                                   sizeof(cpu_set_t), &cpuset);
        }
    }

    sampler_system_general_res command_response = {SAMPLER_CONFIG_START_TCP_STREAMING_RES, command->token_id, SAMPLER_SUCCESS};
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "TCP streaming started for Token ID: " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_start_dma_configuration(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_generic_command))
    {
        std::cerr << "Invalid command data size for SAMPLERStartDmaGlobalCommand" << std::endl;
        return;
    }

    const sampler_generic_command *command = reinterpret_cast<const sampler_generic_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStartDma" << std::endl;
            return;
        }
    }
    driver_interface_.start_DMA_configure(start_dma_configuration_, memory_data_);

    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_START_DMA_CONFIGURATION_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);

    std::cout << "Received SAMPLERStartDmaGlobalCommand command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_start_local_streaming(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_start_local_streaming_command))
    {
        std::cerr << "Invalid command data size for SAMPLER_StartLocalStreaming" << std::endl;
        return;
    }

    const sampler_start_local_streaming_command *command = reinterpret_cast<const sampler_start_local_streaming_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLER_StartLocalStreaming" << std::endl;
            return;
        }
    }

    for (size_t i = 0; i < MAX_LOCAL_STREAMING_CHANNELS; ++i)
    {
        if (i < MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            if (local_rx_streaming_running_[i] && command->channels[i])
            {
                local_rx_streaming_stop_event_[i] = 1;
                local_rx_streaming_running_[i] = false;
                for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i + LOCAL_RX_STREAMING_START_CHANNEL].dma_descriptors_count; ++d)
                {
                    if (local_rx_worker_threads_[i][d].joinable())
                    {
                        local_rx_worker_threads_[i][d].join();
                    }
                }
            }
            local_rx_streaming_running_[i] = command->channels[i];
            if (command->channels[i])
            {
                local_rx_streaming_stop_event_[i] = 0;
            }
        }
        else
        {
            size_t tx_idx = i - MAX_LOCAL_STREAMING_RX_CHANNELS;
            if (local_tx_streaming_running_[tx_idx] && command->channels[i])
            {
                local_tx_streaming_stop_event_[tx_idx] = 1;
                local_tx_streaming_running_[tx_idx] = false;
                for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i].dma_descriptors_count; ++d)
                {
                    if (local_tx_worker_threads_[tx_idx][d].joinable())
                    {
                        local_tx_worker_threads_[tx_idx][d].join();
                    }
                }
            }
            local_tx_streaming_running_[tx_idx] = command->channels[i];
            if (command->channels[i])
            {
                local_tx_streaming_stop_event_[tx_idx] = 0;
            }
        }
    }

    uint32_t test_id = command->test_id;
    for (uint32_t local_channel_idx = 0; local_channel_idx < MAX_LOCAL_STREAMING_CHANNELS; ++local_channel_idx)
    {
        if (!command->channels[local_channel_idx])
            continue;
        uint32_t dma_channel_idx = local_channel_idx;
        for (uint32_t descriptor_idx = 0; descriptor_idx < MAX_NUM_DESCRIPTORS; ++descriptor_idx)
        {
            int event_fd = event_data_.chans[local_channel_idx].events[descriptor_idx].event_handle;
            if (local_channel_idx < MAX_LOCAL_STREAMING_RX_CHANNELS)
            {
                local_rx_worker_threads_[local_channel_idx][descriptor_idx] = std::thread([this, test_id, local_channel_idx, dma_channel_idx, descriptor_idx, event_fd]()
                                                                                          {
                    uint64_t virtual_address = memory_data_.chans[dma_channel_idx].descs[descriptor_idx].buf_va;
                    uint64_t ph_address = memory_data_.chans[dma_channel_idx].descs[descriptor_idx].buf_pa;
                    uint32_t last_seq_id = 0;

                    struct pollfd pfd;
                    pfd.fd = event_fd;
                    pfd.events = POLLIN;

                    while (local_rx_streaming_running_[local_channel_idx]) {
                        if (local_rx_streaming_stop_event_[local_channel_idx]) {
                            break;
                        }
                        
                        int ret = poll(&pfd, 1, -1);
                        if (ret > 0 && (pfd.revents & POLLIN)) 
                        {
                            uint64_t value;
                            read(event_fd, &value, sizeof(value));

                            char* data_buffer = reinterpret_cast<char*>(virtual_address);

                            size_t data_size = 0;
                            pmessage_header header = reinterpret_cast<pmessage_header>(virtual_address);
    
                            if (header->prefix1 == 0xaaaaaaaa) {
                                std::cout << "Local: DMA event signaled for channel " << dma_channel_idx 
                                            << ", descriptor " << descriptor_idx 
                                            << ", header->msg_id = " << header->msg_id 
                                            << ", virtualAddress = 0x" << std::hex << virtual_address << std::endl;
                                
                                if (last_seq_id && (header->msg_id - last_seq_id) != MAX_NUM_DESCRIPTORS) {
                                    std::cerr << "Error in data: header->DMA_id expected: 0x" << std::hex 
                                                << (last_seq_id + MAX_NUM_DESCRIPTORS) << " received: 0x" << header->msg_id << std::endl;
                                }
                                last_seq_id = header->msg_id;
                                data_size = static_cast<size_t>(header->size);
    
                                if (dma_channel_idx == 8 && test_id == SAMPLER_TEST_TX_CP_CH_8_TO_CH_12) {
                                    uint64_t virtual_address_tx = virtual_address;
                                    char* data_buffer_dest = reinterpret_cast<char*>(virtual_address_tx);
                                    memcpy(data_buffer_dest, data_buffer, data_size);
                                }
    
                                if (header->is_counter_mode == 1) {
                                    int offset = 0x140;
                                    bool result = verify_data_integrity(data_buffer, data_size, offset);
                                    std::cout << "Data integrity check " << (result ? "passed" : "failed") << std::endl;
                                }
                            }
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    } });

                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(command->start_cpu_id + dma_channel_idx % std::thread::hardware_concurrency(), &cpuset);
                pthread_setaffinity_np(local_rx_worker_threads_[local_channel_idx][descriptor_idx].native_handle(),
                                       sizeof(cpu_set_t), &cpuset);
            }
            else
            {
                uint32_t tx_idx = local_channel_idx - MAX_LOCAL_STREAMING_RX_CHANNELS;
                local_tx_worker_threads_[tx_idx][descriptor_idx] = std::thread([this, tx_idx, dma_channel_idx, descriptor_idx, event_fd]()
                                                                               {
                    uint64_t virtual_address = memory_data_.chans[dma_channel_idx].descs[descriptor_idx].buf_va;

                    struct pollfd pfd;
                    pfd.fd = event_fd;
                    pfd.events = POLLIN;

                    while (local_tx_streaming_running_[tx_idx]) {

                        int ret = poll(&pfd, 1, -1);
                        if (ret > 0 && (pfd.revents & POLLIN)) {
                            uint64_t value;
                            if (read(event_fd, &value, sizeof(value)) != sizeof(value)) {
                                if (local_tx_streaming_stop_event_[tx_idx]) {
                                    break;
                                }
                            } else {
                                std::cout << "Event received! Value: " << value << std::endl;
                            }
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    } });

                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(command->start_cpu_id + dma_channel_idx % std::thread::hardware_concurrency(), &cpuset);
                pthread_setaffinity_np(local_tx_worker_threads_[tx_idx][descriptor_idx].native_handle(),
                                       sizeof(cpu_set_t), &cpuset);
            }
        }
    }

    sampler_system_general_res command_response = {SAMPLER_CONFIG_START_LOCAL_STREAMING_RX_RES, command->token_id, SAMPLER_SUCCESS};
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Local streaming started for Token ID: " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_stop_tcp_streaming(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_stop_tcp_streaming_command))
    {
        std::cerr << "Invalid command data size for SAMPLERStopTcpStreamingCommand" << std::endl;
        return;
    }

    const sampler_stop_tcp_streaming_command *command = reinterpret_cast<const sampler_stop_tcp_streaming_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStopTcpStreaming" << std::endl;
            return;
        }
    }

    for (size_t i = 0; i < MAX_TCP_STREAMING_CHANNELS; ++i)
    {
        if (command->channels[i])
        {
            tcp_streaming_stop_event_[i] = 1;
            tcp_streaming_running_[i] = false;
            for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i + TCP_STREAMING_START_CHANNEL].dma_descriptors_count; ++d)
            {
                if (tcp_worker_threads_[i][d].joinable())
                {
                    tcp_worker_threads_[i][d].join();
                }
            }
        }
    }

    sampler_system_general_res command_response = {SAMPLER_CONFIG_STOP_TCP_STREAMING_RES, command->token_id, SAMPLER_SUCCESS};
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "TCP streaming stopped for Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_stop_local_streaming(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_stop_local_streaming_command))
    {
        std::cerr << "Invalid command data size for SAMPLERStopLocalRxStreamingCommand" << std::endl;
        return;
    }

    const sampler_stop_local_streaming_command *command = reinterpret_cast<const sampler_stop_local_streaming_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStopLocalRxStreaming" << std::endl;
            return;
        }
    }

    for (size_t i = 0; i < MAX_LOCAL_STREAMING_CHANNELS; ++i)
    {
        if (i < MAX_LOCAL_STREAMING_RX_CHANNELS)
        {
            if (command->channels[i])
            {
                local_rx_streaming_stop_event_[i] = 1;
                local_rx_streaming_running_[i] = false;
                for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i + LOCAL_RX_STREAMING_START_CHANNEL].dma_descriptors_count; ++d)
                {
                    if (local_rx_worker_threads_[i][d].joinable())
                    {
                        local_rx_worker_threads_[i][d].join();
                    }
                }
            }
        }
        else
        {
            size_t tx_idx = i - MAX_LOCAL_STREAMING_RX_CHANNELS;
            if (command->channels[i])
            {
                local_tx_streaming_stop_event_[tx_idx] = 1;
                local_tx_streaming_running_[tx_idx] = false;
                for (size_t d = 0; d < start_dma_configuration_.start_dma_channels[i].dma_descriptors_count; ++d)
                {
                    if (local_tx_worker_threads_[tx_idx][d].joinable())
                    {
                        local_tx_worker_threads_[tx_idx][d].join();
                    }
                }
            }
        }
    }

    sampler_system_general_res command_response = {SAMPLER_CONFIG_STOP_LOCAL_STREAMING_RX_RES, command->token_id, SAMPLER_SUCCESS};
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Local rx streaming stopped for Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_stop_rx_dma_global(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_generic_command))
    {
        std::cerr << "Invalid command data size for HandleSAMPLERStopRxDmaGlobal" << std::endl;
        return;
    }

    const sampler_generic_command *command = reinterpret_cast<const sampler_generic_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStopRxDmaGlobal" << std::endl;
            return;
        }
    }
    driver_interface_.start_stop_DMA_global(false, true);
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_STOP_DMA_GLOBAL_RX_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received HandleSAMPLERStopRxDmaGlobal command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_stop_tx_dma_global(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_generic_command))
    {
        std::cerr << "Invalid command data size for HandleSAMPLERStopTxDmaGlobal" << std::endl;
        return;
    }

    const sampler_generic_command *command = reinterpret_cast<const sampler_generic_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStopTxDmaGlobal" << std::endl;
            return;
        }
    }
    driver_interface_.start_stop_DMA_global(false, false);
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_STOP_DMA_GLOBAL_TX_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received HandleSAMPLERStopTxDmaGlobal command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_start_rx_dma_global(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_generic_command))
    {
        std::cerr << "Invalid command data size for HandleSAMPLERStartDmaGlobal" << std::endl;
        return;
    }

    const sampler_generic_command *command = reinterpret_cast<const sampler_generic_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStartDmaGlobal" << std::endl;
            return;
        }
    }
    driver_interface_.start_stop_DMA_global(true, true);
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_START_DMA_GLOBAL_RX_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received HandleSAMPLERStartDmaGlobal command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_start_tx_dma_global(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_generic_command))
    {
        std::cerr << "Invalid command data size for HandleSAMPLERStartDmaGlobal" << std::endl;
        return;
    }

    const sampler_generic_command *command = reinterpret_cast<const sampler_generic_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStartTxDmaGlobal" << std::endl;
            return;
        }
    }
    driver_interface_.start_stop_DMA_global(true, false);
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_START_DMA_GLOBAL_TX_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received HandleSAMPLERStartTxDmaGlobal command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_start_list_dma_channels(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_start_list_dma_channels_command))
    {
        std::cerr << "Invalid command data size for SAMPLERStartListDmaChannelsCommand" << std::endl;
        return;
    }

    const sampler_start_list_dma_channels_command *command = reinterpret_cast<const sampler_start_list_dma_channels_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for HandleSAMPLERStartListDmaChannels" << std::endl;
            return;
        }
    }
    for (uint8_t i = 0; i < start_dma_configuration_.dma_channels_count; i++)
    {
        if (command->channels[i])
        {
            driver_interface_.start_stop_DMA_channel(i, true, true);
        }
        uint32_t addr = (0x0014 + (i * 0x10)) * 4;
        driver_interface_.write_register(1, addr, command->is_counter_channels[i] ? 2 : 0);
    }
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_START_DMA_LIST_CHANNELS_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received SAMPLERStartListDmaChannelsCommand command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::handle_sampler_stop_list_dma_channels(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_stop_list_dma_channels_command))
    {
        std::cerr << "Invalid command data size for SAMPLERStopListDmaChannelsCommand" << std::endl;
        return;
    }

    const sampler_stop_list_dma_channels_command *command = reinterpret_cast<const sampler_stop_list_dma_channels_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLERStopListDmaChannelsCommand" << std::endl;
            return;
        }
    }
    for (uint8_t i = 0; i < start_dma_configuration_.dma_channels_count; i++)
    {
        if (command->channels[i])
        {
            driver_interface_.start_stop_DMA_channel(i, false, false);
        }
    }
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_SYSTEM_STOP_DMA_LIST_CHANNELS_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received HandleSAMPLERStopListDmaChannels command: Token ID = " << command->token_id << std::endl;
}

void tcp_command_server::initialize_dma(const std::vector<register_data> &register_data, const sampler_system_dma_init_command *init_dma_command)
{
    if (!init_dma_command)
    {
        std::cerr << "Invalid initDmaCommand " << std::endl;
        return;
    }

    for (const auto &reg_data : register_data)
    {
        driver_interface_.write_register(1, reg_data.address, reg_data.value);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    start_dma_configuration_.dma_channels_count = std::min(dma_param_.dma_chan_count, static_cast<uint32_t>(MAX_NUM_CHANNELS));
    start_dma_configuration_.start_cycle = 1;

    for (uint32_t channel = 0; channel < dma_param_.dma_chan_count; channel++)
    {
        if (channel < start_dma_configuration_.dma_channels_count)
        {
            start_dma_configuration_.start_dma_channels[channel].dma_descriptors_count =
                std::min(dma_param_.dma_desc_count, static_cast<uint32_t>(MAX_NUM_DESCRIPTORS));
        }
        else
        {
            start_dma_configuration_.start_dma_channels[channel].dma_descriptors_count = 0;
            continue;
        }

        for (uint32_t descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; descriptor++)
        {
            start_dma_configuration_.start_dma_channels[channel].start_dma_descriptors[descriptor].dma_descriptor_buffer_size =
                std::min(dma_param_.dma_buf_size, static_cast<uint32_t>(DESCRIPTOR_BUF_SIZE));
            start_dma_configuration_.start_dma_channels[channel].start_dma_descriptors[descriptor].is_descriptor_interrupt_enable = 1;
        }
    }
}

/*
void tcp_command_server::send_data_to_client(uint32_t token_id, int channel_index, const char *data_buffer, size_t data_size)
{
    auto it = streaming_servers_.find(token_id);
    if (it != streaming_servers_.end())
    {
        tcp_server *tcp_server = it->second.get();
        if (tcp_server->is_channel_connected(channel_index))
        {
            int res = tcp_server->send_data(channel_index, data_buffer, data_size);
            if (res < 0)
            {
                std::cerr << "Failed to send data on channel " << channel_index << std::endl;
            }
            else
            {
                std::cout << "Data sent successfully on channel " << channel_index << std::endl;
            }
        }
        else
        {
            std::cout << "Channel " << channel_index << " is not connected. Data not sent." << std::endl;
        }
    }
    else
    {
        std::cerr << "No tcp_server found for token ID: " << token_id << std::endl;
    }
}
*/

void tcp_command_server::handle_sampler_read_register(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_read_register_command))
    {
        std::cerr << "Invalid command data size for SAMPLERReadRegisterCommand" << std::endl;
        return;
    }

    const sampler_read_register_command *command = reinterpret_cast<const sampler_read_register_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLERReadRegisterCommand" << std::endl;
            return;
        }
    }

    uint32_t read_value = driver_interface_.read_register(1, command->address);
    sampler_read_register_command_res command_response = {};
    command_response.gen_res.opcode = SAMPLER_CONFIG_READ_REG_BAR1_RES;
    command_response.gen_res.token_id = command->token_id;
    command_response.address = command->address;
    command_response.value = read_value;
    command_response.gen_res.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received SAMPLERReadRegisterCommand command: Token ID = " << command->token_id
              << ", Register Address = " << command->address
              << ", Register Value = " << read_value << std::endl;
}

void tcp_command_server::handle_sampler_write_register(int client_socket, const std::string &command_data)
{
    if (command_data.size() < sizeof(sampler_write_register_command))
    {
        std::cerr << "Invalid command data size for SAMPLERWriteRegisterCommand" << std::endl;
        return;
    }

    const sampler_write_register_command *command = reinterpret_cast<const sampler_write_register_command *>(command_data.data());
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(command->token_id);
        if (it == connections_.end())
        {
            std::cerr << "Invalid token ID for SAMPLERWriteRegisterCommand" << std::endl;
            return;
        }
    }

    driver_interface_.write_register(1, command->address, command->value);
    sampler_system_general_res command_response = {};
    command_response.opcode = SAMPLER_CONFIG_WRITE_REG_BAR1_RES;
    command_response.token_id = command->token_id;
    command_response.status = SAMPLER_SUCCESS;
    std::string response(reinterpret_cast<char *>(&command_response), sizeof(command_response));
    send_data(command->token_id, response);
    std::cout << "Received SAMPLERWriteRegisterCommand command: Token ID = " << command->token_id
              << ", Register Address = " << command->address
              << ", Write Value = " << command->value << std::endl;
}

void tcp_command_server::build_config_from_command(const sampler_start_tcp_streaming_command &command, config_tcp &config)
{
    config.server_ips.clear();
    config.ports.clear();
    for (int i = 0; i < MAX_TCP_STREAMING_CHANNELS; ++i)
    {
        if (command.channels[i])
        {
            std::string ip = command.ip_addresses[i];
            ip.erase(std::remove_if(ip.begin(), ip.end(),
                [](unsigned char c) { return std::isspace(c); }),
                ip.end());
            config.server_ips.push_back(ip);
            config.ports.push_back(command.ports[i]);
        }
    }
}

bool tcp_command_server::verify_data_integrity(char *data_buffer, size_t data_size, int offset)
{
    if (data_size < static_cast<size_t>(offset))
    {
        std::cerr << "Offset is out of the data bounds." << std::endl;
        return false;
    }

    const size_t max_check_size = SIZE_2MB;
    size_t max_index = data_size - offset;
    size_t max_check_index = std::min(max_index, max_check_size);

    std::default_random_engine generator(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<size_t> distribution(0, max_index - max_check_index);
    size_t random_start = distribution(generator);
    random_start = (random_start / 4) * 4;
    size_t check_end_index = random_start + max_check_index;

    int counter = *reinterpret_cast<int *>(data_buffer + random_start + offset);
    for (size_t i = random_start; i < check_end_index; i += 4)
    {
        int byte_verify = *reinterpret_cast<int *>(data_buffer + i + offset);
        if (byte_verify != counter)
        {
            std::cerr << "Error in data: offset 0x" << std::hex << (i + offset)
                      << " from 0x" << max_index
                      << ": expected " << counter << "(0x" << counter
                      << "), received " << byte_verify << "(0x" << byte_verify << ")\n";
            return false;
        }
        counter++;
    }
    return true;
}

/*
void tcp_command_server::print_message_header_from_buffer(const char *data_buffer)
{
    const MessageHeader *header = reinterpret_cast<const MessageHeader *>(data_buffer);
    std::cout << std::hex << std::setfill('0');
    std::cout << "prefix1: 0x" << std::setw(8) << header->prefix1 << std::endl;
    std::cout << "src_id: 0x" << std::setw(2) << static_cast<int>(header->src_id) << std::endl;
    std::cout << "dst_id: 0x" << std::setw(2) << static_cast<int>(header->dst_id) << std::endl;
    std::cout << "size: 0x" << std::setw(8) << header->size << std::endl;
    std::cout << "type: 0x" << std::setw(4) << header->type << std::endl;
    std::cout << "msg_id: 0x" << std::setw(8) << header->msg_id << std::endl;
    std::cout << "spare1: ";
    for (int i = 0; i < 48; ++i)
    {
        std::cout << std::setw(2) << static_cast<int>(header->spare1[i]) << " ";
    }
    std::cout << std::endl;
    std::cout << "prefix2: 0x" << std::setw(8) << header->prefix2 << std::endl;
    std::cout << "dwell_id: 0x" << std::setw(16) << header->dwell_id << std::endl;
    std::cout << "port_id: 0x" << std::setw(8) << header->port_id << std::endl;
    std::cout << "sampling_rate_hz: 0x" << std::setw(16) << header->sampling_rate_hz << std::endl;
    std::cout << "center_frequency_hz: 0x" << std::setw(16) << header->center_frequency_hz << std::endl;
    std::cout << "number_of_samples_dwell: 0x" << std::setw(8) << header->number_of_samples_dwell << std::endl;
    std::cout << "is_counter_mode: 0x" << std::setw(2) << static_cast<int>(header->is_counter_mode) << std::endl;
    std::cout << "snapshots_rate: 0x" << std::setw(2) << static_cast<int>(header->snapshots_rate) << std::endl;
    std::cout << "msg_type: 0x" << std::setw(2) << static_cast<int>(header->msg_type) << std::endl;
    std::cout << "dma_id: 0x" << std::setw(2) << static_cast<int>(header->dma_id) << std::endl;
    std::cout << "fpga_reserved: ";
    for (int i = 0; i < 88; ++i)
    {
        std::cout << std::setw(2) << static_cast<int>(header->fpga_reserved[i]) << " ";
    }
    std::cout << std::endl;
    std::cout << "reserved: ";
    for (int i = 0; i < 128; ++i)
    {
        std::cout << std::setw(2) << static_cast<int>(header->reserved[i]) << " ";
    }
    std::cout << std::endl;
}
*/
