// tcp_client.cpp
#include "tcp_client.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

static const uint32_t g_socket_send_size = 10485760;
static const uint32_t g_keepalive_time = 1000;
static const uint32_t g_keepalive_interval = 1000;

tcp_client::tcp_client(const std::string& server_ip, int port) 
    : server_ip_(server_ip), port_(port), socket_(-1) {
}

tcp_client::~tcp_client() {
    close();
}

bool tcp_client::connect() {
    // Close the existing socket if it's valid
    if (socket_ != -1) {
        ::close(socket_);
        socket_ = -1;
    }

    // Create a new socket
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == -1) {
        std::cerr << "Socket creation failed: " << std::endl;
        return false;
    }

    // Set up the server address
    sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << std::endl;
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    // Set socket to non-blocking for connection attempt
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    // Initiate connection
    int result = ::connect(socket_, (sockaddr*)&server_addr, sizeof(server_addr));
    if (result == -1 && errno != EINPROGRESS) {
        std::cerr << "Connection attempt failed immediately: " << std::endl;
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    if (result == -1) {
        // Connection attempt in progress, use select to wait with timeout
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(socket_, &write_set);

        struct timeval timeout;
        timeout.tv_sec = 2;  // 2 seconds timeout
        timeout.tv_usec = 0;

        result = select(socket_ + 1, NULL, &write_set, NULL, &timeout);
        if (result <= 0) {
            std::cerr << "Connection attempt timed out or failed: " << std::endl;
            ::close(socket_);
            socket_ = -1;
            return false;
        }

        // Check if connection was successful
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
            std::cerr << "Connection failed: " << std::endl;
            ::close(socket_);
            socket_ = -1;
            return false;
        }
    }

    // Set socket back to blocking mode
    fcntl(socket_, F_SETFL, flags);

    configure_socket_options();
    return true;
}

void tcp_client::configure_socket_options() {
    // Set keepalive options
    int keepalive = 1;
    if (setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
        std::cerr << "Failed to set keepalive: " << std::endl;
    }

    int keepalive_time = g_keepalive_time;
    if (setsockopt(socket_, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_time, sizeof(keepalive_time)) < 0) {
        std::cerr << "Failed to set keepalive time: " << std::endl;
    }

    int keepalive_interval = g_keepalive_interval;
    if (setsockopt(socket_, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_interval, sizeof(keepalive_interval)) < 0) {
        std::cerr << "Failed to set keepalive interval: " << std::endl;
    }

    // Set send buffer size
    int send_buffer_size = g_socket_send_size;
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof(send_buffer_size)) < 0) {
        std::cerr << "Failed to set send buffer size: " << std::endl;
    }
}

int tcp_client::send_data(const char* data, size_t data_size) {
    const size_t chunk_size = 32 * 1024 * 1024; // 32MB chunk size
    const int max_retries = 3;
    const int base_delay_ms = 100;
    const int send_timeout_sec = 10;

    std::lock_guard<std::mutex> lock(mutex_);

    // Set send timeout
    struct timeval send_timeout;
    send_timeout.tv_sec = send_timeout_sec;
    send_timeout.tv_usec = 0;
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout)) < 0) {
        std::cerr << "Failed to set send timeout: " << std::endl;
    }

    // Enable TCP_NODELAY
    int tcp_no_delay = 1;
    if (setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, &tcp_no_delay, sizeof(tcp_no_delay)) < 0) {
        std::cerr << "Failed to set TCP_NODELAY: " << std::endl;
    }

    size_t total_sent = 0;
    int retry_count = 0;

    while (total_sent < data_size) {
        size_t current_chunk_size = std::min(chunk_size, data_size - total_sent);
        const char* current_chunk_data = data + total_sent;
        size_t chunk_sent = 0;

        while (chunk_sent < current_chunk_size) {
            ssize_t res = send(socket_, current_chunk_data + chunk_sent, 
                             current_chunk_size - chunk_sent, 0);

            if (res < 0) {
                if ((errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) && 
                    retry_count < max_retries) {
                    retry_count++;
                    int delay = base_delay_ms * (1 << retry_count);
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                    continue;
                }
                std::cerr << "Error sending data: " << std::endl;
                ::close(socket_);
                socket_ = -1;
                return -1;
            }

            chunk_sent += res;
        }

        total_sent += chunk_sent;
        retry_count = 0;
    }

    return static_cast<int>(total_sent);
}

void tcp_client::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (socket_ != -1) {
        ::close(socket_);
        socket_ = -1;
    }
}

bool tcp_client::is_connected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return socket_ != -1;
}

void tcp_client::reconnect_client() {
    close();
    connect();
}