#include "tcp_client.h"
#include <iostream>
#include <cstring> // for std::memset
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// Example buffer sizes & keepalive intervals
static const uint32_t g_socket_send_size   = 10485760; // 10MB send buffer
static const uint32_t g_keepalive_time     = 1000;     // 1 sec
static const uint32_t g_keepalive_interval = 1000;     // 1 sec

tcp_client::tcp_client(const std::string& server_ip, int port)
    : server_ip_(server_ip), port_(port), socket_(-1)
{
}

tcp_client::~tcp_client() {
    close();
}

bool tcp_client::connect() {
    // Close existing if valid
    if (socket_ != -1) {
        ::close(socket_);
        socket_ = -1;
    }

    // Create new socket
    socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == -1) {
        std::cerr << "Socket creation failed, errno=" << errno << std::endl;
        return false;
    }

    // Non-blocking mode for connect
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
    // Set up server address
    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid server IP: " << server_ip_ << std::endl;
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    // Attempt connect
    int result = ::connect(socket_, (sockaddr*)&server_addr, sizeof(server_addr));
    if (result == -1 && errno != EINPROGRESS) {
        std::cerr << "Connect failed immediately, errno=" << errno << std::endl;
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    if (result == -1) {
        // Wait for connect with select()
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(socket_, &write_set);

        struct timeval timeout;
        timeout.tv_sec  = 2;  // 2s
        timeout.tv_usec = 0;

        int sel_res = select(socket_ + 1, nullptr, &write_set, nullptr, &timeout);
        if (sel_res <= 0) {
            std::cerr << "Connect timed out or failed, errno=" << errno << std::endl;
            ::close(socket_);
            socket_ = -1;
            return false;
        }

        // Check if connect was truly successful
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
            std::cerr << "Connect failed, SO_ERROR=" << error << std::endl;
            ::close(socket_);
            socket_ = -1;
            return false;
        }
    }

    // Back to blocking
    fcntl(socket_, F_SETFL, flags);

    configure_socket_options();
    return true;
}

void tcp_client::configure_socket_options() {
    // Keepalive
    int keepalive = 1;
    if (setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
        std::cerr << "Failed to set SO_KEEPALIVE, errno=" << errno << std::endl;
    }

    int keepalive_time = g_keepalive_time; // in seconds
    if (setsockopt(socket_, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_time, sizeof(keepalive_time)) < 0) {
        std::cerr << "Failed to set TCP_KEEPIDLE, errno=" << errno << std::endl;
    }

    int keepalive_interval = g_keepalive_interval; // in seconds
    if (setsockopt(socket_, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_interval, sizeof(keepalive_interval)) < 0) {
        std::cerr << "Failed to set TCP_KEEPINTVL, errno=" << errno << std::endl;
    }

    // Send buffer
    int send_buffer_size = g_socket_send_size;
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof(send_buffer_size)) < 0) {
        std::cerr << "Failed to set SO_SNDBUF, errno=" << errno << std::endl;
    }

    // Optional: enable TCP_NODELAY (disable Nagle’s algorithm)
    int tcp_no_delay = 1;
    if (setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, &tcp_no_delay, sizeof(tcp_no_delay)) < 0) {
        std::cerr << "Failed to set TCP_NODELAY, errno=" << errno << std::endl;
    }
}

int tcp_client::send_data(const char* data, size_t data_size) {
    // We want to send 2 MB in one logical message.  We do a loop in case
    // `send()` writes fewer bytes than requested.
    static const size_t CHUNK_SIZE = 2 * 1024 * 1024; // 2 MB
    // If your data_size is always exactly 2 MB, you could
    // just do one loop. But let's keep it robust:

    if (!is_connected()) {
        return -1;
    }

    size_t total_sent = 0;
    while (total_sent < data_size) {
        // We can do smaller increments if desired, but here we attempt 2MB at once
        size_t to_send = std::min(CHUNK_SIZE, data_size - total_sent);

        ssize_t res = ::send(socket_, data + total_sent, to_send, 0);
        if (res < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                // A short wait then retry
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            std::cerr << "Error sending data, errno=" << errno << std::endl;
            ::close(socket_);
            socket_ = -1;
            return -1;
        }
        total_sent += static_cast<size_t>(res);
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
    return (socket_ != -1);
}

void tcp_client::reconnect_client() {
    close();
    connect();
}
