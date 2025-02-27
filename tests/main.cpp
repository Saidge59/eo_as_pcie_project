#include "tcp_commands_server.h"
#include <iostream>
#include <unistd.h>

int main() {
    shared_state dummy_state; // Placeholder; replace with actual implementation
    const char *devicePath = "/dev/eo_as_fpga_driver";
    tcp_command_server server(dummy_state, 3030, devicePath);

    server.register_callback(SAMPLER_CONNECT_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_connect(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_INIT_DMA_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_dma_system_init(socket, data);
        });
    /*server.register_callback(SAMPLER_CONFIG_START_TCP_STREAMING_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_tcp_streaming(socket, data);
        });*/
    server.register_callback(SAMPLER_CONFIG_STOP_TCP_STREAMING_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_stop_tcp_streaming(socket, data);
        });
    server.register_callback(SAMPLER_CONFIG_START_LOCAL_STREAMING_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_local_streaming(socket, data);
        });
    server.register_callback(SAMPLER_CONFIG_STOP_LOCAL_STREAMING_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_stop_local_streaming(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_START_DMA_CONFIGURATION_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_dma_configuration(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_STOP_DMA_GLOBAL_RX_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_stop_rx_dma_global(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_START_DMA_GLOBAL_RX_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_rx_dma_global(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_STOP_DMA_GLOBAL_TX_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_stop_tx_dma_global(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_START_DMA_GLOBAL_TX_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_tx_dma_global(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_STOP_DMA_LIST_CHANNELS_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_stop_list_dma_channels(socket, data);
        });
    server.register_callback(SAMPLER_SYSTEM_START_DMA_LIST_CHANNELS_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_start_list_dma_channels(socket, data);
        });
    server.register_callback(SAMPLER_CONFIG_READ_REG_BAR1_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_read_register(socket, data);
        });
    server.register_callback(SAMPLER_CONFIG_WRITE_REG_BAR1_REQ, 
        [&](int socket, const std::string &data) {
            server.handle_sampler_write_register(socket, data);
        });
    
    if (server.start()) {
        std::cout << "TCP command server started on port 3030" << std::endl;
    } else {
        std::cout << "Failed to start server" << std::endl;
        return 1;
    }

    std::cout << "Server is running... Press Enter to stop." << std::endl;
    std::cin.get();

    return 0;
}
