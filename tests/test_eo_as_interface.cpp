#include <iostream>
#include <string>
#include <sstream>
#include "eo_as_interface_lib.h"

void print_usage()
{
    std::cout << "Available commands:\n"
              << "  write_register <uint8_t bar> <uint64_t offset (hex)> <uint32_t value>\n"
              << "  read_register <uint8_t bar> <uint64_t offset (hex)>\n"
              //<< "  read_DMA_memory_map_and_event_handles\n"
              //<< "  start_stop_DMA_channel <uint8_t channel> <bool start> <bool cycle>\n"
              //<< "  start_stop_DMA_global <bool start> <bool rx>\n"
              //<< "  start_DMA_configure\n"
              << "  exit (to quit)\n";
}

int main()
{
    try
    {
        const char *devicePath = "/dev/eo_as_fpga_driver";
        driver_interface driver(devicePath);
        std::string command;

        print_usage();
        while (true)
        {
            std::cout << "> ";
            std::getline(std::cin, command);
            std::istringstream iss(command);
            std::string cmd;
            iss >> cmd;

            if (cmd == "write_register")
            {
                uint8_t bar;
                uint64_t offset;
                uint32_t value;
                if (iss >> bar >> std::hex >> offset >> std::dec >> value)
                {
                    driver.write_register(bar, offset, value);
                    std::cout << "Written 0x" << std::hex << value << " at offset 0x" << std::hex << offset << std::dec << "\n";
                }
                else
                {
                    std::cout << "Invalid parameters for write_register\n";
                }
            }
            else if (cmd == "read_register")
            {
                uint8_t bar;
                uint64_t offset;
                if (iss >> bar >> std::hex >> offset)
                {
                    uint32_t value = driver.read_register(bar, offset);
		    std::cout << "Read 0x" << std::hex << value << " from offset 0x" << offset << std::dec << "\n";
                }
                else
                {
                    std::cout << "Invalid parameters for read_register\n";
                }
            }
            /*else if (cmd == "read_DMA_memory_map_and_event_handles")
            {
                struct eo_as_dma_params dmaParams;
                struct eo_as_mem_map memoryData;
                struct eo_as_event_data eventData;
                driver.read_DMA_memory_map_and_event_handles(dmaParams, memoryData, eventData);
                std::cout << "DMA Config: " << dmaParams.dma_chan_count << " channels\n";
            }
            else if (cmd == "start_stop_DMA_channel")
            {
                uint8_t channel;
                bool isStart, isCycle;
                if (iss >> channel >> isStart >> isCycle)
                {
                    driver.start_stop_DMA_channel(channel, isStart, isCycle);
                    std::cout << "DMA channel " << (uint8_t)channel << (isStart ? " started" : " stopped") << "\n";
                }
                else
                {
                    std::cout << "Invalid parameters for start_stop_DMA_channel\n";
                }
            }
            else if (cmd == "start_stop_DMA_global")
            {
                bool isStart, isRx;
                if (iss >> isStart >> isRx)
                {
                    driver.start_stop_DMA_global(isStart, isRx);
                    std::cout << "Global DMA " << (isRx ? "receive" : "transmit") << " " << (isStart ? "started" : "stopped") << "\n";
                }
                else
                {
                    std::cout << "Invalid parameters for start_stop_DMA_global\n";
                }
            }
            else if (cmd == "start_DMA_configure")
            {
                struct global_start_dma_configuration startDmaConfig;
                struct eo_as_mem_map memoryData;
                driver.start_DMA_configure(startDmaConfig, memoryData);
                std::cout << "DMA configured\n";
            }*/
            else if (cmd == "exit")
            {
                break;
            }
            else
            {
                std::cout << "Unknown command\n";
                print_usage();
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
