cmake_minimum_required(VERSION 3.10)
project(eo_as_driver LANGUAGES C)

include_directories(${CMAKE_SOURCE_DIR}/../include)

# Use CMake to run `uname -r` once at configure time, capturing the output
execute_process(
    COMMAND uname -r
    OUTPUT_VARIABLE KERNEL_RELEASE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Now define the kernel build dir using that captured string
set(KERNEL_BUILD_DIR "/lib/modules/${KERNEL_RELEASE}/build" CACHE PATH "Kernel build directory")

# The rest of your config
set(DRIVER_SOURCES
    eo_as_pcie.c
    eo_as_dma.c
    eo_as_ioctl.c
    lin_isr.c
    hal_hwlayer.c
)

add_custom_command(
    OUTPUT eo_as_pci_driver.ko
    COMMAND make
      -C ${KERNEL_BUILD_DIR}
      M=${CMAKE_CURRENT_SOURCE_DIR}
      modules
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${DRIVER_SOURCES}
    COMMENT "Building eo_as_pci_driver.ko out-of-tree on Jetson"
)

add_custom_target(eo_as_driver ALL
    DEPENDS eo_as_pci_driver.ko
    COMMENT "Build target for eo_as_pci_driver.ko"
)

add_custom_target(eo_as_clean
    COMMAND make
      -C ${KERNEL_BUILD_DIR}
      M=${CMAKE_CURRENT_SOURCE_DIR}
      clean
    COMMENT "Cleaning driver build"
)
