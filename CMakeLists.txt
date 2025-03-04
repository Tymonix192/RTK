# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(UdpUartBridge VERSION 1.0)

# Specify the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Add the executable target
add_executable(udp_uart_bridge rtk_pi.c)

# Link the pthread library for threading support
find_package(Threads REQUIRED)
target_link_libraries(udp_uart_bridge PRIVATE Threads::Threads)

# Optionally, set compile options (e.g., warnings)
target_compile_options(udp_uart_bridge PRIVATE -Wall -Wextra -Wpedantic)


# Set the output directory for the executable (optional)
set_target_properties(udp_uart_bridge PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)