cmake_minimum_required(VERSION 3.16)

project(mcu6050Temp LANGUAGES C)

add_executable(mcu6050Temp main.c
    mpu6050.h mpu6050.c
    utils.h utils.c
)

find_library(I2C_LIB i2c)
if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found. Install libi2c-dev (Debian/Ubuntu) or i2c-tools (Arch).")
endif()

target_link_libraries(mcu6050Temp ${I2C_LIB})

include(GNUInstallDirs)
install(TARGETS mcu6050Temp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
