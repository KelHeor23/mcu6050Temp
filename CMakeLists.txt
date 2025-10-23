cmake_minimum_required(VERSION 3.16)

project(mcu6050Temp LANGUAGES C)

add_executable(mcu6050Temp main.c
    mpu6050.h mpu6050.c
    utils.h utils.c
)

include(GNUInstallDirs)
install(TARGETS mcu6050Temp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
