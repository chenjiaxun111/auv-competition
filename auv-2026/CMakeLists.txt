cmake_minimum_required(VERSION 3.5)
project(auv-2026 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(lib/easylogger/inc)
include_directories(lib/libev-4.22)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

aux_source_directory(applications application_srcs)
aux_source_directory(connections connections_srcs)
aux_source_directory(control control_srcs)
aux_source_directory(drivers driver_srcs)
aux_source_directory(user user_srcs)
# 当前未调用，保留给未来的传感器滤波处理。
aux_source_directory(utils utils_srcs)

# 直接编译项目自带的libev源码，不依赖预先安装或单独生成的libev库。
add_library(rov_libev STATIC
    lib/libev-4.22/ev.c
    lib/libev-4.22/event.c)
target_compile_definitions(rov_libev PRIVATE EV_STANDALONE=1)
target_include_directories(rov_libev PUBLIC lib/libev-4.22)
target_link_libraries(rov_libev PRIVATE m)

add_executable(auvmaster     	${application_srcs}
				${connections_srcs}
				${control_srcs}
				${driver_srcs}
				${user_srcs}
				${utils_srcs})

target_compile_options(auvmaster PRIVATE -Wall -Wextra)

target_link_libraries(auvmaster
			m
			pthread
			wiringPi
			easylogger
			rov_libev
			)
