cmake_minimum_required(VERSION 3.16)
project(unilidar_sdk2_package_consumer LANGUAGES CXX)

find_package(unilidar_sdk2 CONFIG REQUIRED)

add_executable(unilidar_sdk2_package_consumer main.cpp)
target_link_libraries(unilidar_sdk2_package_consumer PRIVATE unilidar::sdk2)
