cmake_minimum_required(VERSION 3.10)
project(ur_wrappers)

find_package(ur_client_library REQUIRED)

add_executable(db_client src/db_client.cpp)
target_link_libraries(db_client ur_client_library::urcl)