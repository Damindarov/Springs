cmake_minimum_required(VERSION 3.18)
project(Springs LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)


# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Path to Open3D, if installed in a non-standard location
list(APPEND CMAKE_PREFIX_PATH "D:\\open3d_install")
list(APPEND CMAKE_PREFIX_PATH "D:\\Проекты\\YAML_CPP")

find_package(Open3D REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(Eigen REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS})
# #INCLUDE_DIRECTORIES( "$ENV{EIGEN3_INCLUDE_DIR}" )
# link_directories(${Eigen_LIBRARY_DIRS})
# add_definitions(${Eigen_DEFINITIONS})
include_directories(include)

# add_executable(Springs)
# target_sources(Springs PRIVATE main.cpp)

add_executable(Springs main_ply_load.cpp
        include/filtration.h
        include/clusterization.h
        include/spring_recognition.h
        include/utilities.h
        include/chopping.h
        include/c_interface.h
        include/server.h
        src/filtration.cpp
        src/clusterization.cpp
        src/spring_recognition.cpp
        src/utilities.cpp
        src/chopping.cpp
        src/server.cpp
        )

target_link_libraries(Springs PRIVATE Open3D::Open3D D:\\Проекты\\Springs\\RfSpringVisionSystemLib.lib)
target_link_libraries(Springs PRIVATE D:\\Проекты\\YAML_CPP\\lib\\yaml-cpp.lib)
target_link_libraries(Springs PRIVATE D:\\Microsoft\\SDK\\Lib\\x64\\msmpi.lib)
# target_link_libraries(Springs PRIVATE ws2_32)
#find_package(Open3D REQUIRED)
#target_link_libraries(Springs lib/Open3D.lib)

# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET Draw POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()

