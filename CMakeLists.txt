cmake_minimum_required(VERSION 3.16)

project(ai_safety_modbus_manager VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MODBUS_MANAGER_BUILD_SHARED "Build ai_safety_modbus_manager as a shared library" OFF)
option(MODBUS_MANAGER_BUILD_TESTS "Build tests" ON)
option(MODBUS_MANAGER_REGISTER_HARDWARE_TESTS "Register hardware-dependent tests in CTest" OFF)

set(MODBUS_MANAGER_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party_libraries")
set(MODBUS_MANAGER_LIBMODBUS_DIR "${MODBUS_MANAGER_THIRD_PARTY_DIR}/libmodbus")
set(MODBUS_MANAGER_AI_SAFETY_COMMON_INCLUDE_DIR
    "${MODBUS_MANAGER_THIRD_PARTY_DIR}/ai_safety_common/include")

set(MODBUS_MANAGER_EXTERNAL_INCLUDE_DIRS ""
    CACHE STRING
    "Additional include directories for external headers such as modbus_control.h and ai_safety_common")
set(MODBUS_MANAGER_EXTERNAL_LIBRARIES ""
    CACHE STRING
    "Additional libraries to link, separated by semicolons")

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

find_package(jsoncpp CONFIG QUIET)
if(NOT jsoncpp_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(JSONCPP QUIET jsoncpp)
    endif()
endif()

set(MODBUS_MANAGER_LIBMODBUS_SOURCES
    ${MODBUS_MANAGER_LIBMODBUS_DIR}/modbus.c
    ${MODBUS_MANAGER_LIBMODBUS_DIR}/modbus-data.c
    ${MODBUS_MANAGER_LIBMODBUS_DIR}/modbus-rtu.c
    ${MODBUS_MANAGER_LIBMODBUS_DIR}/modbus-tcp.c
)

add_library(modbus_vendor STATIC ${MODBUS_MANAGER_LIBMODBUS_SOURCES})
add_library(modbus_vendor::modbus_vendor ALIAS modbus_vendor)

target_include_directories(modbus_vendor
    PUBLIC
        ${MODBUS_MANAGER_LIBMODBUS_DIR}
)

set(MODBUS_MANAGER_SOURCES
    src/bridge/shared_memory_bridge.cpp
    src/common/modbus_control.cpp
    src/config/modbus_config.cpp
    src/devices/hookWarning.cpp
    src/devices/multi_turn_encoder_rtu.cpp
    src/devices/trolleyControl.cpp
    src/manager/manager_client.cpp
)

if(MODBUS_MANAGER_BUILD_SHARED)
    add_library(ai_safety_modbus_manager SHARED ${MODBUS_MANAGER_SOURCES})
else()
    add_library(ai_safety_modbus_manager STATIC ${MODBUS_MANAGER_SOURCES})
endif()

add_library(ai_safety_modbus_manager::ai_safety_modbus_manager ALIAS ai_safety_modbus_manager)

target_include_directories(ai_safety_modbus_manager
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MODBUS_MANAGER_AI_SAFETY_COMMON_INCLUDE_DIR}
        ${MODBUS_MANAGER_LIBMODBUS_DIR}
        ${MODBUS_MANAGER_EXTERNAL_INCLUDE_DIRS}
)

target_compile_features(ai_safety_modbus_manager PUBLIC cxx_std_17)

target_link_libraries(ai_safety_modbus_manager
    PUBLIC
        Threads::Threads
        modbus_vendor::modbus_vendor
)

if(TARGET Boost::headers)
    target_link_libraries(ai_safety_modbus_manager PUBLIC Boost::headers)
elseif(TARGET Boost::boost)
    target_link_libraries(ai_safety_modbus_manager PUBLIC Boost::boost)
endif()

if(TARGET jsoncpp_lib)
    target_link_libraries(ai_safety_modbus_manager PUBLIC jsoncpp_lib)
elseif(TARGET JsonCpp::JsonCpp)
    target_link_libraries(ai_safety_modbus_manager PUBLIC JsonCpp::JsonCpp)
elseif(JSONCPP_FOUND)
    target_include_directories(ai_safety_modbus_manager PRIVATE ${JSONCPP_INCLUDE_DIRS})
    target_link_libraries(ai_safety_modbus_manager PUBLIC ${JSONCPP_LIBRARIES})
else()
    message(WARNING "jsoncpp not found by CMake. Set CMAKE_PREFIX_PATH or install jsoncpp development files before building.")
endif()

if(MODBUS_MANAGER_EXTERNAL_LIBRARIES)
    target_link_libraries(ai_safety_modbus_manager PUBLIC ${MODBUS_MANAGER_EXTERNAL_LIBRARIES})
endif()

if(MODBUS_MANAGER_BUILD_TESTS)
    enable_testing()

    add_executable(device_headers_test tests/device_headers_test.cpp)
    target_link_libraries(device_headers_test PRIVATE ai_safety_modbus_manager::ai_safety_modbus_manager)
    target_compile_definitions(device_headers_test
        PRIVATE
            MODBUS_MANAGER_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/modbus_config.json")

    if(MODBUS_MANAGER_REGISTER_HARDWARE_TESTS)
        add_test(NAME device_headers_test COMMAND device_headers_test)
    endif()
endif()

message(STATUS "Extra include dirs: ${MODBUS_MANAGER_EXTERNAL_INCLUDE_DIRS}")
message(STATUS "Extra link libs: ${MODBUS_MANAGER_EXTERNAL_LIBRARIES}")
message(STATUS "Bundled libmodbus dir: ${MODBUS_MANAGER_LIBMODBUS_DIR}")
message(STATUS "Created target ai_safety_modbus_manager (no executable target because no main() was found in this repository)")
