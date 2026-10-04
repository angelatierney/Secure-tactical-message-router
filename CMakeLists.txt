cmake_minimum_required(VERSION 3.16)

project(realtime-embedded-fsm
    VERSION 1.0.0
    DESCRIPTION "Real-time embedded FSM for tactical telemetry nodes"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(fsm_runner
    src/main.cpp
    src/EmbeddedFSM.cpp
    src/Logger.cpp
)

target_include_directories(fsm_runner
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(fsm_runner
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Werror -Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
)

# Prefer pthread on platforms that need an explicit link for std::thread/mutex.
find_package(Threads REQUIRED)
target_link_libraries(fsm_runner PRIVATE Threads::Threads)

install(TARGETS fsm_runner RUNTIME DESTINATION bin)
