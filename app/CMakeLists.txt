include("${_build_cmake_local}")

set(APP_NAME PiSubmarine.Drv8908.Cli.App)

add_executable(${APP_NAME} PiSubmarine/Drv8908/Cli/main.cpp)
PiSubmarineInitModule(${APP_NAME})

target_include_directories(${APP_NAME} PRIVATE ./)
target_link_libraries(${APP_NAME} PRIVATE PiSubmarine.Drv8908.Cli)

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${APP_NAME} PRIVATE spdlog::spdlog)