include("${_build_cmake_local}")

add_library(PiSubmarine.Drv8908.Cli
        PiSubmarine/Drv8908/Cli/Controller.cpp)
target_include_directories(PiSubmarine.Drv8908.Cli PUBLIC "../public")
PiSubmarineInitTarget(PiSubmarine.Drv8908.Cli)

PiSubmarineAddDependency("https://github.com/PiSubmarine/SPI.Linux" "")
target_link_libraries(PiSubmarine.Drv8908.Cli PUBLIC PiSubmarine.SPI.Linux)

PiSubmarineAddDependency("https://github.com/PiSubmarine/GPIO.Linux" "")
target_link_libraries(PiSubmarine.Drv8908.Cli PUBLIC PiSubmarine.GPIO.Linux)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Drv8908" "")
target_link_libraries(PiSubmarine.Drv8908.Cli PUBLIC PiSubmarine.Drv8908)
