#include "PiSubmarine/Drv8908/Cli/Controller.h"
#include "PiSubmarine/SPI/Linux/Driver.h"
#include "PiSubmarine/GPIO/Linux/Driver.h"
#include <spdlog/spdlog.h>

using namespace PiSubmarine::Drv8908::Cli;

int main(int argc, char* argv[])
{
    using namespace PiSubmarine::RegUtils;

    constexpr size_t ThrustersNSleepPin = 5;
    constexpr size_t ThrustersNFaultPin = 6;
    constexpr size_t LampsAndBallastNSleepPin = 16;
    constexpr size_t LampsAndBallastNFaultPin = 20;

    PiSubmarine::GPIO::Linux::Driver gpioDriver("PiSubmarine::Drv8908::Cli");
    auto thrustersPinGroup = gpioDriver.CreatePinGroup("Thrusters", "/dev/gpiochip0", {ThrustersNSleepPin, ThrustersNFaultPin});
    auto lampsAndBallastPinGroup = gpioDriver.CreatePinGroup("LampsAndBallast", "/dev/gpiochip0", {LampsAndBallastNSleepPin, LampsAndBallastNFaultPin});

    PiSubmarine::SPI::Linux::Driver thrustersSpiDriver("/dev/spidev0.0", 5000000,8, SPI_MODE_1, SPI_MODE_1);
    PiSubmarine::SPI::Linux::Driver lampsAndBallastSpiDriver("/dev/spidev0.1", 5000000, 8,SPI_MODE_1, SPI_MODE_1);

    PiSubmarine::Drv8908::Device thrustersDevice(thrustersSpiDriver, *thrustersPinGroup);
    PiSubmarine::Drv8908::Device lampsAndBallastDevice(lampsAndBallastSpiDriver, *lampsAndBallastPinGroup);

    Controller controller(thrustersDevice, lampsAndBallastDevice);
    if (!controller.Init())
    {
        spdlog::critical("Failed to initialize controller");
        return 1;
    }


    spdlog::info("Starting sequence...");

    double power = 0.15;
    controller.SetThrust(Thruster::FrontLeft, power);

    double dutyCycleStep = 0.01;
    uint64_t dutyCycleDelayUs = 1000000;
    while (power < 0.30 && !thrustersDevice.HasFault())
    {
        power += dutyCycleStep;
        spdlog::info("Duty cycle: {}%", power * 100);
        controller.SetThrust(Thruster::FrontRight, power);
        usleep(dutyCycleDelayUs);
    }

    usleep(1000000);

    power = 0.40;
    spdlog::info("Duty cycle: {}%", power * 100);
    controller.SetThrust(Thruster::FrontRight, power);
    usleep(1000000);

    power = 1;
    spdlog::info("Duty cycle: {}%", power * 100);
    controller.SetThrust(Thruster::FrontRight, power);

    for (int i = 0; i < 10 && !thrustersDevice.HasFault(); i++)
    {
        spdlog::info("Working at full power...");
        usleep(1000000);
    }

    if (thrustersDevice.HasFault())
    {
        PiSubmarine::Drv8908::OverCurrentStatus overCurrent;
        auto status = thrustersDevice.GetOvercurrentStatus(overCurrent);
        assert(IsValid(status));
        spdlog::error("Overcurrent status is {}", static_cast<int>(overCurrent));
    }

    PiSubmarine::Drv8908::IcStatus icStat;
    auto status = thrustersDevice.GetStatus(icStat);
    assert(IsValid(status));
    bool hasFault = thrustersDevice.HasFault();

    spdlog::info("[Before enabling OLD] IC_STAT: {}, Fault: {}", static_cast<int>(icStat), hasFault);

    status = thrustersDevice.SetEnabledOpenLoadDetect(PiSubmarine::Drv8908::HalfBridge{0xFF});
    assert(IsValid(status));
    status = thrustersDevice.GetStatus(icStat);
    assert(IsValid(status));
    hasFault = thrustersDevice.HasFault();

    spdlog::info("[After enabling OLD] IC_STAT: {}, Fault: {}", static_cast<int>(icStat), hasFault);
    return 0;
}