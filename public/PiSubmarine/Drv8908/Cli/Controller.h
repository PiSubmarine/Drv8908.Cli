#pragma once
#include "PiSubmarine/Drv8908/Device.h"

namespace PiSubmarine::Drv8908::Cli
{
    enum class Thruster
    {
        FrontLeft = (1 << 0),
        FrontRight = (1 << 1),
        BackLeft = (1 << 2),
        BackRight = (1 << 3)
    };

    class Controller
    {
    public:
        Controller(
            Device& thrustersDrive,
            Device& lampsAndBallastDrive
        );

        ~Controller();

        bool Init();

        void SetThrust(Thruster thrusterMask, double factor) const;

    private:
        Device& m_ThrustersDrive;
        Device& m_LampsAndBallastDrive;

        void VerifyDrive(const Device& drive) const;
        void SetupThrustersDrive(Device& drive);
    };
}
