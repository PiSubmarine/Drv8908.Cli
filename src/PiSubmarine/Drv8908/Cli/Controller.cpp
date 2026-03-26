#include "PiSubmarine/Drv8908/Cli/Controller.h"

#include <cassert>

namespace PiSubmarine::Drv8908::Cli
{
    Controller::Controller(Device& thrustersDrive, Device& lampsAndBallastDrive) :
        m_ThrustersDrive(thrustersDrive),
        m_LampsAndBallastDrive(lampsAndBallastDrive)
    {
    }

    Controller::~Controller()
    {
        m_ThrustersDrive.SetSleeping(true);
        m_LampsAndBallastDrive.SetSleeping(true);
    }

    bool Controller::Init()
    {
        SetupThrustersDrive(m_ThrustersDrive);
        return true;
    }

    void Controller::SetThrust(Thruster thrusterMask, double factor) const
    {
        assert(factor >= 0);
        uint8_t factorByte = static_cast<uint8_t>(255.0 * factor);
        using namespace RegUtils;
        for (size_t i = 0; i < 4; ++i)
        {
            auto thruster = static_cast<Thruster>(1 << i);
            if ((thrusterMask & thruster) != 0)
            {
                auto status = m_ThrustersDrive.SetDutyCycle(i, factorByte);
                assert(IsValid(status));
            }
        }
    }

    void Controller::VerifyDrive(const Device& drive) const
    {
        using namespace RegUtils;

        drive.SetSleeping(false);

        if (drive.HasFault())
        {
            throw std::runtime_error("Fault detected");
        }

        IcStatus icStat;
        auto status = m_ThrustersDrive.GetStatus(icStat);
        assert(IsValid(status));

        ConfigCtrl configCtrl{};
        if (!IsValid(m_ThrustersDrive.GetConfigCtrl(configCtrl)))
        {
            throw std::runtime_error("Failed to initialize Thruster Drive: IC_STATUS test bit not set.");
        }
        if (configCtrl.Id != IcId::DRV8908)
        {
            throw std::runtime_error(
                "Failed to initialize Thruster Drive: wrong device ID: " + std::to_string(ToInt(configCtrl.Id)) +
                ", expected: " + std::to_string(ToInt(IcId::DRV8908)));
        }
    }

    void Controller::SetupThrustersDrive(Device& drive)
    {
        using namespace RegUtils;

        IcStatus status;

        VerifyDrive(drive);

        status = drive.SetEnabledOpenLoadDetect(HalfBridge{0});
        assert(IsValid(status));

        status = drive.SetOpenLoadDetectControl3(OcpDeglitchTime::MicroSeconds60, false);
        assert(IsValid(status));

        PwmGenerator pwmGenerators = PwmGenerator::PwmGenerator1 | PwmGenerator::PwmGenerator2 |
            PwmGenerator::PwmGenerator3 | PwmGenerator::PwmGenerator4;

        // Thruster Front Left  - OUT1, OUT2
        // Thruster Front Right - OUT3, OUT4
        // Thruster Back Left   - OUT5, OUT6
        // Thruster Back Right  - OUT7, OUT8
        status = drive.SetEnabledPwmGenerators(pwmGenerators);
        assert(IsValid(status));

        status = drive.SetPwmFrequency(pwmGenerators, PwmFrequency::Hz2000);
        assert(IsValid(status));

        status = drive.SetHalfBridgePwmModes(HalfBridge{0xFF});
        assert(IsValid(status));

        for (uint8_t i = 0; i < 8; i += 2)
        {
            status = drive.SetHalfBridgeEnabled(i, true, false);
            assert(IsValid(status));

            status = drive.SetHalfBridgeEnabled(i + 1, true, false);
            assert(IsValid(status));

            uint8_t pwm = i / 2;

            status = drive.SetPwmMap(i, pwm);
            assert(IsValid(status));

            status = drive.SetPwmMap(i + 1, pwm);
            assert(IsValid(status));
        }
    }
}
