#pragma once
#include "Common/PrimitiveRequest.hpp"
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/model.hpp> // this is need for the safety monitor thread, which needs access to the model's configuration score for damping calculations
#include <string>
#include <memory> // This is needed for std::unique_ptr
#include <atomic> // For thread-safe flags
#include <thread>

namespace flexiv_executor::robot {

/**
 * @brief Singular encapsulation of all hardware interaction.
 * Implements RAII to ensure robot is stopped safely on destruction.
 */
class RobotExecutor {
public:
    // explicit RobotExecutor(const std::string& robotIP);
    explicit RobotExecutor(const std::string& robotSN);
    ~RobotExecutor();

    // Deleted copy/move constructors for hardware safety
    RobotExecutor(const RobotExecutor&) = delete;
    RobotExecutor& operator=(const RobotExecutor&) = delete;

    void ConnectAndInitialize();
    void ZeroFTSensor(); // to properly support ForceComp primitive
    void Execute(const common::PrimitiveRequest& request);
    void WaitUntilFinished(bool holdIndefinitely = false);
    void Stop();
    

private:
    std::string m_sn;
    std::unique_ptr<flexiv::rdk::Robot> m_robot;
    std::unique_ptr<flexiv::rdk::Model> m_model; // For safety monitoring


    // Background 50Hz Safety Watchdog Properties
    std::thread m_monitorThread;
    std::atomic<bool> m_monitorRunning{false};
    void SafetyMonitorLoop(); // <-- 50Hz Loop
    
    void ClearFaultIfNeeded();
    void EnableAndStandby();
    void SwitchMode();
    
};

} // namespace flexiv_executor::robot