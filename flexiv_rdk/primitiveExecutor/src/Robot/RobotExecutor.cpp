#include "Robot/RobotExecutor.hpp"
#include "Common/Logger.hpp"
#include "Common/SignalHandler.hpp"
#include <thread>
#include <chrono>
#include <stdexcept>

namespace flexiv_executor::robot {

RobotExecutor::RobotExecutor(const std::string& robotSN)
: m_sn(robotSN)
{
    m_robot = std::make_unique<flexiv::rdk::Robot>(m_sn);
    // Instantiate the kinematic model using the initialized robot handle
    m_model = std::make_unique<flexiv::rdk::Model>(*m_robot);
}

RobotExecutor::~RobotExecutor()
{
    Stop();
}

void RobotExecutor::ConnectAndInitialize()
{
    LOG_INFO("Connecting to robot with Serial Number {}...", m_sn);
    ClearFaultIfNeeded();
    EnableAndStandby();
    SwitchMode();

    // Launch the 50Hz Safety Monitor Thread after initializing connection
    m_monitorRunning.store(true);
    m_monitorThread = std::thread(&RobotExecutor::SafetyMonitorLoop, this);
}

void RobotExecutor::SafetyMonitorLoop()
{
    LOG_INFO("[Safety Monitor] 50Hz Robot Kinematics Watchdog Thread Started.");

    // Track print pacing so we don't spam terminal output loops at 50Hz
    auto lastWarningTime = std::chrono::steady_clock::now();

    while (m_monitorRunning.load() && !common::SignalHandler::IsInterrupted()) {
        try {
            if (m_robot->operational()) {
                // Fetch current joint information structure from RDK
                auto robot_states = m_robot->states();

                // Update internal model states before requesting score calculations
                m_model->Update(robot_states.q, robot_states.dq);

                // Fetch configuration scores: returns std::pair{translation, orientation}
                // Fetch configuration scores: returns std::pair{translation, orientation}
                auto scores = m_model->configuration_score();
                double transScore = scores.first;
                double orientScore = scores.second; // <-- Retrieve orientation metric

                // Evaluate both thresholds
                if (transScore < 30.0 || orientScore < 30.0) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed
                        = std::chrono::duration_cast<std::chrono::seconds>(now - lastWarningTime)
                              .count();

                    if (elapsed >= 2) { // Throttle printing
                        if (transScore < 40.0) {
                            LOG_WARN("[SINGULARITY WARNING] Low Translation Score: {:.1f}/100.0",
                                transScore);
                            LOG_WARN(
                                " -> Status: Main arm structure is nearing full extension or max "
                                "compression.");
                            LOG_WARN(
                                " -> Resolution: Bring the target Cartesian position closer to the "
                                "robot base body.");
                        }

                        if (orientScore < 40.0) {
                            LOG_WARN("[SINGULARITY WARNING] Low Orientation Score: {:.1f}/100.0",
                                orientScore);
                            LOG_WARN(
                                " -> Status: Wrist joints (J5, J6, J7) are aligning into a "
                                "co-linear wrist flip configuration.");
                            LOG_WARN(
                                " -> Resolution: Modify the tool orientation (Rx, Ry, Rz) slightly "
                                "to break co-linear joint alignment.");
                        }

                        lastWarningTime = now;
                    }
                }
            }
        } catch (const std::exception& e) {
            LOG_ERROR("[Safety Monitor] Exception inside kinematic checker loop: {}", e.what());
        }

        // Sleep to enforce a ~50Hz monitoring loop pace (1000ms / 50 = 20ms cycles)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    LOG_INFO("[Safety Monitor] Watchdog thread shut down safely.");
}

void RobotExecutor::ClearFaultIfNeeded()
{
    if (m_robot->fault()) {
        LOG_WARN("Robot is currently in FAULT state.");

        if (!m_robot->ClearFault()) {
            std::string logMsg = "Failed to clear robot fault state.";

            // Loop through the vector of events
            auto events = m_robot->event_log();
            if (!events.empty()) {
                logMsg += " Details: ";
                for (const auto& e : events) {
                    logMsg += e.description + " | "; // Extract the specific message text
                }
            }

            throw std::runtime_error(logMsg);
        }

        LOG_INFO("Robot fault cleared successfully.");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void RobotExecutor::EnableAndStandby()
{
    LOG_INFO("Enabling robot...");
    m_robot->Enable();
    while (!m_robot->operational() && !common::SignalHandler::IsInterrupted()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotExecutor::SwitchMode()
{
    LOG_INFO("Switching execution mode...");
    m_robot->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
}

void RobotExecutor::ZeroFTSensor()
{
    LOG_INFO("Calibrating 6DoF Force/Torque Sensor using [ZeroFTSensor]...");
    std::map<std::string, flexiv::rdk::FlexivDataTypes> emptyParams;
    m_robot->ExecutePrimitive("ZeroFTSensor", emptyParams);
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void RobotExecutor::Execute(const common::PrimitiveRequest& request)
{
    m_robot->ExecutePrimitive(request.primitiveName, request.parameters);
}

void RobotExecutor::WaitUntilFinished(bool holdIndefinitely)
{
    if (holdIndefinitely) {
        LOG_INFO("Primitive will hold at target indefinitely. Press Ctrl+C to stop.");
    } else {
        LOG_INFO("Waiting for primitive completion...");
    }

    while (!common::SignalHandler::IsInterrupted()) {
        if (m_robot->fault()) {
            //  Wait 350ms to allow the event log text to sync over the network
            std::this_thread::sleep_for(std::chrono::milliseconds(350));

            std::string logMsg = "Unknown critical server error.";
            auto events = m_robot->event_log();

            if (!events.empty()) {
                logMsg = "";
                for (const auto& e : events) {
                    logMsg += e.description + " | ";
                }
            }

            throw std::runtime_error("Robot entered FAULT state: " + logMsg);
        }

        if (!holdIndefinitely) {
            auto primitive_state = m_robot->primitive_states();
            if (primitive_state.count("reachedTarget")) {
                if (std::get<int>(primitive_state.at("reachedTarget")) == 1) {
                    break;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (common::SignalHandler::IsInterrupted()) {
        LOG_WARN("Execution interrupted by user.");
        return;
    }

    LOG_INFO("Completed Successfully.");
}

void RobotExecutor::Stop()
{
    // Gracefully join and stop background worker thread allocations
    m_monitorRunning.store(false);
    if (m_monitorThread.joinable()) {
        m_monitorThread.join();
    }

    if (m_robot && m_robot->operational()) {
        LOG_INFO("Stopping Robot...");
        m_robot->Stop();
    }
}

} // namespace flexiv_executor::robot