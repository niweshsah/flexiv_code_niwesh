/**
 * ----------------------------------------------------------------------------
 * @file MoveL.cpp
 * @brief Execute a Flexiv MoveL Primitive from the command line.
 *
 * This utility connects to a Flexiv robot, enables it, switches the robot into
 * Primitive Execution mode, and executes a MoveL primitive.
 *
 * Two execution modes are supported:
 *
 *   1. WORLD      (Absolute motion)
 *      Motion target is interpreted in the WORLD coordinate frame.
 *
 *   2. RELATIVE   (Relative motion)
 *      Motion target is interpreted relative to the robot pose at the
 *      beginning of the primitive (TRAJ::START).
 *
 * Example:
 *
 * Absolute:
 *   ./movel Rizon4s-063387 0.50 0.00 0.40 180 0 0
 *
 * Relative:
 *   ./movel Rizon4s-063387 0 0 0.10 0 0 0 RELATIVE
 *
 * Compatible with Flexiv RDK v1.9+
 * ----------------------------------------------------------------------------
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <spdlog/spdlog.h>

#include <atomic>
#include <csignal>
#include <chrono>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

using namespace flexiv;

//==============================================================================
// Default MoveL Parameters
//==============================================================================

constexpr double kDefaultVelocity = 0.25;         // m/s
constexpr double kDefaultAcceleration = 1.5;      // m/s²
constexpr double kDefaultAngularVelocity = 150.0; // deg/s
constexpr double kDefaultJerk = 50.0;             // m/s³

constexpr int kDefaultTargetTolerance = 3;

const std::string kDefaultZoneRadius = "ZFine";

//==============================================================================
// Global interrupt flag
//==============================================================================

std::atomic<bool> g_stop_requested(false);

//==============================================================================
// Signal Handler
//==============================================================================

void SignalHandler(int signal)
{
    spdlog::warn("Received signal {}. Robot stop requested.", signal);
    g_stop_requested = true;
}

//==============================================================================
// Print Help
//==============================================================================

void PrintHelp()
{
    std::cout << "\nMoveL Primitive Execution Utility\n\n"

              << "Usage:\n"
              << "  ./movel <robot_sn> <x> <y> <z> <rx> <ry> <rz> [WORLD|RELATIVE]\n\n"

              << "Arguments:\n"
              << "  robot_sn   Robot serial number\n"
              << "  x y z      Position (meters)\n"
              << "  rx ry rz   Orientation (degrees)\n\n"

              << "Modes:\n"
              << "  WORLD      Absolute coordinates (default)\n"
              << "  RELATIVE   Motion relative to current TCP pose\n\n"

              << "Examples:\n\n"

              << "Absolute Move:\n"
              << "  ./movel Rizon4s-063387 0.5 0 0.4 180 0 0\n\n"

              << "Relative Move:\n"
              << "  ./movel Rizon4s-063387 0 0 0.15 0 0 0 RELATIVE\n"
              << std::endl;
}

//==============================================================================
// Main
//==============================================================================

int main(int argc, char* argv[])
{
    //----------------------------------------------------------------------
    // Parse command-line arguments
    //----------------------------------------------------------------------

    if (argc < 8 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, SignalHandler);

    try {
        //------------------------------------------------------------------
        // Read user inputs
        //------------------------------------------------------------------

        const std::string robot_sn = argv[1];

        const double x = std::stod(argv[2]);
        const double y = std::stod(argv[3]);
        const double z = std::stod(argv[4]);

        const double rx = std::stod(argv[5]);
        const double ry = std::stod(argv[6]);
        const double rz = std::stod(argv[7]);

        //------------------------------------------------------------------
        // Determine reference frame
        //------------------------------------------------------------------

        std::string reference_frame = "WORLD";
        std::string reference_point = "WORLD_ORIGIN";

        if (argc >= 9) {
            const std::string mode = argv[8];

            if (mode == "RELATIVE" || mode == "TRAJ" || mode == "TCP") {
                reference_frame = "TRAJ";
                reference_point = "START";
            }
        }

        //------------------------------------------------------------------
        // Display execution summary
        //------------------------------------------------------------------

        spdlog::info("=================================================");
        spdlog::info("          Flexiv MoveL Execution Utility");
        spdlog::info("=================================================");

        spdlog::info("Robot             : {}", robot_sn);
        spdlog::info("Reference Frame   : {}::{}", reference_frame, reference_point);

        spdlog::info("Target Position   : [{:.4f}, {:.4f}, {:.4f}] m", x, y, z);

        spdlog::info("Target Rotation   : [{:.2f}, {:.2f}, {:.2f}] deg", rx, ry, rz);

        spdlog::info("Velocity          : {:.2f} m/s", kDefaultVelocity);
        spdlog::info("Acceleration      : {:.2f} m/s²", kDefaultAcceleration);
        spdlog::info("Angular Velocity  : {:.2f} deg/s", kDefaultAngularVelocity);
        spdlog::info("Zone Radius       : {}", kDefaultZoneRadius);

        //------------------------------------------------------------------
        // Construct target coordinate
        //------------------------------------------------------------------

        rdk::Coord target({x, y, z}, {rx, ry, rz}, {reference_frame, reference_point});

        //------------------------------------------------------------------
        // Primitive parameters
        //------------------------------------------------------------------

        std::map<std::string, rdk::FlexivDataTypes> primitive_parameters;

        primitive_parameters["target"] = target;
        primitive_parameters["vel"] = kDefaultVelocity;
        primitive_parameters["acc"] = kDefaultAcceleration;
        primitive_parameters["angVel"] = kDefaultAngularVelocity;
        primitive_parameters["jerk"] = kDefaultJerk;
        primitive_parameters["zoneRadius"] = kDefaultZoneRadius;
        primitive_parameters["targetTolerLevel"] = kDefaultTargetTolerance;

        //------------------------------------------------------------------
        // Connect to robot
        //------------------------------------------------------------------

        spdlog::info("Connecting to robot...");

        rdk::Robot robot(robot_sn);

        spdlog::info("Robot connected successfully.");

        //------------------------------------------------------------------
        // Clear fault if necessary
        //------------------------------------------------------------------

        if (robot.fault()) {
            spdlog::warn("Robot is currently in FAULT state.");

            if (!robot.ClearFault()) {
                throw std::runtime_error("Unable to clear robot fault.");
            }

            spdlog::info("Robot fault cleared.");
        }

        //------------------------------------------------------------------
        // Enable robot
        //------------------------------------------------------------------

        spdlog::info("Enabling robot...");

        robot.Enable();

        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        spdlog::info("Robot is operational.");

        //------------------------------------------------------------------
        // Switch execution mode
        //------------------------------------------------------------------

        spdlog::info("Switching to Primitive Execution mode...");

        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        spdlog::info("Mode switched successfully.");

        //------------------------------------------------------------------
        // Execute MoveL
        //------------------------------------------------------------------

        spdlog::info("Executing MoveL primitive...");

        robot.ExecutePrimitive("MoveL", primitive_parameters);

        //------------------------------------------------------------------
        // Wait until execution completes
        //------------------------------------------------------------------

        while (!g_stop_requested) {
            if (robot.fault()) {
                throw std::runtime_error("Robot entered FAULT state during execution.");
            }

            auto primitive_state = robot.primitive_states();

            if (std::get<int>(primitive_state["reachedTarget"]))
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        //------------------------------------------------------------------
        // Handle Ctrl+C
        //------------------------------------------------------------------

        if (g_stop_requested) {
            spdlog::warn("Stopping robot...");

            robot.Stop();

            spdlog::info("Robot motion stopped by user.");
        } else {
            spdlog::info("MoveL completed successfully.");
        }

        spdlog::info("Execution finished.");

        return EXIT_SUCCESS;
    }

    //----------------------------------------------------------------------
    // Invalid numeric arguments
    //----------------------------------------------------------------------

    catch (const std::invalid_argument& e) {
        spdlog::error("Invalid numeric argument supplied.");
    }

    //----------------------------------------------------------------------
    // Out-of-range numeric values
    //----------------------------------------------------------------------

    catch (const std::out_of_range& e) {
        spdlog::error("Numeric argument out of range.");
    }

    //----------------------------------------------------------------------
    // Other runtime errors
    //----------------------------------------------------------------------

    catch (const std::exception& e) {
        spdlog::error("Execution failed: {}", e.what());
    }

    return EXIT_FAILURE;
}