/**
 * ----------------------------------------------------------------------------
 * @file MovePTP.cpp
 * @brief Execute a Flexiv MovePTP Primitive from the command line.
 *
 * This utility connects to a Flexiv robot, enables it, switches the robot into
 * Primitive Execution mode, and executes a MovePTP primitive.
 *
 * Unlike MoveL, MovePTP moves to the target pose using a point-to-point (joint
 * space) trajectory. The user specifies a Cartesian target pose, while the
 * controller computes the joint trajectory internally.
 *
 * Supported Modes:
 *   WORLD      - Absolute pose (default)
 *   RELATIVE   - Relative to the robot pose when the primitive starts
 *
 * Usage:
 *   ./MovePTP <robot_sn> <x> <y> <z> <rx> <ry> <rz> [WORLD|RELATIVE]
 *
 * Examples:
 *
 * Absolute:
 *   ./MovePTP Rizon4s-063387 0.45 0.00 0.35 180 0 180
 *
 * Relative:
 *   ./MovePTP Rizon4s-063387 0 0 0.10 0 0 0 RELATIVE
 *
 * Compatible with Flexiv RDK v1.9+
 * ----------------------------------------------------------------------------
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

using namespace flexiv;

//==============================================================================
// Default Motion Parameters
//==============================================================================

constexpr double kDefaultVelocity = 0.30;          // m/s
constexpr double kDefaultAcceleration = 1.50;      // m/s²
constexpr double kDefaultAngularVelocity = 150.0;  // deg/s
constexpr double kDefaultJerk = 50.0;

constexpr int kDefaultTargetTolerance = 3;

const std::string kDefaultZoneRadius = "ZFine";

//==============================================================================

std::atomic<bool> g_stop_requested(false);

//==============================================================================

void SignalHandler(int signal)
{
    spdlog::warn("Received signal {}. Robot stop requested.", signal);
    g_stop_requested = true;
}

//==============================================================================

void PrintHelp()
{
    std::cout
        << "\nMovePTP Primitive Utility\n\n"

        << "Usage:\n"
        << "  ./MovePTP <robot_sn> <x> <y> <z> <rx> <ry> <rz> [WORLD|RELATIVE]\n\n"

        << "Arguments:\n"
        << "  robot_sn : Robot serial number\n"
        << "  x y z    : Target position (meters)\n"
        << "  rx ry rz : Target orientation (degrees)\n"
        << "  mode     : WORLD (default) or RELATIVE\n\n"

        << "Examples:\n\n"

        << "Absolute:\n"
        << "  ./MovePTP Rizon4s-063387 0.45 0.00 0.35 180 0 180\n\n"

        << "Relative:\n"
        << "  ./MovePTP Rizon4s-063387 0 0 0.10 0 0 0 RELATIVE\n"
        << std::endl;
}

//==============================================================================

int main(int argc, char* argv[])
{
    if (argc < 8 ||
        rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"}))
    {
        PrintHelp();
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, SignalHandler);

    try
    {
        //----------------------------------------------------------------------
        // Parse command-line arguments
        //----------------------------------------------------------------------

        const std::string robot_sn = argv[1];

        const double target_x  = std::stod(argv[2]);
        const double target_y  = std::stod(argv[3]);
        const double target_z  = std::stod(argv[4]);

        const double target_rx = std::stod(argv[5]);
        const double target_ry = std::stod(argv[6]);
        const double target_rz = std::stod(argv[7]);

        //----------------------------------------------------------------------
        // Reference frame
        //----------------------------------------------------------------------

        std::string ref_frame = "WORLD";
        std::string ref_point = "WORLD_ORIGIN";

        if (argc >= 9)
        {
            std::string mode = argv[8];

            if (mode == "RELATIVE" ||
                mode == "TRAJ" ||
                mode == "TCP")
            {
                ref_frame = "TRAJ";
                ref_point = "START";
            }
        }

        //----------------------------------------------------------------------
        // Print execution summary
        //----------------------------------------------------------------------

        spdlog::info("=================================================");
        spdlog::info("        Flexiv MovePTP Execution Utility");
        spdlog::info("=================================================");

        spdlog::info("Robot             : {}", robot_sn);

        spdlog::info(
            "Reference Frame   : {}::{}",
            ref_frame,
            ref_point);

        spdlog::info(
            "Target Position   : [{:.4f}, {:.4f}, {:.4f}] m",
            target_x,
            target_y,
            target_z);

        spdlog::info(
            "Target Rotation   : [{:.2f}, {:.2f}, {:.2f}] deg",
            target_rx,
            target_ry,
            target_rz);

        spdlog::info("Velocity          : {:.2f} m/s", kDefaultVelocity);
        spdlog::info("Acceleration      : {:.2f} m/s²", kDefaultAcceleration);
        spdlog::info("Angular Velocity  : {:.2f} deg/s", kDefaultAngularVelocity);
        spdlog::info("Zone Radius       : {}", kDefaultZoneRadius);

        //----------------------------------------------------------------------
        // Construct target coordinate
        //----------------------------------------------------------------------

        rdk::Coord target_coord(
            {target_x, target_y, target_z},
            {target_rx, target_ry, target_rz},
            {ref_frame, ref_point});

        //----------------------------------------------------------------------
        // Build primitive parameters
        //----------------------------------------------------------------------

        std::map<std::string, rdk::FlexivDataTypes> primitive_parameters;

        primitive_parameters["target"] = target_coord;
        primitive_parameters["vel"] = kDefaultVelocity;
        primitive_parameters["acc"] = kDefaultAcceleration;
        primitive_parameters["angVel"] = kDefaultAngularVelocity;
        primitive_parameters["jerk"] = kDefaultJerk;
        primitive_parameters["zoneRadius"] = kDefaultZoneRadius;
        primitive_parameters["targetTolerLevel"] = kDefaultTargetTolerance;

        //----------------------------------------------------------------------
        // Connect to robot
        //----------------------------------------------------------------------

        spdlog::info("Connecting to robot...");

        rdk::Robot robot(robot_sn);

        spdlog::info("Successfully connected.");

        //----------------------------------------------------------------------
        // Clear fault if necessary
        //----------------------------------------------------------------------

        if (robot.fault())
        {
            spdlog::warn("Robot is currently in FAULT state.");

            if (!robot.ClearFault())
            {
                throw std::runtime_error(
                    "Unable to clear robot fault.");
            }

            spdlog::info("Robot fault cleared.");
        }

        //----------------------------------------------------------------------
        // Enable robot
        //----------------------------------------------------------------------

        spdlog::info("Enabling robot...");

        robot.Enable();

        while (!robot.operational())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }

        spdlog::info("Robot is operational.");

        //----------------------------------------------------------------------
        // Switch to Primitive Execution mode
        //----------------------------------------------------------------------

        spdlog::info("Switching to Primitive Execution mode...");

        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        spdlog::info("Mode switched successfully.");

        //----------------------------------------------------------------------
        // Execute MovePTP
        //----------------------------------------------------------------------

        spdlog::info("Executing MovePTP primitive...");

        robot.ExecutePrimitive(
            "MovePTP",
            primitive_parameters);

        //----------------------------------------------------------------------
        // Wait until target is reached
        //----------------------------------------------------------------------

        while (!g_stop_requested)
        {
            if (robot.fault())
            {
                throw std::runtime_error(
                    "Robot entered FAULT state during execution.");
            }

            auto primitive_state = robot.primitive_states();

            if (std::get<int>(primitive_state["reachedTarget"]))
                break;

            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }

        //----------------------------------------------------------------------
        // Handle user interruption
        //----------------------------------------------------------------------

        if (g_stop_requested)
        {
            spdlog::warn("Stopping robot...");

            robot.Stop();

            spdlog::info("Robot motion stopped by user.");
        }
        else
        {
            spdlog::info("MovePTP completed successfully.");
        }

        return EXIT_SUCCESS;
    }
    catch (const std::invalid_argument&)
    {
        spdlog::error("Invalid numeric argument supplied.");
    }
    catch (const std::out_of_range&)
    {
        spdlog::error("Numeric argument out of range.");
    }
    catch (const std::exception& e)
    {
        spdlog::error("Execution failed: {}", e.what());
    }

    return EXIT_FAILURE;
}