#include <flexiv/rdk/robot.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <map>
// Note: <variant> header is no longer strictly necessary just for the alias,
// as RDK 1.9 headers provide flexiv::rdk::FlexivDataTypes natively.

// Utility function to safely convert Radians to Degrees
double rad2deg(double rad)
{
    return rad * (180.0 / M_PI);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./go_home <robot_sn>" << std::endl;
        std::cerr << "Example: ./go_home Rizon4-123456" << std::endl;
        return -1;
    }

    try
    {
        // 1. Initialize Robot Connection
        std::cout << "[Init] Connecting to Flexiv Robot..." << std::endl;
        flexiv::rdk::Robot robot(argv[1]);

        // 2. Enable the Robot and Clear Faults
        std::cout << "[Init] Enabling robot..." << std::endl;
        robot.Enable();

        // Wait safely until the robot is fully operational
        while (!robot.operational())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "[Init] Robot is fully operational." << std::endl;

        // 3. Switch to Primitive Execution Mode
        std::cout << "[Init] Switching to NRT_PRIMITIVE_EXECUTION mode..." << std::endl;
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        std::vector<double> home_pose_rad = {
            -1.686, -0.408, 0.240, 1.499, -0.177, 0.338, -0.020}; // above table

        // std::vector<double> home_pose_rad = {
        //     0, -0.522, 0.222, 1.684, -0.218, 0.610, 0.021}; // og home

        // Convert to degrees as required by the MoveJ primitive when passing raw vectors
        std::vector<double> home_pose_deg;
        for (double q : home_pose_rad)
        {
            home_pose_deg.push_back(rad2deg(q));
        }

        // 5. Build the parameters map required by RDK v1.9 ExecutePrimitive
        // RDK 1.9 uses the built-in flexiv::rdk::FlexivDataTypes variant
        std::map<std::string, flexiv::rdk::FlexivDataTypes> pt_params;
        pt_params["target"] = home_pose_deg;
        pt_params["max_vel"] = 0.05; // Safe homing speed (5% of max)

        std::cout << "[Motion] Executing MoveJ to home pose..." << std::endl;

        // 6. Execute Primitive
        // RDK 1.9 Signature: ExecutePrimitive(primitive_name, params_map)
        robot.ExecutePrimitive("MoveJ", pt_params);

        // Allow the controller a brief moment to parse the map and begin motion
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[Motion] Moving to home pose... Please wait." << std::endl;

        // 7. Wait for Motion to Complete
        // robot.busy() remains the standard way to check NRT primitive status in 1.9
        while (robot.busy())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[Motion] Home pose reached successfully!" << std::endl;

        // 8. Safely Stop the Robot
        robot.Stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Error] Exception caught: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}