#include <flexiv/rdk/robot.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <map>
#include <variant>

// Utility function to safely convert Radians to Degrees
double rad2deg(double rad) {
    return rad * (180.0 / M_PI);
}

// Exact variant type extracted from the RDK v1.8 compiler definitions
using FlexivParam = std::variant<
    int, double, std::string,
    flexiv::rdk::JPos, flexiv::rdk::Coord,
    std::vector<int>, std::vector<double>, std::vector<std::string>,
    std::vector<flexiv::rdk::JPos>, std::vector<flexiv::rdk::Coord>
>;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./go_home <robot_sn>" << std::endl;
        std::cerr << "Example: ./go_home Rizon4-123456" << std::endl;
        return -1;
    }

    try {
        // 1. Initialize Robot Connection
        std::cout << "[Init] Connecting to Flexiv Robot..." << std::endl;
        flexiv::rdk::Robot robot(argv[1]);

        // 2. Enable the Robot and Clear Faults
        std::cout << "[Init] Enabling robot..." << std::endl;
        robot.Enable();

        // Wait safely until the robot is fully operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "[Init] Robot is fully operational." << std::endl;

        // 3. Switch to Primitive Execution Mode
        std::cout << "[Init] Switching to NRT_PRIMITIVE_EXECUTION mode..." << std::endl;
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        // 4. Define Home Pose in Radians (Extracted from your robot state)
        std::vector<double> home_pose_rad = {
            -1.560, -0.138, 0.351, 1.572, -0.104, -0.240, 0.088
        };

        // Convert to degrees as strictly required by the MoveJ primitive
        std::vector<double> home_pose_deg;
        for (double q : home_pose_rad) {
            home_pose_deg.push_back(rad2deg(q));
        }

        // 5. Build the parameters map required by RDK v1.8 ExecutePrimitive
        std::map<std::string, FlexivParam> pt_params;
        pt_params["target"] = home_pose_deg;
        pt_params["max_vel"] = 0.05; // Safe homing speed (5% of max)

        std::cout << "[Motion] Executing MoveJ to home pose..." << std::endl;

        // 6. Execute Primitive 
        // Signature: ExecutePrimitive(primitive_name, params_map, async_flag)
        // We set async to 'true' so we can poll the busy() state below.
        robot.ExecutePrimitive("MoveJ", pt_params, true);

        // Allow the controller a brief moment to parse the map and begin motion
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[Motion] Moving to home pose... Please wait." << std::endl;
        
        // 7. Wait for Motion to Complete using the correct v1.8 'busy()' call
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[Motion] Home pose reached successfully!" << std::endl;

        // 8. Safely Stop the Robot
        robot.Stop();

    } catch (const std::exception& e) {
        std::cerr << "[Error] Exception caught: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}