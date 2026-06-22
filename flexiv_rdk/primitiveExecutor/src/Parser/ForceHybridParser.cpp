#include "Parser/ForceHybridParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <sstream>

namespace flexiv_executor::parser {

common::PrimitiveRequest ForceHybridParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    if (!config["waypoints"]) {
        throw std::runtime_error("ForceHybrid YAML missing required key: 'waypoints'");
    }

    // 1. Process Multi-Waypoint Array into Flexiv RDK String Format ("X Y Z Rx Ry Rz Frame FrameOrigin : X2 Y2 ...")
    auto waypointsList = config["waypoints"].as<std::vector<std::vector<double>>>();
    if (waypointsList.empty()) {
        throw std::runtime_error("'waypoints' sequence cannot be empty");
    }

    std::string refFrame = "WORLD";
    std::string refPoint = "WORLD_ORIGIN";
    if (config["reference_frame"]) {
        std::string mode = config["reference_frame"].as<std::string>();
        if (mode == "RELATIVE" || mode == "TRAJ" || mode == "TCP") {
            refFrame = "TRAJ";
            refPoint = "START";
        }
    }

    std::stringstream ss;
    for (size_t i = 0; i < waypointsList.size(); ++i) {
        const auto& wp = waypointsList[i];
        if (wp.size() != 6) {
            throw std::runtime_error("Each waypoint vector inside the sequence must contain exactly 6 elements");
        }
        ss << wp[0] << " " << wp[1] << " " << wp[2] << " " 
           << wp[3] << " " << wp[4] << " " << wp[5] << " " 
           << refFrame << " " << refPoint;
        
        if (i < waypointsList.size() - 1) {
            ss << " : "; // Flexiv waypoint separator token
        }
    }

    // 2. Extract Active Force Target Control Properties
    std::vector<double> contactAxis = {0.0, 0.0, 1.0}; // Default direction: TCP Z-axis
    if (config["contact_axis"]) contactAxis = config["contact_axis"].as<std::vector<double>>();
    if (contactAxis.size() != 3) throw std::runtime_error("'contact_axis' requires a 3D direction vector");

    double contactForce = 5.0; // Target active push force in Newtons
    if (config["contact_force"]) contactForce = config["contact_force"].as<double>();

    LOG_INFO("Configuring ForceHybrid active trajectory tracking map.");
    LOG_INFO("Waypoints registered   : {}", waypointsList.size());
    LOG_INFO("Contact Active Axis    : [{} {} {}]", contactAxis[0], contactAxis[1], contactAxis[2]);
    LOG_INFO("Target Closed Loop Force: {:.1f} N", contactForce);

    // 3. Package variant maps natively
    common::PrimitiveRequest req;
    req.primitiveName = "ForceHybrid";
    req.parameters["waypoint"] = ss.str();
    req.parameters["contactAxis"] = contactAxis;
    req.parameters["contactForce"] = contactForce;
    
    // Bind motion parameter constraints
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    req.parameters["jerk"] = motionParams.jerk;

    return req;
}

void ForceHybridParser::PrintHelp() const {
    LOG_INFO("YAML Options: primitive (ForceHybrid), waypoints [Nx6 array], contact_axis [3], contact_force [double]");
}

} // namespace flexiv_executor::parser