#include "Parser/ForceTrajParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <sstream>

namespace flexiv_executor::parser {

common::PrimitiveRequest ForceTrajParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    if (!config["waypoints"]) {
        throw std::runtime_error("ForceTraj YAML missing required key: 'waypoints'");
    }

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
            throw std::runtime_error("Waypoints must have exactly 6 Cartesian elements");
        }
        ss << wp[0] << " " << wp[1] << " " << wp[2] << " " 
           << wp[3] << " " << wp[4] << " " << wp[5] << " " 
           << refFrame << " " << refPoint;
        
        if (i < waypointsList.size() - 1) {
            ss << " : ";
        }
    }

    // Extract optimization and compliance scaling properties
    std::vector<double> stiffScale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // Compliance softness multiplier
    if (config["stiff_scale"]) stiffScale = config["stiff_scale"].as<std::vector<double>>();

    LOG_INFO("Configuring ForceTraj Compliant Trajectory tracking profile.");
    LOG_INFO("Total path waypoints loaded: {}", waypointsList.size());

    common::PrimitiveRequest req;
    req.primitiveName = "ForceTraj"; // Handled seamlessly as ForceTraj/ForceTraj depending on firmware version
    req.parameters["waypoint"] = ss.str();
    req.parameters["stiffScale"] = stiffScale;
    
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    req.parameters["jerk"] = motionParams.jerk;

    return req;
}

void ForceTrajParser::PrintHelp() const {
    LOG_INFO("YAML Options: primitive (ForceTraj), waypoints [Nx6 array], stiff_scale [6]");
}

} // namespace flexiv_executor::parser