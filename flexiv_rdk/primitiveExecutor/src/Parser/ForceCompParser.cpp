#include "Parser/ForceCompParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <vector>

namespace flexiv_executor::parser {

common::PrimitiveRequest ForceCompParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    if (!config["target"]) {
        throw std::runtime_error("YAML Configuration missing required key: 'target'");
    }

    auto targetVec = config["target"].as<std::vector<double>>();
    if (targetVec.size() != 6) {
        throw std::runtime_error("Key 'target' must contain exactly 6 elements [X, Y, Z, Rx, Ry, Rz]");
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

    flexiv::rdk::Coord targetPose(
        {targetVec[0], targetVec[1], targetVec[2]},
        {targetVec[3], targetVec[4], targetVec[5]},
        {refFrame, refPoint}
    );

    std::vector<int> compAxis = {0, 0, 0, 0, 0, 0};
    if (config["comp_axis"]) compAxis = config["comp_axis"].as<std::vector<int>>();

    std::vector<int> enableMaxWrench = {0, 0, 0, 0, 0, 0};
    if (config["enable_max_wrench"]) enableMaxWrench = config["enable_max_wrench"].as<std::vector<int>>();

    std::vector<double> maxContactWrench = {150.0, 150.0, 150.0, 40.0, 40.0, 40.0};
    if (config["max_contact_wrench"]) maxContactWrench = config["max_contact_wrench"].as<std::vector<double>>();

    LOG_INFO("Configuring ForceComp Reference Frame: {}::{}", refFrame, refPoint);
    LOG_INFO("Max Contact Wrench Limits: [{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}]", 
             maxContactWrench[0], maxContactWrench[1], maxContactWrench[2], 
             maxContactWrench[3], maxContactWrench[4], maxContactWrench[5]);

    common::PrimitiveRequest req;
    req.primitiveName = "ForceComp";
    req.parameters["target"] = targetPose;
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    req.parameters["angVel"] = motionParams.angVel;
    req.parameters["jerk"] = motionParams.jerk;
    req.parameters["zoneRadius"] = motionParams.zoneRadius;
    req.parameters["targetTolerLevel"] = motionParams.targetTolerance;
    
    req.parameters["compAxis"] = compAxis;
    req.parameters["enableMaxWrench"] = enableMaxWrench;
    req.parameters["maxContactWrench"] = maxContactWrench;
    
    return req;
}

void ForceCompParser::PrintHelp() const {
    LOG_INFO("YAML Required Keys: primitive, target [size 6]");
    LOG_INFO("YAML Optional Keys: reference_frame, comp_axis, enable_max_wrench, max_contact_wrench");
}

} // namespace flexiv_executor::parser