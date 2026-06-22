#include "Parser/MovePTPParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <vector>

namespace flexiv_executor::parser {

common::PrimitiveRequest MovePTPParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    if (!config["target"]) {
        throw std::runtime_error("YAML Configuration missing required key: 'target'");
    }

    auto targetVec = config["target"].as<std::vector<double>>();
    if (targetVec.size() != 6) {
        throw std::runtime_error("MovePTP 'target' must contain exactly 6 elements [X, Y, Z, Rx, Ry, Rz]");
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

    LOG_INFO("Configuring MovePTP Reference Frame: {}::{}", refFrame, refPoint);
    flexiv::rdk::Coord targetPose(
        {targetVec[0], targetVec[1], targetVec[2]},
        {targetVec[3], targetVec[4], targetVec[5]},
        {refFrame, refPoint}
    );

    common::PrimitiveRequest req;
    req.primitiveName = "MovePTP";
    req.parameters["target"] = targetPose;
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    req.parameters["jerk"] = motionParams.jerk;
    
    return req;
}

void MovePTPParser::PrintHelp() const {
    LOG_INFO("YAML Required Keys: primitive (MovePTP), target [size 6]");
    LOG_INFO("YAML Optional Keys: reference_frame");
}

} // namespace flexiv_executor::parser