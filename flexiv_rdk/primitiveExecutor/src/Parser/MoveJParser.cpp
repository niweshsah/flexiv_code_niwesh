#include "Parser/MoveJParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <vector>

namespace flexiv_executor::parser {

common::PrimitiveRequest MoveJParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    if (!config["target"]) {
        throw std::runtime_error("YAML Configuration missing required key: 'target'");
    }

    auto targetVec = config["target"].as<std::vector<double>>();
    if (targetVec.size() != 7) {
        throw std::runtime_error("MoveJ 'target' must contain exactly 7 joint elements [q1, q2, q3, q4, q5, q6, q7]");
    }

    LOG_INFO("Configuring MoveJ Target: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}] deg", 
             targetVec[0], targetVec[1], targetVec[2], targetVec[3], targetVec[4], targetVec[5], targetVec[6]);

    common::PrimitiveRequest req;
    req.primitiveName = "MoveJ";
    req.parameters["target"] = targetVec; // Inject the 7-element vector directly
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    req.parameters["jerk"] = motionParams.jerk;
    
    return req;
}

void MoveJParser::PrintHelp() const {
    LOG_INFO("YAML Required Keys: primitive (MoveJ), target [size 7]");
}

} // namespace flexiv_executor::parser