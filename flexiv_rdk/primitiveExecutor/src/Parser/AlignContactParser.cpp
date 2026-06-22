#include "Parser/AlignContactParser.hpp"
#include "Common/Logger.hpp"
#include <flexiv/rdk/utility.hpp>
#include <stdexcept>
#include <vector>

namespace flexiv_executor::parser {

common::PrimitiveRequest AlignContactParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    // Validate required search components
    if (!config["search_vel"]) {
        throw std::runtime_error("AlignContact YAML missing required key: 'search_vel'");
    }

    // 1. Parse Search Velocity: VEC_6d [Vx, Vy, Vz, Vrx, Vry, Vrz] (m/s and rad/s)
    auto searchVel = config["search_vel"].as<std::vector<double>>();
    if (searchVel.size() != 6) {
        throw std::runtime_error("'search_vel' must contain exactly 6 elements");
    }

    // 2. Parse Reference Frame (Default: WORLD)
    std::string refFrame = "WORLD";
    std::string refPoint = "WORLD_ORIGIN";
    if (config["reference_frame"]) {
        std::string mode = config["reference_frame"].as<std::string>();
        if (mode == "RELATIVE" || mode == "TRAJ" || mode == "TCP") {
            refFrame = "TRAJ";
            refPoint = "START";
        }
    }

    // 3. Parse Contact Safeguards (with standard industrial defaults if omitted)
    std::vector<double> contactForce = {10.0, 10.0, 10.0}; // Threshold to trigger contact (N)
    if (config["contact_force"]) contactForce = config["contact_force"].as<std::vector<double>>();
    if (contactForce.size() != 3) throw std::runtime_error("'contact_force' must have 3 values [Fx, Fy, Fz]");

    std::vector<double> contactTorque = {2.0, 2.0, 2.0}; // Threshold to trigger contact orientation alignment (Nm)
    if (config["contact_torque"]) contactTorque = config["contact_torque"].as<std::vector<double>>();
    if (contactTorque.size() != 3) throw std::runtime_error("'contact_torque' must have 3 values [Tx, Ty, Tz]");

    // 4. Parse Alignment Stiffness Gains
    std::vector<double> alignStiffness = {100.0, 100.0, 100.0}; 
    if (config["align_stiffness"]) alignStiffness = config["align_stiffness"].as<std::vector<double>>();

    LOG_INFO("Configuring AlignContact Primitive.");
    LOG_INFO("Reference Frame       : {}::{}", refFrame, refPoint);
    LOG_INFO("Search Velocity Lin   : [{:.3f} {:.3f} {:.3f}] m/s", searchVel[0], searchVel[1], searchVel[2]);
    LOG_INFO("Contact Force Trigger : [{:.1f} {:.1f} {:.1f}] N", contactForce[0], contactForce[1], contactForce[2]);

    // Format parameters maps required by Flexiv API variants
    common::PrimitiveRequest req;
    req.primitiveName = "AlignContact";
    
    // Convert base reference frame string to native RDK data structure type wrapper
    flexiv::rdk::Coord frameCoord({0,0,0}, {0,0,0}, {refFrame, refPoint});
    req.parameters["frame"] = frameCoord;

    req.parameters["searchVel"] = searchVel;
    req.parameters["contactForce"] = contactForce;
    req.parameters["contactTorque"] = contactTorque;
    req.parameters["alignStiffness"] = alignStiffness;

    // Map common motion params overrides
    req.parameters["vel"] = motionParams.vel;
    req.parameters["acc"] = motionParams.acc;
    
    return req;
}

void AlignContactParser::PrintHelp() const {
    LOG_INFO("YAML Required Keys: primitive (AlignContact), search_vel [size 6]");
    LOG_INFO("YAML Optional Keys: reference_frame, contact_force [size 3], contact_torque [size 3], align_stiffness [size 3]");
}

} // namespace flexiv_executor::parser