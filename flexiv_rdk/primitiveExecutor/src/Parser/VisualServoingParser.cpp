#include "Parser/VisualServoingParser.hpp"
#include "Common/Logger.hpp"
#include <stdexcept>
#include <vector>

namespace flexiv_executor::parser {

common::PrimitiveRequest VisualServoingParser::ParseYAML(
    const YAML::Node& config,
    const common::MotionDefaults& motionParams) const 
{
    // Validate mandatory fields (*) per specification manual
    if (!config["objName"]) throw std::runtime_error("Missing mandatory key: objName");
    if (!config["targetFeaturePts"]) throw std::runtime_error("Missing mandatory key: targetFeaturePts");
    if (!config["targetDepth"]) throw std::runtime_error("Missing mandatory key: targetDepth");

    common::PrimitiveRequest req;
    req.primitiveName = "HPImageBasedVS";

    // 1. Mandatory Parameters Assignment
    req.parameters["objName"] = config["objName"].as<std::string>();
    
    // FIX: Parse as a 2D vector from YAML, then flatten it for the Flexiv Variant
    auto nestedPts = config["targetFeaturePts"].as<std::vector<std::vector<double>>>();
    std::vector<double> flattenedPts;
    
    for (const auto& point : nestedPts) {
        if (point.size() != 2) {
            throw std::runtime_error("Each feature point entry must contain exactly 2 pixel elements [X, Y]");
        }
        flattenedPts.push_back(point[0]); // Add X
        flattenedPts.push_back(point[1]); // Add Y
    }
    // Assign the flattened 1D vector to satisfy the variant mapping
    req.parameters["targetFeaturePts"] = flattenedPts;

    auto depths = config["targetDepth"].as<std::vector<double>>();
    req.parameters["targetDepth"] = depths;

    // 2. Optional Parameters Assignment (Using manual defaults if omitted)
    req.parameters["objIndex"] = config["objIndex"] ? config["objIndex"].as<int>() : 0;
    
    std::vector<int> vsAxis = {1, 1, 1, 0, 0, 1}; // Default: X, Y, Z, Rz allowed
    if (config["vsAxis"]) vsAxis = config["vsAxis"].as<std::vector<int>>();
    req.parameters["vsAxis"] = vsAxis;

    req.parameters["velScale"] = config["velScale"] ? config["velScale"].as<double>() : 10.0;
    req.parameters["maxVel"] = config["maxVel"] ? config["maxVel"].as<double>() : 0.05;
    req.parameters["maxAngVel"] = config["maxAngVel"] ? config["maxAngVel"].as<double>() : 20.0;
    req.parameters["imageConvToler"] = config["imageConvToler"] ? config["imageConvToler"].as<double>() : 5.0;
    req.parameters["targetConvTimes"] = config["targetConvTimes"] ? config["targetConvTimes"].as<int>() : 4;
    req.parameters["timeoutTime"] = config["timeoutTime"] ? config["timeoutTime"].as<double>() : 3.0;

    LOG_INFO("Configuring HPImageBasedVS Visual Servoing Primitive via YAML.");
    LOG_INFO("Target Tracking Object : {}", config["objName"].as<std::string>());
    LOG_INFO("Total Target Corners   : {}", nestedPts.size());
    LOG_INFO("Target Distance Depth  : {:.3f} m", depths[0]);

    return req;
}

void VisualServoingParser::PrintHelp() const {
    LOG_INFO("YAML Options: primitive (HPImageBasedVS), objName [str], targetFeaturePts [[x,y],...], targetDepth [double]");
}

} // namespace flexiv_executor::parser