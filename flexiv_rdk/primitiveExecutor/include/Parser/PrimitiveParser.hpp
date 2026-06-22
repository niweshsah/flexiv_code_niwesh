#pragma once
#include "Common/PrimitiveRequest.hpp"
#include "Common/MotionDefaults.hpp"
#include <yaml-cpp/yaml.h>

namespace flexiv_executor::parser {

class PrimitiveParser {
public:
    virtual ~PrimitiveParser() = default;

    // Replaces the old Parse method
    virtual common::PrimitiveRequest ParseYAML(
        const YAML::Node& config,
        const common::MotionDefaults& motionParams) const = 0;

    virtual void PrintHelp() const = 0;

    virtual bool RequiresFTSensorZeroing() const { return false; }
};

} // namespace flexiv_executor::parser