#pragma once
#include "Parser/PrimitiveParser.hpp"

namespace flexiv_executor::parser {

class ForceHybridParser : public PrimitiveParser {
public:
    common::PrimitiveRequest ParseYAML(
        const YAML::Node& config,
        const common::MotionDefaults& motionParams) const override;

    void PrintHelp() const override;

    bool RequiresFTSensorZeroing() const override { return true; }
};

} // namespace flexiv_executor::parser