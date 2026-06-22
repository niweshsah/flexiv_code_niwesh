#pragma once
#include "Parser/PrimitiveParser.hpp"

namespace flexiv_executor::parser {

class AlignContactParser : public PrimitiveParser {
public:
    common::PrimitiveRequest ParseYAML(
        const YAML::Node& config,
        const common::MotionDefaults& motionParams) const override;

    void PrintHelp() const override;

    // Like ForceComp, AlignContact interacts with the environment and requires a baseline tare
    bool RequiresFTSensorZeroing() const override { return true; }
};

} // namespace flexiv_executor::parser