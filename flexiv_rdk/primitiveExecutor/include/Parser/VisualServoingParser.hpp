#pragma once
#include "Parser/PrimitiveParser.hpp"

namespace flexiv_executor::parser {

class VisualServoingParser : public PrimitiveParser {
public:
    common::PrimitiveRequest ParseYAML(
        const YAML::Node& config,
        const common::MotionDefaults& motionParams) const override;

    void PrintHelp() const override;
};

} // namespace flexiv_executor::parser