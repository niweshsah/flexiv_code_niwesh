#pragma once
#include "Common/MotionDefaults.hpp"
#include <string>
#include <vector>
#include <map>

namespace flexiv_executor::cli {

/**
 * @brief Handles base separation of positional vs optional (overridden) arguments.
 */
class CLIParser
{
public:
    CLIParser(int argc, char** argv);

    bool HasHelp() const;
    std::string GetPrimitiveName() const;
    // Change GetRobotIP to GetRobotSN
    std::string GetRobotSN() const;
    std::vector<std::string> GetPositionalArgs() const;
    common::MotionDefaults GetMotionParameters() const;

private:
    std::vector<std::string> m_args;
    std::map<std::string, std::string> m_kwargs;
    std::vector<std::string> m_positionalArgs;

    void ParseArgs();
};

} // namespace flexiv_executor::cli