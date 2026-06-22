#include "CLI/CLIParser.hpp"
#include <stdexcept>
#include <algorithm>

namespace flexiv_executor::cli {

CLIParser::CLIParser(int argc, char** argv) {
    for (int i = 0; i < argc; ++i) {
        m_args.emplace_back(argv[i]);
    }
    ParseArgs();
}

void CLIParser::ParseArgs() {
    for (size_t i = 1; i < m_args.size(); ++i) {
        if (m_args[i].rfind("--", 0) == 0) {
            std::string key = m_args[i].substr(2);
            if (i + 1 < m_args.size() && m_args[i+1].rfind("--", 0) != 0) {
                m_kwargs[key] = m_args[++i];
            } else {
                m_kwargs[key] = "true";
            }
        } else {
            m_positionalArgs.push_back(m_args[i]);
        }
    }
}

bool CLIParser::HasHelp() const {
    return m_kwargs.find("help") != m_kwargs.end() || 
           (m_positionalArgs.size() == 1 && m_positionalArgs[0] == "help");
}

std::string CLIParser::GetPrimitiveName() const {
    if (m_positionalArgs.empty()) throw std::runtime_error("Missing Primitive Name");
    return m_positionalArgs[0];
}

// std::string CLIParser::GetRobotIP() const {
//     if (m_positionalArgs.size() < 2) throw std::runtime_error("Missing Robot IP");
//     return m_positionalArgs[1];
// }

std::string CLIParser::GetRobotSN() const {
    if (m_positionalArgs.size() < 2) throw std::runtime_error("Missing Robot Serial Number");
    return m_positionalArgs[1];
}

std::vector<std::string> CLIParser::GetPositionalArgs() const {
    if (m_positionalArgs.size() > 2) {
        return std::vector<std::string>(m_positionalArgs.begin() + 2, m_positionalArgs.end());
    }
    return {};
}

common::MotionDefaults CLIParser::GetMotionParameters() const {
    common::MotionDefaults defaults;
    if (m_kwargs.count("vel")) defaults.vel = std::stod(m_kwargs.at("vel"));
    if (m_kwargs.count("acc")) defaults.acc = std::stod(m_kwargs.at("acc"));
    if (m_kwargs.count("angVel")) defaults.angVel = std::stod(m_kwargs.at("angVel"));
    if (m_kwargs.count("jerk")) defaults.jerk = std::stod(m_kwargs.at("jerk"));
    if (m_kwargs.count("zone")) defaults.zoneRadius = m_kwargs.at("zone");
    if (m_kwargs.count("tol")) defaults.targetTolerance = std::stoi(m_kwargs.at("tol"));

    if (m_kwargs.count("hold")) defaults.hold = true;
    return defaults;
}

} // namespace flexiv_executor::cli