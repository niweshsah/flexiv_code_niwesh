#include "Common/Logger.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace flexiv_executor::common {

std::shared_ptr<spdlog::logger> Logger::s_logger = nullptr;

void Logger::Init() {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    s_logger = std::make_shared<spdlog::logger>("FlexivExecutor", console_sink);
    spdlog::register_logger(s_logger);
    s_logger->set_level(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> Logger::Get() {
    return s_logger;
}

} // namespace flexiv_executor::common