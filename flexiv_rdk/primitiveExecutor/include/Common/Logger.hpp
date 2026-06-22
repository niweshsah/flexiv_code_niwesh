#pragma once
#include <spdlog/spdlog.h>
#include <memory>

namespace flexiv_executor::common {

/**
 * @brief Singleton wrapper for consistent, formatted logging across the application.
 */
class Logger {
public:
    static void Init();
    static std::shared_ptr<spdlog::logger> Get();

private:
    static std::shared_ptr<spdlog::logger> s_logger;
};

} // namespace flexiv_executor::common

// Helper macros for easy logging
#define LOG_INFO(...)  ::flexiv_executor::common::Logger::Get()->info(__VA_ARGS__)
#define LOG_WARN(...)  ::flexiv_executor::common::Logger::Get()->warn(__VA_ARGS__)
#define LOG_ERROR(...) ::flexiv_executor::common::Logger::Get()->error(__VA_ARGS__)
#define LOG_CRIT(...)  ::flexiv_executor::common::Logger::Get()->critical(__VA_ARGS__)