#include "Common/SignalHandler.hpp"
#include "Common/Logger.hpp"
#include <csignal>

namespace flexiv_executor::common {

std::atomic<bool> SignalHandler::s_isInterrupted{false};

void SignalHandler::Init() {
    std::signal(SIGINT, SignalHandler::HandleSignal);
    std::signal(SIGTERM, SignalHandler::HandleSignal);
}

bool SignalHandler::IsInterrupted() {
    return s_isInterrupted.load();
}

void SignalHandler::HandleSignal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        LOG_WARN("Interrupt signal ({}) received. Stopping...", signal);
        s_isInterrupted.store(true);
    }
}

} // namespace flexiv_executor::common