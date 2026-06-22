#pragma once
#include <atomic>

namespace flexiv_executor::common {

/**
 * @brief Global signal handling for Ctrl+C.
 */
class SignalHandler {
public:
    static void Init();
    static bool IsInterrupted();

private:
    static std::atomic<bool> s_isInterrupted;
    static void HandleSignal(int signal);
};

} // namespace flexiv_executor::common