#pragma once

// =============================================================================
// Corium SimLab — Integrated Corium Logging Adapter
// Connects engine logging directly to Corium's zero-heap logging framework
// (corium::logging::ConsoleLogger & corium::logging::LogEvent).
// =============================================================================

#include <corium/logging/logging.hpp>
#include <sstream>
#include <string_view>
#include <utility>

namespace corium_sim {

/// @brief Re-export Corium's LogLevel enum for seamless integration.
using LogLevel = corium::logging::LogLevel;

namespace detail {

/// @brief Global ConsoleLogger instance using Corium's logging service.
inline corium::logging::ConsoleLogger& getConsoleLogger() noexcept
{
    static corium::logging::ConsoleLogger logger{"SimLab"};
    return logger;
}

template <typename... Args>
inline void log(corium::logging::LogLevel level, const char* tag, Args&&... args)
{
    auto& logger = getConsoleLogger();
    logger.setCategory(tag);

    if constexpr (sizeof...(Args) == 1 && std::is_convertible_v<std::tuple_element_t<0, std::tuple<Args...>>, const char*>) {
        logger.log(level, "%s", args...);
    } else {
        std::ostringstream ss;
        (ss << ... << std::forward<Args>(args));
        std::string str = ss.str();
        logger.log(level, "%s", str.c_str());
    }
}

} // namespace detail

/// @brief Set minimum log severity level on Corium's logging service.
inline void setLogLevel(corium::logging::LogLevel level) noexcept
{
    detail::getConsoleLogger().setMinLevel(level);
}

} // namespace corium_sim

// Convenience macros mapping directly to Corium's zero-heap logging service
#define CORIUM_LOG_TRACE(tag, ...) ::corium_sim::detail::log(::corium::logging::LogLevel::Trace, tag, __VA_ARGS__)
#define CORIUM_LOG_DEBUG(tag, ...) ::corium_sim::detail::log(::corium::logging::LogLevel::Debug, tag, __VA_ARGS__)
#define CORIUM_LOG_INFO(tag, ...)  ::corium_sim::detail::log(::corium::logging::LogLevel::Info,  tag, __VA_ARGS__)
#define CORIUM_LOG_WARN(tag, ...)  ::corium_sim::detail::log(::corium::logging::LogLevel::Warn,  tag, __VA_ARGS__)
#define CORIUM_LOG_ERROR(tag, ...) ::corium_sim::detail::log(::corium::logging::LogLevel::Error, tag, __VA_ARGS__)
#define CORIUM_LOG_CRITICAL(tag, ...) ::corium_sim::detail::log(::corium::logging::LogLevel::Critical, tag, __VA_ARGS__)
