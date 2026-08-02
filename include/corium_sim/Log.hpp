#pragma once

// =============================================================================
// Corium SimLab — Structured Logging with Level Filtering
// =============================================================================

#include <cstdint>
#include <iostream>
#include <string_view>

namespace corium_sim {

/// @brief Log severity level for structured log output.
enum class LogLevel : uint8_t {
    Debug = 0,
    Info  = 1,
    Warn  = 2,
    Error = 3
};

namespace detail {

/// @brief Runtime minimum log level (default: Info in Release, Debug in Debug builds).
inline LogLevel& minLogLevel() noexcept
{
#ifdef NDEBUG
    static LogLevel level = LogLevel::Info;
#else
    static LogLevel level = LogLevel::Debug;
#endif
    return level;
}

inline constexpr const char* levelPrefix(LogLevel level) noexcept
{
    switch (level) {
        case LogLevel::Debug: return "\033[90m[DEBUG]\033[0m";
        case LogLevel::Info:  return "\033[36m[INFO]\033[0m ";
        case LogLevel::Warn:  return "\033[33m[WARN]\033[0m ";
        case LogLevel::Error: return "\033[31m[ERROR]\033[0m";
    }
    return "[???]";
}

template <typename... Args>
inline void log(LogLevel level, std::string_view tag, Args&&... args)
{
    if (level < minLogLevel()) return;

    auto& stream = (level >= LogLevel::Error) ? std::cerr : std::cout;
    stream << levelPrefix(level) << " [" << tag << "] ";
    (stream << ... << std::forward<Args>(args));
    stream << '\n';
}

} // namespace detail

/// @brief Set the runtime minimum log level filter.
inline void setLogLevel(LogLevel level) noexcept
{
    detail::minLogLevel() = level;
}

} // namespace corium_sim

// Convenience macros for structured logging with variadic arguments
#define CORIUM_LOG_DEBUG(tag, ...) ::corium_sim::detail::log(::corium_sim::LogLevel::Debug, tag, __VA_ARGS__)
#define CORIUM_LOG_INFO(tag, ...)  ::corium_sim::detail::log(::corium_sim::LogLevel::Info,  tag, __VA_ARGS__)
#define CORIUM_LOG_WARN(tag, ...)  ::corium_sim::detail::log(::corium_sim::LogLevel::Warn,  tag, __VA_ARGS__)
#define CORIUM_LOG_ERROR(tag, ...) ::corium_sim::detail::log(::corium_sim::LogLevel::Error, tag, __VA_ARGS__)
