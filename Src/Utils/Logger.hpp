#pragma once

#ifndef NDEBUG
#include <print>
#endif

#include <source_location>
#include <string_view>

namespace Guch2D
{
    // Base colors
    constexpr std::string_view Black = "\033[30m";
    constexpr std::string_view Red = "\033[31m";
    constexpr std::string_view Green = "\033[32m";
    constexpr std::string_view Yellow = "\033[33m";
    constexpr std::string_view Blue = "\033[34m";
    constexpr std::string_view Magenta = "\033[35m";
    constexpr std::string_view Cyan = "\033[36m";
    constexpr std::string_view White = "\033[37m";

    // Bright colors
    constexpr std::string_view BrightBlack = "\033[90m";
    constexpr std::string_view BrightRed = "\033[91m";
    constexpr std::string_view BrightGreen = "\033[92m";
    constexpr std::string_view BrightYellow = "\033[93m";
    constexpr std::string_view BrightBlue = "\033[94m";
    constexpr std::string_view BrightMagenta = "\033[95m";
    constexpr std::string_view BrightCyan = "\033[96m";
    constexpr std::string_view BrightWhite = "\033[97m";

    // Styles
    constexpr std::string_view Bold = "\033[1m";
    constexpr std::string_view Underline = "\033[4m";

    // Reset all styles and colors
    constexpr std::string_view ResetColor = "\033[0m";

    inline void InfoLog(const std::string_view msg,
                        const std::source_location loc = std::source_location::current())
    {
#ifndef NDEBUG
        std::println("{}[Info] {}: {}{}", ResetColor, loc.function_name(), msg, ResetColor);
#else
        (void)msg;
        (void)loc;
#endif
    }

    inline void WarnLog(const std::string_view msg,
                        const std::source_location loc = std::source_location::current())
    {
#ifndef NDEBUG
        std::println("{}[Warning] {}: {}{}", Yellow, loc.function_name(), msg, ResetColor);
#else
        (void)msg;
        (void)loc;
#endif
    }

    inline void ErrorLog(const std::string_view msg,
                         const std::source_location loc = std::source_location::current())
    {
#ifndef NDEBUG
        std::println("{}[Error] {}: {}{}", Red, loc.function_name(), msg, ResetColor);
#else
        (void)msg;
        (void)loc;
#endif
    }

}   // namespace Guch2D
