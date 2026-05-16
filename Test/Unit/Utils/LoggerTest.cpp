#include "Utils/Logger.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(LoggerTest, InfoLogWritesExpectedPrefixAndMessage)
    {
        constexpr std::string_view Message = "info message";

        testing::internal::CaptureStdout();
        Guch2D::InfoLog(Message);
        const std::string output = testing::internal::GetCapturedStdout();

#ifndef NDEBUG
        EXPECT_NE(output.find("[Info]"), std::string::npos);
        EXPECT_NE(output.find(Message), std::string::npos);
#else
        EXPECT_TRUE(output.empty());
#endif
    }

    TEST(LoggerTest, WarnLogWritesExpectedPrefixAndMessage)
    {
        constexpr std::string_view Message = "warning message";

        testing::internal::CaptureStdout();
        Guch2D::WarnLog(Message);
        const std::string output = testing::internal::GetCapturedStdout();

#ifndef NDEBUG
        EXPECT_NE(output.find("[Warning]"), std::string::npos);
        EXPECT_NE(output.find(Message), std::string::npos);
#else
        EXPECT_TRUE(output.empty());
#endif
    }

    TEST(LoggerTest, ErrorLogWritesExpectedPrefixAndMessage)
    {
        constexpr std::string_view Message = "error message";

        testing::internal::CaptureStdout();
        Guch2D::ErrorLog(Message);
        const std::string output = testing::internal::GetCapturedStdout();

#ifndef NDEBUG
        EXPECT_NE(output.find("[Error]"), std::string::npos);
        EXPECT_NE(output.find(Message), std::string::npos);
#else
        EXPECT_TRUE(output.empty());
#endif
    }
}   // namespace
