/**
 * @file: StyTime.h
 * @brief clock support: >=C++11
 * @author santypilot 
 * @date 2023-2-29
 */
#ifndef _STY_TIME_H
#define _STY_TIME_H
#include <ctime>
#include <chrono>

/*
 * usage:
 *  std::stringstream ss;
 *  const auto now = std::chrono::system_clock::now();
 *  auto time = components::ISTime(now);
 *  ss << '[' << std::setw(2) << 1 + time.month() << std::setw(2) << time.day() << ' '
 *      << std::setw(2) << time.hour() << ':' << std::setw(2) << time.min()
 *      << ':' << std::setw(2) << time.sec() << "." << std::setw(6)
 *      << time.usec() << "] ";
 *  ss.str();
 */
namespace components {
class StyTime {
public:
    StyTime() = default;
    explicit StyTime(std::chrono::system_clock::time_point now);

    const std::chrono::system_clock::time_point& when() const noexcept {
        return timestamp_;
    }
    int sec() const noexcept { return tm_.tm_sec; }
    long usec() const noexcept { return usecs_.count(); }
    int(min)() const noexcept { return tm_.tm_min; }
    int hour() const noexcept { return tm_.tm_hour; }
    int day() const noexcept { return tm_.tm_mday; }
    int month() const noexcept { return tm_.tm_mon; }
    int year() const noexcept { return tm_.tm_year; }
    int dayOfWeek() const noexcept { return tm_.tm_wday; }
    int dayInYear() const noexcept { return tm_.tm_yday; }
    int dst() const noexcept { return tm_.tm_isdst; }
    // gmt: Greenwich Mean Time
    std::chrono::seconds gmtoffset() const noexcept { return gmtoffset_; }
    const std::tm& tm() const noexcept { return tm_; }

private:
    std::tm tm_{};  // Time of creation of LogMessage
    std::chrono::system_clock::time_point
        timestamp_;  // Time in seconds
    std::chrono::microseconds usecs_;
    std::chrono::seconds gmtoffset_;
};

} // components

#endif // _STY_TIME_H
