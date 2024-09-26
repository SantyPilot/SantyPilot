/**
 * @file: StyTime.cpp
 * @brief clock support: >=C++11
 * @author santypilot 
 * @date 2023-2-29
 */
#include "StyTime.h"

namespace components {
StyTime::StyTime(std::chrono::system_clock::time_point now)
    : timestamp_{ now } {
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    usecs_ = std::chrono::duration_cast<std::chrono::microseconds>(
        now - std::chrono::system_clock::from_time_t(timestamp));

    tm_ = *localtime(&timestamp); // Note!!! not thread safe!
    std::tm tm_utc; // gmt time
    tm_utc = *gmtime(&timestamp); // Note!!! not thread safe!
    int isdst = tm_.tm_isdst;
    std::time_t gmt_sec = std::mktime(&tm_utc);
    gmtoffset_ = std::chrono::duration_cast<std::chrono::hours>(
        now - std::chrono::system_clock::from_time_t(gmt_sec) +
        std::chrono::hours(isdst ? 1 : 0));
}

} // components
