/**
 * @file: StyLog.cpp
 * @brief log support
 * @author zhangxin
 * @date 2023-2-29
 */
#include "StyLog.h"
#include "string.h"

namespace components {
	// c++11 can not call ctor directly
	const LogMessage& initLogMessage(const char* file, int line, 
		LogSeverity severity) {
	    LogMessage msg(file, line, severity);
		return msg;
	}

    std::mutex LogMessage::g_log_mtx;

    LogMessage::LogMessage(const char* file, int line, LogSeverity severity):
        data_(new LogMessageData()) {
        Init(file, line, severity);
    }

    void LogMessage::Init(const char* file, int line, LogSeverity severity) {
        data_->severity_ = severity;
        data_->line_ = line;
        data_->outvec_ = nullptr;

        const auto now = std::chrono::system_clock::now();
        time_ = components::StyTime(now);

        data_->num_chars_to_log_ = 0;
        data_->num_chars_to_syslog_ = 0;
        data_->basename_ = constBasename(file);
        data_->fullname_ = file;
        data_->has_been_flushed_ = false;
        data_->thread_id_ = std::this_thread::get_id();

        // If specified, prepend a prefix to each line.  For example:
        //    I20201018 160715 f5d4fbb0 logging.cc:1153]
        //    (log level, GMT year, month, date, time, thread_id, file basename, line)
        // We exclude the thread_id for the default thread.
        std::ios saved_fmt(nullptr);
        saved_fmt.copyfmt(stream());
        stream().fill('0');
        stream() << '[' << std::setw(2) << 1 + time_.month() << std::setw(2) << time_.day() << ' '
            << std::setw(2) << time_.hour() << ':' << std::setw(2) << time_.min()
            << ':' << std::setw(2) << time_.sec() << "." << std::setw(6)
            << time_.usec() << ' ' << std::setfill(' ') << std::setw(5)
            << data_->thread_id_ << std::setfill('0') << ' ' << data_->basename_
            << ':' << data_->line_ << "] ";
        stream().copyfmt(saved_fmt);
        data_->num_prefix_chars_ = data_->stream_.pcount();

        char fileline[128];
        std::snprintf(fileline, sizeof(fileline), "%s:%d", data_->basename_, line);
    }

    LogSeverity LogMessage::severity() const noexcept { return data_->severity_; }

    int LogMessage::line() const noexcept { return data_->line_; }

    const char* LogMessage::constBasename(const char* filepath) {
        const char* base = strrchr(filepath, '/');
#ifdef WIN32  // Look for either path separator in Windows
        if (!base) base = strrchr(filepath, '\\');
#endif // WIN32
        return base ? (base + 1) : filepath;
    }


} // components
