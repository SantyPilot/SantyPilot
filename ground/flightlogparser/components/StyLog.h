/**
 * @file: StyLog.h 
 * @brief log support, 
 * simple implementation of glog
 * refer to: https://github.com/google/glog
 * @author santypilot 
 * @date 2023-2-29
 */
#ifndef _STY_LOG_H
#define _STY_LOG_H

#include <ostream>
#include <fstream>
#include <iomanip>
#include "StyIO.h"
#include "StyTime.h"
#include <mutex> // std::mutex
#include <iostream>
#include <thread>
#include <vector>

#define LOG(severity) COMPACT_STY_LOG_##severity.stream()

#define COMPACT_STY_LOG_INFO components::initLogMessage(__FILE__, __LINE__, components::LogSeverity::STYLOG_INFO)
#define COMPACT_STY_LOG_WARNING components::initLogMessage(__FILE__, __LINE__, components::LogSeverity::STYLOG_WARNING)
#define COMPACT_STY_LOG_ERROR components::initLogMessage(__FILE__, __LINE__, components::LogSeverity::STYLOG_ERROR)
#define COMPACT_STY_LOG_FATAL components::initLogMessage(__FILE__, __LINE__, components::LogSeverity::STYLOG_FATAL)

// #define ENABLE_LOG_FILE // TODO: add this to global config!!!
#ifdef ENABLE_LOG_FILE
    #define LOGFILE_PATH "C:\\Users\\intesim\\Desktop\\islog.txt"
#endif // ENABLE_LOG_FILE

namespace components {
const size_t LOG_MSG_MAX_LEN = 30000;


enum LogSeverity {
	STYLOG_INFO = 0,
	STYLOG_WARNING,
	STYLOG_ERROR,
	STYLOG_FATAL
};
class LogStreamBuf: public std::streambuf {
public:
    // REQUIREMENTS: "len" must be >= 2 to account for the '\n' and '\0'.
    LogStreamBuf(char* buf, int len) { setp(buf, buf + len - 2); }

    // This effectively ignores overflow.
    int_type overflow(int_type ch) { return ch; }

    // Legacy public ostrstream method.
    size_t pcount() const { return static_cast<size_t>(pptr() - pbase()); }
    char* pbase() const { return std::streambuf::pbase(); }
};

class LogStream : public std::ostream {
public:
    LogStream(char* buf, int len, int64_t ctr)
        : std::ostream(nullptr), streambuf_(buf, len), ctr_(ctr), self_(this) {
        rdbuf(&streambuf_);
		ctr = 0;
    }

    LogStream(LogStream&& other) noexcept
        : std::ostream(nullptr),
        streambuf_(std::move(other.streambuf_)),
        ctr_(other.ctr_),
        self_(this) {
        rdbuf(&streambuf_);
		other.ctr_ = 0;
    }

    LogStream& operator=(LogStream&& other) noexcept {
        streambuf_ = std::move(other.streambuf_);
        ctr_ = other.ctr_;
        rdbuf(&streambuf_);
        return *this;
    }

    int64_t ctr() const { return ctr_; }
    void set_ctr(int64_t ctr) { ctr_ = ctr; }
    LogStream* self() const { return self_; }

    // Legacy std::streambuf methods.
    size_t pcount() const { return streambuf_.pcount(); }
    char* pbase() const { return streambuf_.pbase(); }
    char* str() const { return pbase(); }

    LogStream(const LogStream&) = delete;
    LogStream& operator=(const LogStream&) = delete;

private:
    LogStreamBuf streambuf_;
    int64_t ctr_;        // Counter hack (for the LOG_EVERY_X() macro)
    LogStream* self_;  // Consistency check hack
};

struct LogMessageData {
    LogMessageData()
      : stream_(message_text_, LOG_MSG_MAX_LEN, 0) {}
    LogStream stream_;
    LogSeverity severity_;  // What level is this LogMessage logged at?
    int line_;              // line number where logging call is.
    union {  // At most one of these is used: union to keep the size low.
        std::vector<std::string>*
            outvec_;            // nullptr or vector to push message onto
        std::string* message_;  // nullptr or string to write message into
    };
    size_t num_prefix_chars_;     // # of chars of prefix in this message
    size_t num_chars_to_log_;     // # of chars of msg to send to log
    size_t num_chars_to_syslog_;  // # of chars of msg to send to syslog
    const char* basename_;        // basename of file that called LOG
    const char* fullname_;        // fullname of file that called LOG
    bool has_been_flushed_;       // false => data has not been flushed
    bool first_fatal_;            // true => this was first fatal msg
    std::thread::id thread_id_;

    LogMessageData(const LogMessageData&) = delete;
    LogMessageData& operator=(const LogMessageData&) = delete;

    // Buffer space; contains complete message text. last char: \0
    char message_text_[LOG_MSG_MAX_LEN + 1];
};


class LogMessage {
public:
    LogMessage(const char* file, int line, LogSeverity severity);

    ~LogMessage() noexcept(false) {
        {
            std::lock_guard<std::mutex> lock{ g_log_mtx };
            Flush();
        }
        delete data_;
    };

    std::ostream& stream() const { return data_->stream_; }
protected :
    void Init(const char* file, int line, LogSeverity severity);

    // Flush a buffered message to the sink set in the constructor.  Always
    // called by the destructor, it may also be called from elsewhere if
    // needed.  Only the first call is actioned; any later ones are ignored.
    void Flush() {
        data_->num_chars_to_log_ = data_->stream_.pcount();
        data_->message_text_[data_->num_chars_to_log_ + 1] = '\0';
        std::cout << data_->message_text_ << "\r\n";
#ifdef ENABLE_LOG_FILE
        std::ofstream file;
        file.open(LOGFILE_PATH,  std::ios::app | std::ios::in | std::ios::out);
        file << std::string(data_->message_text_, data_->num_chars_to_log_) << std::endl;
        file.close();
#endif
    }

    // These should not be called directly outside of logging.*,
    // only passed as SendMethod arguments to other LogMessage methods:
    void SendToLogFile() {

    }

    LogSeverity severity() const noexcept;
    int line() const noexcept;

    LogMessage(const LogMessage&) = delete;
    LogMessage& operator=(const LogMessage&) = delete;

private:
    const char* constBasename(const char* filepath);
    
    // We keep the data in a separate struct so that each instance of
    // LogMessage uses less stack space.
    LogMessageData* data_;
    StyTime time_;
    static std::mutex g_log_mtx;
};

const LogMessage& initLogMessage(const char* file, int line, 
	LogSeverity severity);

} // components


#endif // _STY_LOG_H

