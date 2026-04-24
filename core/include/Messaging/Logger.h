#ifndef LOGGER_H
#define LOGGER_H

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

class Logger {
  public:
    enum class Level {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    void initialize(const std::string& logDirectory = "logs") {
        std::lock_guard guard(mutex);
        if (initialized) {
            return;
        }

        try {
            std::filesystem::create_directories(logDirectory);
            const std::string stamp = makeFileStamp();
            logFilePath = logDirectory + "/session_" + stamp + ".log";
            logFile.open(logFilePath, std::ios::out | std::ios::app);
            initialized = logFile.is_open();
        } catch (...) {
            initialized = false;
        }

        if (!initialized) {
            std::cerr << "[LOGGER] Failed to initialize log file." << std::endl;
            return;
        }

        writeUnlocked(Level::INFO, "LOGGER", "Initialized log file at " + logFilePath, false);
    }

    void log(Level level,
             const std::string& component,
             const std::string& message,
             bool echoTerminal = false) {
        std::lock_guard guard(mutex);
        writeUnlocked(level, component, message, echoTerminal);
    }

    void debug(const std::string& component, const std::string& message, bool echoTerminal = false) {
        log(Level::DEBUG, component, message, echoTerminal);
    }

    void info(const std::string& component, const std::string& message, bool echoTerminal = false) {
        log(Level::INFO, component, message, echoTerminal);
    }

    void warn(const std::string& component, const std::string& message, bool echoTerminal = false) {
        log(Level::WARN, component, message, echoTerminal);
    }

    void error(const std::string& component, const std::string& message, bool echoTerminal = false) {
        log(Level::ERROR, component, message, echoTerminal);
    }

    std::string getLogFilePath() {
        std::lock_guard guard(mutex);
        return logFilePath;
    }

  private:
    std::mutex mutex;
    std::ofstream logFile;
    bool initialized;
    std::string logFilePath;

    Logger() : mutex(), logFile(), initialized(false), logFilePath() {}

    static std::string levelToString(Level level) {
        switch (level) {
            case Level::DEBUG:
                return "DEBUG";
            case Level::INFO:
                return "INFO";
            case Level::WARN:
                return "WARN";
            case Level::ERROR:
                return "ERROR";
        }

        return "INFO";
    }

    static std::string makeTimeStamp() {
        const auto now = std::chrono::system_clock::now();
        const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
                        std::chrono::seconds(1);

        std::tm localTm{};
#if defined(_WIN32)
        localtime_s(&localTm, &nowTime);
#else
        localtime_r(&nowTime, &localTm);
#endif

        std::ostringstream out;
        out << std::put_time(&localTm, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0')
            << ms.count();
        return out.str();
    }

    static std::string makeFileStamp() {
        const auto now = std::chrono::system_clock::now();
        const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

        std::tm localTm{};
#if defined(_WIN32)
        localtime_s(&localTm, &nowTime);
#else
        localtime_r(&nowTime, &localTm);
#endif

        std::ostringstream out;
        out << std::put_time(&localTm, "%Y%m%d_%H%M%S");
        return out.str();
    }

    void writeUnlocked(Level level,
                       const std::string& component,
                       const std::string& message,
                       bool echoTerminal) {
        const std::string line =
            makeTimeStamp() + " [" + levelToString(level) + "] [" + component + "] " + message;

        if (initialized && logFile.is_open()) {
            logFile << line << '\n';
            if (level != Level::DEBUG) {
                logFile.flush();
            }
        }

        if (!echoTerminal) {
            return;
        }

        if (level == Level::WARN || level == Level::ERROR) {
            std::cerr << line << std::endl;
        } else {
            std::cout << line << std::endl;
        }
    }
};

#endif
