#include "Logger.hpp"

Logger::Logger(const std::string &folderPath) {
    filePath_ = getLogFileName(folderPath);
    logFile_.open(filePath_, std::ios::app);
    if (!logFile_) {
        std::cerr << "Failed to open log file: " << filePath_ << std::endl;
    }
}

Logger::~Logger() {
    if (logFile_.is_open()) {
        logFile_.close();
    }
}

void Logger::logWrite(const std::string &message) {
    // 创建线程并在其中执行日志写入
    std::thread([this, message]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (logFile_.is_open()) {
            logFile_ << currentDateTime() << " - " << message << std::endl;
        }
    }).detach(); // 分离线程
}

std::string Logger::currentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string Logger::getLogFileName(const std::string &folderPath) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::ostringstream oss;
    oss << folderPath << "/systemlog_" << std::put_time(&local_tm, "%Y-%m-%d") << ".txt";
    return oss.str();
}
