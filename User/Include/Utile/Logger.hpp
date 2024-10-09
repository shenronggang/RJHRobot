#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <thread>

class Logger {
public:
    explicit Logger(const std::string &folderPath);
    ~Logger();

    void logWrite(const std::string &message);

private:
    std::string filePath_;
    std::ofstream logFile_;
    std::mutex mutex_;

    std::string currentDateTime();
    std::string getLogFileName(const std::string &folderPath);
};

#endif // LOGGER_H
