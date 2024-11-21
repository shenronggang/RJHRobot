#ifndef DRIVERFACTORY_H
#define DRIVERFACTORY_H

#include "DriverBase.h"
#include "BlackRobot.hpp"
#include "BRobot.hpp"
#include <memory>
#include <string>

class DriverFactory {
public:
    static std::unique_ptr<DriverBase> createDriver(const std::string& type);
};

using DriverFactoryPtr = std::shared_ptr<DriverFactory>;

#endif // DRIVERFACTORY_H