#include "DriverFactory.hpp"

std::unique_ptr<DriverBase> DriverFactory::createDriver(const std::string &type)
{
    if (type == "BlackRobot")
    {
        return std::make_unique<BlackRobot>();
    }
    else if (type == "BRobot")
    {
        return std::make_unique<BRobot>();
        /* code */
    }
    throw std::invalid_argument("Unknown driver type");
}