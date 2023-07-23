#include <memory>
#include <string>
#include <sys/resource.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unistd.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::vector<size_t> getCPUtimes() {
    std::ifstream proc_stat("/proc/stat");
    proc_stat.ignore(5, ' ');
    std::vector<size_t> times;
    for (size_t time; proc_stat >> time; times.push_back(time));
    return times;
}

bool getCPUtimes(size_t &idleTime, size_t &totalTime) {
    const std::vector<size_t> CPUtimes = getCPUtimes();
    if (CPUtimes.size() < 4)
        return false;
    idleTime = CPUtimes[3];
    totalTime = std::accumulate(CPUtimes.begin(), CPUtimes.end(), 0);
    return true;
}

class Publisher : public rclcpp::Node
{
public:
    Publisher()
        : Node("CPU_publisher"), count_(0)
    {
         // Creating a publisher with the topic "CPU_usage" and a message queue size of 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("CPUload", 10);
        // Adding a timer with a duration of 5 seconds
               timer_ = this->create_wall_timer(
            5000ms, std::bind(&Publisher::timer_callback, this));

          // Appending new data to the end of the file stream
        fileStream.open("cpu_load.log", std::ios::app);
        if (!fileStream.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to open log file.");
        }
    }

private:
    void timer_callback()
    {
        
    
        size_t idleTime, totalTime;

        //get  CPU time from /proc/stat
        getCPUtimes(idleTime, totalTime);
        // Calculating the CPU usage in percentage by comparing the previous and current values of idleTime and totalTime
        double cpu_usage = 100.0 * (1.0 - static_cast<double>(idleTime - previous_idleTime_) / static_cast<double>(totalTime - previous_totalTime_));

        double cpu_usage_relative = cpu_usage / get_number_cores();

        fileStream << "Relative CPU Usage : " << cpu_usage_relative << std::endl;

        previous_idleTime_ = idleTime;
        previous_totalTime_ = totalTime;
    }

    size_t get_number_cores() const {

        return std::thread::hardware_concurrency();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    size_t previous_idleTime_;
    size_t previous_totalTime_;
    std::ofstream fileStream;
 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}

