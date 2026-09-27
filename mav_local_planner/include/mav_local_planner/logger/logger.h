#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>

#include <ros/ros.h>

// Helper functions

std::string rosTimeToString(const ros::Time& ros_time)
{
    // Convert ros::Time to std::chrono::system_clock::time_point
    std::chrono::time_point<std::chrono::system_clock> tp =
        std::chrono::time_point<std::chrono::system_clock>(
            std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nsec));

    // Convert time_point to time_t
    std::time_t time = std::chrono::system_clock::to_time_t(tp);

    // Convert time_t to tm structure for formatting
    std::tm tm = *std::localtime(&time);

    // Format the time to a string
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S"); // Change the format as needed
    return oss.str();
}

// End Helper functions

class Logger {
public:
    Logger () {
        logged_msgs_ = 0;
    }

    int GetNumberOfLoggedMsgs()
    {
        return logged_msgs_;
    }

    void Initialze(const std::string log_directory, const std::string log_file) 
    {
        std::cout << "log_directory: " << log_directory << std::endl;
        std::cout << "log_file: " << log_file << std::endl;
        char* home_dir = getenv("HOME");
        if (home_dir) {
            log_directory_ = home_dir + log_directory;
        } else {
            log_directory_ = log_directory;
        }
        log_file_path_ = log_directory_ + log_file;

        logging_start_ = ros::Time::now();

        // Ensure the directory exists
        // Ensure the log directory exists
        if (mkdir(log_directory_.c_str(), 0755) == -1 && errno != EEXIST) {
            std::cerr << "Could not create this directory: " << log_directory_ << std::endl;
        }

        // Open the file in append mode (create if doesn't exist)
        log_stream_.open(log_file_path_, std::ios::app);
        if (!log_stream_.is_open()) {
            std::cerr << "Could not open this file: " << log_file_path_ << std::endl;
        } else {
            ROS_INFO("Logger is set.");
        }

        // Log the starting time
        std::string initial_message = "Starting log at time " + rosTimeToString(logging_start_) + "\n";
        this->Log(initial_message);
    }

    // Destructor to close the log file
    ~Logger()
    {
        if (log_stream_.is_open()) {
            // Log the ending time
            std::string final_message = "Finishing log at time " + rosTimeToString(ros::Time::now()) + "\n" + "Total logged messages = " + std::to_string(logged_msgs_) + "\n";
            this->Log(final_message);
            log_stream_.close();
        }
    }

    void Log(const std::string& message) {
        if (log_stream_.is_open()) {
            log_stream_ << message << std::endl;
            logged_msgs_++;
        } else {
            ROS_ERROR("Log file is not open.");
        }
    }

private:
    std::string log_directory_;
    std::string log_file_path_;
    std::ofstream log_stream_;
    int logged_msgs_;

    ros::Time logging_start_;
};