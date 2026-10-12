#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr std::size_t kColumnCount = 38;

std::string homeDirectory()
{
    const char *home = std::getenv("HOME");
    if (home == nullptr || *home == '\0')
    {
        throw std::runtime_error("HOME is not set; provide an absolute ~output_path");
    }
    return home;
}

std::string defaultOutputPath()
{
    const std::string directory = homeDirectory() + "/.ros";
    if (::mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST)
    {
        throw std::runtime_error("Cannot create " + directory + ": " + std::strerror(errno));
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t wall_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time;
    if (::localtime_r(&wall_time, &local_time) == nullptr)
    {
        throw std::runtime_error("Cannot format the recording timestamp");
    }
    const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count() % 1000000;
    std::ostringstream path;
    path << directory << "/record_curves_"
         << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_'
         << std::setfill('0') << std::setw(6) << microseconds << '_'
         << ::getpid() << ".csv";
    return path.str();
}
} // namespace

class DataRecorder
{
public:
    DataRecorder()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("output_path", output_path_, "");
        if (output_path_.empty())
        {
            output_path_ = defaultOutputPath();
        }
        else if (output_path_.compare(0, 2, "~/") == 0)
        {
            output_path_ = homeDirectory() + output_path_.substr(1);
        }

        // Exclusive creation refuses existing files, including symbolic links.
        // A caller-specified parent directory must already exist.
        const int fd = ::open(output_path_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
        if (fd == -1)
        {
            throw std::runtime_error("Cannot create " + output_path_ + ": " + std::strerror(errno));
        }
        std::FILE *stream = ::fdopen(fd, "w");
        if (stream == nullptr)
        {
            const int saved_errno = errno;
            ::close(fd);
            throw std::runtime_error("Cannot open CSV stream: " + std::string(std::strerror(saved_errno)));
        }
        file_.reset(stream);

        std::ostringstream header;
        for (std::size_t i = 0; i < kColumnCount; ++i)
        {
            if (i != 0)
                header << ',';
            header << "data[" << i << ']';
        }
        header << '\n';
        if (!writeRow(header.str()))
        {
            throw std::runtime_error("Cannot write CSV header to " + output_path_);
        }

        subscriber_ = nh.subscribe("/hat_x_topic", 1000, &DataRecorder::callback, this);
        ROS_INFO_STREAM("Recording 38-column telemetry to " << output_path_);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (failed_ || msg->data.size() < kColumnCount)
            return;

        // Keep the original data[0] through data[37] order and numeric formatting.
        std::ostringstream row;
        for (std::size_t i = 0; i < kColumnCount; ++i)
        {
            if (i != 0)
                row << ',';
            row << msg->data[i];
        }
        row << '\n';
        if (!writeRow(row.str()))
        {
            failed_ = true;
            ROS_FATAL_STREAM("Failed to write recording: " << output_path_);
            ros::shutdown();
        }
    }

    bool finish()
    {
        subscriber_.shutdown();
        if (file_ && std::fclose(file_.release()) != 0)
        {
            failed_ = true;
            ROS_FATAL_STREAM("Failed to close recording: " << output_path_);
        }
        return !failed_;
    }

private:
    bool writeRow(const std::string &row)
    {
        return std::fwrite(row.data(), 1, row.size(), file_.get()) == row.size()
            && std::fflush(file_.get()) == 0;
    }

    ros::Subscriber subscriber_;
    std::unique_ptr<std::FILE, decltype(&std::fclose)> file_{nullptr, &std::fclose};
    std::string output_path_;
    bool failed_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_recorder");
    try
    {
        DataRecorder recorder;
        ros::spin();
        return recorder.finish() ? 0 : 1;
    }
    catch (const std::exception &e)
    {
        ROS_FATAL_STREAM("Recorder failed: " << e.what());
        return 1;
    }
}
