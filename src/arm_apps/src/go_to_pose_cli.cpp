
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
    struct CliOptions
    {
        double x = 0.45;
        double y = 0.0;
        double z = 0.30;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    static void printUsage(const char *prog)
    {
        std::cerr
            << "Usage: " << prog << " [--x X] [--y Y] [--z Z] [--roll R] [--pitch P] [--yaw Y]\n"
            << "Defaults: x=0.45 y=0.0 z=0.30 roll=0 pitch=0 yaw=0\n";
    }

    static double parseDouble(const std::string &value, const std::string &flag)
    {
        try
        {
            size_t idx = 0;
            const double v = std::stod(value, &idx);
            if (idx != value.size())
            {
                throw std::runtime_error("Invalid numeric value for " + flag + ": '" + value + "'");
            }
            return v;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Failed to parse " + flag + ": '" + value + "'");
        }
    }

    static CliOptions parseArgs(int argc, char **argv)
    {
        CliOptions opt;
        for (int i = 1; i < argc; ++i)
        {
            const std::string a(argv[i]);
            if (a == "--help" || a == "-h")
            {
                printUsage(argv[0]);
                std::exit(0);
            }

            auto requireValue = [&](const std::string &flag) -> std::string
            {
                if (i + 1 >= argc)
                {
                    throw std::runtime_error("Missing value after " + flag);
                }
                return std::string(argv[++i]);
            };

            if (a == "--x")
                opt.x = parseDouble(requireValue(a), a);
            else if (a == "--y")
                opt.y = parseDouble(requireValue(a), a);
            else if (a == "--z")
                opt.z = parseDouble(requireValue(a), a);
            else if (a == "--roll")
                opt.roll = parseDouble(requireValue(a), a);
            else if (a == "--pitch")
                opt.pitch = parseDouble(requireValue(a), a);
            else if (a == "--yaw")
                opt.yaw = parseDouble(requireValue(a), a);
            else
            {
                throw std::runtime_error("Unknown argument: " + a);
            }
        }
        return opt;
    }

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        const auto opt = parseArgs(argc, argv);

        auto node = std::make_shared<rclcpp::Node>("go_to_pose");
        node->declare_parameter<std::string>("target_topic", "/target_pose");
        node->declare_parameter<std::string>("frame_id", "base_link");

        const std::string target_topic = node->get_parameter("target_topic").as_string();
        const std::string frame_id = node->get_parameter("frame_id").as_string();

        auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 10);

        // Discovery delay (required for one-shot publishers)
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // Extra robustness: wait briefly for at least one subscriber if possible.
        // Still bounded, so the tool will exit even if no subscriber exists.
        const auto wait_start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && pub->get_subscription_count() == 0 &&
               (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
        {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = frame_id;

        msg.pose.position.x = opt.x;
        msg.pose.position.y = opt.y;
        msg.pose.position.z = opt.z;

        tf2::Quaternion q;
        q.setRPY(opt.roll, opt.pitch, opt.yaw);
        q.normalize();
        msg.pose.orientation = tf2::toMsg(q);

        pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),
                    "Published target pose to %s frame=%s pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] subs=%zu",
                    target_topic.c_str(), frame_id.c_str(),
                    opt.x, opt.y, opt.z,
                    opt.roll, opt.pitch, opt.yaw,
                    pub->get_subscription_count());

        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        printUsage(argv[0]);
        rclcpp::shutdown();
        return 2;
    }

    rclcpp::shutdown();
    return 0;
}
