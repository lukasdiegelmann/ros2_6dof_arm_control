
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cstdlib>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
  /*
   * Define Struct with all CLI options that can be specified. Furthermore
   * the default values are set here.
   */
  struct CliOptions {
    double x = 0.45;
    double y = 0.0;
    double z = 0.30;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
  };

  /*
   * Print usage information to stderr. In case the program was used incorrectly,
   * this function can be called to inform the user about the correct usage.
   */
  static void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " [--x X] [--y Y] [--z Z] [--roll R] [--pitch P] [--yaw Y]\n"
              << "Defaults: x=0.45 y=0.0 z=0.30 roll=0 pitch=0 yaw=0\n";
  }

  /*
   * Parse a string into a double, throwing an exception if parsing fails.
   */
  static double parseDouble(const std::string& value, const std::string& flag) {
    try {
      size_t idx = 0;
      const double v = std::stod(value, &idx);
      if (idx != value.size()) {
        throw std::runtime_error("Invalid numeric value for " + flag + ": '" + value + "'");
      }
      return v;
    } catch (const std::exception& e) {
      throw std::runtime_error("Failed to parse " + flag + ": '" + value + "'");
    }
  }

  /*
   * Parse command line arguments into the CliOptions struct.
   */
  static CliOptions parseArgs(int argc, char** argv) {
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--help" || a == "-h") {
        printUsage(argv[0]);
        std::exit(0);
      }

      auto requireValue = [&](const std::string& flag) -> std::string {
        if (i + 1 >= argc) {
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
      else {
        throw std::runtime_error("Unknown argument: " + a);
      }
    }
    return opt;
  }

}  // namespace

int main(int argc, char** argv) {
  /* Initiating ROS2 Client Library with CLI arguments */
  rclcpp::init(argc, argv);

  /* Try to parse CLI arguments and publish the target pose */
  try {
    /* Parse all CLI arguments, so they can be used to generate the target pose */
    const auto args = parseArgs(argc, argv);

    /*
     * Create a ROS2 node for publishing the target pose. The node will have
     * the name "go_to_pose" and accept two parameters. The "target_topic" parameter
     * specifies the topic to publish the target pose to, and the "frame_id" parameter
     * specifies the reference frame for the target pose.
     */
    auto node = std::make_shared<rclcpp::Node>("go_to_pose");
    node->declare_parameter<std::string>("target_topic", "/target_pose");
    node->declare_parameter<std::string>("frame_id", "base_link");

    const std::string target_topic = node->get_parameter("target_topic").as_string();
    const std::string frame_id = node->get_parameter("frame_id").as_string();

    /*
     * Create a publisher for PoseStamped messages on the target topic. The
     * publisher will remember up to 10 messages if the subscribers are too slow
     * to keep up.
     *
     * The PoseStamped contains a position in 3d space (x,y,z) and an orientation in
     * 3d space (in quaternion representation, (x,y,z,w)).
     */
    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 10);

    /*
     * Discovery delay (required for one-shot publishers). If this is not written
     * the publisher would shoot its message with no subscribers being present yet.
     * In ROS2 it takes some time until the publisher and subscriber discover each other.
     */
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    /*
     * This part is for extra robustness. It makes the publisher wait until at
     * least one subscriber is present (up to 2 seconds). But its still bounded
     * if the publisher waits for more than 2 seconds, it will continue to publish
     * anyway.
     */
    const auto wait_start = std::chrono::steady_clock::now(); /* Initialize timer */
    while (
        /* get current subscription count from publisher */
        rclcpp::ok() && pub->get_subscription_count() == 0 &&
        /* get difference between current time and the timer start, compare to 2 secs */
        (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2)) {
      /* get all updates from ROS2 loop */
      rclcpp::spin_some(node);
      /* reduce CPU usage */
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    /* Create the PoseStamped message */
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node->now(); /* Set the timestamp to the current time */
    msg.header.frame_id = frame_id; /* Set the reference frame for the pose */

    /* Forward position from arguments */
    msg.pose.position.x = args.x;
    msg.pose.position.y = args.y;
    msg.pose.position.z = args.z;

    /* Create Quaternion from roll, pitch, yaw */
    tf2::Quaternion q;
    q.setRPY(args.roll, args.pitch, args.yaw);
    /*
     * Normalize the quaternion to ensure its length is one (unitquaternion). This
     * is to avoid errors that pile up over time.
     */
    q.normalize();
    msg.pose.orientation = tf2::toMsg(q);

    /* Publish the PoseStamped message and log it */
    pub->publish(msg);
    RCLCPP_INFO(
        node->get_logger(),
        "Published target pose to %s frame=%s pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] subs=%zu",
        target_topic.c_str(), frame_id.c_str(), args.x, args.y, args.z, args.roll, args.pitch,
        args.yaw, pub->get_subscription_count());

    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
  /* Should there be an error shutdown the ROS2 Client and print the message */
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    printUsage(argv[0]); /* Inform the user about correct usage */
    rclcpp::shutdown();
    return 2;
  }

  /* Shutdown the ROS2 Client Library */
  rclcpp::shutdown();
  return 0;
}
