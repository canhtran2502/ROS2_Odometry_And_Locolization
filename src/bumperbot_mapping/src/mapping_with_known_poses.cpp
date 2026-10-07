#include "bumperbot_mapping/mapping_with_known_poses.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

using namespace std::placeholders;

namespace bumperbot_mapping
{
    Pose coordinatesToPose(const double px, const double py, const nav_msgs::msg::MapMetaData & map_info)
    {
        Pose pose;
        pose.x = std::round((px - map_info.origin.position.x)/map_info.resolution);
        pose.y = std::round((py - map_info.origin.position.y)/map_info.resolution);
        return pose;
    }

    bool poseOnMap(const Pose & pose, const nav_msgs::msg::MapMetaData & map_info)
    {
        return pose.x < static_cast<int>(map_info.width) && pose.x >= 0 &&
                pose.y < static_cast<int>(map_info.height) && pose.y >= 0;
    }

    unsigned int poseToCell(const Pose & pose, const nav_msgs::msg::MapMetaData &map_info) 
    {
        return map_info.width * pose.y + pose.x;
    }

    MappingWithKnownPoses::MappingWithKnownPoses(const std::string &name) : Node(name)
    {
        declare_parameter<double>("width", 50.0);
        declare_parameter<double>("height", 50.0);
        declare_parameter<double>("resolution", 0.1);

        double width = get_parameter("width").as_double();
        double height = get_parameter("height").as_double();
        map_.info.resolution = get_parameter("resolution").as_double();
        map_.info.width = std::round(width / map_.info.resolution);
        map_.info.height = std::round(height / map_.info.resolution);
        map_.info.origin.position.x = - std::round(width / 2.0);
        map_.info.origin.position.y = - std::round(height / 2.0);
        map_.header.frame_id = "odom";
        map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1);

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&MappingWithKnownPoses::scanCallback, this, _1));
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MappingWithKnownPoses::timerCallback, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan & scan)
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(map_.header.frame_id, scan.header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TimeoutException &exc){
            RCLCPP_ERROR(get_logger(), "Unable to transform between /odom and /base_footprint");
            return;
        }
        
        Pose robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, map_.info);
        if (!poseOnMap(robot_p, map_.info)){
            RCLCPP_ERROR(get_logger(), "The robot is out of the Map");
            return;
        }

        unsigned int robot_cell = poseToCell(robot_p, map_.info);
        map_.data.at(robot_cell) = 100;
    }

    void MappingWithKnownPoses::timerCallback()
    {
        map_.header.stamp = get_clock()->now();
        map_pub_->publish(map_);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_mapping::MappingWithKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}