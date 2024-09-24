#pragma once

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <mutex>
#include <unordered_map>

namespace sensing
{

class LidarImagePointcloud : public rclcpp::Node
{
public:
    LidarImagePointcloud(const rclcpp::NodeOptions &options);

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);

    void subscribe_to_cameras(std::vector<std::string> &cam_names);

    void process_slice(sensor_msgs::msg::PointCloud2 &rgb_pointcloud,
                       sensor_msgs::PointCloud2Iterator<float> iter_x, size_t start_idx,
                       size_t end_idx);

    void extract_transforms(std::vector<std::string> &cam_names);

    rmw_qos_profile_t get_topic_qos_profie(rclcpp::Node *node, const std::string &optic);

    // Data members
    std::unordered_map<std::string, sensor_msgs::msg::Image::ConstSharedPtr> images_;
    std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::ConstSharedPtr> cam_info_map_;
    std::unordered_map<std::string, tf2::Transform> cam_tf_map_;

    std::mutex data_mutex_;

    std::string lidar_frame_;
    std::vector<std::string> cam_names_;
    size_t num_threads_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Subscribers
    std::vector<image_transport::CameraSubscriber> image_subscribers_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgb_pointcloud_pub_;
};
} // namespace sensing