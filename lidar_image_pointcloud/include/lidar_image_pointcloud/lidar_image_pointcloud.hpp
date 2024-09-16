#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "image_transport/image_transport.hpp"
#include <mutex>

#include "image_geometry/pinhole_camera_model.h"



namespace sensing
{
    

class LidarImagePointcloud : public rclcpp::Node
{
public:
    LidarImagePointcloud(const rclcpp::NodeOptions & options);

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    void rectify_image();

    bool extract_transform(const std::string& target_frame, const std::string& source_frame);

    rmw_qos_profile_t get_topic_qos_profie(rclcpp::Node *node, const std::string &optic);

    // Data members
    sensor_msgs::msg::Image::ConstSharedPtr last_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    geometry_msgs::msg::TransformStamped transform_;

    std::mutex image_mutex_;  // Mutex to ensure thread safety when accessing last_image_

    std::string lidar_frame_;
    std::string camera_frame_;
    image_geometry::PinholeCameraModel model_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgb_pointcloud_pub_;
};
} // namespace sensing