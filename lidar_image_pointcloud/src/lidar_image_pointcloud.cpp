#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>

#include "lidar_image_pointcloud/lidar_image_pointcloud.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>

namespace sensing
{

LidarImagePointcloud::LidarImagePointcloud(const rclcpp::NodeOptions &options)
    : Node("rgb_pointcloud_processor", options)
{

    cam_names_ = this->declare_parameter<std::vector<std::string>>("camera_names", {"fsp_l"});

    const std::string lidar_topic = "/sensor/lidar/top/points";
    const std::string rgb_pointcloud_topic = "/sensor/lidar/top/rgb_points";

    lidar_frame_ = "lidar_ouster_top";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10,
        std::bind(&LidarImagePointcloud::pointcloud_callback, this, std::placeholders::_1));

    subscribe_to_cameras(cam_names_);

    rgb_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("rgb_pointcloud", 10);

    extract_transforms(cam_names_);

    RCLCPP_INFO(this->get_logger(), "Lidar Image Pointcloud Initialised!");
}

void LidarImagePointcloud::subscribe_to_cameras(std::vector<std::string> &cam_names)
{

    // Create sync sub for each camera
    for (const auto &camera_name : cam_names)
    {
        std::string cam_topic = "/sensor/camera/" + camera_name + "/image_rect_color";

        image_subscribers_.emplace_back(image_transport::create_camera_subscription(
            this, cam_topic,
            [this, camera_name](const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info_msg)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                images_[camera_name] = image_msg;
                cam_info_map_[camera_name] = cam_info_msg;
                RCLCPP_DEBUG(this->get_logger(), "Image %i x %i received from %s and stored.",
                             image_msg->width, image_msg->height, camera_name.c_str());
            },
            "compressed"));
    }
}

void LidarImagePointcloud::extract_transforms(std::vector<std::string> &cam_names)
{

    for (const auto &camera_name : cam_names)
    {
        while (rclcpp::ok())
        {
            try
            {
                std::string cam_optical_frame = "camera_" + camera_name + "_optical";
                geometry_msgs::msg::TransformStamped transform_msg = tf_buffer_->lookupTransform(
                    cam_optical_frame, lidar_frame_, tf2::TimePointZero);

                tf2::fromMsg(transform_msg.transform, cam_tf_map_[camera_name]);
                RCLCPP_INFO(this->get_logger(), "%s -> %s tf found!", cam_optical_frame.c_str(),
                            lidar_frame_.c_str());
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                rclcpp::sleep_for(std::chrono::seconds(2));
            }
        }
    }
}

void LidarImagePointcloud::pointcloud_callback(
    const sensor_msgs ::msg::PointCloud2::SharedPtr pointcloud_msg)
{
    // In case the node executor is multi-threaded
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto start = std::chrono::high_resolution_clock::now();

    if (images_.size() != cam_names_.size())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for images");
        return;
    }

    if (cam_info_map_.size() != cam_info_map_.size())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for cameras info");
        return;
    }

    // Create RGB PointCloud2
    sensor_msgs::msg::PointCloud2 rgb_pointcloud = *pointcloud_msg;
    sensor_msgs::PointCloud2Modifier pcd_modifier(rgb_pointcloud);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Iterate over point cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(rgb_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(rgb_pointcloud, "rgb");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_rgb)
    {
        float x = iter_x[0];
        float y = iter_x[1];
        float z = iter_x[2];

        tf2::Vector3 lidar_point(x, y, z);

        for (const auto &[cam_name, img_msg] : images_)
        {
            // Transform lidar point into camera frame
            auto cam_cartesian_point = cam_tf_map_[cam_name] * lidar_point;

            // Project into image plane
            float u = (cam_info_map_[cam_name]->k[0] * cam_cartesian_point.x() /
                       cam_cartesian_point.z()) +
                      cam_info_map_[cam_name]->k[2];
            float v = (cam_info_map_[cam_name]->k[4] * cam_cartesian_point.y() /
                       cam_cartesian_point.z()) +
                      cam_info_map_[cam_name]->k[5];

            if (cam_cartesian_point.z() > 0 && u >= 0 && u < img_msg->width && v >= 0 &&
                v < img_msg->height)
            {
                // Get pixel color from image
                int idx = static_cast<int>(v) * img_msg->step + static_cast<int>(u) * 3;
                iter_rgb[0] = img_msg->data[idx];
                iter_rgb[1] = img_msg->data[idx + 1];
                iter_rgb[2] = img_msg->data[idx + 2];
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    RCLCPP_INFO(this->get_logger(), "Computaton time: %f ms", duration.count());
    // Publish the RGB PointCloud
    rgb_pointcloud_pub_->publish(rgb_pointcloud);
}

rmw_qos_profile_t LidarImagePointcloud::get_topic_qos_profie(rclcpp::Node *node,
                                                             const std::string &topic)
{
    /**
    * Given a topic name, get the QoS profile with which it is being published.
￼   * Replaces history and depth settings with default sensor values since they cannot be retrieved.
    * @param node pointer to the ROS node
    * @param topic name of the topic
    * @returns QoS profile of the publisher to the topic. If there are several publishers, it
returns
    *     returns the profile of the first one on the list. If no publishers exist, it returns
    *     the sensor data profile.
    */

    std::string topic_resolved =
        node->get_node_base_interface()->resolve_topic_or_service_name(topic, false);
    auto topics_info = node->get_publishers_info_by_topic(topic_resolved);
    if (topics_info.size())
    {
        auto profile = topics_info[0].qos_profile().get_rmw_qos_profile();
        profile.history = rmw_qos_profile_sensor_data.history;
        profile.depth = rmw_qos_profile_sensor_data.depth;
        return profile;
    }
    else
    {
        return rmw_qos_profile_sensor_data;
    }
}

// void LidarImagePointcloud::rectify_image()
// {
//     std::lock_guard<std::mutex> lock(data_mutex_);

//     auto raw_msg = last_image_;

//     // Convert raw image into CV mat
//     int bit_depth = sensor_msgs::image_encodings::bitDepth(raw_msg->encoding);
//     int type = bit_depth == 8 ? CV_8U : CV_16U;

//     const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
//                         const_cast<uint8_t *>(&raw_msg->data[0]), raw_msg->step);

//     int rgb_step = raw_msg->width * 3 * (bit_depth / 8);
//     size_t rgb_data_vector_size = raw_msg->height * rgb_step;
//     // Create CV mat for color mat
//     std::vector<uint8_t> color_data(rgb_data_vector_size);
//     color_data.resize(rgb_data_vector_size);

//     cv::Mat color(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 3), color_data.data(),
//                   rgb_step);
// }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensing::LidarImagePointcloud)