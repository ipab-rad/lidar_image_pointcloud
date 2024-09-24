
#include "lidar_image_pointcloud/lidar_image_pointcloud.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <thread>

namespace sensing
{

LidarImagePointcloud::LidarImagePointcloud(const rclcpp::NodeOptions &options)
    : Node("rgb_pointcloud_processor", options)
{

    cam_names_ = this->declare_parameter<std::vector<std::string>>("camera_names", {"fsp_l"});
    num_threads_ = this->declare_parameter<int>("num_threads", 4);
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "lidar_ouster_top");
    const std::string lidar_topic =
        this->declare_parameter<std::string>("lidar_topic", "/sensor/lidar/top/points");
    const std::string rgb_pointcloud_topic = this->declare_parameter<std::string>(
        "rgb_pointcloud_topic", "/sensor/lidar/top/rgb_points");

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

void LidarImagePointcloud::process_slice(sensor_msgs::msg::PointCloud2 &rgb_pointcloud,
                                         sensor_msgs::PointCloud2Iterator<float> iter_x,
                                         size_t start_idx, size_t end_idx)
{

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(rgb_pointcloud, "rgb");
    iter_rgb += start_idx;

    iter_x += start_idx;

    for (size_t i = start_idx; i < end_idx; ++i, ++iter_x, ++iter_rgb)
    {
        float x = iter_x[0];
        float y = iter_x[1];
        float z = iter_x[2];

        tf2::Vector3 lidar_point(x, y, z);

        for (const auto &[cam_name, img_msg] : images_)
        {
            // Transform lidar point into camera frame
            auto cam_cartesian_point = cam_tf_map_.at(cam_name) * lidar_point;

            // Project into image plane
            float u = (cam_info_map_.at(cam_name)->k[0] * cam_cartesian_point.x() /
                       cam_cartesian_point.z()) +
                      cam_info_map_.at(cam_name)->k[2];
            float v = (cam_info_map_.at(cam_name)->k[4] * cam_cartesian_point.y() /
                       cam_cartesian_point.z()) +
                      cam_info_map_.at(cam_name)->k[5];

            if (cam_cartesian_point.z() > 0 && u >= 0 && u < img_msg->width && v >= 0 &&
                v < img_msg->height)
            {
                // Get pixel color from image
                int idx = static_cast<int>(v) * img_msg->step + static_cast<int>(u) * 3;
                iter_rgb[0] = img_msg->data[idx];     // Red
                iter_rgb[1] = img_msg->data[idx + 1]; // Green
                iter_rgb[2] = img_msg->data[idx + 2]; // Blue
            }
        }
    }
}

void LidarImagePointcloud::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
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

    sensor_msgs::PointCloud2Iterator<float> iter_x(rgb_pointcloud, "x");

    // Total number of points
    size_t point_count = rgb_pointcloud.width * rgb_pointcloud.height;

    // Divide the work into slices
    size_t pointcloud_slice_size = point_count / num_threads_;
    std::vector<std::thread> threads;

    for (size_t i = 0; i < num_threads_; ++i)
    {
        // Calculate the range for the current thread
        size_t start_idx = i * pointcloud_slice_size;
        size_t end_idx = (i == num_threads_ - 1) ? point_count : start_idx + pointcloud_slice_size;

        // Launch a thread to process each slice
        threads.emplace_back(&LidarImagePointcloud::process_slice, this, std::ref(rgb_pointcloud),
                             iter_x, start_idx, end_idx);
    }

    for (auto &t : threads)
    {
        if (t.joinable())
            t.join();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    RCLCPP_INFO(this->get_logger(), "Computaton time: %f ms", duration.count());

    rgb_pointcloud_pub_->publish(rgb_pointcloud);
}

rmw_qos_profile_t LidarImagePointcloud::get_topic_qos_profie(rclcpp::Node *node,
                                                             const std::string &topic)
{
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

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensing::LidarImagePointcloud)
