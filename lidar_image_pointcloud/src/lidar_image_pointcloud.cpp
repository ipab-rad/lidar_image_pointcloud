#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include "lidar_image_pointcloud/lidar_image_pointcloud.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensing
{

LidarImagePointcloud::LidarImagePointcloud(const rclcpp::NodeOptions &options)
    : Node("rgb_pointcloud_processor", options)
{
    const std::string cam_name = "fsp_l";
    // const std::string cam_name = "rsp_l";
    const std::string lidar_topic = "/sensor/lidar/top/points";
    // const std::string image_raw_topic = "/sensor/camera/" + cam_name+ "/image_raw";
    const std::string image_raw_topic = "/sensor/camera/"+ cam_name + "/image_rect_color";
    const std::string camera_info_topic = "/sensor/camera/" + cam_name +"/camera_info";
    const std::string rgb_pointcloud_topic = "/sensor/lidar/top/rgb_points";

    lidar_frame_ = "lidar_ouster_top";
    camera_frame_ = "camera_" + cam_name + "_optical";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10,
        std::bind(&LidarImagePointcloud::pointcloud_callback, this, std::placeholders::_1));

    auto qos_profile = get_topic_qos_profie(this, image_raw_topic);
    image_sub_ = image_transport::create_subscription(
        this, image_raw_topic,
        std::bind(&LidarImagePointcloud::image_callback, this, std::placeholders::_1), "compressed",
        qos_profile);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&LidarImagePointcloud::camera_info_callback, this, std::placeholders::_1));

    rgb_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("rgb_pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "Lidar Image Pointcloud Initialised!");
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

void LidarImagePointcloud::pointcloud_callback(
    const sensor_msgs ::msg::PointCloud2::SharedPtr pointcloud_msg)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!last_image_ || !camera_info_ || !extract_transform(camera_frame_, lidar_frame_))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for valid image, camera_info, or transform.");
        return;
    }

    // TODO: Rectify image here!
    // rectify_image();

    // Or read recitfied already compressed?


    // Create RGB PointCloud2
    sensor_msgs::msg::PointCloud2 rgb_pointcloud = *pointcloud_msg;
    sensor_msgs::PointCloud2Modifier pcd_modifier(rgb_pointcloud);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Iterate over point cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(rgb_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(rgb_pointcloud, "rgb");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_rgb) {
        float x = iter_x[0];
        float y = iter_x[1];
        float z = iter_x[2];

        // Apply transformation
        tf2::Vector3 point(x, y, z);
        // Convert TransformStamped to tf2::Transform
        tf2::Transform transform;
        tf2::fromMsg(transform_.transform, transform); // transform_ is geometry_msgs::msg::TransformStamped

        point = transform * point;

        // Project into image plane
        float u = (camera_info_->k[0] * point.x() / point.z()) + camera_info_->k[2];
        float v = (camera_info_->k[4] * point.y() / point.z()) + camera_info_->k[5];

        if (point.z() > 0 && u >= 0 && u < last_image_->width && v >= 0 && v < last_image_->height) {
            // Get pixel color from image
            int idx = static_cast<int>(v) * last_image_->step + static_cast<int>(u) * 3;
            iter_rgb[0] = last_image_->data[idx];
            iter_rgb[1] = last_image_->data[idx + 1];
            iter_rgb[2] = last_image_->data[idx + 2];
        }
    }

    // Publish the RGB PointCloud
    rgb_pointcloud_pub_->publish(rgb_pointcloud);
}

void LidarImagePointcloud::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &raw_msg)
{

    std::lock_guard<std::mutex> lock(image_mutex_);
    last_image_ = raw_msg;
}

void LidarImagePointcloud::rectify_image()
{
    std::lock_guard<std::mutex> lock(image_mutex_);

    auto raw_msg = last_image_;

    // Convert raw image into CV mat
    int bit_depth = sensor_msgs::image_encodings::bitDepth(raw_msg->encoding);
    int type = bit_depth == 8 ? CV_8U : CV_16U;

    const cv::Mat bayer(
      raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
      const_cast<uint8_t *>(&raw_msg->data[0]), raw_msg->step);


    int rgb_step = raw_msg->width * 3 * (bit_depth / 8);
    size_t rgb_data_vector_size = raw_msg->height * rgb_step;
    // Create CV mat for color mat
    std::vector<uint8_t> color_data(rgb_data_vector_size);
    color_data.resize(rgb_data_vector_size);

    cv::Mat color(
      raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 3),
      color_data.data(), rgb_step);

    // int bit_depth = sensor_msgs::image_encodings::bitDepth(raw_msg->encoding);
    // // TODO(someone): Fix as soon as bitDepth fixes it
    // if (raw_msg->encoding == sensor_msgs::image_encodings::YUV422)
    // {
    //     bit_depth = 8;
    // }


    // if (sensor_msgs::image_encodings::isMono(raw_msg->encoding))
    // {
    //     // // For monochrome, no processing needed!
    //     // pub_color_.publish(raw_msg);

    //     // Warn if the user asked for color
    //     RCLCPP_WARN(this->get_logger(),
    //                 "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
    //                 pub_color_.getTopic().c_str(), sub_raw_.getTopic().c_str());
    // }
    // else if (sensor_msgs::image_encodings::isColor(raw_msg->encoding))
    // {
    //     // pub_color_.publish(raw_msg);
    // }
    // else if (sensor_msgs::image_encodings::isBayer(raw_msg->encoding))
    // {
    //     int type = bit_depth == 8 ? CV_8U : CV_16U;
    //     const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
    //                         const_cast<uint8_t *>(&raw_msg->data[0]), raw_msg->step);

    //     sensor_msgs::msg::Image::SharedPtr color_msg = std::make_shared<sensor_msgs::msg::Image>();
    //     color_msg->header = raw_msg->header;
    //     color_msg->height = raw_msg->height;
    //     color_msg->width = raw_msg->width;
    //     color_msg->encoding = bit_depth == 8 ? sensor_msgs::image_encodings::BGR8
    //                                          : sensor_msgs::image_encodings::BGR16;
    //     color_msg->step = color_msg->width * 3 * (bit_depth / 8);
    //     color_msg->data.resize(color_msg->height * color_msg->step);

    //     cv::Mat color(color_msg->height, color_msg->width, CV_MAKETYPE(type, 3),
    //                   &color_msg->data[0], color_msg->step);

    //     int debayer_bilinear_ = 0;
    //     int debayer_edgeaware_ = 1;
    //     int debayer_edgeaware_weighted_ = 2;
    //     int debayer_vng_ = 3;
        
    //     int algorithm;
    
    //     // std::loc_guard<std::recursive_mutex> loc(config_mutex_)
    //     algorithm = debayer_vng_;

    //     if (algorithm == debayer_edgeaware_ || algorithm == debayer_edgeaware_weighted_)
    //     {
    //         // These algorithms are not in OpenCV yet
    //         if (raw_msg->encoding != sensor_msgs::image_encodings::BAYER_GRBG8)
    //         {
    //             RCLCPP_WARN(this->get_logger(),
    //                         "Edge aware algorithms currently only support GRBG8 Bayer. "
    //                         "Falling back to bilinear interpolation.");
    //             algorithm = debayer_bilinear_;
    //         }
    //         else
    //         {
    //             if (algorithm == debayer_edgeaware_)
    //             {
    //                 debayerEdgeAware(bayer, color);
    //             }
    //             else
    //             {
    //                 debayerEdgeAwareWeighted(bayer, color);
    //             }
    //         }
    //     }

    //     if (algorithm == debayer_bilinear_ || algorithm == debayer_vng_)
    //     {
    //         int code = -1;

    //         if (raw_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8 ||
    //             raw_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB16)
    //         {
    //             code = cv::COLOR_BayerBG2BGR;
    //         }
    //         else if (raw_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8 || // NOLINT
    //                  raw_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR16)
    //         {
    //             code = cv::COLOR_BayerRG2BGR;
    //         }
    //         else if (raw_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8 || // NOLINT
    //                  raw_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG16)
    //         {
    //             code = cv::COLOR_BayerGR2BGR;
    //         }
    //         else if (raw_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8 || // NOLINT
    //                  raw_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG16)
    //         {
    //             code = cv::COLOR_BayerGB2BGR;
    //         }

    //         if (algorithm == debayer_vng_)
    //         {
    //             code += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;
    //         }

    //         cv::cvtColor(bayer, color, code);
    //     }

    //     // pub_color_.publish(color_msg);
    // }
    // else if (raw_msg->encoding == sensor_msgs::image_encodings::YUV422)
    // {
    //     // Use cv_bridge to convert to BGR8
    //     sensor_msgs::msg::Image::SharedPtr color_msg;

    //     try
    //     {
    //         color_msg =
    //             cv_bridge::toCvCopy(raw_msg, sensor_msgs::image_encodings::BGR8)->toImageMsg();
    //         // pub_color_.publish(color_msg);
    //     }
    //     catch (const cv_bridge::Exception &e)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
    //     }
    // }
    // else if (raw_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC3)
    // {
    //     // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
    //     RCLCPP_WARN(this->get_logger(),
    //                 "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
    //                 "source should set the encoding to 'bgr8' or 'rgb8'.",
    //                 sub_raw_.getTopic().c_str());
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Raw image topic '%s' has unsupported encoding '%s'",
    //                 sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
    // }
}

void LidarImagePointcloud::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{

    camera_info_ = camera_info_msg;
}

bool LidarImagePointcloud::extract_transform(const std::string &target_frame,
                                             const std::string &source_frame)
{
    try
    {
        transform_ = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return false;
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensing::LidarImagePointcloud)