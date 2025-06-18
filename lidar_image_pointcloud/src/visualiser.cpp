#include "lidar_image_pointcloud/timer.hpp"

#include <opencv2/opencv.hpp>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <regex>

namespace fs = std::filesystem;

// Nebular point used in Pandar points
struct PointXYZIRCAEDT {
  float x, y, z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth, elevation, distance;
  std::uint32_t time_stamp;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRCAEDT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
  (std::uint8_t, return_type, return_type)(std::uint16_t, channel, channel)
  (float, azimuth, azimuth)(float, elevation, elevation)(float, distance, distance)
  (std::uint32_t, time_stamp, time_stamp))

using DefaultPointType = PointXYZIRCAEDT;

struct TimedFile {
  fs::path path;
  uint64_t timestamp;
};

std::vector<TimedFile> load_sorted_files(const fs::path& dir, const std::string& ext) {
  std::vector<TimedFile> files;

  for (const auto& entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    if (entry.path().extension() != ext) continue;
    std::string stem = entry.path().stem().string();
    size_t dash_pos = stem.find('-');
    if (dash_pos == std::string::npos) continue;

    try {
      std::string sec_str = stem.substr(0, dash_pos);
      std::string nsec_str = stem.substr(dash_pos + 1);

      // Remove leading zeros from nanoseconds string manually
      size_t first_non_zero = nsec_str.find_first_not_of('0');
      if (first_non_zero != std::string::npos) {
        nsec_str = nsec_str.substr(first_non_zero);
      } else {
        nsec_str = "0"; // all zeros case
      }

      uint64_t sec = std::stoull(sec_str);
      uint64_t nsec = std::stoull(nsec_str);
      uint64_t ts = sec * 1e9 + nsec;
      files.push_back({entry.path(), ts});
    } catch (...) {
      continue;
    }
  }

  std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
    return a.timestamp < b.timestamp;
  });
  return files;
}

class Visualiser {
public:
  Visualiser()
    : cloud_(std::make_shared<pcl::PointCloud<DefaultPointType>>()) {

    // Hardcode the tf and camera intrinsics :)
    Eigen::AngleAxisd roll(-1.664539, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(-0.009410, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(-3.130936, Eigen::Vector3d::UnitZ());
    transform_.setIdentity();
    transform_.block<3,3>(0,0) = (yaw * pitch * roll).toRotationMatrix();
    transform_.block<3,1>(0,3) = Eigen::Vector3d(0.288239, -0.314501, -0.672622);

    K_.setIdentity();
    K_(0,0) = 1451.49531;
    K_(0,2) = 1218.93228;
    K_(1,1) = 1450.7118;
    K_(1,2) = 683.50770999999997;
  }

  void run(const fs::path& pcd_dir, const fs::path& img_dir) {

    std::cout << "Loading files from:\n"
              << "  PCD: " << pcd_dir << "\n"
              << "  IMG: " << img_dir << '\n';
    pcd_files_ = load_sorted_files(pcd_dir, ".pcd");
    img_files_ = load_sorted_files(img_dir, ".jpg");
    if (pcd_files_.empty() || img_files_.empty()) throw std::runtime_error("Missing input files");

    pcd_idx_ = 0;
    img_idx_ = 0;

    loadPCD(pcd_files_[pcd_idx_].path);
    loadImage(img_files_[img_idx_].path);
    double max_distance_m = 15.0; // Adjust as needed

    // project(max_distance_m);
    project_with_marker_dots(max_distance_m);
    display();

    while (true) {
      int key = cv::waitKey(0);
      bool changed = false;
      if (key == 'q' && pcd_idx_ > 0) {
        pcd_idx_--;
        changed = true;
      }
      if (key == 'e' && pcd_idx_ + 1 < pcd_files_.size()) {
        pcd_idx_++;
        changed = true;
      }
      if (key == 'a' && img_idx_ > 0) {
        img_idx_--;
        changed = true;
      }
      if (key == 'd' && img_idx_ + 1 < img_files_.size()) {
        img_idx_++;
        changed = true;
      }
      if (key == 81 && pcd_idx_ > 0 && img_idx_ > 0) {
        pcd_idx_--;
        img_idx_--;
        changed = true;
      }
      if (key == 83 && pcd_idx_ + 1 < pcd_files_.size() && img_idx_ + 1 < img_files_.size()) {
        pcd_idx_++;
        img_idx_++;
        changed = true;
      }
      if (changed) {
        uint64_t pointcloud_ts = pcd_files_[pcd_idx_].timestamp;
        uint64_t image_ts = img_files_[img_idx_].timestamp;
        int64_t diff = image_ts - pointcloud_ts;
        double diff_ms = static_cast<double>(diff) / 1e6;
        std::cout << "[PCD: "<<pcd_idx_<<" ] " << pcd_files_[pcd_idx_].path.filename() << " | [IMG: "<<img_idx_<<"] "
                  << img_files_[img_idx_].path.filename()
                  << " | Time diff (image_time - lidar_time): " << diff_ms << " ms"
                  << '\n';
        loadPCD(pcd_files_[pcd_idx_].path);
        loadImage(img_files_[img_idx_].path);
        // project(max_distance_m);
        project_with_marker_dots(max_distance_m, 2);
        display();
      }
    }
  }

private:
  void loadPCD(const fs::path& file) {
    pcl::io::loadPCDFile<DefaultPointType>(file.string(), *cloud_);
  }

  void loadImage(const fs::path& file) {
    image_ = cv::imread(file.string(), cv::IMREAD_COLOR);
    // image_.setTo(cv::Scalar(0, 0, 0));
  }

  void project(double max_distance_m) {
    for (const auto& pt : *cloud_) {
      if (!pcl::isFinite(pt)) continue;
      Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
      Eigen::Vector4d pc = transform_ * p;
      if (pc(2) <= 0) continue;

      // Remove points that are too far away
      double distance = pc.head<3>().norm();
      if (distance > max_distance_m) continue;
      Eigen::Vector3d pix = K_ * pc.head<3>();
      pix /= pix(2);
      int u = static_cast<int>(pix(0));
      int v = static_cast<int>(pix(1));
      if (u >= 0 && v >= 0 && u < image_.cols && v < image_.rows) {
        // image_.at<cv::Vec3b>(v, u) = turbo_colormap(pt.intensity);
        image_.at<cv::Vec3b>(v, u) = get_rainbow_color(pt.intensity);
      }
    }
  }

  void project_with_marker_dots(double max_distance_m, int radius = 2) {
    for (const auto& pt : *cloud_) {
      if (!pcl::isFinite(pt)) continue;
      Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
      Eigen::Vector4d pc = transform_ * p;
      if (pc(2) <= 0) continue;

      // Remove points that are too far away
      double distance = pc.head<3>().norm();
      if (distance > max_distance_m) continue;

      Eigen::Vector3d pix = K_ * pc.head<3>();
      pix /= pix(2);
      int u = static_cast<int>(pix(0));
      int v = static_cast<int>(pix(1));
      if (u >= 0 && v >= 0 && u < image_.cols && v < image_.rows) {
        // cv::circle(image_, cv::Point(u, v), radius, turbo_colormap(pt.intensity), -1);
        cv::circle(image_, cv::Point(u, v), radius, get_rainbow_color(pt.intensity), -1);
      }
    }
  }

  void display() {
    cv::imshow("Projection", image_);
  }

  cv::Vec3b turbo_colormap(int intensity) {
    constexpr int min_intensity = 1;
    constexpr int max_intensity = 180;

    double normalised = std::clamp(
        static_cast<double>(intensity - min_intensity) / (max_intensity - min_intensity),
        0.0, 1.0);

    double r = 34.61 + normalised * (1172.33 + normalised * (-1079.75 + normalised * (311.0 - 40.0 * normalised)));
    double g = 23.31 + normalised * (557.33 + normalised * (206.0 + normalised * (-677.0 + 510.0 * normalised)));
    double b = 27.2 + normalised * (3211.1 + normalised * (-4574.4 + normalised * (2319.0 - 304.0 * normalised)));

    auto clamp = [](double x) {
      return static_cast<uchar>(std::round(std::clamp(x, 0.0, 255.0)));
    };

    return { clamp(b), clamp(g), clamp(r) };  // OpenCV expects BGR
  }

  // Obtained from https://github.com/ros2/rviz/blob/2ef04d47bcea4dccca68dbcffb4071ab7e94dac9/rviz_default_plugins/include/rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp#L131
  cv::Vec3b get_rainbow_color(double intensity)
  {
    constexpr double kMinIntensity = 1.0;
    constexpr double kMaxIntensity = 180.0;
    double value =
      std::clamp((intensity - kMinIntensity) / (kMaxIntensity - kMinIntensity), 0.0, 1.0);
    double h = value * 5.0 + 1.0;
    int i = static_cast<int>(std::floor(h));
    double f = h - i;
    if ((i & 1) == 0) f = 1 - f;
    double n = 1 - f;

    double r, g, b;
    if (i <= 1) {
      r = n;
      g = 0;
      b = 1;
    } else if (i == 2) {
      r = 0;
      g = n;
      b = 1;
    } else if (i == 3) {
      r = 0;
      g = 1;
      b = n;
    } else if (i == 4) {
      r = n;
      g = 1;
      b = 0;
    } else {
      r = 1;
      g = n;
      b = 0;
    }

    return cv::Vec3b(b * 255, g * 255, r * 255);  // BGR
  }

  std::vector<TimedFile> pcd_files_, img_files_;
  size_t pcd_idx_ = 0, img_idx_ = 0;
  pcl::PointCloud<DefaultPointType>::Ptr cloud_;
  cv::Mat image_;
  Eigen::Matrix4d transform_;
  Eigen::Matrix3d K_;
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: ./visualiser <pcd_dir> <image_dir>\n";
    return 1;
  }
  try {
    Visualiser vis;

    vis.run(argv[1], argv[2]);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
