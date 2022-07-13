#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
// #include <thread>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <time.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "pcl_ros/point_cloud.hpp"

#define pi 3.141592
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std::chrono_literals;

// global variables
PointCloud::Ptr pointCloudMerged(new PointCloud); // 왜 글로벌 변수로 선언하지 않으면 new 에서 에러가 뜨는지??
PointCloud::Ptr transformedPointPtr0(new PointCloud);
PointCloud::Ptr transformedPointPtr1(new PointCloud);
PointCloud::Ptr transformedPointPtr2(new PointCloud);
PointCloud::Ptr transformedPointPtr3(new PointCloud);

class MergePointCloud : public rclcpp::Node
{
public:
  MergePointCloud()
  : Node("merge_point_cloud")
  {
    RCLCPP_INFO(this->get_logger(), "Merge point cloud constructed.");

    this->read_parameters();

    auto subscriber_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    cam0_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cam0_depth_topic_,
      subscriber_qos_profile,
      std::bind(&MergePointCloud::cam0_callback, this, std::placeholders::_1));
    cam1_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cam1_depth_topic_,
      subscriber_qos_profile,
      std::bind(&MergePointCloud::cam1_callback, this, std::placeholders::_1));
    cam2_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cam2_depth_topic_,
      subscriber_qos_profile,
      std::bind(&MergePointCloud::cam2_callback, this, std::placeholders::_1));
    cam3_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cam3_depth_topic_,
      subscriber_qos_profile,
      std::bind(&MergePointCloud::cam3_callback, this, std::placeholders::_1));

    auto publisher_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      publish_topic_, publisher_qos_profile);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::nanoseconds((int)(point_cloud_publish_duration_ * 1e9))),
      std::bind(&MergePointCloud::complete_merging, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    if (!this->initialize()) {
      RCLCPP_INFO(this->get_logger(), "Initialize failed.");
      return;
    }
  }

private:
  void read_parameters()
  {
    RCLCPP_INFO(this->get_logger(), "Read parameters.");

    this->declare_parameter("cam0_depth_topic", "/cam_0/depth/color/points");
    cam0_depth_topic_ = this->get_parameter("cam0_depth_topic").get_value<std::string>();
    this->declare_parameter("cam1_depth_topic", "/cam_1/depth/color/points");
    cam1_depth_topic_ = this->get_parameter("cam1_depth_topic").get_value<std::string>();
    this->declare_parameter("cam2_depth_topic", "/cam_2/depth/color/points");
    cam2_depth_topic_ = this->get_parameter("cam2_depth_topic").get_value<std::string>();
    this->declare_parameter("cam3_depth_topic", "/cam_3/depth/color/points");
    cam3_depth_topic_ = this->get_parameter("cam3_depth_topic").get_value<std::string>();
    this->declare_parameter("publish_topic", "/camera/depth/points");
    publish_topic_ = this->get_parameter("publish_topic").get_value<std::string>();

    this->declare_parameter("use_cam0", false);
    use_cam0_ = this->get_parameter("use_cam0").get_value<bool>();
    this->declare_parameter("use_cam1", false);
    use_cam1_ = this->get_parameter("use_cam1").get_value<bool>();
    this->declare_parameter("use_cam2", false);
    use_cam2_ = this->get_parameter("use_cam2").get_value<bool>();
    this->declare_parameter("use_cam3", false);
    use_cam3_ = this->get_parameter("use_cam3").get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "use_cam0: %s", use_cam0_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_cam1: %s", use_cam1_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_cam2: %s", use_cam2_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_cam3: %s", use_cam3_ ? "true" : "false");
    cam0_merged_ = !use_cam0_;
    cam1_merged_ = !use_cam1_;
    cam2_merged_ = !use_cam2_;
    cam3_merged_ = !use_cam3_;

    this->declare_parameter("max_publish_duration", 0.03);
    max_publish_duration_ = this->get_parameter("max_publish_duration").get_value<double>();
    this->declare_parameter("point_cloud_publish_duration", 0.06);
    point_cloud_publish_duration_ = this->get_parameter("point_cloud_publish_duration").get_value<double>();

    this->declare_parameter("apply_voxel_grid_filter", true);
    apply_voxel_grid_filter_ = this->get_parameter("apply_voxel_grid_filter").get_value<bool>();
    this->declare_parameter("voxel_grid_filter_size", 0.02);
    voxel_grid_filter_size_ = this->get_parameter("voxel_grid_filter_size").get_value<double>();
  }

  bool initialize()
  {
    RCLCPP_DEBUG(this->get_logger(), "Start initialize.");
    sleep(1);

    if(use_cam0_) this->broadcast_static_tf_body2cam0();
    sleep(1);

    // check there exists the transform.
    if (!tf_buffer_->canTransform("body", "cam_0_link", tf2::TimePointZero)) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s", "body", "cam_0_link");
      return false;
    }
    // TODO: waitForTransform은 왜 컴파일 에러가 나는지???
    // tf_buffer_->waitForTransform(std::string("body"), std::string("cam_0_link"), this->now(), rclcpp::Duration(1,0));
    // tf_buffer_->waitForTransform(std::string("body"), std::string("cam_0_link"), tf2::TimePointZero, tf2::durationFromSec(1));

    if(use_cam1_) this->broadcast_static_tf_body2cam1();
    if(use_cam2_) this->broadcast_static_tf_body2cam2();
    if(use_cam3_) this->broadcast_static_tf_body2cam3();

    // initialize last publised time
    last_published_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Done initialize.");

    return true;
  }

  bool check_merging_completed()
  {
    bool completed = cam0_merged_ && cam1_merged_ && cam2_merged_ && cam3_merged_;
    return completed;
  }

  void complete_merging()
  {
    auto duration = this->now() - last_published_time_;
    if (this->check_merging_completed() || (duration.seconds() > max_publish_duration_)) {
      RCLCPP_DEBUG(this->get_logger(), "Complete merging pointcloud start.");

      bool pass_condition = ((cam0_merged_ && use_cam0_) || (cam1_merged_ && use_cam1_) || (cam2_merged_ && use_cam2_) || (cam3_merged_ && use_cam3_));
      if (!pass_condition) return;

      PointCloud point_cloud;

      boost::recursive_mutex::scoped_lock scoped_lock0(cam0_mutex_);
      boost::recursive_mutex::scoped_lock scoped_lock1(cam1_mutex_);
      boost::recursive_mutex::scoped_lock scoped_lock2(cam2_mutex_);
      boost::recursive_mutex::scoped_lock scoped_lock3(cam3_mutex_);

      // if (cam0_merged_ && use_cam0_) *pointCloudMerged += *transformedPointPtr0;
      // if (cam1_merged_ && use_cam1_) *pointCloudMerged += *transformedPointPtr1;
      // if (cam2_merged_ && use_cam2_) *pointCloudMerged += *transformedPointPtr2;
      // if (cam3_merged_ && use_cam3_) *pointCloudMerged += *transformedPointPtr3;

      std_msgs::msg::Header oldest_header;
      oldest_header.stamp.sec = INT_MAX;
      oldest_header.frame_id = "body";
      if (cam0_merged_ && use_cam0_) {
        *pointCloudMerged += *transformedPointPtr0;
        int sec = 1e-6*transformedPointPtr0->header.stamp;
        uint nsec = 1000*transformedPointPtr0->header.stamp - 1e9*sec;
        // RCLCPP_INFO(this->get_logger(), "stamp: %d.%d sec", sec, nsec); // microseconds -> nanoseconds
        // RCLCPP_INFO(this->get_logger(), "stamp: %lld nsec", 1000*transformedPointPtr0->header.stamp); // microseconds -> nanoseconds
        // std::cout << 1000*transformedPointPtr0->header.stamp << std::endl; // just for checking nanoseconds
        if(oldest_header.stamp.sec > sec) {
          oldest_header.stamp.sec = sec;
          oldest_header.stamp.nanosec = nsec;
        }
        else if(oldest_header.stamp.sec == sec) {
          if(oldest_header.stamp.nanosec > nsec) {
            oldest_header.stamp.sec = sec;
            oldest_header.stamp.nanosec = nsec;
          }
        }
      }
      if (cam1_merged_ && use_cam1_) {
        *pointCloudMerged += *transformedPointPtr1;
        int sec = 1e-6*transformedPointPtr1->header.stamp;
        uint nsec = 1000*transformedPointPtr1->header.stamp - 1e9*sec;
        if(oldest_header.stamp.sec > sec) {
          oldest_header.stamp.sec = sec;
          oldest_header.stamp.nanosec = nsec;
        }
        else if(oldest_header.stamp.sec == sec) {
          if(oldest_header.stamp.nanosec > nsec) {
            oldest_header.stamp.sec = sec;
            oldest_header.stamp.nanosec = nsec;
          }
        }
      }
      if (cam2_merged_ && use_cam2_) {
        *pointCloudMerged += *transformedPointPtr2;
        int sec = 1e-6*transformedPointPtr1->header.stamp;
        uint nsec = 1000*transformedPointPtr1->header.stamp - 1e9*sec;
        if(oldest_header.stamp.sec > sec) {
          oldest_header.stamp.sec = sec;
          oldest_header.stamp.nanosec = nsec;
        }
        else if(oldest_header.stamp.sec == sec) {
          if(oldest_header.stamp.nanosec > nsec) {
            oldest_header.stamp.sec = sec;
            oldest_header.stamp.nanosec = nsec;
          }
        }
      }
      if (cam3_merged_ && use_cam3_) {
        *pointCloudMerged += *transformedPointPtr3;
        int sec = 1e-6*transformedPointPtr1->header.stamp;
        uint nsec = 1000*transformedPointPtr1->header.stamp - 1e9*sec;
        if(oldest_header.stamp.sec > sec) {
          oldest_header.stamp.sec = sec;
          oldest_header.stamp.nanosec = nsec;
        }
        else if(oldest_header.stamp.sec == sec) {
          if(oldest_header.stamp.nanosec > nsec) {
            oldest_header.stamp.sec = sec;
            oldest_header.stamp.nanosec = nsec;
          }
        }
      }

      scoped_lock0.unlock();
      scoped_lock1.unlock();
      scoped_lock2.unlock();
      scoped_lock3.unlock();

      pcl::copyPointCloud(*pointCloudMerged, point_cloud);
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(point_cloud, *msg);

      // msg->header.frame_id = "body";
      // msg->header.stamp = this->now();
      msg->header = oldest_header;

      publisher_->publish(*msg);

      last_published_time_ = this->now();
      // RCLCPP_INFO(this->get_logger(), "msg: %d.%d sec", msg->header.stamp.sec, msg->header.stamp.nanosec); // microseconds -> nanoseconds
      // RCLCPP_INFO(this->get_logger(), "now: %f sec", last_published_time_.seconds());

      auto steady_clock = rclcpp::Clock();
      RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock, 5000, "Merged point cloud has (%i points).", static_cast<int>(pointCloudMerged->size()));
      RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock, 5000, "Merged point cloud header stamp is %d.%d sec.", msg->header.stamp.sec, msg->header.stamp.nanosec);
      // RCLCPP_INFO(this->get_logger(), "Merged point cloud frame id is %s.", (msg->header.frame_id).c_str());

      pointCloudMerged->clear(); // clear the merged cloud

      cam0_merged_ = !use_cam0_;
      cam1_merged_ = !use_cam1_;
      cam2_merged_ = !use_cam2_;
      cam3_merged_ = !use_cam3_;
      RCLCPP_DEBUG(this->get_logger(), "Complete merging pointcloud.");
    }
    else return;
  }

  bool filter_point_cloud(const PointCloud::Ptr point_cloud)
  {
    PointCloud temp_point_cloud;

    // remove nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud, temp_point_cloud, indices);
    temp_point_cloud.is_dense = true;
    point_cloud->swap(temp_point_cloud);

    // reduce points using VoxelGrid filter
    if(apply_voxel_grid_filter_) {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (point_cloud);
        double filter_size = voxel_grid_filter_size_; //sensorParameters_.at("voxelgrid_filter_size");
        sor.setLeafSize (filter_size, filter_size, filter_size);
        sor.filter (temp_point_cloud);
        point_cloud->swap(temp_point_cloud);
    }
    rclcpp::Clock clock;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Reduced point cloud to %i points.", static_cast<int>(point_cloud->size()));
    return true;
  }

  void cam0_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_point_cloud)
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam0 callback start.");

    geometry_msgs::msg::TransformStamped transform_stamped;
    this->broadcast_static_tf_body2cam0();

    // convert the point clouds from ROS to PCL.
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*raw_point_cloud, pcl_pc);
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc, *point_cloud);

    // applying voxel grid filter.
    auto start_time = this->now();
    this->filter_point_cloud(point_cloud);
    auto duration = this->now() - start_time;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam0 filter takes %f sec.", duration.seconds());

    // transform the point clouds.
    try {
      transform_stamped = tf_buffer_->lookupTransform("body", "cam_0_depth_optical_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "body", "cam_0_depth_optical_frame", ex.what());
      return;
    }
    Eigen::Matrix4f transform = this->get_transform_matrix(transform_stamped);
    PointCloud transformed_point_cloud;
    pcl::transformPointCloud(*point_cloud, transformed_point_cloud, transform);

    boost::recursive_mutex::scoped_lock scoped_lock(cam0_mutex_);
    *transformedPointPtr0 = transformed_point_cloud;
    scoped_lock.unlock();

    cam0_merged_ = true;
  }

  void cam1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_point_cloud)
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam1 callback start.");

    geometry_msgs::msg::TransformStamped transform_stamped;
    this->broadcast_static_tf_body2cam1();

    // convert the point clouds from ROS to PCL.
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*raw_point_cloud, pcl_pc);
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc, *point_cloud);

    // applying voxel grid filter.
    auto start_time = this->now();
    this->filter_point_cloud(point_cloud);
    auto duration = this->now() - start_time;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam1 filter takes %f sec.", duration.seconds());

    // transform the point clouds.
    try {
      transform_stamped = tf_buffer_->lookupTransform("body", "cam_1_depth_optical_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "body", "cam_1_depth_optical_frame", ex.what());
      return;
    }
    Eigen::Matrix4f transform = this->get_transform_matrix(transform_stamped);
    PointCloud transformed_point_cloud;
    pcl::transformPointCloud(*point_cloud, transformed_point_cloud, transform);

    boost::recursive_mutex::scoped_lock scoped_lock(cam1_mutex_);
    *transformedPointPtr1 = transformed_point_cloud;
    scoped_lock.unlock();

    cam1_merged_ = true;
  }

  void cam2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_point_cloud)
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam2 callback start.");

    geometry_msgs::msg::TransformStamped transform_stamped;
    this->broadcast_static_tf_body2cam2();

    // convert the point clouds from ROS to PCL.
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*raw_point_cloud, pcl_pc);
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc, *point_cloud);

    // applying voxel grid filter.
    auto start_time = this->now();
    this->filter_point_cloud(point_cloud);
    auto duration = this->now() - start_time;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam2 filter takes %f sec.", duration.seconds());

    // transform the point clouds.
    try {
      transform_stamped = tf_buffer_->lookupTransform("body", "cam_2_depth_optical_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "body", "cam_2_depth_optical_frame", ex.what());
      return;
    }
    Eigen::Matrix4f transform = this->get_transform_matrix(transform_stamped);
    PointCloud transformed_point_cloud;
    pcl::transformPointCloud(*point_cloud, transformed_point_cloud, transform);

    boost::recursive_mutex::scoped_lock scoped_lock(cam2_mutex_);
    *transformedPointPtr2 = transformed_point_cloud;
    scoped_lock.unlock();

    cam2_merged_ = true;
  }

  void cam3_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_point_cloud)
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam3 callback start.");

    geometry_msgs::msg::TransformStamped transform_stamped;
    this->broadcast_static_tf_body2cam3();

    // convert the point clouds from ROS to PCL.
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*raw_point_cloud, pcl_pc);
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc, *point_cloud);

    // applying voxel grid filter.
    auto start_time = this->now();
    this->filter_point_cloud(point_cloud);
    auto duration = this->now() - start_time;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 5, "Cam3 filter takes %f sec.", duration.seconds());

    // transform the point clouds.
    try {
      transform_stamped = tf_buffer_->lookupTransform("body", "cam_3_depth_optical_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "body", "cam_3_depth_optical_frame", ex.what());
      return;
    }
    Eigen::Matrix4f transform = this->get_transform_matrix(transform_stamped);
    PointCloud transformed_point_cloud;
    pcl::transformPointCloud(*point_cloud, transformed_point_cloud, transform);

    boost::recursive_mutex::scoped_lock scoped_lock(cam3_mutex_);
    *transformedPointPtr3 = transformed_point_cloud;
    scoped_lock.unlock();

    cam3_merged_ = true;
  }

  void broadcast_static_tf_body2cam0()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;

    if (false) {
      Eigen::Matrix4f transform_mat;

      transform_mat << -0.0801843, -0.62506734, 0.77644143, 0.05548641,
                      -0.99670208, 0.04053544, -0.07029825, -0.02227328,
                        0.01246774, -0.7795176, -0.62625622, -0.03080424,
                        0,          0,          0,       1;
      // Tm <<   -0.0801843, -0.99670208, 0.01246774, -0.01736663,
      //         -0.62506734, 0.04053544, -0.7795176, 0.01157315,
      //         0.77644143, -0.07029825, -0.62625622, -0.06393907,
      //         0,          0,          0,          1;

      tf2::Vector3 origin;
      origin.setValue(static_cast<double>(transform_mat(0,3)),static_cast<double>(transform_mat(1,3)),static_cast<double>(transform_mat(2,3)));

      tf2::Matrix3x3 rotation_mat;
      rotation_mat.setValue(static_cast<double>(transform_mat(0,0)), static_cast<double>(transform_mat(0,1)), static_cast<double>(transform_mat(0,2)),
                            static_cast<double>(transform_mat(1,0)), static_cast<double>(transform_mat(1,1)), static_cast<double>(transform_mat(1,2)),
                            static_cast<double>(transform_mat(2,0)), static_cast<double>(transform_mat(2,1)), static_cast<double>(transform_mat(2,2)));
      tf2::Quaternion quaternion;
      rotation_mat.getRotation(quaternion);

      tf2::Transform transform_body_infra;
      transform_body_infra.setOrigin(origin);
      transform_body_infra.setRotation(quaternion);

      geometry_msgs::msg::TransformStamped transform_stamped_infra_link_msg;
      transform_stamped_infra_link_msg = tf_buffer_->lookupTransform("cam_0_infra1_optical_frame", "cam_0_link", tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> transform_stamped_infra_link;
      tf2::fromMsg(transform_stamped_infra_link_msg, transform_stamped_infra_link);

      tf2::Transform transform_body_link;
      transform_body_link = transform_body_infra * transform_stamped_infra_link;

      // publish static tf
      static_transform_stamped.transform.translation.x = transform_body_link.getOrigin().x();
      static_transform_stamped.transform.translation.y = transform_body_link.getOrigin().y();
      static_transform_stamped.transform.translation.z = transform_body_link.getOrigin().z();

      static_transform_stamped.transform.rotation.x = transform_body_link.getRotation().x();
      static_transform_stamped.transform.rotation.y = transform_body_link.getRotation().y();
      static_transform_stamped.transform.rotation.z = transform_body_link.getRotation().z();
      static_transform_stamped.transform.rotation.w = transform_body_link.getRotation().w();
    }
    else {
    // publish static tf
    static_transform_stamped.header.stamp = this->now(); // this->get_clock()->now() ???
    static_transform_stamped.header.frame_id = "body";
    static_transform_stamped.child_frame_id = "cam_0_link";
    static_transform_stamped.transform.translation.x = 0.3;
    static_transform_stamped.transform.translation.y = 0;
    static_transform_stamped.transform.translation.z =  0;


    tf2::Quaternion quat;
    // double roll = 0;
    double pitch = 35;
    double yaw = 0;
    quat.setRPY(0, pitch*pi/180, yaw*pi/180);
    static_transform_stamped.transform.rotation.x = quat.x();
    static_transform_stamped.transform.rotation.y = quat.y();
    static_transform_stamped.transform.rotation.z = quat.z();
    static_transform_stamped.transform.rotation.w = quat.w();
    }

    tf_publisher_->sendTransform(static_transform_stamped);
  }

  void broadcast_static_tf_body2cam1()
  {
    // get body to cam_0_link transform
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg = tf_buffer_->lookupTransform("body", "cam_0_link", tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> transform_stamped;
    tf2::fromMsg(transform_stamped_msg, transform_stamped);

    tf2::Transform tf_body_0l;
    tf_body_0l.setOrigin(transform_stamped.getOrigin());
    tf_body_0l.setRotation(transform_stamped.getRotation());

    Eigen::Vector3f c0_center;
    c0_center = this->get_camera_center(tf_body_0l.inverse());
    // cout << "cam_0 center: " << endl << c0_center << endl;

    tf2::Transform tf_0l_0cof;
    tf_0l_0cof.setOrigin({-0.000579873, 0.0147138, 5.84737e-05});
    tf_0l_0cof.setRotation({-0.498893, 0.501671, -0.498554, 0.500874});

    tf2::Transform tf_1l_1cof;
    tf_1l_1cof.setOrigin({-0.00058686, 0.0146574, -5.20926e-05});
    tf_1l_1cof.setRotation({-0.49816, 0.500979,-0.500142, 0.50071});

    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.frame_id = "body";
    static_transform_stamped.child_frame_id = "cam_1_link";

    tf2::Transform tf_0l_0cf, tf_0cf_0cof, tf_1cof_0cof, tf_1cf_1cof, tf_1l_1cf, tf_body_1l;
    tf2::Quaternion q_0l_0cf, q_0cf_0cof, q_1cof_0cof, q_1cf_1cof, q_1l_1cf, q_1cof_1cf, q_1cf_1l, q_body_1l;

    tf2::Vector3 t_0l_0cf, t_0cf_0cof, t_1cof_0cof, t_1cf_1cof, t_1l_1cf, t_body_1l;
    tf2::Matrix3x3 tfRot;

    // p:cam0_color_optical_frame, c:cam1_color_optical_frame
    // q_1cof_0cof={0.00762625, -0.58842435, -0.3991386,   0.70312659};
    q_1cof_0cof={-0.00762625, 0.58842435, 0.3991386, 0.70312659};
    t_1cof_0cof={0.46120531, 0.12047342, -0.19447596};
    tf_1cof_0cof.setOrigin(t_1cof_0cof);
    tf_1cof_0cof.setRotation(q_1cof_0cof);

    Eigen::Vector3f center;
    center = this->get_camera_center(tf_1cof_0cof);
    // cout << "cam1cof center in cam0cof frame: " << endl << center << endl;

    // print_tf(tf_1cof_0cof, "tf_1cof_0cof");

    // tf_body_1l= tf_body_0l * tf_0l_0cof * tf_1cof_0cof.inverse() * tf_1l_1cof.inverse();
    tf_body_1l = tf_body_0l * tf_0l_0cof * tf_1cof_0cof.inverse() * tf_1l_1cof.inverse();
    Eigen::Vector3f c1_center;
    c1_center = this->get_camera_center(tf_body_1l.inverse());
    // cout << "cam_1 center: " << endl << c1_center << endl;

    // print_tf(tf_body_1l, "tf_body_1l");
    //p:cam0_link, c:cam1_link
    // tf_body_1l= tf_body_0l * tf_0l_0cf * tf_0cf_0cof * tf_1cof_0cof.inverse() * tf_1cf_1cof.inverse() * tf_1l_1cf.inverse();
    // tfMat=GetTransformMatrix_mk(q_0l_0cf,t_0l_0cf)*GetTransformMatrix_mk(q_0cf_0cof,t_0cf_0cof)*GetTransformMatrix_mk(q_0cof_1cof,t_0cof_1cof)*GetTransformMatrix_mk(q_1cf_1cof,t_1cf_1cof).inverse()*GetTransformMatrix_mk(q_1l_1cf,t_1l_1cf).inverse();
    // q_1cf_1l={-0.00292005832307, 0.0028238222003,  0.00144543300848,  0.999990701675};
    // tf_body_1l = tf_body_1l.inverse();
    q_body_1l=tf_body_1l.getRotation();
    // double roll, pitch, yaw;
    // tfRot.getRPY(roll, pitch, yaw);
    // q_new.setRPY(roll, pitch, yaw);
    double x = tf_body_1l.getOrigin().getX();
    double y = tf_body_1l.getOrigin().getY();
    double z = tf_body_1l.getOrigin().getZ();

    static_transform_stamped.transform.rotation.x = q_body_1l[0];
    static_transform_stamped.transform.rotation.y = q_body_1l[1];
    static_transform_stamped.transform.rotation.z = q_body_1l[2];
    static_transform_stamped.transform.rotation.w = q_body_1l[3];
    static_transform_stamped.transform.translation.x = x;
    static_transform_stamped.transform.translation.y = y;
    static_transform_stamped.transform.translation.z = z;

    tf_publisher_->sendTransform(static_transform_stamped);
  }

  void broadcast_static_tf_body2cam2()
  {
    // get body to cam_0_link transform
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg = tf_buffer_->lookupTransform("body", "cam_0_link", tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> transform_stamped;
    tf2::fromMsg(transform_stamped_msg, transform_stamped);

    tf2::Transform tf_body_0l;
    tf_body_0l.setOrigin(transform_stamped.getOrigin());
    tf_body_0l.setRotation(transform_stamped.getRotation());

    Eigen::Vector3f c0_center;
    c0_center = this->get_camera_center(tf_body_0l.inverse());
    // cout << "cam_0 center: " << endl << c0_center << endl;

    tf2::Transform tf_0l_0cof;
    // tf_0l_0cof.setOrigin({-0.000579873, 0.0147138, 5.84737e-05});
    // tf_0l_0cof.setRotation({-0.498893, 0.501671, -0.498554, 0.500874});
    // TODO: change it to manual values!
    tf_0l_0cof.setOrigin(transform_stamped.getOrigin());
    tf_0l_0cof.setRotation(transform_stamped.getRotation());

    tf2::Transform tf_2l_2cof;
    tf_2l_2cof.setOrigin({-0.000381861, 0.0146603, 0.00030376});
    tf_2l_2cof.setRotation({-0.496401, 0.500766, -0.50067, 0.502145});

    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.frame_id = "body";
    static_transform_stamped.child_frame_id = "cam_2_link";

    tf2::Transform tf_2cof_0cof, tf_body_2l;
    tf2::Quaternion q_2cof_0cof, q_body_2l;

    tf2::Vector3 t_2cof_0cof;
    tf2::Matrix3x3 tfRot;

    // p:cam0_color_optical_frame, c:cam1_color_optical_frame
    q_2cof_0cof={0.009505,    -0.57190487,  -0.40605527,  0.71270863};
    t_2cof_0cof={-0.403613,    0.18562103, -0.24098303};
    tf_2cof_0cof.setOrigin(t_2cof_0cof);
    tf_2cof_0cof.setRotation(q_2cof_0cof);

    Eigen::Vector3f center;
    center = this->get_camera_center(tf_2cof_0cof);
    // cout << "cam2cof center in cam0cof frame: " << endl << center << endl;

    // tf_body_1l= tf_body_0l * tf_0l_0cof * tf_1cof_0cof.inverse() * tf_1l_1cof.inverse();
    // tf_body_2l = tf_body_0l * tf_0l_0cof * tf_2cof_0cof.inverse() * tf_2l_2cof.inverse();
    // TODO: change 'tf_2cof_0cof' to 'tf_2cof_0cof.inverse()'
    tf_body_2l = tf_body_0l * tf_0l_0cof * tf_2cof_0cof * tf_2l_2cof.inverse();
    Eigen::Vector3f c2_center;
    c2_center = this->get_camera_center(tf_body_2l.inverse());
    // cout << "cam_2 center: " << endl << c2_center << endl;

    // print_tf(tf_body_2l, "tf_body_2l");

    q_body_2l=tf_body_2l.getRotation();
    // double roll, pitch, yaw;
    // tfRot.getRPY(roll, pitch, yaw);
    // q_new.setRPY(roll, pitch, yaw);
    double x = tf_body_2l.getOrigin().getX();
    double y = tf_body_2l.getOrigin().getY();
    double z = tf_body_2l.getOrigin().getZ();

    static_transform_stamped.transform.rotation.x = q_body_2l[0];
    static_transform_stamped.transform.rotation.y = q_body_2l[1];
    static_transform_stamped.transform.rotation.z = q_body_2l[2];
    static_transform_stamped.transform.rotation.w = q_body_2l[3];
    static_transform_stamped.transform.translation.x = x;
    static_transform_stamped.transform.translation.y = y;
    static_transform_stamped.transform.translation.z = z;

    tf_publisher_->sendTransform(static_transform_stamped);
  }

  void broadcast_static_tf_body2cam3()
  {
    // get body to cam_0_link transform
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg = tf_buffer_->lookupTransform("body", "cam_0_link", tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> transform_stamped;
    tf2::fromMsg(transform_stamped_msg, transform_stamped);

    tf2::Transform tf_body_0l;
    tf_body_0l.setOrigin(transform_stamped.getOrigin());
    tf_body_0l.setRotation(transform_stamped.getRotation());

    Eigen::Vector3f c0_center;
    c0_center = this->get_camera_center(tf_body_0l.inverse());
    // cout << "cam_0 center: " << endl << c0_center << endl;

    tf2::Transform tf_0l_0cof;
    tf_0l_0cof.setOrigin({-0.000579873, 0.0147138, 5.84737e-05});
    tf_0l_0cof.setRotation({-0.498893, 0.501671, -0.498554, 0.500874});

    tf2::Transform tf_3l_3cof;
    tf_3l_3cof.setOrigin({-0.000596907, 0.0147666, 0.000408532});
    tf_3l_3cof.setRotation({0.49853, -0.49721, 0.504945, -0.499279});

    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.frame_id = "body";
    static_transform_stamped.child_frame_id = "cam_3_link";

    tf2::Transform tf_3cof_0cof, tf_3cof_1cof, tf_1cof_0cof, tf_body_3l;
    tf2::Quaternion q_1cof_0cof, q_3cof_1cof, q_body_3l;

    tf2::Vector3 t_1cof_0cof, t_3cof_1cof;
    tf2::Matrix3x3 tfRot;

    // p:cam0_color_optical_frame, c:cam1_color_optical_frame
    q_1cof_0cof={-0.00762625, 0.58842435, 0.3991386,   0.70312659};
    t_1cof_0cof={0.46120531,  0.12047342, -0.19447596};
    tf_1cof_0cof.setOrigin(t_1cof_0cof);
    tf_1cof_0cof.setRotation(q_1cof_0cof);

    //   tf::Vector3 t_3cof_0cof;
    //   tf::Quaternion q_3cof_0cof;
    // 2 Vert Cams TF setup
    //   q_3cof_0cof={0.32876835, -0.00803235, -0.00639744, 0.94435477};
    //   t_3cof_0cof={0.00203677, -0.05767851, 0.00653627};
    //   tf_3cof_0cof.setOrigin(t_3cof_0cof);
    //   tf_3cof_0cof.setRotation(q_3cof_0cof);

    // Original 4-cam based cam3
    q_3cof_1cof={0.00402333, 0.5985427,  0.39182399,  0.69871626};
    t_3cof_1cof={0.26441368,  0.20489213, -0.34793621};
    tf_3cof_1cof.setOrigin(t_3cof_1cof);
    tf_3cof_1cof.setRotation(q_3cof_1cof);

    tf_3cof_0cof=tf_3cof_1cof*tf_1cof_0cof;
    Eigen::Vector3f center;
    center = this->get_camera_center(tf_3cof_0cof);
    // cout << "cam3cof center in cam0cof frame: " << endl << center << endl;



    // tf_body_1l= tf_body_0l * tf_0l_0cof * tf_1cof_0cof.inverse() * tf_1l_1cof.inverse();
    tf_body_3l = tf_body_0l * tf_0l_0cof * tf_3cof_0cof.inverse() * tf_3l_3cof.inverse();
    Eigen::Vector3f c3_center;
    c3_center = this->get_camera_center(tf_body_3l.inverse());
    // cout << "cam_3 center: " << endl << c3_center << endl;

    // print_tf(tf_body_3l, "tf_body_3l");
    // p:cam0_link, c:cam1_link
    // tf_body_1l= tf_body_0l * tf_0l_0cf * tf_0cf_0cof * tf_1cof_0cof.inverse() * tf_1cf_1cof.inverse() * tf_1l_1cf.inverse();
    // tfMat=GetTransformMatrix_mk(q_0l_0cf,t_0l_0cf)*GetTransformMatrix_mk(q_0cf_0cof,t_0cf_0cof)*GetTransformMatrix_mk(q_0cof_1cof,t_0cof_1cof)*GetTransformMatrix_mk(q_1cf_1cof,t_1cf_1cof).inverse()*GetTransformMatrix_mk(q_1l_1cf,t_1l_1cf).inverse();
    // q_1cf_1l={-0.00292005832307, 0.0028238222003,  0.00144543300848,  0.999990701675};
    // tf_body_1l = tf_body_1l.inverse();
    q_body_3l = tf_body_3l.getRotation();
    // double roll, pitch, yaw;
    // tfRot.getRPY(roll, pitch, yaw);
    // q_new.setRPY(roll, pitch, yaw);
    double x = tf_body_3l.getOrigin().getX();
    double y = tf_body_3l.getOrigin().getY();
    double z = tf_body_3l.getOrigin().getZ();

    static_transform_stamped.transform.rotation.x = q_body_3l[0];
    static_transform_stamped.transform.rotation.y = q_body_3l[1];
    static_transform_stamped.transform.rotation.z = q_body_3l[2];
    static_transform_stamped.transform.rotation.w = q_body_3l[3];
    static_transform_stamped.transform.translation.x = x;
    static_transform_stamped.transform.translation.y = y;
    static_transform_stamped.transform.translation.z = z;

    tf_publisher_->sendTransform(static_transform_stamped);
  }

  void print_tf(tf2::Transform transform_tf, std::string name)
  {
    tf2::Quaternion q = transform_tf.getRotation();
    // q.normalize();

    tf2::Matrix3x3 rotMat;
    rotMat.setRotation(q);

    Eigen::Matrix4f TransformMat;
    TransformMat << rotMat.getRow(0).getX(), rotMat.getRow(0).getY(), rotMat.getRow(0).getZ(), transform_tf.getOrigin().x()
                  , rotMat.getRow(1).getX(), rotMat.getRow(1).getY(), rotMat.getRow(1).getZ(), transform_tf.getOrigin().y()
                  , rotMat.getRow(2).getX(), rotMat.getRow(2).getY(), rotMat.getRow(2).getZ(), transform_tf.getOrigin().z()
                  , 0, 0, 0, 1;

    std::cout << name + ": " << std::endl << TransformMat << std::endl;
  }

  Eigen::Matrix4f get_transform_matrix_old(geometry_msgs::msg::TransformStamped transform_stamped)
  {
    auto quaternion = transform_stamped.transform.rotation;
    auto quaternion_eigen = Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    auto rotation_mat = quaternion_eigen.normalized().toRotationMatrix();

    auto translation = transform_stamped.transform.translation;

    Eigen::Matrix4f transform_mat;
    transform_mat << (float)rotation_mat(0,0), (float)rotation_mat(0,1), (float)rotation_mat(0,2), (float)translation.x,
                     (float)rotation_mat(1,0), (float)rotation_mat(1,1), (float)rotation_mat(1,2), (float)translation.y,
                     (float)rotation_mat(2,0), (float)rotation_mat(2,1), (float)rotation_mat(2,2), (float)translation.z,
                     0, 0, 0, 1;

    // Eigen::Matrix4f TransformMat;
    // TransformMat << rotMat.getRow(0).getX(), rotMat.getRow(0).getY(), rotMat.getRow(0).getZ(), transform_tf.getOrigin().x()
    //               , rotMat.getRow(1).getX(), rotMat.getRow(1).getY(), rotMat.getRow(1).getZ(), transform_tf.getOrigin().y()
    //               , rotMat.getRow(2).getX(), rotMat.getRow(2).getY(), rotMat.getRow(2).getZ(), transform_tf.getOrigin().z()
    //               , 0, 0, 0, 1;
    // TransformMat << 1-2*(pow(q.y(),2) + pow(q.z(),2)) , 2*(q.x()*q.y()-q.z()*q.w()) , 2*(q.x()*q.z()+q.y()*q.w()) , transform_tf.getOrigin().x()
    //                     , 2*(q.x()*q.y() + q.z()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.z(),2)) , 2*(q.y()*q.z()-q.x()*q.w()) , transform_tf.getOrigin().y()
    //                     , 2*(q.x()*q.z()-q.y()*q.w()) , 2*(q.y()*q.z()+q.x()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.y(),2)) , transform_tf.getOrigin().z()
    //                     , 0 , 0 , 0 , 1;

    return transform_mat;
  }

  Eigen::Matrix4f get_transform_matrix(geometry_msgs::msg::TransformStamped transform_stamped_msg)
  {
    tf2::Stamped<tf2::Transform> transform_stamped;
    tf2::fromMsg(transform_stamped_msg, transform_stamped);

    tf2::Matrix3x3 rotation_mat = transform_stamped.getBasis();
    tf2::Vector3 translation = transform_stamped.getOrigin();

    Eigen::Matrix4f transform_mat;
    transform_mat << (float)rotation_mat[0][0], (float)rotation_mat[0][1], (float)rotation_mat[0][2], (float)translation.x(),
                     (float)rotation_mat[1][0], (float)rotation_mat[1][1], (float)rotation_mat[1][2], (float)translation.y(),
                     (float)rotation_mat[2][0], (float)rotation_mat[2][1], (float)rotation_mat[2][2], (float)translation.z(),
                     0, 0, 0, 1;

    // Eigen::Matrix4f TransformMat;
    // TransformMat << rotMat.getRow(0).getX(), rotMat.getRow(0).getY(), rotMat.getRow(0).getZ(), transform_tf.getOrigin().x()
    //               , rotMat.getRow(1).getX(), rotMat.getRow(1).getY(), rotMat.getRow(1).getZ(), transform_tf.getOrigin().y()
    //               , rotMat.getRow(2).getX(), rotMat.getRow(2).getY(), rotMat.getRow(2).getZ(), transform_tf.getOrigin().z()
    //               , 0, 0, 0, 1;
    // TransformMat << 1-2*(pow(q.y(),2) + pow(q.z(),2)) , 2*(q.x()*q.y()-q.z()*q.w()) , 2*(q.x()*q.z()+q.y()*q.w()) , transform_tf.getOrigin().x()
    //                     , 2*(q.x()*q.y() + q.z()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.z(),2)) , 2*(q.y()*q.z()-q.x()*q.w()) , transform_tf.getOrigin().y()
    //                     , 2*(q.x()*q.z()-q.y()*q.w()) , 2*(q.y()*q.z()+q.x()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.y(),2)) , transform_tf.getOrigin().z()
    //                     , 0 , 0 , 0 , 1;

    return transform_mat;
  }

  Eigen::Vector3f get_camera_center(tf2::Transform tf_)
  {
      tf2::Quaternion q = tf_.getRotation();
      // q.normalize();

      tf2::Matrix3x3 rotMat;
      rotMat.setRotation(q);

      Eigen::Matrix3f rot_eigen;
      rot_eigen << rotMat.getRow(0).getX(), rotMat.getRow(0).getY(), rotMat.getRow(0).getZ()
                    , rotMat.getRow(1).getX(), rotMat.getRow(1).getY(), rotMat.getRow(1).getZ()
                    , rotMat.getRow(2).getX(), rotMat.getRow(2).getY(), rotMat.getRow(2).getZ();

      Eigen::Vector3f trans_eigen(tf_.getOrigin().x(), tf_.getOrigin().y(), tf_.getOrigin().z());

      Eigen::Vector3f center;
      center = -rot_eigen.inverse() * trans_eigen;
      // TransformMat << 1-2*(pow(q.y(),2) + pow(q.z(),2)) , 2*(q.x()*q.y()-q.z()*q.w()) , 2*(q.x()*q.z()+q.y()*q.w()) , transform_tf.getOrigin().x()
      //                     , 2*(q.x()*q.y() + q.z()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.z(),2)) , 2*(q.y()*q.z()-q.x()*q.w()) , transform_tf.getOrigin().y()
      //                     , 2*(q.x()*q.z()-q.y()*q.w()) , 2*(q.y()*q.z()+q.x()*q.w()) , 1-2*( pow(q.x(),2)+pow(q.y(),2)) , transform_tf.getOrigin().z()
      //                     , 0 , 0 , 0 , 1;
      return center;
  }

  bool use_cam0_;
  bool use_cam1_;
  bool use_cam2_;
  bool use_cam3_;
  bool cam0_merged_;
  bool cam1_merged_;
  bool cam2_merged_;
  bool cam3_merged_;
  std::string cam0_depth_topic_;
  std::string cam1_depth_topic_;
  std::string cam2_depth_topic_;
  std::string cam3_depth_topic_;
  std::string publish_topic_;
  bool apply_voxel_grid_filter_;
  double voxel_grid_filter_size_;
  double max_publish_duration_;
  double point_cloud_publish_duration_;

  boost::recursive_mutex mutex_;
  boost::recursive_mutex cam0_mutex_;
  boost::recursive_mutex cam1_mutex_;
  boost::recursive_mutex cam2_mutex_;
  boost::recursive_mutex cam3_mutex_;

  rclcpp::Time last_published_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam0_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam1_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam2_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam3_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MergePointCloud>();

  // TODO: mutli-thread executor
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
