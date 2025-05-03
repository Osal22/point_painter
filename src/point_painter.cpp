// Copyright 2025 Huzaifa Osal.

#include "point_painter.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/objects/grid.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <string>

namespace point_painter
{
PointPainter::PointPainter()
{
  frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Reference Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame this grid will use for its origin.", this, nullptr, true);
}

void PointPainter::onEnable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  setup_ros_subscriptions();
}

PointPainter::~PointPainter()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
}

void PointPainter::onInitialize()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  auto rviz_ros_node = context_->getRosNodeAbstraction();

  image_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Image Topic", "/image", "sensor_msgs/msg/Image", "Topic for Gear Data", this,
    SLOT(topic_updated_image()));
  image_topic_property_->initialize(rviz_ros_node);

  camera_info_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "CameraInfo Topic", "/camera_info", "sensor_msgs/msg/CameraInfo", "Topic for Gear Data", this,
    SLOT(topic_updated_camera_info()));
  camera_info_topic_property_->initialize(rviz_ros_node);

  pointcloud_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "PointCloud2 Topic", "/pointcloud", "sensor_msgs/msg/PointCloud2", "Topic for Gear Data", this,
    SLOT(topic_updated_pointcloud()));
  pointcloud_topic_property_->initialize(rviz_ros_node);

  frame_property_->setFrameManager(context_->getFrameManager());

  point_cloud_render_ = std::make_shared<rviz_rendering::PointCloud>();
  scene_node_->attachObject(point_cloud_render_.get());
}

void PointPainter::image_pointcloud_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud_msg)
{
  if (camera_info_msg_->header.frame_id != "") {
    project_point_ploud_to_image(input_pointcloud_msg, input_image_msg, camera_info_msg_);
  }
}

void PointPainter::topic_updated_image()
{
  image_topic_name_ = image_topic_property_->getTopicStd();
  setup_ros_subscriptions();
}

void PointPainter::topic_updated_camera_info()
{
  camera_info_topic_name_ = camera_info_topic_property_->getTopicStd();
  setup_ros_subscriptions();
}

void PointPainter::topic_updated_pointcloud()
{
  pointcloud_topic_name_ = pointcloud_topic_property_->getTopicStd();
  setup_ros_subscriptions();
}

void PointPainter::setup_ros_subscriptions()
{
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
               .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  if (image_topic_name_ != "" && pointcloud_topic_name_ != "" && camera_info_topic_name_ != "") {
    camera_info_sub_.reset();
    image_sub_.subscribe(
      rviz_ros_node->get_raw_node(), image_topic_name_, qos.get_rmw_qos_profile());
    cloud_sub_.subscribe(
      rviz_ros_node->get_raw_node(), pointcloud_topic_name_, qos.get_rmw_qos_profile());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rviz_ros_node->get_raw_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Define ApproximateTime policy with queue size of 10
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), image_sub_, cloud_sub_);

    sync_->registerCallback(std::bind(
      &PointPainter::image_pointcloud_callback, this, std::placeholders::_1,
      std::placeholders::_2));

    camera_info_sub_ =
      rviz_ros_node->get_raw_node()->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_name_, qos,
        std::bind(&PointPainter::camera_info_callback, this, std::placeholders::_1));

    pc_color_pub_ = rviz_ros_node->get_raw_node()->create_publisher<sensor_msgs::msg::PointCloud2>(
      "painted_cloud", 10);
    pc_color_intensity_pub_ =
      rviz_ros_node->get_raw_node()->create_publisher<sensor_msgs::msg::PointCloud2>(
        "painted_intensity_cloud", 10);
  }
}

void PointPainter::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  camera_info_msg_ = msg;
}

void PointPainter::reset()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
}

void PointPainter::project_point_ploud_to_image(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  if (cloud->header.frame_id != image->header.frame_id) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        image->header.frame_id, cloud->header.frame_id, tf2::TimePointZero);

      tf2::doTransform(*cloud, transformed_cloud, transform_stamped);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        rclcpp::get_logger("point_painter"), "Could not transform point cloud: %s", ex.what());
      return;
    }

    // Extract camera intrinsic parameters
    const auto & K = camera_info->k;  // 3x3 row-major intrinsic matrix
    double fx = K[0], fy = K[4];
    double cx = K[2], cy = K[5];

    // Ensure image is in an accessible format (e.g., CV_8UC3 for RGB)
    cv::Mat cv_image(
      image->height, image->width, CV_8UC3, const_cast<unsigned char *>(image->data.data()));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_pcl(new pcl::PointCloud<pcl::PointXYZI>);

    // Iterate through the point cloud_pcl
    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(transformed_cloud, "x"),
         it_y(transformed_cloud, "y"), it_z(transformed_cloud, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      float x = *it_x, y = *it_y, z = *it_z;

      // Skip points with no valid depth
      if (z <= 0) continue;

      // Project 3D point to 2D image plane
      int u = static_cast<int>((fx * x / z) + cx);
      int v = static_cast<int>((fy * y / z) + cy);

      // Check if the projected point is within image bounds
      if (
        u >= 0 && u < static_cast<int>(image->width) && v >= 0 &&
        v < static_cast<int>(image->height)) {
        pcl::PointXYZRGB p_rgb;
        rviz_rendering::PointCloud::Point new_point;
        new_point.position.x = *it_x;
        new_point.position.y = *it_y;
        new_point.position.z = *it_z;
        p_rgb.x = x;
        p_rgb.y = y;
        p_rgb.z = z;

        // Draw point (set pixel to a specific color, e.g., red)
        // cv_image.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
        cv::Vec3b pixel = cv_image.at<cv::Vec3b>(v, u);  // OpenCV stores (y, x) format

        p_rgb.r = pixel[2];
        p_rgb.g = pixel[1];
        p_rgb.b = pixel[0];
        // new_point.color = rviz_rendering::Color(p_rgb.r, p_rgb.g, p_rgb.b);  // Default white
        // point_cloud_render_->addPoints(new_point);
        cloud_pcl->points.push_back(p_rgb);
      }
    }

    sensor_msgs::msg::PointCloud2 painted_cloud_msg;
    sensor_msgs::msg::PointCloud2 painted_intensity_cloud_msg;

    pcl::toROSMsg(*cloud_pcl, painted_cloud_msg);
    // pcl::toROSMsg(*cloud_intensity_pcl, painted_cloud_msg);

    painted_cloud_msg.header = image->header;
    // painted_intensity_cloud_msg.header = image->header;

    pc_color_pub_->publish(painted_cloud_msg);
    // pc_color_intensity_pub_->publish(painted_intensity_cloud_msg);
  }
}

void PointPainter::update(float, float)
{
  if (point_cloud_render_) {
    point_cloud_render_->setVisible(true);
  }
}

}  // namespace point_painter

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(point_painter::PointPainter, rviz_common::Display)
