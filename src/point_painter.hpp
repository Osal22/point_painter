// Copyright 2025 Huzaifa Osal.

#ifndef POINT_PAINTER_HPP_
#define POINT_PAINTER_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace point_painter
{
/**
 * \class PointPainter
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class PointPainter : public rviz_common::Display
{
  Q_OBJECT

public:
  PointPainter();
  ~PointPainter() override;

  // Overrides from Display
  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void update(float wall_dt, float ros_dt) override;

  void setup_ros_subscriptions();

private Q_SLOTS:
  void topic_updated_image();
  void topic_updated_camera_info();
  void topic_updated_pointcloud();

private:
  void image_pointcloud_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud_msg);

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  void project_point_ploud_to_image(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

  Ogre::ManualObject * rings_manual_object_{nullptr};
  Ogre::ManualObject * wave_manual_object_{nullptr};
  float wave_range_{0.0};
  std::mutex property_mutex_;
  rviz_common::properties::TfFrameProperty * frame_property_;

  std::unique_ptr<rviz_common::properties::RosTopicProperty> image_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> camera_info_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> pointcloud_topic_property_;

  std::string image_topic_name_;
  std::string camera_info_topic_name_;
  std::string pointcloud_topic_name_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rviz_rendering::PointCloud> point_cloud_render_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_color_intensity_pub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace point_painter

#endif  // POINT_PAINTER_HPP_
