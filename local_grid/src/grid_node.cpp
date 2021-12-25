#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <backward.hpp>
#include <chrono>
#include <cmath>

#include "raycast_ring_buffer.h"

namespace backward {
backward::SignalHandling sh;
}

const int POW = 7;  // 2^7 = 128
const double resolution = 0.03;

bool initialized = false;

grid::RaycastRingBuffer<POW>::Ptr local_3d_map;

ros::Publisher occ_marker_pub, free_marker_pub;
tf::TransformListener* listener;

cv::Mat depth_image;

void odomDpethCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg) {
  // Solve all of perception here...
  // ROS_INFO("Sync_Callback");
  // Step 1 get T_w_b T_b_camera
  tf::Transform tf_w_b;
  tf::StampedTransform tf_b_camera, tf_b_lidar;
  // get T_w_b
  tf_w_b.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                               odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z));
  tf_w_b.setRotation(tf::Quaternion(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
  Eigen::Affine3d dT_w_b;
  tf::transformTFToEigen(tf_w_b, dT_w_b);
  Eigen::Affine3f T_w_b = dT_w_b.cast<float>();
  // get T_b_camera
  listener->lookupTransform(odom_msg->child_frame_id,
                            depth_msg->header.frame_id, depth_msg->header.stamp,
                            tf_b_camera);
  Eigen::Affine3d dT_b_camera;
  tf::transformTFToEigen(tf_b_camera, dT_b_camera);
  Eigen::Affine3f T_b_camera = dT_b_camera.cast<float>();
  // get Origin (base_link)
  Eigen::Vector3f origin = (T_w_b * Eigen::Vector4f(0, 0, 0, 1)).head<3>();

  // Step 2 depth_msg convert to cloud
  grid::RaycastRingBuffer<POW>::PointCloud cloud;

  // configuration: 640*480 D435 & T265
  // const float fx = 384.6916198730469;
  // const float fy = 384.6916198730469;
  // const float cx = 320.2483215332031;
  // const float cy = 239.5032501220703;

  // configuration: 848*480 D435 & T265
  const float fx = 424.763671875;
  const float fy = 424.763671875;
  const float cx = 424.27420043945310;
  const float cy = 239.45150756835938;
  // sensor_msgs::Image convert to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
    }
    cv_ptr->image.copyTo(depth_image);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const float inv_factor = 1.0 / 1000.0;
  uint16_t* row_ptr;
  int cols = depth_image.cols;
  int rows = depth_image.rows;
  const int depth_filter_margin = 5;
  const int skip_pixel = 2;
  const float depth_filter_mindist = 0.1;
  const float depth_filter_maxdist = 3.5;  // 3.84/2*sqrt(3)=3.33m
  float depth;
  for (int v = depth_filter_margin; v < rows - depth_filter_margin;
       v += skip_pixel) {
    row_ptr = depth_image.ptr<uint16_t>(v) + depth_filter_margin;
    for (int u = depth_filter_margin; u < cols - depth_filter_margin;
         u += skip_pixel) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel;
      if (*row_ptr == 0) {
        continue;
      } else if (depth < depth_filter_mindist) {
        continue;
      } else if (depth > depth_filter_maxdist) {
        depth = depth_filter_maxdist;
      }
      // project to world frame
      Eigen::Vector4f pt_camera;
      pt_camera(0) = (u - cx) * depth / fx;
      pt_camera(1) = (v - cy) * depth / fy;
      pt_camera(2) = depth;
      pt_camera(3) = 1.0;
      pt_camera = T_w_b * T_b_camera * pt_camera;
      cloud.push_back(pt_camera);
    }
  }

  ROS_INFO_STREAM("Cloud size: " << cloud.size());

  // Step 3
  if (!initialized) {
    ROS_INFO("Initializing Volume");
    Eigen::Vector3i origin_idx;
    local_3d_map->getIdx(origin, origin_idx);
    local_3d_map->setCenter(origin_idx);  // offset != origin == center ?
    initialized = true;
  } else {
    Eigen::Vector3i origin_idx, center, diff;
    local_3d_map->getIdx(origin, origin_idx);
    center = local_3d_map->getVolumeCenter();
    diff = origin_idx - center;
    while (diff.array().any()) {
      ROS_INFO("Moving Volume");
      local_3d_map->moveVolume(diff);
      center = local_3d_map->getVolumeCenter();
      diff = origin_idx - center;
    }
  }

  ROS_INFO_STREAM("Insert " << cloud.size() << " PointClouds ......");
  local_3d_map->insertPointCloud(cloud, origin);
  ROS_INFO_STREAM("InsertPointCloud finished.");

  // VISUALIZATION
  if (occ_marker_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 m_occ;
    local_3d_map->getMarkerOccupied(m_occ);
    occ_marker_pub.publish(m_occ);
  }
  if (free_marker_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 m_free;
    local_3d_map->getMarkerFree(m_free);
    free_marker_pub.publish(m_free);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ring_buffer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  listener = new tf::TransformListener;
  occ_marker_pub =
      nh.advertise<sensor_msgs::PointCloud2>("ring_buffer/occupied", 5);
  free_marker_pub =
      nh.advertise<sensor_msgs::PointCloud2>("ring_buffer/free", 5);

  // 同步
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(
      nh, "/t265/odom/sample", 100);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(
      nh, "/d400/depth/image_rect_raw", 100);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          sensor_msgs::Image>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub,
                                                   depth_sub);
  sync.registerCallback(boost::bind(&odomDpethCallback, _1, _2));

  /*
  ┌──128*0.03=64*0.06=3.84m────┐
  │                            │
  │                            │
  │                            │
  │                            │
  │           ┌────┐           │
  │           └────┘           │
  │                            │
  │                            │
  │                            │
  │                            │
  └────────────────────────────┘
   */
  local_3d_map.reset(new grid::RaycastRingBuffer<POW>(resolution));

  ros::spin();
  return 0;
}