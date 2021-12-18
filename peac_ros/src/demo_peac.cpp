#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/Eigen>
#include <chrono>
// #include <opencv2/imgproc/imgproc.hpp>

#include "AHCPlaneFitter.hpp"
#include "plane_msg/Plane.h"
#include "plane_msg/VecPlane.h"

using namespace std;
using namespace std::chrono;
struct OrganizedImage3D {
  const cv::Mat_<cv::Vec3f>& cloud;
  // note: ahc::PlaneFitter assumes mm as unit!!!
  OrganizedImage3D(const cv::Mat_<cv::Vec3f>& c) : cloud(c) {}
  inline int width() const { return cloud.cols; }
  inline int height() const { return cloud.rows; }
  inline bool get(const int row, const int col, double& x, double& y,
                  double& z) const {
    const cv::Vec3f& p = cloud.at<cv::Vec3f>(row, col);
    x = p[0];
    y = p[1];
    z = p[2];
    return z > 0 && isnan(z) == 0;  // return false if current depth is NaN
  }
};
typedef ahc::PlaneFitter<OrganizedImage3D> PlaneFitter;

class peac_ros {
 public:
  peac_ros(const ros::NodeHandle& private_nh = ros::NodeHandle("~"))
      : private_nh_(private_nh), it_(private_nh) {
    pub_vec_planes_ = private_nh_.advertise<plane_msg::VecPlane>("vecPlane", 5);
    pub_pcl2_depth_ =
        private_nh_.advertise<sensor_msgs::PointCloud2>("depth_pcl", 5);
    pub_img_planes_ = it_.advertise("vec_plane_img", 1);

    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        private_nh_, "/d400/depth/image_rect_raw", 50));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        private_nh_, "/t265/odom/sample", 100));
    sync_image_odom_.reset(
        new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(
        boost::bind(&peac_ros::depthOdomCallback, this, _1, _2));
  }

  void depthOdomCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                         const nav_msgs::OdometryConstPtr& odom) {
    // Get d400 pose in world (t265 as body)
    Eigen::Quaterniond body_rotation_q = Eigen::Quaterniond(
        odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    Eigen::Matrix3d body_rotation_m = body_rotation_q.toRotationMatrix();
    Eigen::Matrix4d body2world;  // from world to body
    body2world.block<3, 3>(0, 0) = body_rotation_m;
    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;
    body2world(3, 3) = 1.0;

    Eigen::Matrix4d cam2body;  // from body to cam
    Eigen::Quaterniond cam2body_rotation_q =
        Eigen::Quaterniond(0.500, -0.500, 0.500, -0.500);
    cam2body.block<3, 3>(0, 0) = cam2body_rotation_q.toRotationMatrix();
    cam2body(0, 3) = -0.009375589;
    cam2body(1, 3) = 0.015890727;
    cam2body(2, 3) = 0.028273059;
    cam2body(3, 3) = 1.0;

    Eigen::Matrix4d cam_world = body2world * cam2body;
    Eigen::Vector3d camera_pos;
    camera_pos(0) = cam_world(0, 3);
    camera_pos(1) = cam_world(1, 3);
    camera_pos(2) = cam_world(2, 3);
    Eigen::Quaterniond camera_rotation_q =
        Eigen::Quaterniond(cam_world.block<3, 3>(0, 0));
    Eigen::Matrix3d camera_rotation_m = camera_rotation_q.toRotationMatrix();

    cv::Mat depth;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
      }
      cv_ptr->image.copyTo(depth);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // configuration: 640*480 D435 & T265
    const float fx = 384.6916198730469;
    const float fy = 384.6916198730469;
    const float cx = 320.2483215332031;
    const float cy = 239.5032501220703;
    const float min_use_range = 0.1;  // unit: m
    const float max_use_range = 3.5;  // unit: m

    // only for visulization
    pcl::PointCloud<pcl::PointXYZ> pcl_viz;

    cv::Mat_<cv::Vec3f> cloud(depth.rows, depth.cols);
    for (int r = 0; r < depth.rows; r++) {
      uint16_t* depth_ptr = depth.ptr<uint16_t>(r);
      cv::Vec3f* pt_ptr = cloud.ptr<cv::Vec3f>(r);
      for (int c = 0; c < depth.cols; c++) {
        float z = (float)depth_ptr[c] / 1000.0;

        if (z < min_use_range || z > max_use_range) z = 0;

        Eigen::Vector3d proj_pt;
        proj_pt(0) = (c - cx) * z / fx;
        proj_pt(1) = (r - cy) * z / fy;
        proj_pt(2) = z;
        // note: Do not transform before PlaneFitter!!!
        pt_ptr[c][0] = proj_pt(0) * 1000.0;  // unit: m->mm
        pt_ptr[c][1] = proj_pt(1) * 1000.0;  // unit: m->mm
        pt_ptr[c][2] = proj_pt(2) * 1000.0;  // unit: m->mm

        // only for visulization
        if (z > 0.0) {
          proj_pt = camera_rotation_m * proj_pt + camera_pos;
          pcl::PointXYZ pt;
          pt.x = proj_pt(0);
          pt.y = proj_pt(1);
          pt.z = proj_pt(2);
          pcl_viz.points.push_back(pt);
        }
      }
    }

    // only for visulization
    if (pub_pcl2_depth_.getNumSubscribers() > 0) {
      pcl_viz.width = pcl_viz.points.size();
      pcl_viz.height = 1;
      pcl_viz.is_dense = true;
      sensor_msgs::PointCloud2 pcl_ros;
      pcl::toROSMsg(pcl_viz, pcl_ros);
      pcl_ros.header.frame_id = "t265_odom_frame";
      pcl_ros.header.stamp = depth_msg->header.stamp;
      pub_pcl2_depth_.publish(pcl_ros);
    }

    pf.minSupport = 3000;
    pf.windowWidth = 10;
    pf.windowHeight = 10;
    pf.doRefine = true;

    auto t1 = high_resolution_clock::now();
    std::vector<std::vector<int>> Membership;
    cv::Mat Seg(depth.rows, depth.cols, CV_8UC3);
    OrganizedImage3D Ixyz(cloud);
    pf.run(&Ixyz, &Membership, &Seg, 0, true);
    auto duration =
        duration_cast<microseconds>(high_resolution_clock::now() - t1);
    cout << "PlaneFitter: " << duration.count() / 1000.0 << " ms" << endl;

    if (pub_img_planes_.getNumSubscribers() > 0) {
      cv_bridge::CvImage out_msg;
      out_msg.header = depth_msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = Seg;
      pub_img_planes_.publish(out_msg.toImageMsg());
    }

    // GenerateVecPlanes
    plane_msg::VecPlane vecPlane;

    size_t plane_number = Membership.size();
    vecPlane.vecPlane.resize(plane_number);
    vecPlane.header.stamp = depth_msg->header.stamp;
    vecPlane.header.frame_id = "t265_odom_frame";

    for (size_t i = 0; i < plane_number; i++) {
      pcl::PointCloud<pcl::PointXYZ> pclcloud;
      uint32_t point_number = Membership[i].size();
      for (size_t j = 0; j < point_number; j++) {
        int index = Membership[i][j];
        int i_x = index / depth.cols;
        int i_y = index % depth.cols;
        double x, y, z;
        Ixyz.get(i_x, i_y, x, y, z);
        Eigen::Vector3d proj_pt(x / 1000.0, y / 1000.0, z / 1000.0);
        proj_pt = camera_rotation_m * proj_pt + camera_pos;
        pclcloud.points.push_back(
            pcl::PointXYZ(proj_pt(0), proj_pt(1), proj_pt(2)));
      }
      pclcloud.width = point_number;
      pclcloud.height = 1;

      pcl::toROSMsg(pclcloud, vecPlane.vecPlane[i].cloud);
      vecPlane.vecPlane[i].cloud.header = vecPlane.header;
    }

    pub_vec_planes_.publish(vecPlane);
  }

  // data structure
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>>
      SynchronizerImageOdom;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImageOdom sync_image_odom_;

  ros::NodeHandle private_nh_;
  ros::Publisher pub_vec_planes_;
  ros::Publisher pub_pcl2_depth_;

  image_transport::ImageTransport it_;
  image_transport::Publisher pub_img_planes_;

  PlaneFitter pf;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_peac");
  ros::NodeHandle private_nh("~");

  peac_ros peacRos(private_nh);

  ros::spin();
  ros::shutdown();
  return 0;
}
