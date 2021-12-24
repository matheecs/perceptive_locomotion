#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stair_modeling/stair_modeling_paramConfig.h>

#include <Eigen/Eigen>
#include <backward.hpp>
#include <chrono>

#include "plane_msg/Plane.h"
#include "plane_msg/VecPlane.h"
#include "stair_info_msg/stair_info.h"
#include "stair_modeling/StairPerception.hpp"

namespace backward {
backward::SignalHandling sh;
}

using namespace std;
using namespace std::chrono;

enum PolygonShowMode { polygon_line_mode = 0, polygon_surface_mode };

class StairModeling {
 public:
  explicit StairModeling(
      const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
      const std::string &topicVecPlane = "vec_planes",
      const std::string &topicOdom = "odom")
      : private_nh(private_nh),
        queueSize(5),
        count(0),
        topicVecPlane(topicVecPlane),
        topicOdom(topicOdom),
        has_stair(false),
        height(0),
        depth(0),
        hd(0),
        vd(0),
        time_window(2),
        server(new dynamic_reconfigure::Server<
               stair_modeling::stair_modeling_paramConfig>(private_nh)) {}

  void init() {
    ROS_INFO("############# StairModeling start #############");

    sub_vec_plane.reset(new message_filters::Subscriber<plane_msg::VecPlane>(
        private_nh, topicVecPlane, 10));
    sub_t265_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        private_nh, topicOdom, 100));
    sync_plane_odom.reset(
        new message_filters::Synchronizer<SyncPolicyPlaneOdom>(
            SyncPolicyPlaneOdom(100), *sub_vec_plane, *sub_t265_odom));
    sync_plane_odom->registerCallback(
        boost::bind(&StairModeling::planeOdomCallback, this, _1, _2));

    f = boost::bind(&StairModeling::cfg_callback, this, _1, _2);
    server->setCallback(f);

    pub_stair_info =
        private_nh.advertise<stair_info_msg::stair_info>("stair_info", 5);
    pub_stair_poly = private_nh.advertise<jsk_recognition_msgs::PolygonArray>(
        "stair_polygon_array", 1);

    time_record.open("modeling_time_record.csv");
    time_record << "TIME_STAMP"
                << ","
                << "pre-process"
                << ","
                << "pre-modeling"
                << ","
                << "stair-modeling" << endl;

    stair_record.open("modeling_result_record.csv");
    stair_record << "TIME_STAMP"
                 << ","
                 << "height"
                 << ","
                 << "depth"
                 << ","
                 << "h_dis"
                 << ","
                 << "v_dis" << endl;

    output_record.open("modeling_output_record.csv");
    output_record << "TIME_STAMP"
                  << ","
                  << "height"
                  << ","
                  << "depth"
                  << ","
                  << "h_dis"
                  << ","
                  << "v_dis" << endl;
  }

  void cfg_callback(stair_modeling::stair_modeling_paramConfig &config,
                    uint32_t level) {
    stair_detector.setDown_sample_points_number(
        config.down_sample_points_number);
    stair_detector.setAngle_h(pcl::deg2rad(config.angle_h));
    stair_detector.setAngle_v(pcl::deg2rad(config.angle_v));
    stair_detector.setGround_height_th(config.ground_height_th);
    stair_detector.setMin_ground_plane_points_num(
        config.min_ground_plane_points_num);
    stair_detector.setMerge_dis_th(config.merge_dis_th);
    stair_detector.setMerge_angle_th(pcl::deg2rad(config.merge_angle_th));
    stair_detector.setMerge_distance_th(config.merge_distance_th);
    stair_detector.setA1D_th(config.a1D_th);
    stair_detector.setDecay_rate(config.decay_rate);
    stair_detector.setVertical_plane_normal_angle_th(
        pcl::deg2rad(config.vertical_plane_normal_angle_th));
    stair_detector.setVertical_plane_normal_est_angle_th(
        pcl::deg2rad(config.vertical_plane_normal_est_angle_th));
    stair_detector.setCv_angle_horizontal_th(
        pcl::deg2rad(config.cv_angle_horizontal_th));
    stair_detector.setCv_dir_cluster_angle_th(
        pcl::deg2rad(config.cv_dir_cluster_angle_th));
    stair_detector.setPlane_max_length(config.plane_max_length);
    stair_detector.setPlane_min_length(config.plane_min_length);
    stair_detector.setPlane_max_width(config.plane_max_width);
    stair_detector.setPlane_min_width(config.plane_min_width);
    stair_detector.setStair_max_height(config.stair_max_height);
    stair_detector.setStair_max_width(config.stair_max_width);
    stair_detector.setStair_plane_max_distance(config.stair_plane_max_distance);
    stair_detector.setStair_cv_angle_diff_th(
        pcl::deg2rad(config.stair_cv_angle_diff_th));

    time_window = config.time_window;
  }

  // Do not use Eigen
  float fromQuaternion2yaw(Eigen::Quaternionf q) {
    float yaw =
        atan2(2 * (q.x() * q.y() + q.w() * q.z()),
              q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
  }

  void planeOdomCallback(const plane_msg::VecPlane::ConstPtr &pvec_plane,
                         const nav_msgs::OdometryConstPtr &odom) {
    Eigen::Vector3f camera_position;

    // Step1 get pose of camera
    {
      Eigen::Quaternionf body_rotation_q = Eigen::Quaternionf(
          odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
          odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
      Eigen::Matrix3f body_rotation_m = body_rotation_q.toRotationMatrix();
      Eigen::Matrix4f body2world;  // from world to body
      body2world.block<3, 3>(0, 0) = body_rotation_m;
      body2world(0, 3) = odom->pose.pose.position.x;
      body2world(1, 3) = odom->pose.pose.position.y;
      body2world(2, 3) = odom->pose.pose.position.z;
      body2world(3, 3) = 1.0;

      Eigen::Matrix4f cam2body;  // from body to cam
      Eigen::Quaternionf cam2body_rotation_q = Eigen::Quaternionf(1, 0, 0, 0);
      cam2body.block<3, 3>(0, 0) = cam2body_rotation_q.toRotationMatrix();
      cam2body(0, 3) = -0.009375589;
      cam2body(1, 3) = 0.015890727;
      cam2body(2, 3) = 0.028273059;
      cam2body(3, 3) = 1.0;

      Eigen::Matrix4f cam_world = body2world * cam2body;
      camera_position(0) = cam_world(0, 3);
      camera_position(1) = cam_world(1, 3);
      camera_position(2) = cam_world(2, 3);
      Eigen::Quaternionf camera_rotation_q =
          Eigen::Quaternionf(cam_world.block<3, 3>(0, 0));

      // update down_direction, right_direction, forward_direction
      stair_detector.down_direction = Eigen::Vector3f(0, 0, -1);

      float yaw = fromQuaternion2yaw(camera_rotation_q);
      Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());
      stair_detector.forward_direction =
          q.toRotationMatrix() * Eigen::Vector3f(1, 0, 0);

      stair_detector.right_direction =
          stair_detector.down_direction.cross(stair_detector.forward_direction);
      stair_detector.right_direction.normalize();
    }
    // Step2 stair modeling
    std::vector<double> vectime;

    count++;
    stair_perception::Stair stair;
    if (!pvec_plane->vecPlane.empty()) {
      std::ostringstream oss;
      oss.precision(3);
      vsp_plane.clear();

      auto t1 = system_clock::now();

      convertCloud(*pvec_plane, vsp_plane, camera_position);

      auto t2 = system_clock::now();
      auto duration = duration_cast<microseconds>(t2 - t1);
      ROS_INFO("Convert cloud time: %.2f ms", duration.count() / 1000.0);
      oss << "Convert cloud time: " << duration.count() / 1000.0 << " ms"
          << std::endl;

      has_stair = stair_detector.process(vsp_plane, stair, keyInfo, vectime);

      auto t3 = system_clock::now();
      duration = duration_cast<microseconds>(t3 - t2);
      ROS_INFO("Stair modeling time: %.2f ms",
               float(duration.count()) / 1000.0);
      oss << "Stair modeling time: " << duration.count() / 1000.0 << " ms"
          << std::endl;

      if (has_stair) {
        ROS_INFO("Find stair");
        detial_stair_model = stair_detector.getDetialStairModelString(stair);
        est_stair_param = stair_detector.getEstimatedParamString(stair);

        stair_record << pvec_plane->vecPlane[0].cloud.header.stamp << ",";
        stair_record << stair_detector.getEstimatedParamStringRcd(stair);

        send2robot(stair, pvec_plane->vecPlane[0].cloud.header.stamp);

        publish_rviz_stair(vsp_plane,
                           pvec_plane->vecPlane[0].cloud.header.stamp);
      } else {
        detial_stair_model = "No stair";
        est_stair_param = "No stair";
      }

      time_record << pvec_plane->vecPlane[0].cloud.header.stamp << ",";
      for (auto &t : vectime) time_record << t << ",";
      time_record << std::endl;

      running_time_str = oss.str();
    }
  }

  void convertCloud(const plane_msg::VecPlane &vec_plane,
                    std::vector<stair_perception::Plane> &vsp_plane,
                    Eigen::Vector3f camera_position) {
    unsigned long plane_number = vec_plane.vecPlane.size();
    vsp_plane.resize(plane_number);

    for (size_t i = 0; i < plane_number; i++) {
      pcl::fromROSMsg(vec_plane.vecPlane[i].cloud, vsp_plane[i].cloud);

      vsp_plane[i].camera_position = camera_position;
      vsp_plane[i].computeStatistics();
      vsp_plane[i].computePlaneInfo();
    }
  }

  void send2robot(stair_perception::Stair &stair, ros::Time ctime) {
    double height_now, depth_now, hd_now, vd_now;

    using namespace stair_perception;
    std::shared_ptr<Node> pnext;
    stair.reset();

    while (stair.readNext(pnext)) {
      if (pnext->pcurrent_type == step_node) {
        std::shared_ptr<Node> &pstep = pnext;
        height_now = pstep->step.height;
        depth_now = pstep->step.depth;
        hd_now = pstep->step.line.h;
        vd_now = pstep->step.line.d;

        if (!pstep->step.good_h) return;

        break;
      } else if (pnext->pcurrent_type == concaveline_node) {
        std::shared_ptr<Node> &pline = pnext;
      }
    }

    vec_time.push_back(ctime);
    vec_height.push_back(height_now);
    vec_depth.push_back(depth_now);

    if (count == 1) {
      height = height_now;
      depth = depth_now;
      hd = hd_now;
      vd = vd_now;
      ws = 1;
    } else {
      int start_idx = 0;
      for (auto i = 0; i < vec_time.size() - 1; i++) {
        if (fabs(fabs(ctime.toNSec() - vec_time[i].toNSec()) -
                 time_window * 1000000000ull) < 0.05) {
          start_idx = i;
          break;
        }
      }
      int bak_start_index = start_idx;

      if (start_idx == 0) start_idx = vec_time.size() - 1 - ws;

      ws = vec_time.size() - 1 - start_idx;

      double sum_h = 0, sum_d = 0, cnt = 0;
      for (auto i = start_idx; i < vec_time.size(); i++) {
        if (fabs(height - vec_height[i]) > 0.04 ||
            fabs(depth - vec_depth[i]) > 0.04)
          continue;
        cnt += 1;
        sum_h += vec_height[i];
        sum_d += vec_depth[i];
      }

      if (cnt) {
        height = sum_h / cnt;
        depth = sum_d / cnt;
      }

      if (fabs(height - height_now) < 0.04 && fabs(depth - depth_now) < 0.04) {
        hd = hd_now;
        vd = vd_now;
      }

      // remove outdated data
      if (bak_start_index > 0) {
        vec_time.erase(vec_time.begin(), vec_time.begin() + bak_start_index);
        vec_height.erase(vec_height.begin(),
                         vec_height.begin() + bak_start_index);
        vec_depth.erase(vec_depth.begin(), vec_depth.begin() + bak_start_index);
      }
    }

    output_record << ctime << "," << height << "," << depth << "," << hd << ","
                  << vd << endl;
    stair_info_msg::stair_info msg;
    msg.has_stair = has_stair;
    msg.height = height;
    msg.depth = depth;
    msg.v_height = hd;
    msg.v_depth = vd;

    pub_stair_info.publish(msg);
  }

  void publish_rviz_stair(
      const std::vector<stair_perception::Plane> &vsp_plane_viz, ros::Time t) {
    jsk_recognition_msgs::PolygonArray array_msg;
    array_msg.header.stamp = t;
    array_msg.header.frame_id = "t265_odom_frame";

    if (!vsp_plane_viz.empty()) {
      for (unsigned int i = 0; i < vsp_plane_viz.size(); i++) {
        if ((vsp_plane_viz)[i].ptype ==
            stair_perception::Plane::Ptype::stair_component) {
          geometry_msgs::PolygonStamped polygon_stamped;
          polygon_stamped.header.stamp = t;
          polygon_stamped.header.frame_id = "t265_odom_frame";
          if (vsp_plane_viz[i].counter.points.size() == 4) {
            for (unsigned int j = 0; j < 4; j++) {
              geometry_msgs::Point32 point32;
              point32.x = vsp_plane_viz[i].counter.points[j].x;
              point32.y = vsp_plane_viz[i].counter.points[j].y;
              point32.z = vsp_plane_viz[i].counter.points[j].z;
              polygon_stamped.polygon.points.push_back(point32);
            }
          }
          array_msg.polygons.push_back(polygon_stamped);
        } else if ((vsp_plane_viz)[i].ptype ==
                   stair_perception::Plane::Ptype::others) {
        } else if ((vsp_plane_viz)[i].ptype ==
                   stair_perception::Plane::Ptype::ground) {
        } else {
          // pstair_component
        }
      }
    }
    pub_stair_poly.publish(array_msg);
  }

 private:
  int count;

  double ws;  // window size
  double time_window;
  std::vector<ros::Time> vec_time;
  std::vector<double> vec_height, vec_depth;
  double height, depth, hd, vd;
  std::ofstream time_record, stair_record, output_record;

  /*********** ros related ***********/
  uint32_t queueSize;
  ros::NodeHandle private_nh;
  std::string topicVecPlane;
  std::string topicOdom;

  typedef message_filters::sync_policies::ApproximateTime<plane_msg::VecPlane,
                                                          nav_msgs::Odometry>
      SyncPolicyPlaneOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyPlaneOdom>>
      SynchronizerPlaneOdom;
  shared_ptr<message_filters::Subscriber<plane_msg::VecPlane>> sub_vec_plane;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_t265_odom;
  SynchronizerPlaneOdom sync_plane_odom;

  ros::Publisher pub_stair_info, pub_stair_poly;

  std::vector<stair_perception::Plane> vsp_plane;

  // dynamic paramter configure
  dynamic_reconfigure::Server<stair_modeling::stair_modeling_paramConfig>
      *server;
  dynamic_reconfigure::Server<
      stair_modeling::stair_modeling_paramConfig>::CallbackType f;

  /*********** stair modeling ***********/
  stair_perception::StairDetection stair_detector;
  stair_perception::KeyInfo keyInfo;
  bool has_stair;

  std::string detial_stair_model, est_stair_param, running_time_str;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "stair_modeling");
  ros::NodeHandle private_nh("~");

  StairModeling stairModeling(private_nh, "/demo_peac/vecPlane",
                              "/t265/odom/sample");
  stairModeling.init();

  ros::spin();
  ros::shutdown();
  return 0;
}
