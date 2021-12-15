#include <dynamic_reconfigure/server.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stair_modeling/stair_modeling_paramConfig.h>

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
      const std::string &topicVecPlane = "vec_planes")
      : private_nh(private_nh),
        queueSize(5),
        running(true),
        count(0),
        topicVecPlane(topicVecPlane),
        has_stair(false),
        height(0),
        depth(0),
        hd(0),
        vd(0),
        time_window(2),
        show_key_directions(true),
        show_cloud(true),
        show_counter(true),
        show_downsampled_cloud(false),
        show_plane_info(true),
        show_center(true),
        show_detial_model(false),
        show_est_param(false),
        polygonShowMode(polygon_line_mode),
        server(new dynamic_reconfigure::Server<
               stair_modeling::stair_modeling_paramConfig>(private_nh)) {}

  ~StairModeling() { running = false; }

  void init() {
    ROS_INFO("############# StairModeling start #############");

    sub = private_nh.subscribe(topicVecPlane, queueSize,
                               &StairModeling::callback, this);

    f = boost::bind(&StairModeling::cfg_callback, this, _1, _2);
    server->setCallback(f);

    pub_stair_info =
        private_nh.advertise<stair_info_msg::stair_info>("stair_info", 5);

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

    show_key_directions = config.show_key_directions;
    show_plane_info = config.show_plane_info;
    show_cloud = config.show_cloud;
    show_downsampled_cloud = config.show_downsampled_cloud;
    show_counter = config.show_counter;
    show_center = config.show_center;
    show_detial_model = config.show_detial_model;
    show_est_param = config.show_est_param;
    show_run_time = config.show_run_time;
    show_stair_cloud = config.show_stair_cloud;
    time_window = config.time_window;
    if (config.polygonShowMode == 0)
      polygonShowMode = polygon_line_mode;
    else
      polygonShowMode = polygon_surface_mode;
  }

  void callback(const plane_msg::VecPlane::ConstPtr &pvec_plane) {
    std::vector<double> vectime;

    count++;
    stair_perception::Stair stair;
    if (!pvec_plane->vecPlane.empty()) {
      std::ostringstream oss;
      oss.precision(3);
      vsp_plane.clear();

      auto t1 = system_clock::now();

      convertCloud(*pvec_plane, vsp_plane);

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
        detial_stair_model = stair_detector.getDetialStairModelString(stair);
        est_stair_param = stair_detector.getEstimatedParamString(stair);

        stair_record << pvec_plane->vecPlane[0].cloud.header.stamp << ",";
        stair_record << stair_detector.getEstimatedParamStringRcd(stair);

        send2robot(stair, pvec_plane->vecPlane[0].cloud.header.stamp);
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
                    std::vector<stair_perception::Plane> &vsp_plane) {
    unsigned long plane_number = vec_plane.vecPlane.size();
    cout << "plane_number: " << plane_number << endl;
    vsp_plane.resize(plane_number);

    // omp_set_num_threads(4);
    // #pragma omp parallel for
    for (size_t i = 0; i < plane_number; i++) {
      pcl::fromROSMsg(vec_plane.vecPlane[i].cloud, vsp_plane[i].cloud);

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
  ros::Subscriber sub;
  ros::Publisher pub_stair_info;

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

  /*********** PCLVisualizer ***********/
  bool running;
  bool show_key_directions;
  bool show_plane_info;
  bool show_cloud;
  bool show_downsampled_cloud;
  bool show_counter;
  bool show_center;
  bool show_detial_model;
  bool show_est_param;
  bool show_run_time;
  bool show_stair_cloud;
  PolygonShowMode polygonShowMode;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "stair_modeling");
  ros::NodeHandle private_nh("~");

  StairModeling stairModeling(private_nh, "/demo_peac/vecPlane");

  stairModeling.init();

  ros::spin();

  ros::shutdown();
  return 0;
}
