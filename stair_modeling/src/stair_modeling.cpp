//
// Created by zxm on 18-8-8.
//
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include "plane_msg/Plane.h"
#include "plane_msg/VecPlane.h"
#include "stair_modeling/StairPerception.hpp"

#include "stair_info_msg/stair_info.h"

#include <thread>
#include <mutex>
#include <chrono>

#include <stair_modeling/stair_modeling_paramConfig.h>

#include "stair_modeling/viewer_custom_func.h"

using namespace std;
using namespace std::chrono;

#define VIEWER_INFO_SHOW

class StairModeling
{
public:
    explicit StairModeling(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
                           const std::string &topicVecPlane = "vec_planes") :
            private_nh(private_nh), queueSize(5), running(true), count(0),
            topicVecPlane(topicVecPlane), has_stair(false),
            height(0), depth(0), hd(0), vd(0), time_window(2),
            show_key_directions(true), show_cloud(true), show_counter(true),
            show_downsampled_cloud(false), show_plane_info(true), show_center(true),
            show_detial_model(false), show_est_param(false), polygonShowMode(polygon_line_mode),
            server(new dynamic_reconfigure::Server<stair_modeling::stair_modeling_paramConfig>(private_nh)) {}

    ~StairModeling()
    {
        running = false;
        viz_thread.join();
    }

    void init()
    {
        ROS_INFO("############# StairModeling start #############");

        sub = private_nh.subscribe(topicVecPlane, queueSize, &StairModeling::callback, this);

        f = boost::bind(&StairModeling::cfg_callback, this, _1, _2);
        server->setCallback(f);

        pub_stair_info = private_nh.advertise<stair_info_msg::stair_info>("stair_info", 5);

        time_record.open("modeling_time_record.csv");
        time_record << "TIME_STAMP" << "," << "pre-process" << "," << "pre-modeling" << "," << "stair-modeling"
                    << endl;

        stair_record.open("modeling_result_record.csv");
        stair_record << "TIME_STAMP" << "," << "height" << "," << "depth" << "," << "h_dis" << "," << "v_dis"
                     << endl;

        output_record.open("modeling_output_record.csv");
        output_record << "TIME_STAMP" << "," << "height" << "," << "depth" << "," << "h_dis" << "," << "v_dis"
                      << endl;

        viz_thread = thread(&StairModeling::rviz_loop, this);
    }

    void cfg_callback(stair_modeling::stair_modeling_paramConfig &config, uint32_t level)
    {
//        ROS_INFO("cfg_callback");
        stair_detector.setDown_sample_points_number(config.down_sample_points_number);
        stair_detector.setAngle_h(pcl::deg2rad(config.angle_h));
        stair_detector.setAngle_v(pcl::deg2rad(config.angle_v));
        stair_detector.setGround_height_th(config.ground_height_th);
        stair_detector.setMin_ground_plane_points_num(config.min_ground_plane_points_num);
        stair_detector.setMerge_dis_th(config.merge_dis_th);
        stair_detector.setMerge_angle_th(pcl::deg2rad(config.merge_angle_th));
        stair_detector.setMerge_distance_th(config.merge_distance_th);
        stair_detector.setA1D_th(config.a1D_th);
        stair_detector.setDecay_rate(config.decay_rate);
        stair_detector.setVertical_plane_normal_angle_th(pcl::deg2rad(config.vertical_plane_normal_angle_th));
        stair_detector.setVertical_plane_normal_est_angle_th(pcl::deg2rad(config.vertical_plane_normal_est_angle_th));
        stair_detector.setCv_angle_horizontal_th(pcl::deg2rad(config.cv_angle_horizontal_th));
        stair_detector.setCv_dir_cluster_angle_th(pcl::deg2rad(config.cv_dir_cluster_angle_th));
        stair_detector.setPlane_max_length(config.plane_max_length);
        stair_detector.setPlane_min_length(config.plane_min_length);
        stair_detector.setPlane_max_width(config.plane_max_width);
        stair_detector.setPlane_min_width(config.plane_min_width);
        stair_detector.setStair_max_height(config.stair_max_height);
        stair_detector.setStair_max_width(config.stair_max_width);
        stair_detector.setStair_plane_max_distance(config.stair_plane_max_distance);
        stair_detector.setStair_cv_angle_diff_th(pcl::deg2rad(config.stair_cv_angle_diff_th));

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

    void callback(const plane_msg::VecPlane::ConstPtr &pvec_plane)
    {
        std::vector<double> vectime;

        count++;
        stair_perception::Stair stair;
        if (!pvec_plane->vecPlane.empty())
        {
            std::ostringstream oss;
            oss.precision(3);
            vsp_plane.clear();

            auto t1 = system_clock::now();

//            if(pvec_plane->header.stamp.sec==1538726795 &&pvec_plane->header.stamp.nsec==574559773)
//            {
//                // error frame?
//                has_stair = false;
//                return;
//            }

            convertCloud(*pvec_plane, vsp_plane);

            auto t2 = system_clock::now();
            auto duration = duration_cast<microseconds>(t2 - t1);
            ROS_INFO("Convert cloud time: %.2f ms", duration.count() / 1000.0);
            oss << "Convert cloud time: " /*<< setprecision(3)*/ << duration.count() / 1000.0 << " ms" << std::endl;

            has_stair = stair_detector.process(vsp_plane, stair, keyInfo, vectime);

            auto t3 = system_clock::now();
            duration = duration_cast<microseconds>(t3 - t2);
            ROS_INFO("Stair modeling time: %.2f ms", float(duration.count()) / 1000.0);
            oss << "Stair modeling time: " /*<< setprecision(3)*/ << duration.count() / 1000.0 << " ms" << std::endl;

            if (has_stair)
            {
                detial_stair_model = stair_detector.getDetialStairModelString(stair);
                est_stair_param = stair_detector.getEstimatedParamString(stair);

                stair_record << pvec_plane->vecPlane[0].cloud.header.stamp << ",";
                stair_record << stair_detector.getEstimatedParamStringRcd(stair);

                send2robot(stair,pvec_plane->vecPlane[0].cloud.header.stamp);
            } else
            {
                detial_stair_model = "No stair";
                est_stair_param = "No stair";
            }

            time_record << pvec_plane->vecPlane[0].cloud.header.stamp << ",";
            for (auto &t : vectime)
                time_record << t << ",";
            time_record << std::endl;

            running_time_str = oss.str();

            update_rviz_cloud(vsp_plane);
        }
    }

    void convertCloud(const plane_msg::VecPlane &vec_plane, std::vector<stair_perception::Plane> &vsp_plane)
    {
        unsigned long plane_number = vec_plane.vecPlane.size();
        vsp_plane.resize(plane_number);

        omp_set_num_threads(4);
#pragma omp parallel for
        for (size_t i = 0; i < plane_number; i++)
        {
            pcl::fromROSMsg(vec_plane.vecPlane[i].cloud, vsp_plane[i].cloud);

            vsp_plane[i].computeStatistics();
            vsp_plane[i].computePlaneInfo();
        }
    }

    void rviz_loop()
    {
        viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
        viewer->setPosition(0, 0);
        viewer->initCameraParameters();
        viewer->setBackgroundColor(1, 1, 1);
        viewer->addCoordinateSystem(0.3);
        viewer->setCameraPosition(-0.596784, -2.03596, 2.79617, -0.949447, 0.215143, -0.228612);
        viewer->setPosition(100,100);
        viewer->setSize(780,540);

        while (running)
        {
//            ROS_INFO("rviz_loop in");

            std::string viewr_cloud_name = "cloud_";

            mutex_vsp.lock();
            if (!vsp_plane_viz.empty())
            {
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                srand(0);
                for (unsigned int i = 0; i < vsp_plane_viz.size(); i++)
                {
                    if ((vsp_plane_viz)[i].ptype == stair_perception::Plane::Ptype::stair_component)
                    {
                        double r, g, b;
                        r = int(255.0 * rand() / (RAND_MAX + 1.0));
                        g = int(255.0 * rand() / (RAND_MAX + 1.0));
                        b = int(255.0 * rand() / (RAND_MAX + 1.0));
                        pcl::visualization::PointCloudColorHandlerCustom<stair_perception::PointType>
                                single_color((vsp_plane_viz)[i].cloud.makeShared(), r, g, b);

                        std::stringstream ss;

                        ss << viewr_cloud_name << i;

                        if (show_stair_cloud)
                        {
                            if (vsp_plane_viz[i].cloud.points.size())
                            {
                                // add cloud
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_downsampled_cloud)
                        {
                            if (vsp_plane_viz[i].random_down_sample_cloud.points.size())
                            {
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].random_down_sample_cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_center)
                        {
                            // add center
                            ss << "c";
                            viewer->addSphere((vsp_plane_viz)[i].center, 0.02, r / 255.0, g / 255.0, b / 255.0,
                                              ss.str());
                        }

                        if (show_plane_info)
                        {
                            // add plane number (TODO: just for debug)
                            ss << "c";
                            std::stringstream nss;
                            nss << "P" << i;
                            if (vsp_plane_viz[i].type == stair_perception::Plane::Type::horizontal)
                                nss << "-H";
                            else if (vsp_plane_viz[i].type == stair_perception::Plane::Type::vertical)
                                nss << "-V";
                            else
                                nss << "-S";
                            nss << "-Stair";
                            viewer->addText3D(nss.str(), vsp_plane_viz[i].center, 0.02, 0, 0, 0, ss.str());

                            // add plane normal (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ start, end;
                            start = (vsp_plane_viz)[i].center;
                            end.x = (vsp_plane_viz)[i].center.x + 0.1 * (vsp_plane_viz)[i].coefficients.values[0];
                            end.y = (vsp_plane_viz)[i].center.y + 0.1 * (vsp_plane_viz)[i].coefficients.values[1];
                            end.z = (vsp_plane_viz)[i].center.z + 0.1 * (vsp_plane_viz)[i].coefficients.values[2];
                            viewer->addLine<pcl::PointXYZ>(start, end, r / 255.0, g / 255.0, b / 255.0, ss.str());

                            // add plane eigen vectors (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ center, p1;
                            center = vsp_plane_viz[i].center;
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].x * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].y * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].z * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                            ss << "c";
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].x * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].y * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].z * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                        }

                        if (show_counter)
                        {
                            // add counter
                            ss << "c";
//                            viewer->addPolygon<pcl::PointXYZ>((vsp_plane_viz)[i].counter.makeShared(), r / 255.0,
//                                                              g / 255.0,
//                                                              b / 255.0, ss.str());

                            myaddPolygon(*viewer, vsp_plane_viz[i].counter.makeShared(), polygonShowMode,
                                         r / 255.0, g / 255.0, b / 255.0, ss.str());
                        }

                    } else if ((vsp_plane_viz)[i].ptype == stair_perception::Plane::Ptype::others)
                    {
                        pcl::visualization::PointCloudColorHandlerCustom<stair_perception::PointType>
                                single_color((vsp_plane_viz)[i].cloud.makeShared(), 255, 0, 0);

                        std::stringstream ss;

                        ss << viewr_cloud_name << i;

                        if (show_cloud)
                        {
                            if (vsp_plane_viz[i].cloud.points.size())
                            {
                                // add cloud
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_downsampled_cloud)
                        {
                            if (vsp_plane_viz[i].random_down_sample_cloud.points.size())
                            {
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].random_down_sample_cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_plane_info)
                        {
                            // add plane number (TODO: just for debug)
                            ss << "c";
                            std::stringstream nss;
                            nss << "P" << i;
                            if (vsp_plane_viz[i].type == stair_perception::Plane::Type::horizontal)
                                nss << "-H";
                            else if (vsp_plane_viz[i].type == stair_perception::Plane::Type::vertical)
                                nss << "-V";
                            else
                                nss << "-S";
                            nss << "-Others-" << vsp_plane_viz[i].info;
                            viewer->addText3D(nss.str(), vsp_plane_viz[i].center, 0.02, 0, 0, 0, ss.str());

                            // add plane normal (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ start, end;
                            start = (vsp_plane_viz)[i].center;
                            end.x = (vsp_plane_viz)[i].center.x + 0.1 * (vsp_plane_viz)[i].coefficients.values[0];
                            end.y = (vsp_plane_viz)[i].center.y + 0.1 * (vsp_plane_viz)[i].coefficients.values[1];
                            end.z = (vsp_plane_viz)[i].center.z + 0.1 * (vsp_plane_viz)[i].coefficients.values[2];
                            viewer->addLine<pcl::PointXYZ>(start, end, 1, 0, 0, ss.str());

                            // add plane eigen vectors (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ center, p1;
                            center = vsp_plane_viz[i].center;
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].x * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].y * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].z * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                            ss << "c";
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].x * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].y * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].z * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                        }

                    } else if ((vsp_plane_viz)[i].ptype == stair_perception::Plane::Ptype::ground)
                    {
                        pcl::visualization::PointCloudColorHandlerCustom<stair_perception::PointType>
                                single_color((vsp_plane_viz)[i].cloud.makeShared(), 0, 255, 0);

                        std::stringstream ss;

                        ss << viewr_cloud_name << i;

                        if (show_cloud)
                        {
                            if (vsp_plane_viz[i].cloud.points.size())
                            {
                                // add cloud
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_downsampled_cloud)
                        {
                            if (vsp_plane_viz[i].random_down_sample_cloud.points.size())
                            {
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].random_down_sample_cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_plane_info)
                        {
                            // add plane number (TODO: just for debug)
                            ss << "c";
                            std::stringstream nss;
                            nss << "P" << i;
                            if (vsp_plane_viz[i].type == stair_perception::Plane::Type::horizontal)
                                nss << "-H";
                            else if (vsp_plane_viz[i].type == stair_perception::Plane::Type::vertical)
                                nss << "-V";
                            else
                                nss << "-S";
                            nss << "-Ground";
                            viewer->addText3D(nss.str(), vsp_plane_viz[i].center, 0.02, 0, 0, 0, ss.str());

                            // add plane normal (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ start, end;
                            start = (vsp_plane_viz)[i].center;
                            end.x = (vsp_plane_viz)[i].center.x + 0.1 * (vsp_plane_viz)[i].coefficients.values[0];
                            end.y = (vsp_plane_viz)[i].center.y + 0.1 * (vsp_plane_viz)[i].coefficients.values[1];
                            end.z = (vsp_plane_viz)[i].center.z + 0.1 * (vsp_plane_viz)[i].coefficients.values[2];
                            viewer->addLine<pcl::PointXYZ>(start, end, 0, 1, 0, ss.str());

                            // add plane eigen vectors (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ center, p1;
                            center = vsp_plane_viz[i].center;
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].x * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].y * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].z * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                            ss << "c";
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].x * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].y * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].z * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                        }
                    } else
                    {
                        double r, g, b;
                        r = int(200.0 * rand() / (RAND_MAX + 1.0));
                        g = int(200.0 * rand() / (RAND_MAX + 1.0));
                        b = int(200.0 * rand() / (RAND_MAX + 1.0));
                        pcl::visualization::PointCloudColorHandlerCustom<stair_perception::PointType>
                                single_color((vsp_plane_viz)[i].cloud.makeShared(), r, g, b);

                        std::stringstream ss;

                        ss << viewr_cloud_name << i;

                        if (show_cloud)
                        {
                            if (vsp_plane_viz[i].cloud.points.size())
                            {
                                // add cloud
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_downsampled_cloud)
                        {
                            if (vsp_plane_viz[i].random_down_sample_cloud.points.size())
                            {
                                ss << "c";
                                viewer->addPointCloud<stair_perception::PointType>(
                                        (vsp_plane_viz)[i].random_down_sample_cloud.makeShared(),
                                        single_color, ss.str());
                                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                         3,
                                                                         ss.str());
                            }
                        }

                        if (show_center)
                        {
                            // add center
                            ss << "c";
                            viewer->addSphere((vsp_plane_viz)[i].center, 0.02, r / 255.0, g / 255.0, b / 255.0,
                                              ss.str());
                        }


                        if (show_counter)
                        {
                            // add counter
                            ss << "c";
//                            viewer->addPolygon<pcl::PointXYZ>((vsp_plane_viz)[i].counter.makeShared(),
//                                                              r / 255.0, g / 255.0, b / 255.0, ss.str());

                            myaddPolygon(*viewer, vsp_plane_viz[i].counter.makeShared(), polygonShowMode,
                                         r / 255.0, g / 255.0, b / 255.0, ss.str());
                        }

                        if (show_plane_info)
                        {
                            // add plane number (TODO: just for debug)
                            ss << "c";
                            std::stringstream nss;
                            nss << "P" << i;
                            if (vsp_plane_viz[i].type == stair_perception::Plane::Type::horizontal)
                                nss << "-H";
                            else if (vsp_plane_viz[i].type == stair_perception::Plane::Type::vertical)
                                nss << "-V";
                            else
                                nss << "-S";
                            nss << "-pStair";
                            viewer->addText3D(nss.str(), vsp_plane_viz[i].center, 0.02, 0, 0, 0, ss.str());
                            // add plane eigen vectors (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ center, p1;
                            center = vsp_plane_viz[i].center;
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].x * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].y * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[1].z * sqrtf(vsp_plane_viz[i].eigen_values[1]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());
                            ss << "c";
                            p1.x = center.x +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].x * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.y = center.y +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].y * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            p1.z = center.z +
                                   0.01 * vsp_plane_viz[i].eigen_vectors[2].z * sqrtf(vsp_plane_viz[i].eigen_values[2]);
                            viewer->addLine<pcl::PointXYZ>(center, p1, 0, 0, 0, ss.str());

                            // add plane normal (TODO: just for debug)
                            ss << "c";
                            pcl::PointXYZ start, end;
                            start = (vsp_plane_viz)[i].center;
                            end.x = (vsp_plane_viz)[i].center.x + 0.1 * (vsp_plane_viz)[i].coefficients.values[0];
                            end.y = (vsp_plane_viz)[i].center.y + 0.1 * (vsp_plane_viz)[i].coefficients.values[1];
                            end.z = (vsp_plane_viz)[i].center.z + 0.1 * (vsp_plane_viz)[i].coefficients.values[2];
                            viewer->addLine<pcl::PointXYZ>(start, end, r / 255.0, g / 255.0, b / 255.0, ss.str());
                        }
                    }
                }

                if (!viz_key_info.init)
                {
                    // add stair key directions (TODO: just for debug)

                    if (show_key_directions)
                    {
                        std::stringstream ss;
                        ss << "normal_";
                        pcl::PointXYZ start, vertical_plane_normal, horizontal_plane_direction, main_center_diff_vector, slide_direction;

                        start = pcl::PointXYZ(1, 0, 0.8);
                        ss << "c";
                        viewer->addSphere(start, 0.015, 0, 0, 0, ss.str());

                        vertical_plane_normal.x = start.x + 0.2 * viz_key_info.main_vertical_plane_normal.normal[0];
                        vertical_plane_normal.y = start.y + 0.2 * viz_key_info.main_vertical_plane_normal.normal[1];
                        vertical_plane_normal.z = start.z + 0.2 * viz_key_info.main_vertical_plane_normal.normal[2];
                        ss << "vertical_plane_normal";
                        viewer->addLine<pcl::PointXYZ>(start, vertical_plane_normal, 255, 0, 0, ss.str());
                        ss << "c";
                        viewer->addText3D("vertical_plane_normal", vertical_plane_normal, 0.02, 0, 0, 0, ss.str());


                        horizontal_plane_direction.x =
                                start.x + 0.2 * viz_key_info.horizontal_plane_direction.normal[0];
                        horizontal_plane_direction.y =
                                start.y + 0.2 * viz_key_info.horizontal_plane_direction.normal[1];
                        horizontal_plane_direction.z =
                                start.z + 0.2 * viz_key_info.horizontal_plane_direction.normal[2];
                        ss << "horizontal_plane_direction";
                        viewer->addLine<pcl::PointXYZ>(start, horizontal_plane_direction, 255, 0, 0, ss.str());
                        ss << "c";
                        viewer->addText3D("horizontal_plane_direction", horizontal_plane_direction, 0.02, 0, 0, 0,
                                          ss.str());

                        main_center_diff_vector.x =
                                start.x + 0.2 * viz_key_info.main_center_diff_vector.normal[0];
                        main_center_diff_vector.y =
                                start.y + 0.2 * viz_key_info.main_center_diff_vector.normal[1];
                        main_center_diff_vector.z =
                                start.z + 0.2 * viz_key_info.main_center_diff_vector.normal[2];
                        ss << "main_center_diff_vector";
                        viewer->addLine<pcl::PointXYZ>(start, main_center_diff_vector, 255, 0, 0, ss.str());
                        ss << "c";
                        viewer->addText3D("main_center_diff_vector", main_center_diff_vector,
                                          0.02, 0, 0, 0, ss.str());
                    }
                }

                pcl::visualization::Camera camera{};
                viewer->getCameraParameters(camera);

                if (show_run_time)
                {
                    myaddText(*viewer, running_time_str, up_right,
                              camera.window_size[0] - 10, camera.window_size[1] - 10, 12, 0, 0, 0, "running_time_str");
                }

                if (has_stair)
                {
                    std::stringstream ss;
                    ss << "model_";

                    if (show_detial_model)
                    {
                        ss << "c";
//                        viewer->addText(detial_stair_model,0,camera.window_size[1]/2,12,0,0,0,ss.str());
                        myaddText(*viewer, detial_stair_model, up_left, 10, camera.window_size[1] - 10, 12, 0, 0, 0,
                                  ss.str());
                    }

                    if (show_est_param)
                    {
                        ss << "c";
//                        viewer->addText(est_stair_param,0,0,12,0,0,0,ss.str());
                        myaddText(*viewer, est_stair_param, down_left, 10, 10, 12, 0, 0, 0, ss.str());
                    }
                }
            }
//            ROS_INFO("rviz_loop out");
            viewer->spinOnce(2);
            mutex_vsp.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void update_rviz_cloud(std::vector<stair_perception::Plane> &vsp_plane)
    {
        mutex_vsp.lock();
        vsp_plane_viz = vsp_plane;
        viz_key_info = keyInfo;
        mutex_vsp.unlock();
    }

    void send2robot(stair_perception::Stair &stair, ros::Time ctime)
    {
        double height_now, depth_now, hd_now, vd_now;

        using namespace stair_perception;
        std::shared_ptr<Node> pnext;
        stair.reset();

        while (stair.readNext(pnext))
        {
            if (pnext->pcurrent_type == step_node)
            {
                std::shared_ptr<Node> &pstep = pnext;
                height_now = pstep->step.height;
                depth_now = pstep->step.depth;
                hd_now = pstep->step.line.h;
                vd_now = pstep->step.line.d;

                if(!pstep->step.good_h)
                    return;

                break;
            } else if (pnext->pcurrent_type == concaveline_node)
            {
                std::shared_ptr<Node> &pline = pnext;
            }
        }

        vec_time.push_back(ctime);
        vec_height.push_back(height_now);
        vec_depth.push_back(depth_now);

        if (count == 1)
        {
            height = height_now;
            depth = depth_now;
            hd = hd_now;
            vd = vd_now;
            ws = 1;
        } else
        {
            int start_idx = 0;
            for (auto i = 0; i < vec_time.size() - 1; i++)
            {
                if (fabs(fabs(ctime.toNSec() - vec_time[i].toNSec()) - time_window * 1000000000ull) < 0.05)
                {
                    start_idx = i;
                    break;
                }
            }
            int bak_start_index = start_idx;

            if (start_idx == 0)
                start_idx = vec_time.size() - 1 - ws;

            ws = vec_time.size() - 1 - start_idx;

            double sum_h = 0, sum_d = 0, cnt = 0;
            for (auto i = start_idx; i < vec_time.size(); i++)
            {
                if (fabs(height - vec_height[i]) > 0.04 || fabs(depth - vec_depth[i]) > 0.04)
                    continue;
                cnt += 1;
                sum_h += vec_height[i];
                sum_d += vec_depth[i];
            }

            if (cnt)
            {
                height = sum_h / cnt;
                depth = sum_d / cnt;
            }

            if (fabs(height - height_now) < 0.04 && fabs(depth - depth_now) < 0.04)
            {
                hd = hd_now;
                vd = vd_now;
            }

            // remove outdated data
            if (bak_start_index > 0)
            {
                vec_time.erase(vec_time.begin(), vec_time.begin() + bak_start_index);
                vec_height.erase(vec_height.begin(), vec_height.begin() + bak_start_index);
                vec_depth.erase(vec_depth.begin(), vec_depth.begin() + bak_start_index);
            }
        }

        output_record << ctime << "," << height << "," << depth << "," << hd << "," << vd << endl;
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

    double ws;// window size
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
    ros::Publisher pub_cloud;

    std::vector<stair_perception::Plane> vsp_plane;

    // dynamic paramter configure
    dynamic_reconfigure::Server<stair_modeling::stair_modeling_paramConfig> *server;
    dynamic_reconfigure::Server<stair_modeling::stair_modeling_paramConfig>::CallbackType f;

    /*********** stair modeling ***********/
    stair_perception::StairDetection stair_detector;
    stair_perception::KeyInfo keyInfo;
    bool has_stair;

    std::string detial_stair_model, est_stair_param, running_time_str;

    /*********** PCLVisualizer ***********/
    thread viz_thread;
    mutex mutex_vsp;
    bool running;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    std::vector<stair_perception::Plane> vsp_plane_viz;
    stair_perception::KeyInfo viz_key_info;
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

class StairModelingNodelet : public nodelet::Nodelet
{
public:
    StairModelingNodelet() : Nodelet(), pstair_modeling(nullptr) {}

    ~StairModelingNodelet() override { delete pstair_modeling; }

private:
    void onInit() override
    {
        pstair_modeling = new StairModeling(getPrivateNodeHandle());
        pstair_modeling->init();
    }

    StairModeling *pstair_modeling;
};

PLUGINLIB_DECLARE_CLASS(StairModeling, StairModelingNodelet, StairModelingNodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_modeling");
    ros::NodeHandle private_nh("~");

    StairModeling stairModeling(private_nh, "vec_planes");

    stairModeling.init();

    ros::spin();

    ros::shutdown();
    return 0;
}
