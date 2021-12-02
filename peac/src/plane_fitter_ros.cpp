//
// Created by zxm on 18-7-31.
//


#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <math.h> //fabs

#include <dynamic_reconfigure/server.h>
#include <peac/peac_paramConfig.h>
#include "plane_msg/Plane.h"
#include "plane_msg/VecPlane.h"

#include "AHCPlaneFitter.hpp"

#include <chrono>
#include <thread>
#include <mutex>

using namespace std;
using namespace std::chrono;

namespace plane_fitter
{

    template<class PointT>
    struct OrganizedImage3D
    {
        const pcl::PointCloud<PointT> &cloud;
        //note: ahc::PlaneFitter assumes mm as unit!!!
        const double unitScaleFactor;

        OrganizedImage3D(const pcl::PointCloud<PointT> &c) : cloud(c), unitScaleFactor(1) {}

        OrganizedImage3D(const OrganizedImage3D &other) : cloud(other.cloud), unitScaleFactor(other.unitScaleFactor) {}

        inline int width() const { return cloud.width; }

        inline int height() const { return cloud.height; }

        inline bool get(const int row, const int col, double &x, double &y, double &z) const
        {
            const PointT &pt = cloud.at(col, row);
            x = pt.x * unitScaleFactor;
            y = pt.y * unitScaleFactor;
            z = pt.z * unitScaleFactor; //TODO: will this slowdown the speed?
            return pcl_isnan(z) == 0; //return false if current depth is NaN
        }
    };

    typedef OrganizedImage3D<pcl::PointXYZRGBA> ImageXYZRGBA;
    typedef ahc::PlaneFitter<ImageXYZRGBA> PlaneFitter;
    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

    class peac_ros
    {
    public:
        peac_ros(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
                 const std::string topicPointCloud2 = "cloud_in",
                 const std::string topicAdImage = "seg_image",
                 const std::string topicVecPlane = "vec_planes") :
                queueSize(5), topicPointCloud2(topicPointCloud2), show_viewer(true),running(true),
                topicAdImage(topicAdImage), topicVecPlane(topicVecPlane),
                private_nh(private_nh), count(0), process_time(0),
                distance_min(50), distance_max(500),
                server(new dynamic_reconfigure::Server<peac::peac_paramConfig>(private_nh)) {}

        ~peac_ros()
        {
            running = false;
            if (show_viewer)
                viz_thread.join();
        }

        void start()
        {
            ROS_INFO("############# peac_ros start #############");

            sub = private_nh.subscribe(topicPointCloud2, queueSize, &peac_ros::callback, this);

            f = boost::bind(&peac_ros::cfg_callback, this, _1, _2);
            server->setCallback(f);

            pub_img = private_nh.advertise<sensor_msgs::Image>(topicAdImage, queueSize);
            pub_vec_planes = private_nh.advertise<plane_msg::VecPlane>(topicVecPlane, queueSize);

            private_nh.getParam("show_seg_viewer", show_viewer);

            ROS_INFO("show_seg_viewer: %d", show_viewer);

            time_record.open("peac_time_record.csv");
            time_record << "TIME_STAMP" << "," << "init" << "," << "cluster" << "," << "refine" << endl;

            if (show_viewer)
            {
                viz_thread = thread(&peac_ros::rviz_loop, this);
            }
        }

        void cfg_callback(peac::peac_paramConfig &config, uint32_t level)
        {
            distance_min = config.distance_min;
            distance_max = config.distance_max;

            //setup fitter
            pf.minSupport = config.minSupport;
            pf.windowWidth = config.windowWidth;
            pf.windowHeight = config.windowWidth;
            pf.doRefine = config.doRefine;

            if (config.initType == 0)
                pf.params.initType = ahc::INIT_STRICT;
            else
                pf.params.initType = ahc::INIT_LOOSE;

            //T_mse
            pf.params.stdTol_merge = config.stdTol_merge;
            pf.params.stdTol_init = config.stdTol_init;
            pf.params.depthSigma = config.depthSigma;

            //T_dz
            pf.params.depthAlpha = config.depthAlpha;
            pf.params.depthChangeTol = config.depthChangeTol;

            //T_ang
            pf.params.z_near = config.z_near;
            pf.params.z_far = config.z_far;
            pf.params.angle_near = MACRO_DEG2RAD(config.angle_near);
            pf.params.angle_far = MACRO_DEG2RAD(config.angle_far);
            pf.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(config.similarityTh_merge));
            pf.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(config.similarityTh_refine));

            ROS_INFO("param change");
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr cloud_ros)
        {
            std::vector<double> vectime;

            count++;

            pcl::fromROSMsg(*cloud_ros, cloud_in_pcl_xyzrgba);
            ClipCloud(cloud_in_pcl_xyzrgba);
            cv::Mat seg(cloud_in_pcl_xyzrgba.height, cloud_in_pcl_xyzrgba.width, CV_8UC3);
            ImageXYZRGBA Ixyzrgba(cloud_in_pcl_xyzrgba);
            std::vector<std::vector<int>> pMembership;

            auto t1 = system_clock::now();
            pf.run(&Ixyzrgba, vectime, &pMembership, &seg, 0, false);
            auto t2 = system_clock::now();
            auto duration = duration_cast<microseconds>(t2 - t1);
            process_time = duration.count();
            ROS_INFO("Average plane segmentation time: %.2f ms", process_time / 1000.0);

            // publish segmentation image
            sensor_msgs::Image msgImage;
            MatImg2RosImage(seg, msgImage, sensor_msgs::image_encodings::RGB8);
//            Ixyz2RosImage(Ixyz,msgImage,17);
            pub_img.publish(msgImage);

            // publish segmented planes
            plane_msg::VecPlane vecPlane;
            GenerateVecPlanes(pMembership, count, vecPlane, cloud_ros->header.stamp);
            pub_vec_planes.publish(vecPlane);

            mutex_vsp.lock();
            vsp_plane_viz = vecPlane;
            mutex_vsp.unlock();

            time_record << cloud_ros->header.stamp << ",";
            for (auto &t : vectime)
                time_record << t << ",";
            time_record << std::endl;
        }

        void MatImg2RosImage(const cv::Mat &cvmat, sensor_msgs::Image &rosimg, std::string encoding)
        {
            rosimg.encoding = encoding;
            int step = cvmat.cols * cvmat.elemSize();
            int size = cvmat.rows * step;
            rosimg.height = cvmat.rows;
            rosimg.width = cvmat.cols;
            rosimg.is_bigendian = false;
            rosimg.step = step;
            rosimg.data.resize(size);
            memcpy(rosimg.data.data(), cvmat.data, size);
        }

        void Ixyz2RosImage(const ImageXYZRGBA &Ixyzrgba, sensor_msgs::Image &rosimg, double scale)
        {
            rosimg.encoding = sensor_msgs::image_encodings::BGR8;

            rosimg.height = Ixyzrgba.height();
            rosimg.width = Ixyzrgba.width();
            rosimg.is_bigendian = false;
            rosimg.step = Ixyzrgba.width() * 3;
            rosimg.data.resize(rosimg.height * rosimg.step);

            /*double max=0;*/
            for (size_t i = 0; i < rosimg.height; i++)
            {
                for (size_t j = 0; j < rosimg.width; j++)
                {
                    double x, y, z, d;
                    Ixyzrgba.get(i, j, x, y, z);
                    if (!(std::isnan(x) || std::isnan(y) || std::isnan(z)))
                    {
                        d = sqrt(x * x + y * y + z * z);
                        rosimg.data.data()[rosimg.step * i + 3 * j] = uint8_t(d * scale);
                        rosimg.data.data()[rosimg.step * i + 3 * j + 1] = uint8_t(d * scale);
                        rosimg.data.data()[rosimg.step * i + 3 * j + 2] = uint8_t(d * scale);
                        /*if(d>max)max=d;*/
                    } else
                    {
                        rosimg.data.data()[rosimg.step * i + 3 * j] = 0;
                        rosimg.data.data()[rosimg.step * i + 3 * j + 1] = 0;
                        rosimg.data.data()[rosimg.step * i + 3 * j + 2] = 0;
                    }
                }
            }

            /*ROS_INFO("max distance: %f", max);*/
        }

        void GenerateVecPlanes(const std::vector<std::vector<int>> &pMembership,
                               const unsigned long count, plane_msg::VecPlane &vec_planes,
                               const ros::Time &stamp)
        {
            size_t plane_number = pMembership.size();
            vec_planes.vecPlane.resize(plane_number);

            vec_planes.header.seq = count;
            vec_planes.header.stamp = stamp;
            vec_planes.header.frame_id = "imu_pose";

            // plane information
            for (size_t i = 0; i < plane_number; i++)
            {
                // get the plane points from original cloud with pMembership
                pcl::PointCloud<pcl::PointXYZRGBA> pclcloud;
                uint32_t point_number = pMembership[i].size();
                for (size_t j = 0; j < point_number; j++)
                {
                    int index = pMembership[i][j];
                    pclcloud.points.push_back(cloud_in_pcl_xyzrgba.points[index]);
                }
                pclcloud.width = point_number;
                pclcloud.height = 1;

                pcl::toROSMsg(pclcloud, vec_planes.vecPlane[i].cloud);
                vec_planes.vecPlane[i].cloud.header = vec_planes.header;

                // we do not publish the statistics of plane here, only publish the plane points
                // the statistics will computed in the process node
            }
        }

        // cut cloud based on distance_min and distance_max
        void ClipCloud(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
        {
            size_t height = cloud.height;
            size_t width = cloud.width;
            float x, y, z, distance2;
            for (size_t i = 0; i < height; i++)
            {
                for (size_t j = 0; j < width; j++)
                {
                    x = cloud.points[i * width + j].x * 100;// m -> cm
                    y = cloud.points[i * width + j].y * 100;
                    z = cloud.points[i * width + j].z * 100;
                    distance2 = x * x + y * y + z * z;
                    if (distance2 < distance_min * distance_min || distance2 > distance_max * distance_max)
                    {
                        cloud.points[i * width + j].x = std::numeric_limits<float>::quiet_NaN();
                        cloud.points[i * width + j].y = std::numeric_limits<float>::quiet_NaN();
                        cloud.points[i * width + j].z = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }
        }

        void rviz_loop()
        {
            viewer.reset(new pcl::visualization::PCLVisualizer("seg_viewer"));
            viewer->setPosition(0, 0);
            viewer->initCameraParameters();
            viewer->setBackgroundColor(0.8, 0.8, 0.8);
            viewer->addCoordinateSystem(0.3);
            viewer->setCameraPosition(-0.596784, -2.03596, 2.79617, -0.949447, 0.215143, -0.228612);

            while (running)
            {
                std::string viewr_cloud_name = "cloud_";

                mutex_vsp.lock();
                if (!vsp_plane_viz.vecPlane.empty())
                {
                    viewer->removeAllPointClouds();
                    viewer->removeAllShapes();

                    srand(0);
                    for (unsigned int i = 0; i < vsp_plane_viz.vecPlane.size(); i++)
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA> pointCloud;
                        pcl::fromROSMsg(vsp_plane_viz.vecPlane[i].cloud,pointCloud);

                        double r, g, b;
                        r = int(255.0 * rand() / (RAND_MAX + 1.0));
                        g = int(255.0 * rand() / (RAND_MAX + 1.0));
                        b = int(255.0 * rand() / (RAND_MAX + 1.0));
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
                                single_color(pointCloud.makeShared(), r, g, b);

                        std::stringstream ss;

                        ss << viewr_cloud_name << i;

                        if (pointCloud.points.size())
                        {
                            // add cloud
                            ss << "c";
                            viewer->addPointCloud<pcl::PointXYZRGBA>(pointCloud.makeShared(), single_color,
                                                                     ss.str());
                            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                                     ss.str());
                        }
                    }
                }
                mutex_vsp.unlock();

                viewer->spinOnce(30);
            }
        }


        std::ofstream time_record;

        double distance_min, distance_max;

        int queueSize;

        ros::NodeHandle private_nh;

        std::string topicPointCloud2, topicAdImage, topicVecPlane;

        PlaneFitter pf;

        ros::Subscriber sub;
        ros::Publisher pub_img, pub_vec_planes;

        dynamic_reconfigure::Server<peac::peac_paramConfig> *server;
        dynamic_reconfigure::Server<peac::peac_paramConfig>::CallbackType f;

        tf::TransformBroadcaster broadcaster;
        tf::StampedTransform stVecPlane;

        pcl::PointCloud<pcl::PointXYZRGBA> cloud_in_pcl_xyzrgba;

        bool show_viewer;
        thread viz_thread;
        mutex mutex_vsp;
        bool running;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> vsp_plane_viz;
        plane_msg::VecPlane vsp_plane_viz;

        unsigned long count;
        unsigned long process_time;
    };

    class peac_ros_nodelet : public nodelet::Nodelet
    {
    private:
        peac_ros *ppeac_ros;

    public:
        peac_ros_nodelet() : Nodelet(), ppeac_ros(nullptr)
        {
        }

        ~peac_ros_nodelet()
        {
            delete ppeac_ros;
        }

    private:
        virtual void onInit()
        {
            ppeac_ros = new peac_ros(getPrivateNodeHandle());
            ppeac_ros->start();
        }
    };

    PLUGINLIB_DECLARE_CLASS(peac, peac_ros_nodelet, plane_fitter::peac_ros_nodelet, nodelet::Nodelet);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "peac_ros");
    ros::NodeHandle private_nh("~");

//    plane_fitter::peac_ros peacRos(private_nh,"/peac_cloud_rotation/cloud_xyzrgba");
    plane_fitter::peac_ros peacRos(private_nh, "/read_pub_cloud/reader_cloud");

    peacRos.start();

    ros::spin();

    ros::shutdown();
    return 0;
}
