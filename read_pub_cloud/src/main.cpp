//
// Created by zxm on 18-8-8.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/common.h>

#include <string>
#include <ostream>

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

class Reader
{
public:
    Reader(std::string file_path):
            file_path(file_path),count(0),read_file_count(0),ok(false)
    {
        if (access(file_path.c_str(), 0) == -1)
        {
            ROS_ERROR("Can't access : %s", file_path.c_str());
            ok = false;
        } else
        {
            count = 0;
            // get number of files
            DIR *dir;
            struct dirent *ptr;
            dir = opendir(file_path.c_str());
            while ((ptr = readdir(dir)) != nullptr)
                count++;
            closedir(dir);

            ROS_INFO("Find %d files in %s", count - 2, file_path.c_str());
        }
        read_file_count = 0;

        ok = true;
    }

    void next_one(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
    {
        if(ok)
        {
            read_file_count++;
            if (read_file_count >= (count - 2) / 4)
                read_file_count = 0;
            read_one(cloud);
        }
    }

    void prev_one(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
    {
        if(ok)
        {
            read_file_count--;
            if (read_file_count < 0)
                read_file_count = (count - 2) / 4 - 1;
            read_one(cloud);
        }
    }

    void read_one(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
    {
        ostringstream oss;
        oss << file_path << std::setfill('0') << "/" << std::setw(4) << read_file_count;
        const std::string cloudName = oss.str() + "_cloud.pcd";

        ROS_INFO("Read file : %s", cloudName.c_str());

        if (pcl::io::loadPCDFile(cloudName, cloud) == -1)
        {
            ROS_ERROR("Can not able to open file : %s", cloudName.c_str());
            return;
        }

        remove_NAN(cloud);
    }

    void remove_NAN(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
    {
        size_t height = cloud.height;
        size_t width = cloud.width;
        for(size_t i = 0; i < height; i++)
        {
            for(size_t j = 0; j < width; j++)
            {
                if(cloud.points[i*width + j].r == 0 ||
                   cloud.points[i*width + j].g == 0 ||
                   cloud.points[i*width + j].b == 0)
                {
                    cloud.points[i*width + j].x = std::numeric_limits<float>::quiet_NaN();
                    cloud.points[i*width + j].y = std::numeric_limits<float>::quiet_NaN();
                    cloud.points[i*width + j].z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }

    bool is_ok(){return ok;}

private:
    int count,read_file_count;
    std::string file_path;
    bool ok;
};

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_pub_cloud");
    ros::NodeHandle private_nh("~");

    std::string file_path;

    if(argc == 2)
        file_path = argv[1];
    else{
        private_nh.getParam("file_path", file_path);
    }

    if(file_path.empty())
    {
        ROS_INFO("No file path");
        return 0;
    }

    Reader reader(file_path.c_str());

    if(!reader.is_ok())
        return 0;

    ros::Publisher cloud_pub = private_nh.advertise<sensor_msgs::PointCloud2>("reader_cloud", 2);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        int c = getch();

        if(c != -1)
        {
            // <-
            if(c == 68)
            {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::PointCloud<pcl::PointXYZRGBA> cloud_pcl;
                reader.prev_one(cloud_pcl);
                pcl::toROSMsg(cloud_pcl,cloud_msg);

                cloud_msg.header.frame_id = "original";

                cloud_pub.publish(cloud_msg);
            }
            // ->
            else if(c == 67)
            {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::PointCloud<pcl::PointXYZRGBA> cloud_pcl;
                reader.next_one(cloud_pcl);
                pcl::toROSMsg(cloud_pcl,cloud_msg);

                cloud_msg.header.frame_id = "original";

                cloud_pub.publish(cloud_msg);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}