/*
 * StairDetection.hpp
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef STAIRDETECTION_HPP_
#define STAIRDETECTION_HPP_

#include "../common.h"
#include <time.h>
#include "Stair.hpp"

namespace stair_perception
{
    struct PlaneNormalInfo
    {
        Eigen::Vector3f normal;
        int plane_points_numbers; // for main vertical plane normal
        float eigen_value; // for coarse side plane normal
        float weight;
        int count;

        PlaneNormalInfo() : normal(Eigen::Vector3f(0, 0, 0)), plane_points_numbers(0), eigen_value(0), weight(0) {}
    };

    struct PlaneNormalCluster
    {
        Eigen::Vector3f normal;
        int count;
        int plane_points_numbers; // for main vertical plane normal
        float eigen_value; // for coarse side plane normal
    };

    struct KeyInfo
    {
        bool init;
        pcl::ModelCoefficients ground_coeff;
        PlaneNormalInfo main_vertical_plane_normal;
        // if find two cluster vertical plane normal, then second_vertical_plane_normal will be used
        PlaneNormalInfo second_vertical_plane_normal;
        PlaneNormalInfo horizontal_plane_direction;
        PlaneNormalInfo main_center_diff_vector;

        pcl::ModelCoefficients side_plane_left;
        pcl::ModelCoefficients side_plane_right;

        KeyInfo() : init(true) {}
    };

    struct Relation
    {
        Eigen::Vector3f center_direction; // the center vector diff (no normalization)
        float distance; // estimated rough distance of two point cloud
        float delta_h; // delta height
        float delta_d; // distance in stair vertical plane normal direction
    };

    struct int2
    {
        int from;
        int to;
    };

    class StairDetection
    {
    private:

        /********************** run time parameters ****************************/
        // the down direction and ref right direction which parallel with gravity
        Eigen::Vector3f down_direction, right_direction, forward_diretion;

        // remain point number of each plane after random down sample
        int down_sample_points_number;

        // horizontal plane: plane angle [0,angle_h)
        // slope plane:      plane angle [angle_h,angle_v)
        // vertical plane:   plane angle [angle_v,PI/2]
        float angle_h, angle_v;

        float ground_height_th;
        int min_ground_plane_points_num;

        // the threshold to merge two planes (m/reg)
        float merge_dis_th, merge_angle_th, merge_distance_th;

        // the threshold of a1D in paper DIMENSIONALITY BASED SCALE SELECTION IN 3D LIDAR POINT CLOUDS
        // which define the line like of a set of points (0~1)
        float a1D_th;

        // history info decay rate
        float decay_rate;

        // vertical plane normal cluster angle threshold
        float vertical_plane_normal_angle_th;
        // the vertical_plane_normal should be close to eigen_vector[1] of stair horizontal plane in vertical_plane_normal_est_angle_th
        float vertical_plane_normal_est_angle_th;

        // center vector direction angle with horizontal_plane_direction in 90 degrees threshold
        // the center vector direction should be perpendicular with horizontal_plane_direction
        float cv_angle_horizontal_th;

        // cluster main center vector angle threshold
        float cv_dir_cluster_angle_th;

        // plane length, width and height constraint
        float plane_min_length, plane_max_length;
        float plane_min_width, plane_max_width;
        // stair width and height constraint
        float stair_max_width,stair_max_height;

        // the max distance of two planes in a stair, exceed this distance, the two plane will not be judge as in a stair
        float stair_plane_max_distance;
        // the max angle difference threshold of center vectors of two neighbor planes in stair when modeling
        float stair_cv_angle_diff_th;


        std::vector<std::vector<Relation>> confuse_matrix;
        std::vector<int2> fragment_vector;
        std::vector<int> stair_plane_index_list;

    public:
        bool process(std::vector<Plane> &vector_plane, Stair &stair, KeyInfo &keyInfo, std::vector<double> &vectime);

        /** \brief print stair model
         * \param[in] stair: stair model
         */
        std::string getDetialStairModelString(Stair &stair);

        std::string getEstimatedParamString(Stair &stair);

        std::string getEstimatedParamStringRcd(Stair &stair);


        StairDetection() :
                down_direction(Eigen::Vector3f(1, 0, 0)),
                right_direction(Eigen::Vector3f(0, 0, 1)),
                forward_diretion(Eigen::Vector3f(0, 1, 0)),
                down_sample_points_number(200),
                angle_h(pcl::deg2rad(10.0)), angle_v(pcl::deg2rad(70.0)),
                ground_height_th(0.06),min_ground_plane_points_num(50000),
                merge_dis_th(0.02), merge_angle_th(pcl::deg2rad(12.0)), merge_distance_th(0.8),
                a1D_th(0.85), decay_rate(0.5),
                vertical_plane_normal_angle_th(pcl::deg2rad(15.0)),
                vertical_plane_normal_est_angle_th(pcl::deg2rad(5.0)),
                cv_angle_horizontal_th(pcl::deg2rad(10.0)),
                cv_dir_cluster_angle_th(pcl::deg2rad(5.0)),
                plane_min_length(0.4), plane_max_length(1.4), plane_min_width(0.18), plane_max_width(0.80),
                stair_max_height(0.5),stair_max_width(0.33),
                stair_plane_max_distance(0.4),stair_cv_angle_diff_th(pcl::deg2rad(30.0))
                {}

        void setDown_sample_points_number(int down_sample_points_number);

        void setAngle_h(float angle_h);

        void setAngle_v(float angle_v);

        void setGround_height_th(float ground_height_th);

        void setMin_ground_plane_points_num(int min_ground_plane_points_num);

        void setMerge_dis_th(float merge_dis_th);

        void setMerge_angle_th(float merge_angle_th);

        void setMerge_distance_th(float merge_center_dis_th);

        void setA1D_th(float a1D_th);

        void setDecay_rate(float decay_rate);

        void setVertical_plane_normal_angle_th(float vertical_plane_normal_angle_th);

        void setCv_angle_horizontal_th(float cv_angle_horizontal_th);

        void setVertical_plane_normal_est_angle_th(float vertical_plane_normal_est_angle_th);

        void setCv_dir_cluster_angle_th(float cv_dir_cluster_angle_th);

        void setPlane_min_length(float min_length);

        void setPlane_max_length(float max_length);

        void setPlane_min_width(float min_width);

        void setPlane_max_width(float max_width);

        void setStair_max_height(float max_height);

        void setStair_max_width(float stair_max_width);

        void setStair_plane_max_distance(float stair_plane_max_distance);

        void setStair_cv_angle_diff_th(float stair_cv_angle_diff_th);

        ~StairDetection()
        {
#ifdef TIME_RECORD
            ofile.close();
#endif
        }

    private:

        void clearStatus();

        /*
         * mark the type (horizontal, vertical, slope) based on plane normal
         */
        void markHVType(std::vector<Plane> &vector_plane);

        /*
         * remove vertical planes which maybe vertical poles, and remove slope planes
         */
        void removeVerticalPoleSlopePlanes(std::vector<Plane> &vector_plane);

        /*
         * sort planes by height
         */
        void sortPlanesByHeight(std::vector<Plane> &vector_plane);

        /*
         * random sample points form full size cloud (construct random_down_sample_cloud in Plane)
         * so that the latter compute can do some approximate computation efficently
         */
        void randomSampleSomePoints(std::vector<Plane> &vector_plane);

        /*
         * find and merge ground plane
         */
        void findMergeGroudPlanes(std::vector<Plane> &vector_plane, KeyInfo &keyInfo);

        /*
         * merge the horizontal planes separated by obstacles
         */
        void mergeSeparatedPlanes(std::vector<Plane> &vector_plane);

        bool shouldMerge(Plane &p1, Plane &p2, float dis_th);

        /*
         * compute min max related info in struct Plane, this maybe time consuming
         */
        void computePlanesMinMax(std::vector<Plane> &vector_plane);

        /*
         * compute the confuse matrix of each plane (for all the planes in current frame)
         */
        void computeConfuseMatrix(const std::vector<Plane> &vector_plane,
                                  std::vector<std::vector<Relation>> &confuse_matrix);

        float minDistaceOfTwoCloud(const Plane &plane1, const Plane &plane2);

        /*
         * find the dominate vertical plane normal in the current frame (and refine it using history info)
         */
        void findDominateVerticalPlaneNormal(std::vector<Plane> &vector_plane, KeyInfo &keyInfo);

        /*
         * find a coarse side plane normal using stair steps length direction
         */
        void findHorizontalPlaneDirection(std::vector<Plane> &vector_plane, KeyInfo &keyInfo);

        /*
         * refine the key directions (in case there is no vertical plane be found)
         */
        bool refineKeyDirections(KeyInfo &keyInfo);

        /*
         * find the dominate center diff vector in the current frame (and refine it using history info)
         */
        void findDominateCenterDiffVector(const std::vector<Plane> &vector_plane,
                                          std::vector<std::vector<Relation>> &confuse_matrix, KeyInfo &keyInfo);


        /** \brief find the point whose projection in on vector is max and min
         * \param[in] plane: include input cloud
         * \param[in] vector: (vx,vy,vz),the projection direction
         * \param[out] point: max_point and min_point
         */
        inline void findMinMaxProjPoint(const Plane &plane, const Eigen::Vector3f &ve,
                                        PointType &min_point, PointType &max_point,
                                        float &min_proj,float &max_proj);

        /*
         * mark others plane based on main_vertical_normals
         */
        void filterPlanesWithKeyInfo(std::vector<Plane> &vector_plane, KeyInfo &keyInfo);

        /*
         * discard some planes which have unreasonable size
         */
        void filterPlanesWithSizeConstraint(std::vector<Plane> &vector_plane);

        /*
         * find all the connect plane fragments
         */
        void findConnectFragment(std::vector<Plane> &vector_plane, const std::vector<std::vector<Relation>> &confuse_matrix,
                                 KeyInfo &keyInfo, std::vector<int2> &fragment_vector);

        /*
         * find (or stitch) the longest from the connect plane fragments
         */
        void findStairPlaneIndexList(std::vector<int2> &fragment_vector,std::vector<int> &stair_plane_index_list);

        void findFragments(std::vector<int2> &fragment_vector, int index,
                                           std::vector<std::vector<int>> &vector_index_list, int vec_index);

        /*
         * mark plane and TODO: refine the main vertical normal?
         */
        void filterPlanesWithStairPlaneList(std::vector<Plane> &vector_plane,std::vector<int> &stair_plane_index_list);

        /*
         * compute stair side plane
         */
        void computeStairSidePlane(std::vector<Plane> &vector_plane,KeyInfo &keyInfo);

        /*
         * modeling stair
         */
        bool modelingStair(std::vector<Plane> &vector_plane,std::vector<int> &stair_plane_index_list,
                KeyInfo &keyInfo,Stair &stair);

        void computePlaneCounter(Stair &stair,KeyInfo &keyInfo);

        /** \brief compute cross line of two planes
         * \param[in] coefficients_plane1: input plane coefficients
         * \param[in] coefficients_plane2: input plane coefficients
         * \param[out] coefficients_line: output line coefficients
         */
        void computeLineFrom2Planes(
                const pcl::ModelCoefficients &coefficients_plane1,
                const pcl::ModelCoefficients &coefficients_plane2,
                KeyInfo keyInfo,
                Line &line);

        inline void crossPointOfLineAndPlane(
                const float &x0, const float &y0, const float &z0, const float &a, const float &b, const float &c,
                pcl::ModelCoefficients plane_coefficients,
                pcl::PointXYZ &pc);

    };

}

#endif /* STAIRDETECTION_HPP_ */
