/*
 * common.h
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <omp.h>

//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/common/time.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

namespace stair_perception
{
#define ABS(x) ((x)>0?(x):-1*(x))
    typedef pcl::PointXYZ PointType;

#define ptv pcl::PointCloud<PointType>::VectorType

    inline ptv operator+(ptv &a, ptv &b)
    {
        ptv ret;
        ret.insert(ret.end(), a.begin(), a.end());
        ret.insert(ret.end(), b.begin(), b.end());
        return ret;
    }

    struct ResultDef
    {
        bool has_stair;
        float height;
        float width;
        float v_height;
        float v_depth;
        double time;
    };

    struct Plane
    {
        enum Type
        {
            vertical = 0,
            horizontal,
            slope
        };

        enum Ptype
        {
            stair_component = 0,
            pstair_component,
            ground,
            others
        };

        /******************* basic plane properties *******************/
        pcl::PointCloud<PointType> cloud;
        pcl::PointCloud<PointType> random_down_sample_cloud;
        pcl::ModelCoefficients coefficients;
        PointType center;

        float sx, sy, sz, //sum of x/y/z
                sxx, syy, szz, //sum of xx/yy/zz
                sxy, syz, sxz; //sum of xy/yz/xz

        size_t N; //#points in this PlaneSeg

        float mse, curvature;

        float eigen_values[3];
        PointType eigen_vectors[3];

        /******************* extend plane properties *******************/
        // min and max distance in the direction of eigen vector
        float min[3], max[3];
        // and associated points
        PointType points_min[3], points_max[3];

        // plane counter
        pcl::PointCloud<PointType> counter;

        /******************* stair model related properties *******************/
        Ptype ptype;
        Type type;
        std::string info;

        /******************* useful inline functions *******************/
        inline void computeStatistics()
        {
            sx = sy = sz = sxx = syy = szz = sxy = syz = sxz = 0;
            for (size_t i = 0; i < cloud.size(); i++)
            {
                float &x = cloud.points[i].x;
                float &y = cloud.points[i].y;
                float &z = cloud.points[i].z;
                sx += x;
                sy += y;
                sz += z;
                sxx += x * x;
                syy += y * y;
                szz += z * z;
                sxy += x * y;
                syz += y * z;
                sxz += x * z;
            }
            N = cloud.size();
        }

        inline void computePlaneInfo()
        {
            const float sc = ((float) 1.0) / N;
            // center
            center.x = sx * sc;
            center.y = sy * sc;
            center.z = sz * sc;
            float K[3][3] = {
                    {sxx - sx * sx * sc,sxy - sx * sy * sc,sxz - sx * sz * sc},
                    {0,                 syy - sy * sy * sc,syz - sy * sz * sc},
                    {0,                 0,                 szz - sz * sz * sc}};

            K[1][0] = K[0][1];
            K[2][0] = K[0][2];
            K[2][1] = K[1][2];
            float V[3][3] = {0};

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(
                    Eigen::Map<Eigen::Matrix3f>(K[0], 3, 3));
            Eigen::Map<Eigen::Vector3f>(eigen_values, 3, 1) = es.eigenvalues();
            //below we need to specify row major since V!=V'
            Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(V[0], 3, 3) = es.eigenvectors();

            // coefficients
            coefficients.values.resize(4);
            if (V[0][0] * center.x
                + V[1][0] * center.y
                + V[2][0] * center.z <= 0)
            {//enforce dot(normal,center)<00 so normal always points towards camera
                coefficients.values[0] = V[0][0];
                coefficients.values[1] = V[1][0];
                coefficients.values[2] = V[2][0];
            } else
            {
                coefficients.values[0] = -V[0][0];
                coefficients.values[1] = -V[1][0];
                coefficients.values[2] = -V[2][0];
            }
            coefficients.values[3] = -1 *
                                     (coefficients.values[0] *
                                      center.x +
                                      coefficients.values[1] *
                                      center.y +
                                      coefficients.values[2] *
                                      center.z);

            mse = eigen_values[0] * sc;
            curvature = eigen_values[0] / (eigen_values[0] + eigen_values[1] + eigen_values[2]);

            eigen_vectors[0].x = V[0][0];
            eigen_vectors[0].y = V[1][0];
            eigen_vectors[0].z = V[2][0];
            eigen_vectors[1].x = V[0][1];
            eigen_vectors[1].y = V[1][1];
            eigen_vectors[1].z = V[2][1];
            eigen_vectors[2].x = V[0][2];
            eigen_vectors[2].y = V[1][2];
            eigen_vectors[2].z = V[2][2];
        }

        Plane():sx(0), sy(0), sz(0),
                    sxx(0), syy(0), szz(0),
                    sxy(0), syz(0), sxz(0), N(0),
                    mse(0),curvature(0)
        {
            ptype = Ptype::pstair_component;
        }

/*        //merge two other Planes, only the basic plane properties are merged
        struPlane(const struPlane& a, const struPlane& b) :
                sx(a.sx+b.sx), sy(a.sy+b.sy), sz(a.sz+b.sz),
                sxx(a.sxx+b.sxx), syy(a.syy+b.syy), szz(a.szz+b.szz),
                sxy(a.sxy+b.sxy), syz(a.syz+b.syz), sxz(a.sxz+b.sxz), N(a.N+b.N)
        {
            this->cloud = a.cloud + b.cloud;
            this->computePlaneInfo();
        }*/

        Plane& operator+=(const Plane& other)
        {
            this->sx += other.sx;
            this->sy += other.sy;
            this->sz += other.sz;
            this->sxx += other.sxx;
            this->syy += other.syy;
            this->szz += other.szz;
            this->sxy += other.sxy;
            this->syz += other.syz;
            this->sxz += other.sxz;
            this->N += other.N;
            this->cloud += other.cloud;
            this->random_down_sample_cloud += other.random_down_sample_cloud;
            this->computePlaneInfo();
        }
    };
}

#endif /* COMMON_H_ */
