#include <common.h>
#include "stair_modeling/StairPerception.hpp"

#include <ros/ros.h>
#include <chrono>
#include <stair_modeling/StairPerception.hpp>


using namespace std;
using namespace std::chrono;

//#define SHOWTIME

double tic_toc_ms()
{
    static auto last = system_clock::now();
    auto duration = duration_cast<microseconds>(system_clock::now() - last);
    last = system_clock::now();
    return duration.count() / 1000.0;
}

//#define DEBUG_STAIR_MODEL
namespace stair_perception
{
    bool StairDetection::process(std::vector<Plane> &vector_plane, Stair &stair, KeyInfo &keyInfo,
                                 std::vector<double> &vectime)
    {

        double t;
#ifdef SHOWTIME
        ROS_INFO("***************** stair modeling process in ***************** ");
#endif

        bool has_stair = false;

        clearStatus();

        /********************** pre-process: do some easy things **********************/
        // 1. mark the type(horizontal,vertical,slope) based on plane normal
        tic_toc_ms();
        markHVType(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("markHVType time: %.2f ms", tic_toc_ms());
#endif
        // 2. remove vertical planes which maybe vertical poles, and remove slope planes
        removeVerticalPoleSlopePlanes(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("removeVerticalPoleSlopePlanes time: %.2f ms", tic_toc_ms());
#endif

        // 3. sort planes by height
        sortPlanesByHeight(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("sortPlanesByHeight time: %.2f ms", tic_toc_ms());
#endif

        // 4. random sample points form full size cloud (construct random_down_sample_cloud in Plane)
        randomSampleSomePoints(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("randomSampleSomePoints time: %.2f ms", tic_toc_ms());
#endif

        // 5. find and merge ground plane
        findMergeGroudPlanes(vector_plane, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("findMergeGroudPlanes time: %.2f ms", tic_toc_ms());
#endif

        // 6. merge the planes separated by obstacles
        mergeSeparatedPlanes(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("mergeSeparatedPlanes time: %.2f ms", tic_toc_ms());
#endif

        // 7. compute min max related info in struct Plane, this maybe time consuming
        computePlanesMinMax(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("computePlanesMinMax time: %.2f ms", tic_toc_ms());
#endif

        // 8. discard some planes which have unreasonable size
        filterPlanesWithSizeConstraint(vector_plane);

#ifdef SHOWTIME
        ROS_INFO("filterPlanesWithSizeConstraint time: %.2f ms", tic_toc_ms());
#endif

        t = tic_toc_ms();
        vectime.push_back(t);
        /************\ pre-modeling: get key directions of staircase for modeling********/
        // 9. compute the confuse matrix of each plane (for all the planes in current frame)
        computeConfuseMatrix(vector_plane, confuse_matrix);

#ifdef SHOWTIME
        ROS_INFO("computeConfuseMatrix time: %.2f ms", tic_toc_ms());
#endif

        // 10. find the dominate vertical plane normal in the current frame (add history dominate vertical plane normal)
        findDominateVerticalPlaneNormal(vector_plane, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("findDominateVerticalPlaneNormal time: %.2f ms", tic_toc_ms());
#endif

        // 11. find horizontal plane direction (add horizontal plane direction)
        findHorizontalPlaneDirection(vector_plane, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("findHorizontalPlaneDirection time: %.2f ms", tic_toc_ms());
#endif

        // 12. refine the key directions (in case there is no vertical plane be found)
        //     in most stair cases, the horizontal plane direction is always correct
        if (!refineKeyDirections(keyInfo))
            return has_stair;

#ifdef SHOWTIME
        ROS_INFO("refineKeyDirections time: %.2f ms", tic_toc_ms());
#endif

        // 13. find the dominate center diff vector in the current frame (add dominate center diff vector)
        findDominateCenterDiffVector(vector_plane, confuse_matrix, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("findDominateCenterDiffVector time: %.2f ms", tic_toc_ms());
#endif

        // 14. mark others plane based on keyInfo
        filterPlanesWithKeyInfo(vector_plane, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("filterPlanesWithKeyInfo time: %.2f ms", tic_toc_ms());
#endif

        t = tic_toc_ms();
        vectime.push_back(t);
        /********************** stair modeling  **********************/
        // 15. find all the connect plane fragments
        findConnectFragment(vector_plane, confuse_matrix, keyInfo, fragment_vector);

#ifdef SHOWTIME
        ROS_INFO("findConnectFragment time: %.2f ms", tic_toc_ms());
#endif

        // 16. find (or stitch) the longest from the connect plane fragments
        findStairPlaneIndexList(fragment_vector, stair_plane_index_list);

#ifdef SHOWTIME
        ROS_INFO("findStairPlaneIndexList time: %.2f ms", tic_toc_ms());
#endif

        // 17. mark plane and TODO: refine the main vertical normal?
        filterPlanesWithStairPlaneList(vector_plane, stair_plane_index_list);

#ifdef SHOWTIME
        ROS_INFO("filterPlanesWithStairPlaneList time: %.2f ms", tic_toc_ms());
#endif

        // 18. compute stair side plane
        computeStairSidePlane(vector_plane, keyInfo);

#ifdef SHOWTIME
        ROS_INFO("computeStairSidePlane time: %.2f ms", tic_toc_ms());
#endif

        // 19. modeling stair: construct stair link list model
        has_stair = modelingStair(vector_plane, stair_plane_index_list, keyInfo, stair);

#ifdef SHOWTIME
        ROS_INFO("modelingStair time: %.2f ms", tic_toc_ms());
#endif

        // 20. compute the counter of the planes in the stair (for visualization)
        if (has_stair)
            computePlaneCounter(stair, keyInfo);

        keyInfo.init = false;

        t = tic_toc_ms();
        vectime.push_back(t);
#ifdef SHOWTIME
        ROS_INFO("*****************  stair modeling  process out ***************** ");
#endif

        return has_stair;
    }

    /* \brief clear the inner varibale status
    */
    void StairDetection::clearStatus()
    {
        confuse_matrix.clear();
        fragment_vector.clear();
        stair_plane_index_list.clear();
    }

    /*
     * mark the type (horizontal, vertical, slope) based on plane normal
     * runtime param: down_direction,angle_h,angle_v
     */
    void StairDetection::markHVType(std::vector<Plane> &vector_plane)
    {
        for (auto &i : vector_plane)
        {
            Eigen::Vector3f plane_normal(i.coefficients.values[0],
                                         i.coefficients.values[1],
                                         i.coefficients.values[2]);

            double angle = acos(plane_normal.dot(down_direction));

            // horizontal plane
            if ((angle < angle_h) || (M_PI - angle < angle_h))
            {
                i.type = Plane::horizontal;
            }
                // vertical plane
            else if (fabs(angle - M_PI / 2) < M_PI / 2 - angle_v)
            {
                i.type = Plane::vertical;
            }
                // slope plane
            else
            {
                i.type = Plane::slope;
            }
        }
    }

    /*
     * remove vertical planes which maybe vertical poles
     * vertical poles are common things in real stair case
     * and remove slope planes
     */
    void StairDetection::removeVerticalPoleSlopePlanes(std::vector<Plane> &vector_plane)
    {
        for (auto &i : vector_plane)
        {
            if (i.type == Plane::slope)
            {
                i.ptype = Plane::others;
            } else if (i.type == Plane::vertical)
            {
                // compute a1D in paper DIMENSIONALITY BASED SCALE SELECTION IN 3D LIDAR POINT CLOUDS
                float sigma1 = sqrtf(i.eigen_values[2]);
                float sigma2 = sqrtf(i.eigen_values[1]);
                float sigma3 = sqrtf(i.eigen_values[0] < 0 ? 0 : i.eigen_values[0]);
                float a1D = (sigma1 - sigma2) / sigma1;
                float a2D = (sigma2 - sigma3) / sigma1;

                if (a1D > a1D_th && a1D > a2D)
                {
                    // the vector in the length deriction
                    Eigen::Vector3f length_vector(i.eigen_vectors[2].x,
                                                  i.eigen_vectors[2].y,
                                                  i.eigen_vectors[2].z);

                    // the angle between length_vector and down_direction
                    // if the angle is in the vertical threshold then delete this plane
                    float dot_product = length_vector.dot(down_direction);
                    float angle = acos(dot_product);
                    if ((angle < M_PI / 2 - angle_v) || (M_PI - angle < M_PI / 2 - angle_v))
                    {
                        i.ptype = Plane::others;
                        i.info = "{slope}";
                    }
                }
            }
        }
    }

    /*
    * sort planes by height
    */
    void StairDetection::sortPlanesByHeight(std::vector<Plane> &vector_plane)
    {
        // this is very slow, due to heavey copy of data
        /*std::sort(vector_plane.begin(), vector_plane.end(),
                  [&](Plane pa, Plane pb)
                  {
                      Eigen::Vector3f diff_center(pa.center.x - pb.center.x,
                                                  pa.center.y - pb.center.y,
                                                  pa.center.z - pb.center.z);
                      return diff_center.dot(down_direction) > 0;
                  });*/

        struct IndexCenter
        {
            int index;
            pcl::PointXYZ center;
        };

        std::vector<IndexCenter> vec_index_centers;

//        tic_toc_ms();
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            IndexCenter indexCenter;
            indexCenter.index = i;
            indexCenter.center = vector_plane[i].center;

            vec_index_centers.push_back(indexCenter);
        }
//        ROS_INFO("\tcreate vec_index_centers time: %.2f ms", tic_toc_ms());

        std::sort(vec_index_centers.begin(), vec_index_centers.end(),
                  [&](IndexCenter pa, IndexCenter pb)
                  {
                      Eigen::Vector3f diff_center(pa.center.x - pb.center.x,
                                                  pa.center.y - pb.center.y,
                                                  pa.center.z - pb.center.z);
                      return diff_center.dot(down_direction) > 0;
                  });
//        ROS_INFO("\tsort vec_index_centers time: %.2f ms", tic_toc_ms());

        // the current index list
        std::vector<int> curr_index;
        for (size_t i = 0; i < vector_plane.size(); i++)
            curr_index.push_back(i);

        for (size_t i = 0; i < vec_index_centers.size(); i++)
        {
            for (size_t j = 0; j < curr_index.size(); j++)
            {
                if (curr_index[j] == vec_index_centers[i].index)
                {
                    //swap
                    std::swap(vector_plane[j], vector_plane[i]);
                    std::swap(curr_index[j], curr_index[i]);
                    break;
                }
            }
        }
//        ROS_INFO("\tvector_plane time: %.2f ms", tic_toc_ms());


//        std::vector<Plane> copy_vector_plane = vector_plane;

//        for (size_t i = 0; i < vec_index_centers.size(); i++)
//            vector_plane[i] = copy_vector_plane[vec_index_centers[i].index];

    }

    /*
     * random sample points form full size cloud (construct random_down_sample_cloud in Plane)
     * so that the latter compute can do some approximate computation efficently
     */
    void StairDetection::randomSampleSomePoints(std::vector<Plane> &vector_plane)
    {
        for (auto &i : vector_plane)
        {
            for (size_t j = 0; j < down_sample_points_number; j++)
            {
                PointType sample_point = i.cloud.points[random() % i.cloud.points.size()];
                i.random_down_sample_cloud.points.push_back(sample_point);
            }
            i.random_down_sample_cloud.width = i.random_down_sample_cloud.points.size();
            i.random_down_sample_cloud.height = 1;
        }
    }

    /*
     * find and merge ground plane
     */
    void StairDetection::findMergeGroudPlanes(std::vector<Plane> &vector_plane, KeyInfo &keyInfo)
    {
        pcl::ModelCoefficients ref;

        if (keyInfo.init || keyInfo.ground_coeff.values.empty())
        {
            // find the lowest plane as ref plane
            for (auto &i : vector_plane)
            {
                if (i.type == Plane::Type::horizontal)
                {
                    ref = i.coefficients;
                    break;
                }
            }
        } else
        {
            // TODO: maybe need the estimated camera motion pose
            ref = keyInfo.ground_coeff;
        }

        // compare the lowest horizontal planes with ref ground plane
        if (vector_plane[0].type == Plane::Type::horizontal)
        {
            float a, b, c, d, px, py, pz, distance;
            a = ref.values[0];
            b = ref.values[1];
            c = ref.values[2];
            d = ref.values[3];

            size_t count = 0;
            for (auto &point : vector_plane[0].random_down_sample_cloud.points)
            {
                // sampled points from cloud 0
                px = point.x;
                py = point.y;
                pz = point.z;
                // compute the distance to cloud 0
                distance = fabs(a * px + b * py + c * pz + d);
                if (distance <= ground_height_th)
                    count++;
            }

            // near ref ground plane
            if (count > vector_plane[0].random_down_sample_cloud.points.size() - 5)
            {
                for (int j = 1; j < vector_plane.size(); j++)
                {
                    // judge should merge or not
                    if (vector_plane[j].type == Plane::Type::horizontal
                        && shouldMerge(vector_plane[0], vector_plane[j], ground_height_th))
                    {
                        // 1) satisfy the above conditions, then merge the two plane.
                        vector_plane[0] += vector_plane[j];

                        // 2) erase plane j.
                        vector_plane.erase(vector_plane.begin() + j);

                        j = 1;
                    }
                }
            }

            if (vector_plane[0].cloud.points.size() > min_ground_plane_points_num)
            {
                vector_plane[0].ptype = Plane::Ptype::ground;

                keyInfo.ground_coeff.values.resize(4);
                keyInfo.ground_coeff = vector_plane[0].coefficients;
            }
        }
    }

    /*
     * merge the planes separated by obstacles
     * runtime param: merge_dis_th,merge_angle_th
     */
    void StairDetection::mergeSeparatedPlanes(std::vector<Plane> &vector_plane)
    {
        for (int i = 0; i < vector_plane.size(); i++)
        {
            for (int j = i + 1; j < vector_plane.size(); j++)
            {
                // judge should merge or not
                if (shouldMerge(vector_plane[i], vector_plane[j], merge_dis_th))
                {
                    if (vector_plane[i].type == vector_plane[j].type)
                    {
                        // if the distance of two plane lager than merge_distance_th
                        // do not merge, otherwise will be a large disturb plane
                        float distance = minDistaceOfTwoCloud(vector_plane[i], vector_plane[j]);
                        if (distance > merge_distance_th)
                            continue;

                        // 1) satisfy the above conditions, then merge the two plane.
                        vector_plane[i] += vector_plane[j];

                        // 2) erase plane j.
                        vector_plane.erase(vector_plane.begin() + j);

                        // finish one combination,then restart this process
                        i--;
                        break;
                    }
                }
            }
        }
    }

    /*
     * judge two planes shold merge or not, based on ave dis and normal dis
     * input: two planes
     * return: should merge or not
     */
    bool StairDetection::shouldMerge(Plane &p1, Plane &p2, float dis_th)
    {
        pcl::ModelCoefficients &coeff1 = p1.coefficients;
        pcl::ModelCoefficients &coeff2 = p2.coefficients;

        Eigen::Vector3f normal1(coeff1.values[0], coeff1.values[1], coeff1.values[2]);
        Eigen::Vector3f normal2(coeff2.values[0], coeff2.values[1], coeff2.values[2]);

        float dot_product = normal1.dot(normal2);
        float angle = acos(dot_product);
        if ((angle < merge_angle_th) || (M_PI - angle < merge_angle_th))
        {
            float a1, b1, c1, d1, px, py, pz;
            float a2, b2, c2, d2, distance;
            a1 = coeff1.values[0];
            b1 = coeff1.values[1];
            c1 = coeff1.values[2];
            d1 = coeff1.values[3];
            a2 = coeff2.values[0];
            b2 = coeff2.values[1];
            c2 = coeff2.values[2];
            d2 = coeff2.values[3];

            //compute distance to cloudj form 20 points sampled from cloudi
            size_t count = 0;
            for (auto &point : p1.random_down_sample_cloud.points)
            {
                // sampled points from cloudi
                px = point.x;
                py = point.y;
                pz = point.z;
                // compute the distance to cloudj
                distance = fabs(a2 * px + b2 * py + c2 * pz + d2);
                if (distance <= dis_th)
                    count++;
            }

            for (auto &point : p2.random_down_sample_cloud.points)
            {
                // sampled points from cloudj
                px = point.x;
                py = point.y;
                pz = point.z;
                // compute the distance to cloudi
                distance = fabs(a1 * px + b1 * py + c1 * pz + d1);
                if (distance <= dis_th)
                    count++;
            }

            // judge should merge or not
            if (count > (p1.random_down_sample_cloud.points.size() + p2.random_down_sample_cloud.points.size()) / 2 - 2)
            {
                return true;
            }
        }

        return false;
    }

    /*
     * compute min max related info in struct Plane, this maybe time consuming
     */
    void StairDetection::computePlanesMinMax(std::vector<Plane> &vector_plane)
    {
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            vector_plane[i].max[0] = vector_plane[i].max[1] = vector_plane[i].max[2] = -FLT_MAX;
            vector_plane[i].min[0] = vector_plane[i].min[1] = vector_plane[i].min[2] = FLT_MAX;
            for (size_t index = 0; index < vector_plane[i].random_down_sample_cloud.points.size(); index++)
            {
                // vector form center to point
                Eigen::Vector3f cp;
                cp[0] = vector_plane[i].random_down_sample_cloud.points[index].x - vector_plane[i].center.x;
                cp[1] = vector_plane[i].random_down_sample_cloud.points[index].y - vector_plane[i].center.y;
                cp[2] = vector_plane[i].random_down_sample_cloud.points[index].z - vector_plane[i].center.z;
                // eigen vectors
                Eigen::Vector3f ev0 = Eigen::Vector3f(vector_plane[i].eigen_vectors[0].x,
                                                      vector_plane[i].eigen_vectors[0].y,
                                                      vector_plane[i].eigen_vectors[0].z);
                Eigen::Vector3f ev1 = Eigen::Vector3f(vector_plane[i].eigen_vectors[1].x,
                                                      vector_plane[i].eigen_vectors[1].y,
                                                      vector_plane[i].eigen_vectors[1].z);
                Eigen::Vector3f ev2 = Eigen::Vector3f(vector_plane[i].eigen_vectors[2].x,
                                                      vector_plane[i].eigen_vectors[2].y,
                                                      vector_plane[i].eigen_vectors[2].z);
                // cp projection to eigen vectors
                float pj0 = cp.dot(ev0);
                float pj1 = cp.dot(ev1);
                float pj2 = cp.dot(ev2);

                // compare and save
                if (pj0 > vector_plane[i].max[0])
                {
                    vector_plane[i].max[0] = pj0;
                    vector_plane[i].points_max[0] = vector_plane[i].random_down_sample_cloud.points[index];
                } else if (pj0 < vector_plane[i].min[0])
                {
                    vector_plane[i].min[0] = pj0;
                    vector_plane[i].points_min[0] = vector_plane[i].random_down_sample_cloud.points[index];
                }

                if (pj1 > vector_plane[i].max[1])
                {
                    vector_plane[i].max[1] = pj1;
                    vector_plane[i].points_max[1] = vector_plane[i].random_down_sample_cloud.points[index];
                } else if (pj1 < vector_plane[i].min[1])
                {
                    vector_plane[i].min[1] = pj1;
                    vector_plane[i].points_min[1] = vector_plane[i].random_down_sample_cloud.points[index];
                }

                if (pj2 > vector_plane[i].max[2])
                {
                    vector_plane[i].max[2] = pj2;
                    vector_plane[i].points_max[2] = vector_plane[i].random_down_sample_cloud.points[index];
                } else if (pj2 < vector_plane[i].min[2])
                {
                    vector_plane[i].min[2] = pj2;
                    vector_plane[i].points_min[2] = vector_plane[i].random_down_sample_cloud.points[index];
                }
            }
        }
    }

    /*
     * compute the confuse matrix of each plane (for all the planes in current frame)
     */
    void StairDetection::computeConfuseMatrix(const std::vector<Plane> &vector_plane,
                                              std::vector<std::vector<Relation>> &confuse_matrix)
    {
        unsigned long d = vector_plane.size();
        confuse_matrix.resize(d);
        for (size_t i = 0; i < d; i++)
            confuse_matrix[i].resize(d);

        // for all planes
        for (size_t i = 0; i < d; i++)
        {
            for (size_t j = i + 1; j < d; j++)
            {
                // compute matrix
                confuse_matrix[i][j].distance = minDistaceOfTwoCloud(vector_plane[i], vector_plane[j]);
                confuse_matrix[i][j].delta_h = fabs(vector_plane[i].center.x -
                                                    vector_plane[j].center.x);
                confuse_matrix[i][j].center_direction = Eigen::Vector3f(
                        vector_plane[j].center.x - vector_plane[i].center.x,
                        vector_plane[j].center.y - vector_plane[i].center.y,
                        vector_plane[j].center.z - vector_plane[i].center.z);
            }
        }
    }

    float StairDetection::minDistaceOfTwoCloud(const Plane &plane1, const Plane &plane2)
    {
        float min_distance2 = FLT_MAX;

        std::vector<PointType> sample_points_p1, sample_points_p2;

        // random sampled points
        for (const auto &point : plane1.random_down_sample_cloud.points)
            sample_points_p1.push_back(point);

        for (const auto &point : plane2.random_down_sample_cloud.points)
            sample_points_p2.push_back(point);

        // compute min distance
        for (auto &i : sample_points_p1)
        {
            for (auto &j : sample_points_p2)
            {
                float distance2 = (i.x - j.x) *
                                  (i.x - j.x) +
                                  (i.y - j.y) *
                                  (i.y - j.y) +
                                  (i.z - j.z) *
                                  (i.z - j.z);
                if (distance2 < min_distance2) min_distance2 = distance2;
            }
        }

        return sqrtf(min_distance2);
    }


    /*
     * find the dominate vertical plane normal in the current frame (and refine it using history info)
     */
    void StairDetection::findDominateVerticalPlaneNormal(std::vector<Plane> &vector_plane, KeyInfo &keyInfo)
    {
        std::vector<PlaneNormalInfo> vec_vertical_plane_normals;

        // history dominate vertical plane normal
        if (!keyInfo.init)
        {
            if (!keyInfo.main_vertical_plane_normal.normal.norm())
            {
                // decay the history point number so the history weight will be decay
                keyInfo.main_vertical_plane_normal.plane_points_numbers *= (1 - decay_rate);
                vec_vertical_plane_normals.push_back(keyInfo.main_vertical_plane_normal);
            }
        }

        for (auto &i : vector_plane)
        {
            if (i.type == Plane::Type::vertical
                && i.ptype == Plane::Ptype::pstair_component)
            {
                Eigen::Vector3f normal = Eigen::Vector3f(
                        i.coefficients.values[0],
                        i.coefficients.values[1],
                        i.coefficients.values[2]);

                PlaneNormalInfo plane_normal_info;

                if (normal.dot(forward_diretion) < 0)
                    plane_normal_info.normal = normal;
                else
                    plane_normal_info.normal = -normal;

                plane_normal_info.plane_points_numbers = i.cloud.points.size();
                plane_normal_info.eigen_value = 0; // not use

                vec_vertical_plane_normals.push_back(plane_normal_info);
            }
        }

        if (!vec_vertical_plane_normals.empty())
        {
            // compute weight
            int total_point_number = 0;
            for (auto &vec_vertical_plane_normal : vec_vertical_plane_normals)
                total_point_number += vec_vertical_plane_normal.plane_points_numbers;
            for (auto &vec_vertical_plane_normal : vec_vertical_plane_normals)
                vec_vertical_plane_normal.weight =
                        (float) vec_vertical_plane_normal.plane_points_numbers / total_point_number;


            std::vector<PlaneNormalCluster> vec_vertical_plane_normals_cluster;

            // for all vertical normals
            for (auto &vec_vertical_plane_normal : vec_vertical_plane_normals)
            {
                Eigen::Vector3f &plane_vector_normal = vec_vertical_plane_normal.normal;
                float &weight = vec_vertical_plane_normal.weight;

                // for all normals in the vec_vertical_plane_normals_cluster
                bool exist = false;
                for (auto &j : vec_vertical_plane_normals_cluster)
                {
                    Eigen::Vector3f cluster_vertical_normal = j.normal;
                    cluster_vertical_normal.normalize();

                    float dot_product = plane_vector_normal.dot(cluster_vertical_normal);
                    float angle = acos(dot_product);

                    // add and count exist one
                    if (angle < vertical_plane_normal_angle_th)
                    {
                        j.normal += weight * plane_vector_normal;
                        j.count += 1;
                        j.plane_points_numbers += vec_vertical_plane_normal.plane_points_numbers;
                        exist = true;
                        break;
                    } else if (M_PI - angle < vertical_plane_normal_angle_th)
                    {
                        j.normal -= weight * plane_vector_normal;
                        j.count += 1;
                        j.plane_points_numbers += vec_vertical_plane_normal.plane_points_numbers;
                        exist = true;
                        break;
                    }
                }

                // not exist in cluster, add a new one
                if (!exist)
                {
                    PlaneNormalCluster planeNormalCluster;
                    planeNormalCluster.normal = weight * plane_vector_normal;
                    planeNormalCluster.count = 1;
                    planeNormalCluster.plane_points_numbers = vec_vertical_plane_normal.plane_points_numbers;
                    vec_vertical_plane_normals_cluster.push_back(planeNormalCluster);
                }
            }

            for (auto &i : vec_vertical_plane_normals_cluster)
                i.normal.normalize();

            if (!vec_vertical_plane_normals_cluster.empty())
            {
                // sort based on the counts
                std::sort(vec_vertical_plane_normals_cluster.begin(), vec_vertical_plane_normals_cluster.end(),
                          [&](PlaneNormalCluster a, PlaneNormalCluster b)
                          {
                              return a.count > b.count;
                          });

                PlaneNormalInfo main_vertical_plane_normal;
                main_vertical_plane_normal.eigen_value = 0; // not use
                main_vertical_plane_normal.count = vec_vertical_plane_normals_cluster[0].count;
                main_vertical_plane_normal.normal = vec_vertical_plane_normals_cluster[0].normal;
                main_vertical_plane_normal.plane_points_numbers = vec_vertical_plane_normals_cluster[0].plane_points_numbers;

                keyInfo.main_vertical_plane_normal = main_vertical_plane_normal;

                // if find two cluster vertical plane normal, then second_vertical_plane_normal will be used
                if (vec_vertical_plane_normals_cluster.size() > 1)
                {
                    PlaneNormalInfo second_vertical_plane_normal;
                    second_vertical_plane_normal.eigen_value = 0; // not use
                    second_vertical_plane_normal.count = vec_vertical_plane_normals_cluster[1].count;
                    second_vertical_plane_normal.normal = vec_vertical_plane_normals_cluster[1].normal;
                    second_vertical_plane_normal.plane_points_numbers = vec_vertical_plane_normals_cluster[1].plane_points_numbers;

                    keyInfo.second_vertical_plane_normal = second_vertical_plane_normal;
                } else
                {
                    PlaneNormalInfo empty_dir;
                    keyInfo.second_vertical_plane_normal = empty_dir;
                }
            } else
            {
                PlaneNormalInfo empty_dir;
                keyInfo.main_vertical_plane_normal = empty_dir;
            }
        }
    }

    /*
     * find a coarse side plane normal using stair steps length direction
     */
    void StairDetection::findHorizontalPlaneDirection(std::vector<Plane> &vector_plane, KeyInfo &keyInfo)
    {
        std::vector<PlaneNormalInfo> vec_horizontal_plane_directions;

        // history horizontal_plane_direction
        if (!keyInfo.init)
        {
            if (!keyInfo.horizontal_plane_direction.normal.norm())
            {
                // decay the eigen_value so the history weight will be decay
                keyInfo.horizontal_plane_direction.eigen_value *= (1 - decay_rate);
                vec_horizontal_plane_directions.push_back(keyInfo.horizontal_plane_direction);
            }
        }

        for (auto &i : vector_plane)
        {
            if (i.type == Plane::horizontal && i.ptype == Plane::pstair_component)
            {
                // the length direction of plane
                Eigen::Vector3f direction = Eigen::Vector3f(
                        i.eigen_vectors[2].x,
                        i.eigen_vectors[2].y,
                        i.eigen_vectors[2].z);

                PlaneNormalInfo plane_direction_info;

                if (direction.dot(right_direction) < 0)
                    plane_direction_info.normal = -direction;
                else
                    plane_direction_info.normal = direction;
                plane_direction_info.plane_points_numbers = 0; // not use
                plane_direction_info.eigen_value = sqrtf(i.eigen_values[2]);

                vec_horizontal_plane_directions.push_back(plane_direction_info);
            }
        }

        if (!vec_horizontal_plane_directions.empty())
        {
            // compute weight
            float total_eigen_value = 0;
            for (auto &vec_horizontal_plane_direction : vec_horizontal_plane_directions)
                total_eigen_value += fabs(vec_horizontal_plane_direction.eigen_value);
            for (auto &vec_horizontal_plane_direction : vec_horizontal_plane_directions)
                vec_horizontal_plane_direction.weight =
                        fabs(vec_horizontal_plane_direction.eigen_value) / total_eigen_value;


            std::vector<PlaneNormalCluster> vec_horizontal_plane_direction_cluster;

            // for all vertical normals
            for (auto &vec_horizontal_plane_direction : vec_horizontal_plane_directions)
            {
                Eigen::Vector3f &plane_vector_normal = vec_horizontal_plane_direction.normal;
                float &weight = vec_horizontal_plane_direction.weight;

                // for all normals in the vec_horizontal_plane_direction_cluster
                bool exist = false;
                for (auto &j : vec_horizontal_plane_direction_cluster)
                {
                    Eigen::Vector3f cluster_vertical_normal = j.normal;
                    cluster_vertical_normal.normalize();

                    float dot_product = plane_vector_normal.dot(cluster_vertical_normal);
                    float angle = acos(dot_product);

                    // add and count exist one
                    if (angle < vertical_plane_normal_angle_th)
                    {
                        j.normal += weight * plane_vector_normal;
                        j.count += 1;
                        j.eigen_value += vec_horizontal_plane_direction.eigen_value;
                        exist = true;
                        break;
                    } else if (M_PI - angle < vertical_plane_normal_angle_th)
                    {
                        j.normal -= weight * plane_vector_normal;
                        j.count += 1;
                        j.eigen_value += vec_horizontal_plane_direction.eigen_value;
                        exist = true;
                        break;
                    }
                }

                // not exist in cluster, add a new one
                if (!exist)
                {
                    PlaneNormalCluster hplaneDirCluster;
                    hplaneDirCluster.normal = weight * plane_vector_normal;
                    hplaneDirCluster.count = 1;
                    hplaneDirCluster.eigen_value = vec_horizontal_plane_direction.eigen_value;
                    vec_horizontal_plane_direction_cluster.push_back(hplaneDirCluster);
                }
            }

            for (auto &i : vec_horizontal_plane_direction_cluster)
                i.normal.normalize();

            if (!vec_horizontal_plane_direction_cluster.empty())
            {
                // sort based on the counts
                std::sort(vec_horizontal_plane_direction_cluster.begin(), vec_horizontal_plane_direction_cluster.end(),
                          [&](PlaneNormalCluster a, PlaneNormalCluster b)
                          {
                              return a.count > b.count;
                          });

                PlaneNormalInfo horizontal_plane_direction;
                horizontal_plane_direction.plane_points_numbers = 0; // not use
                horizontal_plane_direction.normal = vec_horizontal_plane_direction_cluster[0].normal;
                horizontal_plane_direction.eigen_value = vec_horizontal_plane_direction_cluster[0].eigen_value;

                keyInfo.horizontal_plane_direction = horizontal_plane_direction;
            } else
            {
                PlaneNormalInfo empty_dir;
                keyInfo.horizontal_plane_direction = empty_dir;
            }
        }
    }

    /*
     * refine the key directions (in case there is no vertical plane be found)
     */
    bool StairDetection::refineKeyDirections(KeyInfo &keyInfo)
    {
        if (keyInfo.horizontal_plane_direction.normal.norm())
        {
            Eigen::Vector3f estimated_vertical_plane_normal;
            estimated_vertical_plane_normal = keyInfo.horizontal_plane_direction.normal.cross(
                    -1 * down_direction);
            estimated_vertical_plane_normal.normalize();

            // no vertical plane found, then we use horizontal_plane_direction to estimate the vertical_plane_normal
            if (keyInfo.main_vertical_plane_normal.normal.norm() == 0)
            {
                keyInfo.main_vertical_plane_normal.normal = estimated_vertical_plane_normal;
                keyInfo.main_vertical_plane_normal.plane_points_numbers = 1000;

            } else
            {
                float dot_product = keyInfo.main_vertical_plane_normal.normal.dot(estimated_vertical_plane_normal);
                float angle = acos(dot_product);

                // if there has a second_vertical_plane_normal
                if (keyInfo.second_vertical_plane_normal.normal.norm())
                {
                    // compare the two vertical plane normals with horizontal_plane_direction,
                    // choose the most match one to refine
                    float dot_product2 = keyInfo.second_vertical_plane_normal.normal.dot(
                            estimated_vertical_plane_normal);
                    float angle2 = acos(dot_product2);

                    if (angle2 < angle)
                    {
                        angle = angle2;
                        keyInfo.main_vertical_plane_normal = keyInfo.second_vertical_plane_normal;
                    }
                }

                // main_vertical_plane_normal should be close to estimated_vertical_plane_normal,
                // if the angle are two large, it is possibly the horizontal plane are be cut due to bad camera view
                if (fabs(angle) > vertical_plane_normal_est_angle_th)
                {
                    // so, we check the vertical_plane_normal count, if the vertical_plane_normal is computed by more than one
                    // vertical plane, then we should trust vertical_plane_normal more than estimated_vertical_plane_normal;
                    // if the vertical_plane_normal are only estimated from one plane, we should trust horizontal_plane_direction more
                    if (keyInfo.main_vertical_plane_normal.count >= 2)
                    {
                        keyInfo.horizontal_plane_direction.normal = keyInfo.main_vertical_plane_normal.normal.cross(
                                down_direction);
                        keyInfo.horizontal_plane_direction.normal.normalize();
                        keyInfo.horizontal_plane_direction.eigen_value = 0.5;
                    } else
                    {
                        keyInfo.main_vertical_plane_normal.normal = estimated_vertical_plane_normal;
                        keyInfo.main_vertical_plane_normal.plane_points_numbers = 1000;
                    }
                }
            }

//            ROS_INFO("11111111111111111111111111111");
//            keyInfo.main_vertical_plane_normal.normal[0]=0;
//            keyInfo.main_vertical_plane_normal.normal.normalize();
//            keyInfo.horizontal_plane_direction.normal[0]=0;
//            keyInfo.horizontal_plane_direction.normal.normalize();
//            ROS_INFO("22222222222222222222222222222");
            return true;
        }

        return false;
    }

    /*
     * find the dominate center diff vector in the current frame (and refine it using history info)
     */
    void StairDetection::findDominateCenterDiffVector(const std::vector<Plane> &vector_plane,
                                                      std::vector<std::vector<Relation>> &confuse_matrix,
                                                      KeyInfo &keyInfo)
    {
        // for all planes, compute delta_d
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            for (size_t j = i + 1; j < vector_plane.size(); j++)
            {
                confuse_matrix[i][j].delta_d = fabs(
                        keyInfo.main_vertical_plane_normal.normal.dot(confuse_matrix[i][j].center_direction));
            }
        }

        std::vector<PlaneNormalInfo> vec_main_center_diff_vectors;

        // history main_center_diff_vector
        if (!keyInfo.init)
        {
            if (keyInfo.main_center_diff_vector.normal.norm())
            {
                keyInfo.main_center_diff_vector.weight *= (1 - decay_rate);
                keyInfo.main_center_diff_vector.normal *= keyInfo.main_center_diff_vector.weight;
                vec_main_center_diff_vectors.push_back(keyInfo.main_center_diff_vector);
            }
        }

        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            if (vector_plane[i].ptype == Plane::pstair_component)
            {
                for (size_t j = i + 1; j < vector_plane.size(); j++)
                {
                    if (vector_plane[j].ptype == Plane::pstair_component)
                    {
                        Eigen::Vector3f normalized_cv_direction = confuse_matrix[i][j].center_direction;
                        normalized_cv_direction.normalize();

                        float dot_product, angle_with_horizontal_plane_direction;

                        dot_product = normalized_cv_direction.dot(keyInfo.horizontal_plane_direction.normal);
                        angle_with_horizontal_plane_direction = acos(dot_product);

                        dot_product = normalized_cv_direction.dot(keyInfo.main_vertical_plane_normal.normal);

                        // center vector direction angle with horizontal_plane_direction in 90 degrees threshold
                        // the center vector direction should be perpendicular with horizontal_plane_direction
                        if (fabs(M_PI / 2 - angle_with_horizontal_plane_direction) < cv_angle_horizontal_th)
                        {
                            // the main_vertical_plane_normal are point to -y, but the cv_direction should point to y
                            if (dot_product < 0)
                            {
                                PlaneNormalInfo cv_direction;
                                cv_direction.normal = confuse_matrix[i][j].center_direction;
                                cv_direction.weight = 1;
                                cv_direction.eigen_value = 0; // not use
                                cv_direction.plane_points_numbers = 1; //not use

                                vec_main_center_diff_vectors.push_back(cv_direction);
                            }
                        }
                    }
                }
            }
        }

        // cluster main center diff vector
        std::vector<PlaneNormalCluster> vec_cv_direction_clusters;
        for (auto &vec_main_center_diff_vector : vec_main_center_diff_vectors)
        {
            Eigen::Vector3f normalized_cv_direction = vec_main_center_diff_vector.normal;
            normalized_cv_direction.normalize();

            PlaneNormalCluster cv_direction_cluster;

            bool exist = false;
            for (auto &vec_cv_direction_cluster : vec_cv_direction_clusters)
            {
                Eigen::Vector3f normalized_cluster_direction = vec_cv_direction_cluster.normal;
                normalized_cluster_direction.normalize();

                float dot_product = normalized_cluster_direction.dot(normalized_cv_direction);
                float angle = acos(dot_product);

                // add and count exist one
                if (fabs(angle) < cv_dir_cluster_angle_th)
                {
                    vec_cv_direction_cluster.normal += vec_main_center_diff_vector.normal;
                    vec_cv_direction_cluster.count += 1;
                    vec_cv_direction_cluster.plane_points_numbers = 0; // not use
                    vec_cv_direction_cluster.eigen_value = 0; // not use
                    exist = true;
                    break;
                }
            }

            // not exist in cluster, add a new one
            if (!exist)
            {
                PlaneNormalCluster planeNormalCluster;
                planeNormalCluster.normal = vec_main_center_diff_vector.normal;
                planeNormalCluster.count = 1;
                planeNormalCluster.plane_points_numbers = 0; // not use
                planeNormalCluster.eigen_value = 0; // not use
                vec_cv_direction_clusters.push_back(planeNormalCluster);
            }
        }

        if (!vec_cv_direction_clusters.empty())
        {
            // sort based on the counts
            std::sort(vec_cv_direction_clusters.begin(), vec_cv_direction_clusters.end(),
                      [&](PlaneNormalCluster a, PlaneNormalCluster b)
                      {
                          return a.count > b.count;
                      });

            PlaneNormalInfo main_center_diff_vector;
            main_center_diff_vector.eigen_value = 0; // not use
            main_center_diff_vector.plane_points_numbers = 0; // not use
            main_center_diff_vector.weight = vec_cv_direction_clusters[0].count;
            main_center_diff_vector.normal = vec_cv_direction_clusters[0].normal;
            main_center_diff_vector.normal.normalize();

            // refine main_center_diff_vector using horizontal_plane_direction
            // the main_center_diff_vector should be prependiculr with horizontal_plane_direction
            // sometimes the main_center_diff_vector maybe deflect from correct direction
            // so we project current main_center_diff_vector to the plane of horizontal_plane_direction
            float prj_len = main_center_diff_vector.normal.dot(keyInfo.horizontal_plane_direction.normal);
            Eigen::Vector3f prj = prj_len * keyInfo.horizontal_plane_direction.normal;
            main_center_diff_vector.normal = main_center_diff_vector.normal - prj;
            main_center_diff_vector.normal.normalize();

            keyInfo.main_center_diff_vector = main_center_diff_vector;
        } else
        {
            PlaneNormalInfo empty_dir;
            keyInfo.main_center_diff_vector = empty_dir;
        }

    }

    /*
* mark others plane based on main_vertical_normals
*/
    void StairDetection::filterPlanesWithKeyInfo(std::vector<Plane> &vector_plane, KeyInfo &keyInfo)
    {
        for (auto &i : vector_plane)
        {
            if (i.type == Plane::vertical)
            {
                Eigen::Vector3f plane_normal;
                plane_normal[0] = i.coefficients.values[0];
                plane_normal[1] = i.coefficients.values[1];
                plane_normal[2] = i.coefficients.values[2];

                float dot_product = keyInfo.main_vertical_plane_normal.normal.dot(plane_normal);
                if (dot_product > 1)dot_product = 1;
                else if (dot_product < -1)dot_product = -1;
                float angle = acos(dot_product);

                if ((fabs(angle) < vertical_plane_normal_est_angle_th) ||
                    (fabs(M_PI - angle) < vertical_plane_normal_est_angle_th)) {}
                else
                {
                    i.ptype = Plane::others;
                    i.info = "{vertical_plane_normal}";
                }
            }
        }
    }

    /*
     * discard some planes which have unreasonable size
     */
    void StairDetection::filterPlanesWithSizeConstraint(std::vector<Plane> &vector_plane)
    {
        for (auto &i : vector_plane)
        {
            if (i.ptype == Plane::pstair_component)
            {
                // length constraint
                if (i.max[2] - i.min[2] < plane_min_length)
                {
                    i.ptype = Plane::others;
                    i.info = "{length < min_length}";
                } else if (i.max[2] - i.min[2] > plane_max_length)
                {
                    i.ptype = Plane::others;
                    i.info = "{length > max_length}";
                }

                // width constraint

                if (i.max[1] - i.min[1] < plane_min_width)
                {
                    i.ptype = Plane::others;
                    i.info = "{width < min_width}";
                } else if (i.max[1] - i.min[1] > plane_max_width)
                {
                    i.ptype = Plane::others;
                    i.info = "{width > max_width}";
                }
            }
        }
    }

    /*
     * find all the connect plane fragments
     */
    void StairDetection::findConnectFragment(std::vector<Plane> &vector_plane,
                                             const std::vector<std::vector<Relation>> &confuse_matrix,
                                             KeyInfo &keyInfo, std::vector<int2> &fragment_vector)
    {
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            for (size_t j = i + 1; j < vector_plane.size(); j++)
            {
                if (vector_plane[i].ptype == Plane::pstair_component &&
                    vector_plane[j].ptype == Plane::pstair_component)
                {
                    Eigen::Vector3f normal_i, normal_j;
                    normal_i[0] = vector_plane[i].coefficients.values[0];
                    normal_i[1] = vector_plane[i].coefficients.values[1];
                    normal_i[2] = vector_plane[i].coefficients.values[2];
                    normal_j[0] = vector_plane[j].coefficients.values[0];
                    normal_j[1] = vector_plane[j].coefficients.values[1];
                    normal_j[2] = vector_plane[j].coefficients.values[2];

                    Eigen::Vector3f dir = confuse_matrix[i][j].center_direction;
                    dir.normalize();

                    // the angle between the two plane center vector and main_center_diff_vector
                    float dot_product = keyInfo.main_center_diff_vector.normal.dot(dir);
                    float angle = acos(dot_product);

                    // TODO: stair_cv_angle_diff_th and stair_plane_max_distance
                    if ((angle < stair_cv_angle_diff_th) && (confuse_matrix[i][j].distance < stair_plane_max_distance)
                        && (confuse_matrix[i][j].delta_h < stair_max_height) &&
                        (confuse_matrix[i][j].delta_d < stair_max_width))
                    {
                        int2 ft{};
                        ft.from = i;
                        ft.to = j;
                        fragment_vector.push_back(ft);
                    }
                }
            }
        }

        int i = 0;
    }

    /*
     * find (or stitch) the longest from the connect plane fragments
     */
    void StairDetection::findStairPlaneIndexList(std::vector<int2> &fragment_vector,
                                                 std::vector<int> &stair_plane_index_list)
    {
        if (fragment_vector.size() > 1)
        {
            std::vector<std::vector<int>> vector_index_list;
            int last_from = -1;
            for (size_t i = 0; i < fragment_vector.size(); i++)
            {
                if (fragment_vector[i].from != last_from)
                {
                    std::vector<std::vector<int>> vector_index_list_i;
                    std::vector<int> start_index_list;
                    start_index_list.push_back(-1); // -1 is a virtual point, for easy logic in findFragments
                    start_index_list.push_back(fragment_vector[i].from); // start from fragment_vector[i].from
                    vector_index_list_i.push_back(start_index_list);

                    findFragments(fragment_vector, i - 1, vector_index_list_i, 0);

                    vector_index_list.insert(vector_index_list.end(), vector_index_list_i.begin(),
                                             vector_index_list_i.end());
                }
                last_from = fragment_vector[i].from;
            }

            for (auto &i : vector_index_list)
                i.erase(i.begin());

            sort(vector_index_list.begin(), vector_index_list.end(),
                 [&](std::vector<int> a, std::vector<int> b)
                 {
                     return a.size() > b.size();
                 });

            stair_plane_index_list = vector_index_list[0];
        } else if (fragment_vector.size() == 1)
        {
            stair_plane_index_list.push_back(fragment_vector[0].from);
            stair_plane_index_list.push_back(fragment_vector[0].to);
        } else
            return;
    }

    void StairDetection::findFragments(std::vector<int2> &fragment_vector, int index,
                                       std::vector<std::vector<int>> &vector_index_list, int vec_index)
    {
        // destination
        int start_from = vector_index_list[vec_index][vector_index_list[vec_index].size() - 2];
        int start_to = vector_index_list[vec_index][vector_index_list[vec_index].size() - 1];
        std::vector<int> bak_index_list;

        bool first = true;
        for (int i = index + 1; i < fragment_vector.size(); i++)
        {
            // start_from -> start_to , find a connect list, add to the current list
            if (fragment_vector[i].from == start_to)
            {
                if (first)
                {
                    first = false;

                    // backup the list before insert a new data, for second use
                    bak_index_list = vector_index_list[vec_index];

                    vector_index_list[vec_index].push_back(fragment_vector[i].to);

                    if (i < fragment_vector.size() - 1)
                        findFragments(fragment_vector, i, vector_index_list, vec_index);
                } else
                {
                    bak_index_list.push_back(fragment_vector[i].to);
                    vector_index_list.push_back(bak_index_list);
                    bak_index_list.pop_back();

                    if (i < fragment_vector.size() - 1)
                        findFragments(fragment_vector, i, vector_index_list, vector_index_list.size() - 1);
                }
            }
                // from > start_to means that we should break,
                // because the list is non decrease
            else if (fragment_vector[i].from > start_to)
            {
                break;
            }
        }
    }

    /*
     * mark plane and TODO: refine the main vertical normal?
     */
    void StairDetection::filterPlanesWithStairPlaneList(std::vector<Plane> &vector_plane,
                                                        std::vector<int> &stair_plane_index_list)
    {
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            if (vector_plane[i].ptype == Plane::pstair_component)
            {
                vector_plane[i].ptype = Plane::others;

                vector_plane[i].info = "{not in stair_plane_index_list}";

                for (int j : stair_plane_index_list)
                {
                    if (i == j)
                    {
                        vector_plane[i].info = "";
                        vector_plane[i].counter.points.resize(4);
                        vector_plane[i].counter.height = 1;
                        vector_plane[i].ptype = Plane::stair_component;
                        break;
                    }
                }
            }
        }
    }

    /*
     * compute stair side plane
     */
    void StairDetection::computeStairSidePlane(std::vector<Plane> &vector_plane, KeyInfo &keyInfo)
    {
        std::vector<float> vec_left_intercepts;
        std::vector<float> vec_right_intercepts;

        float cluster_intercept_threshold = 0.1;

        if (!keyInfo.init)
        {
            if (!keyInfo.side_plane_left.values.empty() && decay_rate < 0.99)
            {
                vec_left_intercepts.push_back(keyInfo.side_plane_left.values[3]);
                vec_right_intercepts.push_back(keyInfo.side_plane_right.values[3]);
            }
        }

        for (auto &i : vector_plane)
        {
            if (i.ptype == Plane::stair_component)
            {
                PointType min_point, max_point;
                float min_proj, max_proj;

                findMinMaxProjPoint(i, keyInfo.horizontal_plane_direction.normal,
                                    min_point, max_point, min_proj, max_proj);

                vec_right_intercepts.push_back(-max_proj);
                vec_left_intercepts.push_back(-min_proj);
            }
        }

        /*sort(vec_right_intercepts.begin(), vec_right_intercepts.end(),
             [&](float a, float b)
             {
                 return fabs(a) > fabs(b);
             });

        sort(vec_left_intercepts.begin(), vec_left_intercepts.end(),
             [&](float a, float b)
             {
                 return fabs(a) > fabs(b);
             });

        keyInfo.side_plane_left.values.resize(4);
        keyInfo.side_plane_left.values[0] = keyInfo.horizontal_plane_direction.normal[0];
        keyInfo.side_plane_left.values[1] = keyInfo.horizontal_plane_direction.normal[1];
        keyInfo.side_plane_left.values[2] = keyInfo.horizontal_plane_direction.normal[2];
        keyInfo.side_plane_left.values[3] = vec_left_intercepts[0];

        keyInfo.side_plane_right.values.resize(4);
        keyInfo.side_plane_right.values[0] = keyInfo.horizontal_plane_direction.normal[0];
        keyInfo.side_plane_right.values[1] = keyInfo.horizontal_plane_direction.normal[1];
        keyInfo.side_plane_right.values[2] = keyInfo.horizontal_plane_direction.normal[2];
        keyInfo.side_plane_right.values[3] = vec_right_intercepts[0];*/

        struct InterceptCount
        {
            float intercept;
            int count;
        };

        if (!vec_right_intercepts.empty())
        {
            std::vector<InterceptCount> vec_left_intercept_counts, vec_right_intercept_counts;

            for (float vec_left_intercept : vec_left_intercepts)
            {
                bool exist = false;
                for (auto &vec_left_intercept_count : vec_left_intercept_counts)
                {
                    if (fabs(vec_left_intercept - vec_left_intercept_count.intercept) <
                        cluster_intercept_threshold)
                    {
                        exist = true;
                        vec_left_intercept_count.intercept =
                                (vec_left_intercept_count.intercept * vec_left_intercept_count.count
                                 + vec_left_intercept) / (vec_left_intercept_count.count + 1);
                        vec_left_intercept_count.count++;
                    }
                }

                if (!exist)
                {
                    InterceptCount newInterceptCount{};
                    newInterceptCount.intercept = vec_left_intercept;
                    newInterceptCount.count = 1;
                    vec_left_intercept_counts.push_back(newInterceptCount);
                }
            }

            for (float vec_right_intercept : vec_right_intercepts)
            {
                bool exist = false;
                for (auto &vec_right_intercept_count : vec_right_intercept_counts)
                {
                    if (fabs(vec_right_intercept - vec_right_intercept_count.intercept) <
                        cluster_intercept_threshold)
                    {
                        exist = true;
                        vec_right_intercept_count.intercept =
                                (vec_right_intercept_count.intercept * vec_right_intercept_count.count
                                 + vec_right_intercept) / (vec_right_intercept_count.count + 1);
                        vec_right_intercept_count.count++;
                    }
                }

                if (!exist)
                {
                    InterceptCount newInterceptCount{};
                    newInterceptCount.intercept = vec_right_intercept;
                    newInterceptCount.count = 1;
                    vec_right_intercept_counts.push_back(newInterceptCount);
                }
            }

            sort(vec_left_intercept_counts.begin(), vec_left_intercept_counts.end(),
                 [&](InterceptCount a, InterceptCount b)
                 {
                     return a.count > b.count;
                 });

            sort(vec_right_intercept_counts.begin(), vec_right_intercept_counts.end(),
                 [&](InterceptCount a, InterceptCount b)
                 {
                     return a.count > b.count;
                 });

            if (vec_left_intercept_counts.size() > 1)
                if (vec_left_intercept_counts[0].count == vec_left_intercept_counts[1].count)
                    if (abs(vec_left_intercept_counts[0].intercept) < abs(vec_left_intercept_counts[1].intercept))
                        swap(vec_left_intercept_counts[0], vec_left_intercept_counts[1]);

            if (vec_right_intercept_counts.size() > 1)
                if (vec_right_intercept_counts[0].count == vec_right_intercept_counts[1].count)
                    if (abs(vec_right_intercept_counts[0].intercept) < abs(vec_right_intercept_counts[1].intercept))
                        swap(vec_right_intercept_counts[0], vec_right_intercept_counts[1]);

            keyInfo.side_plane_left.values.resize(4);
            keyInfo.side_plane_left.values[0] = keyInfo.horizontal_plane_direction.normal[0];
            keyInfo.side_plane_left.values[1] = keyInfo.horizontal_plane_direction.normal[1];
            keyInfo.side_plane_left.values[2] = keyInfo.horizontal_plane_direction.normal[2];
            keyInfo.side_plane_left.values[3] = vec_left_intercept_counts[0].intercept;

            keyInfo.side_plane_right.values.resize(4);
            keyInfo.side_plane_right.values[0] = keyInfo.horizontal_plane_direction.normal[0];
            keyInfo.side_plane_right.values[1] = keyInfo.horizontal_plane_direction.normal[1];
            keyInfo.side_plane_right.values[2] = keyInfo.horizontal_plane_direction.normal[2];
            keyInfo.side_plane_right.values[3] = vec_right_intercept_counts[0].intercept;
        }
    }

    /*
     * modeling stair
     */
    bool StairDetection::modelingStair(std::vector<Plane> &vector_plane, std::vector<int> &stair_plane_index_list,
                                       KeyInfo &keyInfo, Stair &stair)
    {
        std::shared_ptr<Node> p_concaveline = nullptr;
        std::shared_ptr<Node> p_step = nullptr;

        std::shared_ptr<Node> prev_step = nullptr;
        std::shared_ptr<Node> prev_concave_line = nullptr;

        if (stair_plane_index_list.empty())
            return false;

        // has a ground plane?
        bool find_ground = false;
        int gound_index = -1;
        for (size_t i = 0; i < vector_plane.size(); i++)
        {
            if (vector_plane[i].ptype == Plane::ground)
            {
                find_ground = true;
                gound_index = i;
                vector_plane[i].counter.resize(4);
                vector_plane[i].counter.height = 1;
                vector_plane[i].counter.width = 0;
                break;
            }
        }

        int count = 0;
        for (size_t i = 0; i < stair_plane_index_list.size() - 1; i++)
        {
            int index = stair_plane_index_list[i];
            int index_next = stair_plane_index_list[i + 1];

            // has a ground plane and i==0
            if (find_ground && !i)
            {
                // first step is groud to horizontal
                if (vector_plane[index].type == Plane::horizontal)
                {
                    Eigen::Vector3f center_diff(vector_plane[gound_index].center.x - vector_plane[index].center.x,
                                                vector_plane[gound_index].center.y - vector_plane[index].center.y,
                                                vector_plane[gound_index].center.z - vector_plane[index].center.z);
                    float dot_product = center_diff.dot(down_direction);

                    if (fabs(dot_product) < 2.5 * ground_height_th)
                    {
                        Plane virtual_virtcal_plane;
                        PointType point_h1, tmp;
                        float min_p, max_p;

                        /********************* virtual_virtcal_plane ********************/

                        // find the front point of the upper plane
                        findMinMaxProjPoint(vector_plane[index], keyInfo.main_vertical_plane_normal.normal,
                                            tmp, point_h1, min_p, max_p);

                        // "center"
                        virtual_virtcal_plane.center.x = point_h1.x;
                        virtual_virtcal_plane.center.y = point_h1.y;
                        virtual_virtcal_plane.center.z = point_h1.z;

                        // compute the coefficients of virtual vertical plane
                        float d = -point_h1.x * keyInfo.main_vertical_plane_normal.normal[0] -
                                  point_h1.y * keyInfo.main_vertical_plane_normal.normal[1] -
                                  point_h1.z * keyInfo.main_vertical_plane_normal.normal[2];

                        virtual_virtcal_plane.coefficients.values.push_back(
                                keyInfo.main_vertical_plane_normal.normal[0]);
                        virtual_virtcal_plane.coefficients.values.push_back(
                                keyInfo.main_vertical_plane_normal.normal[1]);
                        virtual_virtcal_plane.coefficients.values.push_back(
                                keyInfo.main_vertical_plane_normal.normal[2]);
                        virtual_virtcal_plane.coefficients.values.push_back(d);
                        // get the min max point
                        virtual_virtcal_plane.points_min[1] = point_h1;
                        virtual_virtcal_plane.points_max[1] = point_h1;

                        virtual_virtcal_plane.counter.points.resize(4);
                        virtual_virtcal_plane.counter.width = 0;
                        virtual_virtcal_plane.counter.height = 1;

                        // push back the virtual_virtcal_plane to vector_plane
                        vector_plane.push_back(virtual_virtcal_plane);

                        // construct a node object, set the type with concaveline_node
                        p_concaveline = std::make_shared<Node>();
                        p_concaveline->pcurrent_type = PartType::concaveline_node;

                        Line line1;
                        pcl::ModelCoefficients plane1_coefficients, plane2_coefficients;

                        plane1_coefficients = vector_plane[gound_index].coefficients;
                        plane2_coefficients = virtual_virtcal_plane.coefficients;
                        computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line1);

                        p_concaveline->concave_line.plane_h = &vector_plane[gound_index];
                        p_concaveline->concave_line.plane_v = &vector_plane[vector_plane.size() - 1];
                        p_concaveline->concave_line.line = line1;

                        stair.pushBack(p_concaveline);

                        /////////////////////Step
                        // construct a node object, set the type with step_node
                        p_step = std::make_shared<Node>();
                        p_step->pcurrent_type = PartType::step_node;

                        Line line2;
                        plane1_coefficients = vector_plane[index].coefficients;
                        computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line2);

                        p_step->step.plane_v = &vector_plane[vector_plane.size() - 1];
                        p_step->step.plane_h = &vector_plane[index];
                        p_step->step.line = line2;
                        p_step->step.count = ++count;

//                        p_step->step.height = fabs(dot_product);

                        p_step->step.height = fabs(p_step->step.line.h - p_concaveline->concave_line.line.h);
                        p_step->step.good_h = true;

                        stair.pushBack(p_step);

                        prev_step = stair.getCurPoint();
                    }
                } else if (vector_plane[index].type == Plane::vertical)
                {
                    Eigen::Vector3f center_diff(vector_plane[gound_index].center.x - vector_plane[index].center.x,
                                                vector_plane[gound_index].center.y - vector_plane[index].center.y,
                                                vector_plane[gound_index].center.z - vector_plane[index].center.z);
                    float dot_product = center_diff.dot(down_direction);

                    if (fabs(dot_product) < 1.6 * ground_height_th)
                    {
                        // construct a node object, set the type with concaveline_node
                        p_concaveline = std::make_shared<Node>();
                        p_concaveline->pcurrent_type = PartType::concaveline_node;

                        Line line1;
                        pcl::ModelCoefficients plane1_coefficients, plane2_coefficients;

                        plane1_coefficients = vector_plane[gound_index].coefficients;
                        plane2_coefficients = vector_plane[index].coefficients;
                        computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line1);

                        p_concaveline->concave_line.plane_h = &vector_plane[gound_index];
                        p_concaveline->concave_line.plane_v = &vector_plane[index];
                        p_concaveline->concave_line.line = line1;

                        stair.pushBack(p_concaveline);

                        prev_step = stair.getCurPoint();
                    }
                }
            }


            if ((vector_plane[index].type == Plane::horizontal)
                && ((vector_plane[index_next].type == Plane::vertical)))
            {
                // construct a node object, set the type with concaveline_node
                p_concaveline = std::make_shared<Node>();
                p_concaveline->pcurrent_type = PartType::concaveline_node;

                Line line;
                pcl::ModelCoefficients plane1_coefficients, plane2_coefficients;

                plane1_coefficients = vector_plane[index].coefficients;
                plane2_coefficients = vector_plane[index_next].coefficients;

                computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line);

                p_concaveline->concave_line.plane_h = &vector_plane[index];
                p_concaveline->concave_line.plane_v = &vector_plane[index_next];
                p_concaveline->concave_line.line = line;

                if (prev_step)
                {
//                    PointType &prev_center = prev_step->step.plane_v->center;
//                    PointType &now_center = p_concaveline->concave_line.plane_v->center;
//
//                    Eigen::Vector3f diff(now_center.x - prev_center.x,
//                                         now_center.y - prev_center.y,
//                                         now_center.z - prev_center.z);
//
//                    prev_step->step.depth = fabs(diff.dot(keyInfo.main_vertical_plane_normal.normal));
                    prev_step->step.depth = fabs(line.d - prev_step->step.line.d);
                    prev_step->step.good_d = true;
                }

                stair.pushBack(p_concaveline);

                prev_concave_line = stair.getCurPoint();
            } else if ((vector_plane[index].type == Plane::vertical)
                       && ((vector_plane[index_next].type == Plane::horizontal)))
            {
                // construct a node object, set the type with step_node
                p_step = std::make_shared<Node>();
                p_step->pcurrent_type = PartType::step_node;

                Line line;
                pcl::ModelCoefficients plane1_coefficients, plane2_coefficients;

                plane1_coefficients = vector_plane[index].coefficients;
                plane2_coefficients = vector_plane[index_next].coefficients;

                computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line);

                p_step->step.plane_v = &vector_plane[index];
                p_step->step.plane_h = &vector_plane[index_next];
                p_step->step.line = line;
                p_step->step.count = ++count;

                if (prev_concave_line)
                {
//                    PointType &prev_center = prev_concave_line->concave_line.plane_h->center;
//                    PointType &now_center = p_step->step.plane_h->center;
//
//                    Eigen::Vector3f diff(now_center.x - prev_center.x,
//                                         now_center.y - prev_center.y,
//                                         now_center.z - prev_center.z);
//
//                    p_step->step.height = fabs(diff.dot(down_direction));

                    p_step->step.height = fabs(line.h - prev_concave_line->concave_line.line.h);
                } else
                {
                    PointType point_max, point_min;
                    float min_p, max_p;
                    findMinMaxProjPoint(vector_plane[index], down_direction, point_min, point_max, min_p, max_p);
                    p_step->step.height = fabs(point_max.x - line.h);
                    p_step->step.good_h = false;
//                    Eigen::Vector3f diff(point_min.x - point_max.x,
//                                         point_min.y - point_max.y,
//                                         point_min.z - point_max.z);
//
//                    p_step->step.height = fabs(diff.dot(down_direction));
                }

                stair.pushBack(p_step);

                prev_step = stair.getCurPoint();
            } else if ((vector_plane[index].type == Plane::horizontal)
                       && ((vector_plane[index_next].type == Plane::horizontal)))
            {
                Plane virtual_virtcal_plane;
                PointType point_h0, point_h1, tmp;
                float min_p, max_p;

                // find the back point of the lower plane
                findMinMaxProjPoint(vector_plane[index], keyInfo.main_vertical_plane_normal.normal, point_h0, tmp,
                                    min_p, max_p);

                // find the front point of the upper plane
                findMinMaxProjPoint(vector_plane[index_next], keyInfo.main_vertical_plane_normal.normal, tmp, point_h1,
                                    min_p, max_p);

                // compute center point of virtual vertical plane
                float cx, cy, cz;
                cx = (point_h0.x + point_h1.x) / 2;
                cy = (point_h0.y + point_h1.y) / 2;
                cz = (point_h0.z + point_h1.z) / 2;

                virtual_virtcal_plane.center.x = cx;
                virtual_virtcal_plane.center.y = cy;
                virtual_virtcal_plane.center.z = cz;

                // compute the coefficients of virtual vertical plane
                float d = -cx * keyInfo.main_vertical_plane_normal.normal[0] -
                          cy * keyInfo.main_vertical_plane_normal.normal[1] -
                          cz * keyInfo.main_vertical_plane_normal.normal[2];
                virtual_virtcal_plane.coefficients.values.push_back(keyInfo.main_vertical_plane_normal.normal[0]);
                virtual_virtcal_plane.coefficients.values.push_back(keyInfo.main_vertical_plane_normal.normal[1]);
                virtual_virtcal_plane.coefficients.values.push_back(keyInfo.main_vertical_plane_normal.normal[2]);
                virtual_virtcal_plane.coefficients.values.push_back(d);
                // get the min max point
                virtual_virtcal_plane.points_min[1] = point_h0;
                virtual_virtcal_plane.points_max[1] = point_h1;

                virtual_virtcal_plane.counter.points.resize(4);
                virtual_virtcal_plane.counter.width = 0;
                virtual_virtcal_plane.counter.height = 1;

                // push back the virtual_virtcal_plane to vector_plane
                vector_plane.push_back(virtual_virtcal_plane);

                // construct a node object, set the type with concaveline_node
                p_concaveline = std::make_shared<Node>();
                p_concaveline->pcurrent_type = PartType::concaveline_node;

                Line line1;
                pcl::ModelCoefficients plane1_coefficients, plane2_coefficients;

                plane1_coefficients = vector_plane[index].coefficients;
                plane2_coefficients = virtual_virtcal_plane.coefficients;
                computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line1);

                p_concaveline->concave_line.plane_h = &vector_plane[index];
                p_concaveline->concave_line.plane_v = &vector_plane[vector_plane.size() - 1];
                p_concaveline->concave_line.line = line1;

                if (prev_step)
                {
//                    PointType &prev_center = prev_step->step.plane_v->center;
//                    PointType &now_center = p_concaveline->concave_line.plane_v->center;
//
//                    Eigen::Vector3f diff(now_center.x - prev_center.x,
//                                         now_center.y - prev_center.y,
//                                         now_center.z - prev_center.z);
//
//                    prev_step->step.depth = fabs(diff.dot(keyInfo.main_vertical_plane_normal.normal));
                    prev_step->step.depth = fabs(line1.d - prev_step->step.line.d);
                    prev_step->step.good_d = true;
                }

                stair.pushBack(p_concaveline);
                prev_concave_line = stair.getCurPoint();

                /////////////////////Step
                // construct a node object, set the type with step_node
                p_step = std::make_shared<Node>();
                p_step->pcurrent_type = PartType::step_node;

                Line line2;
                plane1_coefficients = vector_plane[index_next].coefficients;
                computeLineFrom2Planes(plane1_coefficients, plane2_coefficients, keyInfo, line2);

                p_step->step.plane_v = &vector_plane[vector_plane.size() - 1];
                p_step->step.plane_h = &vector_plane[index_next];
                p_step->step.line = line2;
                p_step->step.count = ++count;

                if (prev_concave_line)
                {
//                    PointType &prev_center = prev_concave_line->concave_line.plane_h->center;
//                    PointType &now_center = p_step->step.plane_h->center;
//
//                    Eigen::Vector3f diff(now_center.x - prev_center.x,
//                                         now_center.y - prev_center.y,
//                                         now_center.z - prev_center.z);
//
//                    p_step->step.height = fabs(diff.dot(down_direction));
                    p_step->step.height = fabs(line2.h - prev_concave_line->concave_line.line.h);
                    p_step->step.good_h = true;
                } else
                {
                    // show not come here
                }

                stair.pushBack(p_step);

                prev_step = stair.getCurPoint();
            }
        }
        stair.pushBackEnd();

        return count > 0;
    }

    /** \brief find the point whose projection in on vector is max and min
    * \param[in] plane: include input cloud
    * \param[in] vector: (vx,vy,vz),the projection direction
    * \param[out] point: max_point and min_point
    */
    inline void StairDetection::findMinMaxProjPoint(const Plane &plane, const Eigen::Vector3f &ve,
                                                    PointType &min_point, PointType &max_point,
                                                    float &min_proj, float &max_proj)
    {
        max_proj = -FLT_MAX, min_proj = FLT_MAX;
        for (size_t i = 0; i < plane.random_down_sample_cloud.points.size(); i++)
        {
            pcl::PointXYZ center = plane.center;
            Eigen::Vector3f vp(plane.random_down_sample_cloud.points[i].x/* - center.x*/,
                               plane.random_down_sample_cloud.points[i].y/* - center.y*/,
                               plane.random_down_sample_cloud.points[i].z/* - center.z*/);
            float proj = ve.dot(vp);
            if (proj > max_proj)
            {
                max_proj = proj;
                max_point.x = plane.random_down_sample_cloud.points[i].x;
                max_point.y = plane.random_down_sample_cloud.points[i].y;
                max_point.z = plane.random_down_sample_cloud.points[i].z;
            }
            if (proj < min_proj)
            {
                min_proj = proj;
                min_point.x = plane.random_down_sample_cloud.points[i].x;
                min_point.y = plane.random_down_sample_cloud.points[i].y;
                min_point.z = plane.random_down_sample_cloud.points[i].z;
            }
        }
    }


    std::string StairDetection::getDetialStairModelString(Stair &stair)
    {
//        std::string str;
//
//        str = "Stair Model:\n";
//
//        std::shared_ptr<Node> pnext;
//        stair.reset();
//        while (stair.readNext(pnext))
//        {
//            if (pnext->pcurrent_type == PartType::step_node)
//            {
//                std::shared_ptr<Node> &pstep = pnext;
//                str = str + "\n";
//                str = str + "[STEP" + std::to_string(pstep->step.count) + "]:\n";
//                str = str + "    point cloud vertical" + "\n";
//                str = str + "        points number: " + std::to_string(pstep->step.plane_v->cloud.points.size()) + "\n";
//                str = str + "        height: " + std::to_string(pstep->step.height) + "\n";
//                str = str + "    point cloud horizontal" + "\n";
//                str = str + "        points number: " + std::to_string(pstep->step.plane_h->cloud.points.size()) + "\n";
//                str = str + "        depth:  " + std::to_string(pstep->step.depth) + "\n";
//                str = str + "    convex line: " + "\n";
//                str = str + "        " + "coeff: ";
//                for (float value : pstep->step.line.coeff.values)
//                    str = str + std::to_string(value) + "   ";
//                str = str + "\n" + "        h:" + std::to_string(pstep->step.line.h);
//                str = str + "\n" + "        d:" + std::to_string(pstep->step.line.d);
//                str = str + "\n";
//            } else if (pnext->pcurrent_type == PartType::concaveline_node)
//            {
//                std::shared_ptr<Node> &pline = pnext;
//                str = str + "\n";
//                str = str + "[CONCAVE LINE]: " + "\n";
//                str = str + "        " + "coeff: ";
//                for (float value : pline->concave_line.line.coeff.values)
//                    str = str + std::to_string(value) + "   ";
//                str = str + "\n" + "        h: " + std::to_string(pline->concave_line.line.h);
//                str = str + "\n" + "        d: " + std::to_string(pline->concave_line.line.d);
//                str = str + "\n";
//            }
//        }
//
//        return str;

        std::stringstream oss;
        oss.setf(ios::fixed);
        oss.precision(4);

        std::shared_ptr<Node> pnext;
        stair.reset();
        oss << "Stair Model:" << std::endl;

        while (stair.readNext(pnext))
        {
            if (pnext->pcurrent_type == PartType::step_node)
            {
                std::shared_ptr<Node> &pstep = pnext;
                oss << std::endl;
                oss << "[STEP" << pstep->step.count << "]:" << std::endl;
                oss << "    point cloud vertical" << std::endl;
                oss << "        points number: " << pstep->step.plane_v->cloud.points.size() << std::endl;
                oss << "        height: " /*<< setprecision(4)*/ << pstep->step.height << std::endl;
                oss << "    point cloud horizontal" << std::endl;
                oss << "        points number: " << pstep->step.plane_h->cloud.points.size() << std::endl;
                oss << "        depth:  " /*<< setprecision(4)*/ << pstep->step.depth << std::endl;
                oss << "    convex line: " << std::endl;
                oss << "        " << "coeff: ";
                for (float value : pstep->step.line.coeff.values)
                    oss /*<< setprecision(4)*/ << value << "   ";
                oss << std::endl << "        h:" /*<< setprecision(4)*/ << pstep->step.line.h;
                oss << std::endl << "        d:" /*<< setprecision(4)*/ << pstep->step.line.d;
                oss << std::endl;
            } else if (pnext->pcurrent_type == PartType::concaveline_node)
            {
                std::shared_ptr<Node> &pline = pnext;
                oss << std::endl;
                oss << "[CONCAVE LINE]: " << std::endl;
                oss << "        " << "coeff: ";
                for (float value : pline->concave_line.line.coeff.values)
                    oss << value << "   ";
                oss << std::endl << "        h: " /*<< setprecision(4)*/ << pline->concave_line.line.h;
                oss << std::endl << "        d: " /*<< setprecision(4)*/ << pline->concave_line.line.d;
                oss << std::endl;
            }
        }
        std::string str = oss.str();

        return str;
    }

    std::string StairDetection::getEstimatedParamString(Stair &stair)
    {
//        std::string str;
//
//        std::shared_ptr<Node> pnext;
//        stair.reset();
//
//        str = "Estimated Paramters:\n";
//        str = str + "Height    Depth    V_Distance  H_Distance" + "\n";
//        while (stair.readNext(pnext))
//        {
//            if (pnext->pcurrent_type == step_node)
//            {
//                std::shared_ptr<Node> &pstep = pnext;
//                str = str + std::to_string(pstep->step.height);
//                if (pstep->step.good_h)
//                    str = str + "g   ";
//                else
//                    str = str + "b   ";
//                str = str + std::to_string(pstep->step.depth);
//                if (pstep->step.good_d)
//                    str = str + "g   ";
//                else
//                    str = str + "b   ";
//                str = str + std::to_string(pstep->step.line.h) + "      " + std::to_string(pstep->step.line.d) + "\n";
//            }
//        }
//
//        return str;

        std::stringstream oss;
        oss.setf(ios::fixed);
        oss.precision(4);

        std::shared_ptr<Node> pnext;
        stair.reset();

        oss << "Estimated Paramters:" << std::endl;
        oss << "Height    Depth    V_Distance  H_Distance" << std::endl;
        while (stair.readNext(pnext))
        {
            if (pnext->pcurrent_type == step_node)
            {
                std::shared_ptr<Node> &pstep = pnext;
                oss /*<< setprecision(4)*/ << pstep->step.height;
                if (pstep->step.good_h)
                    oss << "g   ";
                else
                    oss << "b   ";
                oss /*<< setprecision(4)*/ << pstep->step.depth;
                if (pstep->step.good_d)
                    oss << "g   ";
                else
                    oss << "b   ";
                oss /*<< setprecision(4)*/ << pstep->step.line.h << "      "
                                           /*<< setprecision(4)*/ << pstep->step.line.d << std::endl;
            } else if (pnext->pcurrent_type == concaveline_node)
            {
                std::shared_ptr<Node> &pline = pnext;
//                oss << pline->concave_line.line.h << "\t" << pline->concave_line.line.d << std::endl;
            }
        }

        std::string str = oss.str();

        return str;
    }

    std::string StairDetection::getEstimatedParamStringRcd(Stair &stair)
    {
        std::stringstream oss;
        oss.setf(ios::fixed);
        oss.precision(4);

        std::shared_ptr<Node> pnext;
        stair.reset();

        bool first = true;
        while (stair.readNext(pnext))
        {
            if (pnext->pcurrent_type == step_node)
            {
                std::shared_ptr<Node> &pstep = pnext;
                if (first)
                    first = false;
                else
                    oss << ",";
                oss /*<< setprecision(4)*/ << pstep->step.height << ","
                                           /*<< setprecision(4)*/ << pstep->step.depth << ","
                                           /*<< setprecision(4)*/ << pstep->step.line.h << ","
                                           /*<< setprecision(4)*/ << pstep->step.line.d << std::endl;
            } else if (pnext->pcurrent_type == concaveline_node)
            {
                std::shared_ptr<Node> &pline = pnext;
//                oss << pline->concave_line.line.h << "\t" << pline->concave_line.line.d << std::endl;
            }
        }

        std::string str = oss.str();

        return str;
    }


/** \brief compute cross line of two planes
    * \param[in] coefficients_plane1: input plane coefficients
    * \param[in] coefficients_plane2: input plane coefficients
    * \param[out] coefficients_line: output line coefficients
    */
    void StairDetection::computeLineFrom2Planes(
            const pcl::ModelCoefficients &coefficients_plane1,
            const pcl::ModelCoefficients &coefficients_plane2,
            KeyInfo keyInfo,
            Line &line)
    {
        float a1, b1, c1, d1, a2, b2, c2, d2;
        a1 = coefficients_plane1.values[0];
        a2 = coefficients_plane2.values[0];
        b1 = coefficients_plane1.values[1];
        b2 = coefficients_plane2.values[1];
        c1 = coefficients_plane1.values[2];
        c2 = coefficients_plane2.values[2];
        d1 = coefficients_plane1.values[3];
        d2 = coefficients_plane2.values[3];

        // compute line normal
        Eigen::Vector3f normal1, normal2, normal_line;
        normal1 = Eigen::Vector3f(a1, b1, c1);
        normal2 = Eigen::Vector3f(a2, b2, c2);

        normal_line = normal1.cross(normal2);

        // compute line point (in xoy plane)
        float x0 = (b2 * d1 - b1 * d2) / (b1 * a2 - b2 * a1);
        float y0 = (a2 * d1 - a1 * d2) / (a1 * b2 - a2 * b1);
        float z0 = 0;

        // line : (x-x0)/n1=(y-y0)/n2=(z-z0)/n3=t
        // plane: x0*n1+y0*n2+z0*n3=0
        // compute cross point between line and plane perpendicular to this line with in origin
        float t = -1 * (normal_line[0] * x0 + normal_line[1] * y0 + normal_line[2] * z0)
                  /
                  (normal_line[0] * normal_line[0] + normal_line[1] * normal_line[1] + normal_line[2] * normal_line[2]);

        float x = normal_line[0] * t + x0;
        float y = normal_line[1] * t + y0;
        float z = normal_line[2] * t + z0;

        // set line coefficients
        line.coeff.values.push_back(x0);
        line.coeff.values.push_back(y0);
        line.coeff.values.push_back(z0);
        line.coeff.values.push_back(normal_line[0]);
        line.coeff.values.push_back(normal_line[1]);
        line.coeff.values.push_back(normal_line[2]);

        line.h = fabs(Eigen::Vector3f(x, y, z).dot(down_direction));
        line.d = fabs(Eigen::Vector3f(x, y, z).dot(keyInfo.main_vertical_plane_normal.normal));
    }


    inline void StairDetection::crossPointOfLineAndPlane(
            const float &x0, const float &y0, const float &z0, const float &a, const float &b, const float &c,
            const pcl::ModelCoefficients plane_coefficients,
            pcl::PointXYZ &pc)
    {
        float A, B, C, D;
        A = plane_coefficients.values[0];
        B = plane_coefficients.values[1];
        C = plane_coefficients.values[2];
        D = plane_coefficients.values[3];
        float t = -1 * (A * x0 + B * y0 + C * z0 + D);
        t = t / (A * a + B * b + C * c);
        pc.x = a * t + x0;
        pc.y = b * t + y0;
        pc.z = c * t + z0;
    }

    void StairDetection::computePlaneCounter(Stair &stair, KeyInfo &keyInfo)
    {
        std::shared_ptr<Node> pnext;
        pcl::PointXYZ point;

        stair.reset();

        while (stair.readNext(pnext))
        {
            if (pnext->pcurrent_type == step_node)
            {
                std::shared_ptr<Node> &pstep = pnext;

                if (pstep->step.plane_v->counter.width == 0)
                {
                    // 台阶垂直面上边沿直线参数
                    float xu = pstep->step.line.coeff.values[0];
                    float yu = pstep->step.line.coeff.values[1];
                    float zu = pstep->step.line.coeff.values[2];
                    float nux = pstep->step.line.coeff.values[3];
                    float nuy = pstep->step.line.coeff.values[4];
                    float nuz = pstep->step.line.coeff.values[5];

                    // 台阶垂直面下边沿直线参数
                    PointType max_point, min_point;
                    float max_prj, min_prj;
                    findMinMaxProjPoint(*pstep->step.plane_v, down_direction, min_point, max_point, min_prj, max_prj);
                    float xd, yd, zd, ndx, ndy, ndz;
                    xd = max_point.x;
                    yd = max_point.y;
                    zd = max_point.z;
                    ndx = nux;
                    ndy = nuy;
                    ndz = nuz;

                    // 左下侧交点
                    crossPointOfLineAndPlane(xd, yd, zd, ndx, ndy, ndz, keyInfo.side_plane_left, point);
                    pstep->step.plane_v->counter.points[0] = point;
                    // 右下侧交点
                    crossPointOfLineAndPlane(xd, yd, zd, ndx, ndy, ndz, keyInfo.side_plane_right, point);
                    pstep->step.plane_v->counter.points[1] = point;
                    // 右上侧交点
                    crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_right, point);
                    pstep->step.plane_v->counter.points[2] = point;
                    // 左上侧交点
                    crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_left, point);
                    pstep->step.plane_v->counter.points[3] = point;

                    pstep->step.plane_v->counter.width = 4;
                } else if (pstep->step.plane_v->counter.width == 2)
                {
                    // 台阶垂直面上边沿直线参数
                    float xu = pstep->step.line.coeff.values[0];
                    float yu = pstep->step.line.coeff.values[1];
                    float zu = pstep->step.line.coeff.values[2];
                    float nux = pstep->step.line.coeff.values[3];
                    float nuy = pstep->step.line.coeff.values[4];
                    float nuz = pstep->step.line.coeff.values[5];

                    // 右上侧交点
                    crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_right, point);
                    pstep->step.plane_v->counter.points[2] = point;
                    // 左上侧交点
                    crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_left, point);
                    pstep->step.plane_v->counter.points[3] = point;

                    pstep->step.plane_v->counter.width = 4;
                }

                if (pstep->step.plane_h->counter.width == 0)
                {
                    // 台阶水平面前沿直线参数
                    float xf = pstep->step.line.coeff.values[0];
                    float yf = pstep->step.line.coeff.values[1];
                    float zf = pstep->step.line.coeff.values[2];
                    float nfx = pstep->step.line.coeff.values[3];
                    float nfy = pstep->step.line.coeff.values[4];
                    float nfz = pstep->step.line.coeff.values[5];

                    // 左前侧交点
                    crossPointOfLineAndPlane(xf, yf, zf, nfx, nfy, nfz, keyInfo.side_plane_left, point);
                    pstep->step.plane_h->counter.points[0] = point;
                    // 右前侧交点
                    crossPointOfLineAndPlane(xf, yf, zf, nfx, nfy, nfz, keyInfo.side_plane_right, point);
                    pstep->step.plane_h->counter.points[1] = point;

                    pstep->step.plane_h->counter.width = 2;
                    // last one
                    if (pnext->pnext->pnext == nullptr)
                    {
                        PointType max_point, min_point;
                        // 找点云在 main_vertical_plane_normal 方向上的最远点和最近点
                        float max_prj, min_prj;
                        findMinMaxProjPoint(*pstep->step.plane_h, keyInfo.main_vertical_plane_normal.normal, min_point,
                                            max_point, min_prj, max_prj);

                        // 后侧交点
                        float xb, yb, zb, nbx, nby, nbz;
                        xb = min_point.x;
                        yb = min_point.y;
                        zb = min_point.z;
                        nbx = nfx;
                        nby = nfy;
                        nbz = nfz;

                        // 右后侧交点
                        crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_right, point);
                        pstep->step.plane_h->counter.points[2] = point;
                        // 左后侧交点
                        crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_left, point);
                        pstep->step.plane_h->counter.points[3] = point;

                        pstep->step.plane_h->counter.width = 4;
                    }
                }
            } else if (pnext->pcurrent_type == concaveline_node)
            {
                std::shared_ptr<Node> &pline = pnext;

                if (pline->concave_line.plane_h->counter.width == 0)
                {
                    // 平面后边沿直线参数
                    float xb = pline->concave_line.line.coeff.values[0];
                    float yb = pline->concave_line.line.coeff.values[1];
                    float zb = pline->concave_line.line.coeff.values[2];
                    float nbx = pline->concave_line.line.coeff.values[3];
                    float nby = pline->concave_line.line.coeff.values[4];
                    float nbz = pline->concave_line.line.coeff.values[5];

                    PointType max_point, min_point;
                    float max_prj, min_prj;
                    // 找点云在 main_vertical_plane_normal 方向上的最远点和最近点
                    findMinMaxProjPoint(*pline->concave_line.plane_h, keyInfo.main_vertical_plane_normal.normal,
                                        min_point, max_point, min_prj, max_prj);


                    // 前侧交点
                    float xf, yf, zf, nfx, nfy, nfz;
                    xf = max_point.x;
                    yf = max_point.y;
                    zf = max_point.z;
                    nfx = nbx;
                    nfy = nby;
                    nfz = nbz;

                    // 左前侧交点
                    crossPointOfLineAndPlane(xf, yf, zf, nfx, nfy, nfz, keyInfo.side_plane_left, point);
                    pline->concave_line.plane_h->counter.points[0] = point;
                    // 右前侧交点
                    crossPointOfLineAndPlane(xf, yf, zf, nfx, nfy, nfz, keyInfo.side_plane_right, point);
                    pline->concave_line.plane_h->counter.points[1] = point;

                    // 右后侧交点
                    crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_right, point);
                    pline->concave_line.plane_h->counter.points[2] = point;
                    // 左后侧交点
                    crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_left, point);
                    pline->concave_line.plane_h->counter.points[3] = point;

                    pline->concave_line.plane_h->counter.width = 4;
                } else if (pline->concave_line.plane_h->counter.width == 2)
                {
                    // 平面后边沿直线参数
                    float xb = pline->concave_line.line.coeff.values[0];
                    float yb = pline->concave_line.line.coeff.values[1];
                    float zb = pline->concave_line.line.coeff.values[2];
                    float nbx = pline->concave_line.line.coeff.values[3];
                    float nby = pline->concave_line.line.coeff.values[4];
                    float nbz = pline->concave_line.line.coeff.values[5];

                    // 右后侧交点
                    crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_right, point);
                    pline->concave_line.plane_h->counter.points[2] = point;
                    // 左后侧交点
                    crossPointOfLineAndPlane(xb, yb, zb, nbx, nby, nbz, keyInfo.side_plane_left, point);
                    pline->concave_line.plane_h->counter.points[3] = point;

                    pline->concave_line.plane_h->counter.width = 4;
                }

                if (pline->concave_line.plane_v->counter.width == 0)
                {
                    // 平面下边沿直线参数
                    float xd = pline->concave_line.line.coeff.values[0];
                    float yd = pline->concave_line.line.coeff.values[1];
                    float zd = pline->concave_line.line.coeff.values[2];
                    float ndx = pline->concave_line.line.coeff.values[3];
                    float ndy = pline->concave_line.line.coeff.values[4];
                    float ndz = pline->concave_line.line.coeff.values[5];

                    // 左下侧交点
                    crossPointOfLineAndPlane(xd, yd, zd, ndx, ndy, ndz, keyInfo.side_plane_left, point);
                    pline->concave_line.plane_v->counter.points[0] = point;
                    // 右下侧交点
                    crossPointOfLineAndPlane(xd, yd, zd, ndx, ndy, ndz, keyInfo.side_plane_right, point);
                    pline->concave_line.plane_v->counter.points[1] = point;

                    pline->concave_line.plane_v->counter.width = 2;
                    // last one
                    if (pnext->pnext->pnext == nullptr)
                    {
                        // 上侧面所过的点
                        PointType max_point, min_point;
                        float max_prj, min_prj;
                        findMinMaxProjPoint(*pline->concave_line.plane_v, down_direction, min_point, max_point, min_prj,
                                            max_prj);
                        float xu, yu, zu, nux, nuy, nuz;
                        xu = min_point.x;
                        yu = min_point.y;
                        zu = min_point.z;
                        nux = ndx;
                        nuy = ndy;
                        nuz = ndz;

                        // 右上侧交点
                        crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_right, point);
                        pline->concave_line.plane_v->counter.points[2] = point;
                        // 左上侧交点
                        crossPointOfLineAndPlane(xu, yu, zu, nux, nuy, nuz, keyInfo.side_plane_left, point);
                        pline->concave_line.plane_v->counter.points[3] = point;

                        pline->concave_line.plane_v->counter.width = 4;
                    }
                }
            }
        }
    }

    void StairDetection::setDown_sample_points_number(int down_sample_points_number)
    {
        StairDetection::down_sample_points_number = down_sample_points_number;
    }

    void StairDetection::setAngle_h(float angle_h)
    {
        StairDetection::angle_h = angle_h;
    }

    void StairDetection::setAngle_v(float angle_v)
    {
        StairDetection::angle_v = angle_v;
    }

    void StairDetection::setGround_height_th(float ground_height_th)
    {
        StairDetection::ground_height_th = ground_height_th;
    }

    void StairDetection::setMerge_dis_th(float merge_dis_th)
    {
        StairDetection::merge_dis_th = merge_dis_th;
    }

    void StairDetection::setMerge_angle_th(float merge_angle_th)
    {
        StairDetection::merge_angle_th = merge_angle_th;
    }

    void StairDetection::setMerge_distance_th(float merge_distance_th)
    {
        StairDetection::merge_distance_th = merge_distance_th;
    }

    void StairDetection::setA1D_th(float a1D_th)
    {
        StairDetection::a1D_th = a1D_th;
    }

    void StairDetection::setDecay_rate(float decay_rate)
    {
        StairDetection::decay_rate = decay_rate;
    }

    void StairDetection::setVertical_plane_normal_angle_th(float vertical_plane_normal_angle_th)
    {
        StairDetection::vertical_plane_normal_angle_th = vertical_plane_normal_angle_th;
    }

    void StairDetection::setCv_angle_horizontal_th(float cv_angle_horizontal_th)
    {
        StairDetection::cv_angle_horizontal_th = cv_angle_horizontal_th;
    }

    void StairDetection::setCv_dir_cluster_angle_th(float cv_dir_cluster_angle_th)
    {
        StairDetection::cv_dir_cluster_angle_th = cv_dir_cluster_angle_th;
    }

    void StairDetection::setVertical_plane_normal_est_angle_th(float vertical_plane_normal_est_angle_th)
    {
        StairDetection::vertical_plane_normal_est_angle_th = vertical_plane_normal_est_angle_th;
    }

    void StairDetection::setPlane_min_length(float min_length)
    {
        StairDetection::plane_min_length = min_length;
    }

    void StairDetection::setPlane_max_length(float max_length)
    {
        StairDetection::plane_max_length = max_length;
    }

    void StairDetection::setPlane_min_width(float min_width)
    {
        StairDetection::plane_min_width = min_width;
    }

    void StairDetection::setPlane_max_width(float max_width)
    {
        StairDetection::plane_max_width = max_width;
    }

    void StairDetection::setStair_max_height(float max_height)
    {
        StairDetection::stair_max_height = max_height;
    }

    void StairDetection::setStair_plane_max_distance(float stair_plane_max_distance)
    {
        StairDetection::stair_plane_max_distance = stair_plane_max_distance;
    }

    void StairDetection::setStair_cv_angle_diff_th(float stair_cv_angle_diff_th)
    {
        StairDetection::stair_cv_angle_diff_th = stair_cv_angle_diff_th;
    }

    void StairDetection::setStair_max_width(float stair_max_width)
    {
        StairDetection::stair_max_width = stair_max_width;
    }

    void StairDetection::setMin_ground_plane_points_num(int min_ground_plane_points_num)
    {
        StairDetection::min_ground_plane_points_num = min_ground_plane_points_num;
    }


}