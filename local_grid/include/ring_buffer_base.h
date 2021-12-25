#ifndef RING_BUFFER_BASE_H_
#define RING_BUFFER_BASE_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace grid {

template <int _POW, typename _Datatype, typename _Scalar = float>
class RingBufferBase {
 public:
  static const int _N = (1 << _POW);  // 2 to the power of POW
  static const int _N_2 = _N / 2;
  static const int _MASK = (_N - 1);    // 11...1
  static const int _NEG_MASK = ~_MASK;  // 00...0

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<int, 3, 1> Vector3i;

  explicit RingBufferBase(const _Scalar &resolution,
                          const _Datatype &empty_element = _Datatype())
      : resolution_(resolution),
        empty_element_(empty_element),
        offset_(-_N_2, -_N_2, -_N_2),
        buffer_(_N * _N * _N) {
    std::fill(buffer_.begin(), buffer_.end(), empty_element_);
  }

  virtual ~RingBufferBase() {}

  void setEmptyElement(const _Datatype &e) { empty_element_ = e; }

  virtual void setOffset(const Vector3i &offset) { offset_ = offset; }

  virtual void setCenter(const Vector3i &center) {
    static const Vector3i center2offset(-_N_2, -_N_2, -_N_2);
    offset_ = center + center2offset;
  }

  inline void getIdx(const Vector3 &point, Vector3i &idx) const {
    idx = (point / resolution_).array().floor().template cast<int>();
  }

  inline void getPoint(const Vector3i &idx, Vector3 &point) const {
    point = idx.template cast<_Scalar>();
    point.array() += _Scalar(0.5);
    point *= resolution_;
  }

  inline void getOffset(Vector3i &offset) const { offset = offset_; }

  // Moves 1 step in the direction
  // 每次只移动一个单位
  virtual void moveVolume(const Vector3i &direction) {
    for (int axis = 0; axis < 3; axis++) {
      if (direction[axis] != 0) {
        int slice;

        if (direction[axis] > 0) {
          ++offset_[axis];
          slice = offset_[axis] + _N - 1;
        } else {
          --offset_[axis];
          slice = offset_[axis];
        }

        switch (axis) {
          case 0:
            setXSlice(slice, empty_element_);
            break;
          case 1:
            setYSlice(slice, empty_element_);
            break;
          case 2:
            setZSlice(slice, empty_element_);
            break;
        }
      }
    }
  }

  void setXSlice(int slice_idx, const _Datatype &data) {
    for (int i = 0; i < _N; i++) {
      for (int j = 0; j < _N; j++) {
        Vector3i idx(slice_idx, i, j);
        this->at(idx) = data;
      }
    }
  }

  void setYSlice(int slice_idx, const _Datatype &data) {
    for (int i = 0; i < _N; i++) {
      for (int j = 0; j < _N; j++) {
        Vector3i idx(i, slice_idx, j);
        this->at(idx) = data;
      }
    }
  }

  void setZSlice(int slice_idx, const _Datatype &data) {
    for (int i = 0; i < _N; i++) {
      for (int j = 0; j < _N; j++) {
        Vector3i idx(i, j, slice_idx);
        this->at(idx) = data;
      }
    }
  }

  inline bool insideVolume(const Vector3i &coord) {
    static const Vector3i neg_mask_vec(_NEG_MASK, _NEG_MASK, _NEG_MASK);

    bool res = true;

    for (int i = 0; i < 3; i++) {
      res &= !((coord[i] - offset_[i]) & neg_mask_vec[i]);
    }

    return res;
  }

  inline _Datatype &at(const Vector3i &coord) {
    Vector3i idx;

    for (int i = 0; i < 3; i++) {
      idx[i] = coord[i] & _MASK;
    }

    return buffer_[_N * _N * idx[0] + _N * idx[1] + idx[2]];
  }

  inline _Datatype at(const Vector3i &coord) const {
    Vector3i idx;

    for (int i = 0; i < 3; i++) {
      idx[i] = coord[i] & _MASK;
    }

    return buffer_[_N * _N * idx[0] + _N * idx[1] + idx[2]];
  }

  inline Vector3i getVolumeCenter() {
    static const Vector3i offset2center(_N_2, _N_2, _N_2);
    return offset_ + offset2center;
  }

  template <typename F>
  void getMarkerHelper(sensor_msgs::PointCloud2 &m, const std::string &ns,
                       int id, const Vector4 &color, F func) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;

    Vector3 offset_point;
    getPoint(offset_, offset_point);

    for (int x = 0; x < _N; x++) {
      for (int y = 0; y < _N; y++) {
        for (int z = 0; z < _N; z++) {
          Vector3i coord(x, y, z);
          coord += offset_;

          geometry_msgs::Point p;
          p.x = x * resolution_;
          p.y = y * resolution_;
          p.z = z * resolution_;

          _Datatype &data = this->at(coord);

          if (func(data)) {
            pcl::PointXYZ pt;
            pt.x = p.x + offset_point.x();
            pt.y = p.y + offset_point.y();
            pt.z = p.z + offset_point.z();
            cloudMap.points.push_back(pt);
          }
        }
      }
    }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;
    pcl::toROSMsg(cloudMap, m);
    m.header.frame_id = "t265_odom_frame";
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  _Scalar resolution_;
  _Datatype empty_element_;

  Vector3i offset_;
  std::vector<_Datatype> buffer_;
};

}  // namespace grid

#endif  // RING_BUFFER_BASE_H_
