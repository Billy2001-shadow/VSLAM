#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;

struct Feature;

/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
struct MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long id_ = 0; // ID
  bool is_outlier_ = false;
  Vec3 pos_ = Vec3::Zero(); // Position in world
  std::mutex data_mutex_;
  std::mutex update_get_mutex_;
  int observed_times_ = 0; // being observed by feature matching algo.
  int active_observed_times_ = 0;
  std::list<std::weak_ptr<Feature>> observations_;
  std::list<std::weak_ptr<Feature>> active_observations_;

  MapPoint() {}

  MapPoint(long id, Vec3 position);

  Vec3 Pos() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return pos_;
  }

  void SetPos(const Vec3 &pos) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    pos_ = pos;
  };

  void AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
  }

  void RemoveObservation(std::shared_ptr<Feature> feat);
  void RemoveActiveObservation(std::shared_ptr<Feature> feature);
  std::list<std::weak_ptr<Feature>> GetObs() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return observations_;
  }

  std::list<std::weak_ptr<Feature>> GetActiveObservations() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_observations_;
  }

  // factory function
  static MapPoint::Ptr CreateNewMappoint();
};
} // namespace myslam

#endif // MYSLAM_MAPPOINT_H
