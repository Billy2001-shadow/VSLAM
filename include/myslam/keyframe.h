//
// Created by chenwu on 23-8-9.
//

#ifndef MYSLAM_KEYFRAME_H
#define MYSLAM_KEYFRAME_H

#include "DBoW2/BowVector.h"
#include "myslam/frame.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
namespace myslam {
struct Frame;

struct KeyFrame : public Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<KeyFrame> Ptr;

  KeyFrame() {}
  KeyFrame(std::shared_ptr<Frame> frame);

  KeyFrame::Ptr CreateKeyFrame(std::shared_ptr<Frame> frame);

  SE3 getPose();
  void SetPose(const SE3 &pose);

  std::vector<cv::KeyPoint> GetKeyPoints();
  //   unsigned long frame_id_;     //可继承自Frame里的id_
  unsigned long key_frame_id_; //关键帧的id

  // for pose graph optimization
  SE3 relativate_pose_to_last_KF_;      //相对于上一关键帧的位姿
  SE3 relativate_pose_to_last_loop_KF_; //相对于最近一次闭环的位姿

  std::weak_ptr<KeyFrame> loop_key_frame_; //最近一次闭环的关键帧
  std::weak_ptr<KeyFrame> last_key_frame_; //上一关键帧

  // pyramid keypoints only for computing ORB descriptors and doing matching
  std::vector<cv::KeyPoint> pyramid_key_points_; //金字塔关键点
  //   std::vector<std::shared_ptr<Feature>>
  //       features_left_; //左图特征点 这个可以继承自Frame_吗？

  cv::Mat ORBDescriptors_; // ORB描述子
                           //   cv::Mat image_left_;     //左图

  DBoW2::BowVector bow2_vec_;

  SE3 pose_;
  std::mutex update_get_pose_mutex_;
};

} // namespace myslam

#endif