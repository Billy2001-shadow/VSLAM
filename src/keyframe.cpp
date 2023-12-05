#include "myslam/keyframe.h"
#include "myslam/feature.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame) {
  // set id
  static unsigned long FactoryId = 0;
  key_frame_id_ = FactoryId++;

  // copy some members from Frame
  id_ = frame->id_;
  time_stamp_ = frame->id_;
  left_img_ = frame->left_img_;
  features_left_ = frame->features_left_;
}

KeyFrame::Ptr KeyFrame::CreateKeyFrame(std::shared_ptr<Frame> frame) {
  // new KeyFrame(frame) 构造了一个KeyFrame对象，并copy了一些属性
  KeyFrame::Ptr new_key_frame(new KeyFrame(frame));

  /// link Feature->keyframe_ to the current KF
  /// add the feature to Feature->MapPoint->observation
  for (size_t i = 0, N = new_key_frame->features_left_.size(); i < N; i++) {
    auto feat = new_key_frame->features_left_[i];
    feat->keyframe_ = new_key_frame; // feature 关联一个关键帧

    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->AddObservation(feat);
      new_key_frame->features_left_[i]->map_point_ = mp;
    }
  }

  return new_key_frame;
}

SE3 KeyFrame::getPose() {
  std::unique_lock<std::mutex> lck(update_get_pose_mutex_);
  return pose_;
}

void KeyFrame::SetPose(const SE3 &pose) {
  std::unique_lock<std::mutex> lck(update_get_pose_mutex_);
  pose_ = pose;
}

std::vector<cv::KeyPoint> KeyFrame::GetKeyPoints() {
  std::vector<cv::KeyPoint> kps(features_left_.size());
  for (size_t i = 0, N = features_left_.size(); i < N; i++) {
    kps[i] = features_left_[i]->position_;
  }
  return kps;
}

} // namespace myslam