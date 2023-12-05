#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/orbextractor.h"
#include "myslam/orbvocabulary.h"
#include <memory>
namespace myslam {
//后续考虑换成KeyFrame，因为回环只处理关键帧，帧中需要添加新的成员如DBoW向量
struct Frame;

class LoopClosing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<LoopClosing> Ptr;
  //构造函数中启动回环线程并挂起
  LoopClosing();
  ~LoopClosing() = default;
  void LoopClosingThread();
  void Stop();
  void SetMap(shared_ptr<Map> map) { map_ = map; }
  void SetBackend(const std::shared_ptr<Backend> backend) {
    backend_ = backend;
  }
  // void SetFrontend(const std::shared_ptr<FrontEnd> frontend) {
  //   frontend_ = frontend;
  // }

private:
  void LoadParam();
  void GenerateORBextractor();
  bool HasNewKeyFrame();
  void ProcessNewKeyframe();
  std::atomic<bool> loop_thread_is_running_; //这个是否放在public更好？
  std::thread loop_closing_thread_;
  std::mutex all_new_keyframe_mutex_;

  cv::Ptr<cv::DescriptorMatcher> cv_matcher_;
  std::shared_ptr<ORBextractor> orb_extractor_ = nullptr;
  std::unique_ptr<ORBVocabulary> dbow2_vocabulary_ = nullptr;
  std::list<std::shared_ptr<Frame>> all_new_keyframes_;
  std::map<unsigned long, std::shared_ptr<Frame>>
      key_frame_database_;                            //需要改为关键帧
  std::shared_ptr<Frame> current_keyframe_ = nullptr; //需要改为关键帧
  std::shared_ptr<Map> map_ = nullptr;
  std::shared_ptr<Backend> backend_ = nullptr;
  bool need_correct_loop_pose_ = true;
  bool open_loop_closing_ = true;
  bool show_loop_closing_result_ = false;
  float loop_threshold_heigher_;
  float loop_threshold_lower_; //暂时没用
  int pyramid_level_num_ = 8;
  unsigned int keyframe_database_min_size_ = 50;
  std::string orb_voc_path_;
};

} // namespace myslam

#endif