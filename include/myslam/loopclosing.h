#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/keyframe.h"
#include "myslam/map.h"
#include "myslam/orbextractor.h"
#include "myslam/orbvocabulary.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <memory>
namespace myslam {

struct KeyFrame;
class Frontend;
class Backend;
class Map;
class ORBextractor;

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
  void SetFrontend(const std::shared_ptr<Frontend> frontend) {
    frontend_ = frontend;
  }
  void SetSteroCamera(std::shared_ptr<Camera> left,
                      std::shared_ptr<Camera> right);
  // void SetFrontend(const std::shared_ptr<FrontEnd> frontend) {
  //   frontend_ = frontend;
  // }
  void InsertNewKeyFrame(const std::shared_ptr<KeyFrame> new_kf);
  void LoadParam();
  void GenerateORBextractor();
  bool HasNewKeyFrame();
  void ProcessNewKeyframe();
  bool DetectLoop();
  bool MatchFeatures();
  bool ComputeCorrectPose();
  int OptimizeCurrentPose();
  void LoopCorrect();
  void CorrectActivateKeyframeAndMappoint();
  void PoseGraphOptimization();
  void AddToKeyframeDatabase();

private:
  std::vector<cv::Mat> ConvertToDescriptorVector(const cv::Mat &descriptors);

  std::shared_ptr<Camera> left_camera_ = nullptr;
  std::shared_ptr<Camera> right_camera_ = nullptr;
  std::shared_ptr<KeyFrame> current_keyframe_ = nullptr;
  std::shared_ptr<KeyFrame> last_closed_keyframe_ = nullptr;
  std::atomic<bool> loop_thread_is_running_; //这个是否放在public更好？
  std::thread loop_closing_thread_;
  std::mutex all_new_keyframe_mutex_;

  cv::Ptr<cv::DescriptorMatcher> cv_matcher_;
  std::shared_ptr<ORBextractor> orb_extractor_ = nullptr;
  std::unique_ptr<ORBVocabulary> dbow2_vocabulary_ = nullptr;
  std::list<std::shared_ptr<KeyFrame>> all_new_keyframes_;
  std::map<unsigned long, std::shared_ptr<KeyFrame>> key_frame_database_;
  std::shared_ptr<KeyFrame> loop_keyframe_ = nullptr;
  std::shared_ptr<Map> map_ = nullptr;
  std::weak_ptr<Backend> backend_;
  std::shared_ptr<Frontend> frontend_ = nullptr;
  std::set<std::pair<int, int>> set_valid_feature_matches_; // TODO
  //非常重要的 全都是围绕这个优化位姿来展开
  Sophus::SE3d corrected_current_pose_;

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