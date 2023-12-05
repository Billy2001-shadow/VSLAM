#include "myslam/config.h"
#include "myslam/orbvocabulary.h"
#include <memory>
#include <myslam/loopclosing.h>

namespace myslam {

LoopClosing::LoopClosing() {
  LoadParam();
  GenerateORBextractor();
  loop_thread_is_running_.store(true);

  cv_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

  LOG_ASSERT(orb_voc_path_.empty() == false);
  if (open_loop_closing_) {
    dbow2_vocabulary_ = std::make_unique<ORBVocabulary>();
    LOG(WARNING) << "Reading ORB Vocabulary txt File, Wait a Second.....";
    std::cout << "orb_voc_path_ = " << orb_voc_path_ << std::endl;
    dbow2_vocabulary_->loadFromTextFile(orb_voc_path_);
    // if (dbow2_vocabulary_ == nullptr) {
    //   std::cerr << "dbow2_vocabulary_ is not initialized.\n";
    // } else if (access(orb_voc_path_.c_str(), F_OK) == -1) {
    //   std::cerr << "The file " << orb_voc_path_
    //             << " does not exist or cannot be accessed.\n";
    // } else {
    //   try {
    //     dbow2_vocabulary_->loadFromTextFile(orb_voc_path_);
    //   } catch (const std::exception &e) {
    //     std::cerr << "An exception occurred: " << e.what() << '\n';
    //   }
    // }
    // dbow2_vocabulary_->loadFromTextFile(orb_voc_path_);
    loop_closing_thread_ =
        std::thread(std::bind(&LoopClosing::LoopClosingThread, this));
  }
}

void LoopClosing::LoopClosingThread() {
  while (loop_thread_is_running_.load()) {
    if (HasNewKeyFrame()) {
      ProcessNewKeyframe();

      bool confirm_loop_closing = false; //是否检测到闭环
    }
    usleep(1000);
  }
}

void LoopClosing::Stop() {
  while (HasNewKeyFrame()) {
    usleep(1e5);
  }
  loop_thread_is_running_.store(false);
  loop_closing_thread_.join();
}

void LoopClosing::ProcessNewKeyframe() {
  {
    std::unique_lock<std::mutex> lck(all_new_keyframe_mutex_);
    current_keyframe_ = all_new_keyframes_.front();

    all_new_keyframes_.pop_front();
  }

  // //处理新的关键帧
  // std::vector<cv::KeyPoint> pyramid_points;
  // pyramid_points.reserve(pyramid_level_num_ *
  //                        current_keyframe_->features_left_.size());
  // for (size_t i = 0; i < current_keyframe_->features_left_.size(); i++) {
  //   /// The class id marks the serial number
  //   current_keyframe_->features_left_[i]->kp_position_.class_id = i;
  //   for (int level = 0; level < pyramid_level_num_; level++) {
  //     cv::KeyPoint kp(current_keyframe_->features_left_[i]->kp_position_);
  //     kp.octave = level;
  //     kp.response = -1;
  //     kp.class_id = i;
  //     pyramid_points.emplace_back(kp);
  //   }
  // }
  // // remove the pyramid keypoints which are not FAST corner or beyond borders
  // // compute their orientations and sizes
  // orb_extractor_->ScreenAndComputeKPsParams(
  //     current_keyframe_->image_left_, pyramid_points,
  //     current_keyframe_->pyramid_key_points_);

  // // calculate the orb descriptors of all valid pyramid keypoints
  // orb_extractor_->CalcDescriptors(current_keyframe_->image_left_,
  //                                 current_keyframe_->pyramid_key_points_,
  //                                 current_keyframe_->ORBDescriptors_);
  // std::vector<cv::Mat> desc =
  //     ConvertToDescriptorVector(current_keyframe_->ORBDescriptors_);
  // //将特征描述子转换为bow2向量，并存储在当前关键帧中
  // dbow2_vocabulary_->transform(desc, current_keyframe_->bow2_vec_);
}

bool LoopClosing::HasNewKeyFrame() {
  std::unique_lock<std::mutex> lck(all_new_keyframe_mutex_);
  return (!all_new_keyframes_.empty());
}

void LoopClosing::GenerateORBextractor() {
  int num_orb_bew_features = Config::Get<int>("ORBextractor.nNewFeatures");
  float scale_factor = Config::Get<float>("ORBextractor.scaleFactor");
  int n_levels = Config::Get<int>("ORBextractor.nLevels");
  int fIniThFAST = Config::Get<int>("ORBextractor.iniThFAST");
  int fMinThFAST = Config::Get<int>("ORBextractor.minThFAST");
  orb_extractor_ = ORBextractor::Ptr(new ORBextractor(
      num_orb_bew_features, scale_factor, n_levels, fIniThFAST, fMinThFAST));
}

void LoopClosing::LoadParam() {
  //在这里加判断，如果没读取到就提示，不然出现段错误都不知道哪里的bug
  open_loop_closing_ = Config::Get<int>("Loop.Closing.Open");
  show_loop_closing_result_ = Config::Get<int>("Loop.Show.Closing.Result");
  loop_threshold_heigher_ = Config::Get<float>("Loop.Threshold.Heigher");
  loop_threshold_lower_ = Config::Get<float>("Loop.Threshold.Lower");
  pyramid_level_num_ = Config::Get<int>("Pyramid.Level");
  keyframe_database_min_size_ =
      Config::Get<int>("Loop.Closig.Keyframe.Database.Min.Size");
  orb_voc_path_ = Config::Get<std::string>("DBOW2.VOC.Path");
  //   LOG(INFO) << "Loop.Closing.Open: " << open_loop_closing_;
  //   LOG(INFO) << "Loop.Show.Closing.Result: " << show_loop_closing_result_;
  //   LOG(INFO) << "Loop.Threshold.Heigher: " << loop_threshold_heigher_;
  //   LOG(INFO) << "Loop.Threshold.Lower: " << loop_threshold_lower_;
  //   LOG(INFO) << "Pyramid.Level: " << pyramid_level_num_;
  //   LOG(INFO) << "Loop.Closig.Keyframe.Database.Min.Size: " <<
  //   keyframe_database_min_size_;
}

} // namespace myslam