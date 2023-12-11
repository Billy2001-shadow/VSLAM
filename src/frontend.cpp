//
// Created by gaoxiang on 19-5-2.
//

#include <mutex>
#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/keyframe.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

Frontend::Frontend() {
  GenerateORBextractor();
  num_features_init_ = Config::Get<int>("num_features_init");
  num_features_ = Config::Get<int>("num_features");
  show_orb_detect_result_ = Config::Get<int>("View.ORB.Extractor.Result");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
  current_frame_ = frame;

  switch (status_) {
  case FrontendStatus::INITING:
    StereoInit();
    break;
  case FrontendStatus::TRACKING_GOOD:
  case FrontendStatus::TRACKING_BAD:
    Track();
    break;
  case FrontendStatus::LOST:
    Reset();
    break;
  }

  last_frame_ = current_frame_;
  return true;
}

bool Frontend::Track() {
  // 0.先对当前帧与参考关键帧之间的相对位姿进行一个先验估计(恒速模型)
  // 使用这个之后为什么直接跟丢了呀？
  if (last_frame_) {
    current_frame_->SetRelativePose(relative_motion_ *
                                    last_frame_->getRelativePose());
    // current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
  }
  // 1.利用光流法跟踪上一帧的特征点
  int num_track_last = TrackLastFrame(); // current_frame_->Pose() TODO
  // 2.利用跟踪到的特征点(成功三角化了的)进行BA优化，得到当前帧的位姿
  tracking_inliers_ = EstimateCurrentPose();
  // 3.根据优化位姿的情况得到的内点数来切换状态
  if (tracking_inliers_ > num_features_tracking_) {
    // tracking good
    status_ = FrontendStatus::TRACKING_GOOD;
  } else if (tracking_inliers_ > num_features_tracking_bad_) {
    // tracking bad
    status_ = FrontendStatus::TRACKING_BAD;
  } else {
    // lost
    status_ = FrontendStatus::LOST;
  }
  // 4.更新恒速模型的相对运动量(当前帧与上一帧的位姿均是相对于参考关键帧的)
  relative_motion_ = current_frame_->getRelativePose() *
                     last_frame_->getRelativePose().inverse();

  // 5.需要插入关键帧(特殊情况，如果没跟踪上就插入关键帧)
  if (tracking_inliers_ < num_features_needed_for_keyframe_) {
    // not have enough features, don't insert keyframe
    DetectFeatures();
    FindFeaturesInRight();
    TriangulateNewPoints();
    InsertKeyframe();
  }

  // 6.与初始化模块一样，在处理完当前帧之后，将该帧加入到可视化线程中(这个时候的位姿是已经局部BA矫正过后的)
  if (viewer_)
    viewer_->AddCurrentFrame(current_frame_);

  return true;
}

bool Frontend::InsertKeyframe() {

  KeyFrame::Ptr new_key_frame = KeyFrame::CreateKeyFrame(current_frame_);
  Eigen::Matrix<double, 6, 1> se3_zero = Eigen::Matrix<double, 6, 1>::Zero();

  if (status_ == FrontendStatus::INITING) {
    new_key_frame->SetPose(Sophus::SE3d::exp(se3_zero));
  }

  else {
    //需要注意reference_kf_在哪些地方更新了？
    std::unique_lock<std::mutex> lck(getset_reference_kp_numtex_);
    new_key_frame->SetPose(current_frame_->getRelativePose() *
                           reference_kf_->getPose());
    new_key_frame->last_key_frame_ = reference_kf_;
    new_key_frame->relative_pose_to_last_KF_ =
        current_frame_->getRelativePose();
  }

  map_->InsertKeyFrame(new_key_frame); //将新的关键帧插入地图中

  if (viewer_) {
    //插入关键帧之后要及时更新地图(因为有新的点加入了，需要刷新一下关键帧列表，不然新加入的地图点是显示不出来的)
    viewer_->UpdateMap();
  }

  if (backend_) {
    backend_->UpdateMap(); //通知一下地图进行了更新
  }

  if (loopclosing_) {
    loopclosing_->InsertNewKeyFrame(new_key_frame);
  }

  //更新一些变量
  {
    std::unique_lock<std::mutex> lck(getset_reference_kp_numtex_);
    reference_kf_ = new_key_frame;
  }
  //因为当前帧就是关键帧，与最近的关键帧之间的相对位姿关系是se3_zero
  current_frame_->SetRelativePose(Sophus::SE3d::exp(se3_zero));

  return true;
}

void Frontend::SetObservationsForKeyFrame() {
  for (auto &feat : current_frame_->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp)
      mp->AddObservation(feat);
  }
}

int Frontend::TriangulateNewPoints() {
  std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
  // SE3 current_pose_Twc = current_frame_->Pose().inverse();
  //使用相对位姿
  SE3 current_pose_Twc;
  std::unique_lock<std::mutex> lck(getset_reference_kp_numtex_);
  current_pose_Twc =
      (current_frame_->getRelativePose() * reference_kf_->getPose()).inverse();

  int cnt_triangulated_pts = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.expired() &&
        current_frame_->features_right_[i] != nullptr) {
      // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
      std::vector<Vec3> points{
          camera_left_->pixel2camera(
              Vec2(current_frame_->features_left_[i]->position_.pt.x,
                   current_frame_->features_left_[i]->position_.pt.y)),
          camera_right_->pixel2camera(
              Vec2(current_frame_->features_right_[i]->position_.pt.x,
                   current_frame_->features_right_[i]->position_.pt.y))};
      Vec3 pworld = Vec3::Zero();

      if (triangulation(poses, points, pworld) && pworld[2] > 0) {
        auto new_map_point = MapPoint::CreateNewMappoint();
        pworld = current_pose_Twc * pworld;
        new_map_point->SetPos(pworld);
        new_map_point->AddObservation(current_frame_->features_left_[i]);
        new_map_point->AddObservation(current_frame_->features_right_[i]);

        current_frame_->features_left_[i]->map_point_ = new_map_point;
        current_frame_->features_right_[i]->map_point_ = new_map_point;

        map_->InsertMapPoint(new_map_point);
        cnt_triangulated_pts++;
      }
    }
  }
  LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
  return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose() {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // vertex
  VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
  vertex_pose->setId(0);
  /// T{i_k} * T{k_w} = T{i_w}
  {
    std::unique_lock<std::mutex> lck(getset_reference_kp_numtex_);
    vertex_pose->setEstimate(current_frame_->getRelativePose() *
                             reference_kf_->getPose());
  }
  // vertex_pose->setEstimate(current_frame_->Pose());
  optimizer.addVertex(vertex_pose);

  // K
  Mat33 K = camera_left_->K();

  // edges
  int index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Feature::Ptr> features;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    auto mp = current_frame_->features_left_[i]->map_point_.lock();
    //没有三角化的特征点是无法形成BA约束的
    if (mp) {
      features.push_back(current_frame_->features_left_[i]);
      EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(
          toVec2(current_frame_->features_left_[i]->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.push_back(edge);
      optimizer.addEdge(edge);
      index++;
    }
  }

  // estimate the Pose the determine the outliers
  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iteration = 0; iteration < 4; ++iteration) {
    // vertex_pose->setEstimate(current_frame_->Pose());  CH13中为什么要有这句呢
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cnt_outlier = 0;

    // count the outliers
    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->is_outlier_) {
        e->computeError();
      }
      if (e->chi2() > chi2_th) {
        features[i]->is_outlier_ = true;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->is_outlier_ = false;
        e->setLevel(0);
      };

      if (iteration == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }

  LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
            << features.size() - cnt_outlier;
  // Set pose and outlier
  current_frame_->SetPose(vertex_pose->estimate());

  LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

  /// T{i_k} = T{i_w} * T{k_w}.inverse()
  {
    std::unique_lock<std::mutex> lck(getset_reference_kp_numtex_);

    current_frame_->SetRelativePose(vertex_pose->estimate() *
                                    reference_kf_->getPose().inverse());
  }

  //这里可以map_->AddOutlierMapPoint(mp->id_); 后续研究一下
  for (auto &feat : features) {
    if (feat->is_outlier_) {
      feat->map_point_.reset();
      feat->is_outlier_ = false; // maybe we can still use it in future
    }
  }
  return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_last, kps_current;
  for (auto &kp : last_frame_->features_left_) {
    if (kp->map_point_.lock()) {
      // use project point
      auto mp = kp->map_point_.lock();
      // auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
      auto px = camera_left_->world2pixel(mp->pos_,
                                          current_frame_->getRelativePose() *
                                              reference_kf_->getPose());

      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(cv::Point2f(px[0], px[1]));
    } else {
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  Mat error;
  cv::calcOpticalFlowPyrLK(
      last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_current[i], 7);
      Feature::Ptr feature(new Feature(current_frame_, kp));
      feature->map_point_ = last_frame_->features_left_[i]->map_point_;
      current_frame_->features_left_.push_back(feature);
      num_good_pts++;
    }
  }

  LOG(INFO) << "Find " << num_good_pts << " in the last image.";
  return num_good_pts;
}

bool Frontend::StereoInit() {
  int num_features_left = DetectFeatures();
  int num_coor_features = FindFeaturesInRight();
  if (num_coor_features < num_features_init_) {
    LOG(WARNING) << "Too few init feature points";
    return false;
  }

  bool build_map_success = BuildInitMap();
  if (build_map_success) {
    status_ = FrontendStatus::TRACKING_GOOD;
    if (viewer_) {
      viewer_->AddCurrentFrame(current_frame_);
      viewer_->UpdateMap();
    }
    return true;
  }
  return false;
}

int Frontend::DetectFeatures() {
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto &feat : current_frame_->features_left_) {
    cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                  feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  // gftt_->detect(current_frame_->left_img_, keypoints, mask);
  if (status_ == FrontendStatus::INITING)
    orb_init_extractor_->Detect(current_frame_->left_img_, mask, keypoints);
  else
    orb_extractor_->Detect(current_frame_->left_img_, mask, keypoints);
  int cnt_detected = 0;
  for (auto &kp : keypoints) {
    current_frame_->features_left_.push_back(
        Feature::Ptr(new Feature(current_frame_, kp)));
    cnt_detected++;
  }

  if (show_orb_detect_result_ && cnt_detected > 0) {
    cv::Mat show_img(current_frame_->left_img_.size(), CV_8UC1);
    cv::drawKeypoints(current_frame_->left_img_, keypoints, show_img,
                      cv::Scalar(0, 255, 0));
    cv::imshow("orb_detect_result", show_img);
    // cv::waitKey(1);
  }

  LOG(INFO) << "Detect " << cnt_detected << " new features";
  return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_left, kps_right;
  for (auto &kp : current_frame_->features_left_) {
    kps_left.push_back(kp->position_.pt);
    auto mp = kp->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_right.push_back(cv::Point2f(px[0], px[1]));
    } else {
      // use same pixel in left iamge
      kps_right.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_img_, current_frame_->right_img_, kps_left,
      kps_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_right[i], 7);
      Feature::Ptr feat(new Feature(current_frame_, kp));
      feat->is_on_left_image_ = false;
      current_frame_->features_right_.push_back(feat);
      num_good_pts++;
    } else {
      current_frame_->features_right_.push_back(nullptr);
    }
  }
  LOG(INFO) << "Find " << num_good_pts << " in the right image.";
  return num_good_pts;
}

bool Frontend::BuildInitMap() {
  std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
  size_t cnt_init_landmarks = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_right_[i] == nullptr)
      continue;
    // create map point from triangulation
    std::vector<Vec3> points{
        camera_left_->pixel2camera(
            Vec2(current_frame_->features_left_[i]->position_.pt.x,
                 current_frame_->features_left_[i]->position_.pt.y)),
        camera_right_->pixel2camera(
            Vec2(current_frame_->features_right_[i]->position_.pt.x,
                 current_frame_->features_right_[i]->position_.pt.y))};
    Vec3 pworld = Vec3::Zero();

    if (triangulation(poses, points, pworld) && pworld[2] > 0) {
      auto new_map_point = MapPoint::CreateNewMappoint();
      new_map_point->SetPos(pworld);
      new_map_point->AddObservation(current_frame_->features_left_[i]);
      new_map_point->AddObservation(current_frame_->features_right_[i]);
      current_frame_->features_left_[i]->map_point_ = new_map_point;
      current_frame_->features_right_[i]->map_point_ = new_map_point;
      cnt_init_landmarks++;
      map_->InsertMapPoint(new_map_point);
    }
  }

  //第一帧默认为关键帧
  InsertKeyframe();

  LOG(INFO) << "Initial map created with " << cnt_init_landmarks
            << " map points";

  return true;
}

void Frontend::GenerateORBextractor() {
  int num_orb_bew_features = Config::Get<int>("ORBextractor.nNewFeatures");
  float scale_factor = Config::Get<float>("ORBextractor.scaleFactor");
  int n_levels = Config::Get<int>("ORBextractor.nLevels");
  int fIniThFAST = Config::Get<int>("ORBextractor.iniThFAST");
  int fMinThFAST = Config::Get<int>("ORBextractor.minThFAST");
  orb_extractor_ = ORBextractor::Ptr(new ORBextractor(
      num_orb_bew_features, scale_factor, n_levels, fIniThFAST, fMinThFAST));
  int num_features_init = Config::Get<int>("ORBextractor.nInitFeatures");
  orb_init_extractor_ = ORBextractor::Ptr(new ORBextractor(
      num_features_init, scale_factor, n_levels, fIniThFAST, fMinThFAST));
}

bool Frontend::Reset() {
  LOG(INFO) << "Reset is not implemented. ";
  return true;
}

} // namespace myslam