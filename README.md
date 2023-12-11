# VSLAM
以SLAM十四将第13章的工程代码为基础框架进行进一步开发





### 对Feature进行修改

在Feature结构体里关联 关键帧。





### 关键帧相关

```
unsigned long keyframe_id_ = 0;
is_keyframe_ = false;
SetKeyFrame(){
	static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}
```

#### 后端处理关键帧

关键帧策略：

- 初始化时插入关键帧，并把当前关键帧插入到局部地图中去。因为添加了新的关键帧，所以在后端里面运行UpdateMap()更新一下局部地图，启动一次局部地图的BA优化。最后把显示线程中的地图点也更新一下，对应的Viewer::UpdateMap()函数
- 大于18个内点 小于50个内点时  追踪状态为BAD。并不一定是BAD状态下就插入关键帧。而是内点数小于80就插入关键帧。 把当前关键之插入到局部地图中去，因为添加了新的关键帧，所以在后端里面运行UpdateMap()更新一下局部地图，启动一次局部地图的BA优化。最后把显示线程中的地图点也更新一下，对应的Viewer::UpdateMap()函数



- 回环的关键帧需要新增什么内容呢？
  - 回环检测插入的关键帧是后端处理过后的吗？

- 局部地图除了关键帧和mapPoint还有什么需要维护呢？

  - 最重要的就是Landmark和Keyframes了。使用哈希表存储。

  - ```
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;
    ```

- 第一步保证检测到了回环 第二步保证回环起了作用

reference_kf_ 在哪里维护？





### 回环检测流程

回环类构造函数

- 加载回环参数
- 创建ORB提取器
- 创建描述子匹配器  用Hamming距离当做匹配的度量
- 加载ORB特征的DBoW字典 ORB词汇

进入回环线程

- 检测是否有新的关键帧

- 如果有新的关键帧，就处理新的关键帧(在关键帧上提取bow2向量)

  - 从新关键帧队列中弹出最新的关键帧
  - 创建金字塔，金子塔的点数为 =  金子塔层数×图像特征点个数
  - 遍历左图的特征点
    - 对关键点的序号赋值为 i （在第i轮遍历时）
    - 遍历金字塔的不同层
      - 由当前特征点构造出每一层上的关键点，额外赋予新的属性 即关键点的层数octave,关键点的响应值 -1 关键点的序号 i 
      - 最后把上面构造好的关键点对象送入金子塔点的容器中pyramid_points.emplace_back(kp);
  - 由orb_extractor_ 计算pyramid_points的orientations and sizes，计算得到的结果存进current_keyframe_->pyramid_key_points_ 
  - 由orb_extractor_ 计算所有current_keyframe_->pyramid_key_points_ 的orb descriptors ，并将计算结果存放进current_keyframe_->ORBDescriptors_
  - 将描述子转换成向量形式ConvertToDescriptorVector(current_keyframe_->ORBDescriptors_)；
  - 再将cv::Mat形式的向量转换成bow2向量，并存储在当前关键帧transform(desc, current_keyframe_->bow2_vec_);

- 如果有足够多的关键帧 (即key_frame_database_的大小大于50帧)

  - 检测是否存在回环

    - 在key_frame_database_这个std::map中寻找是否存在与当前关键帧相似的
    - 如果当前帧与查询的关键帧之间的id小于20，直接break
    - 计算当前关键帧与查询关键帧之间的相似性得分，并找出最高得分
    - 如果最高得分小于阈值loop_threshold_heigher_ 则认为不存在回环，反正则认为得到最高分的查询帧是当前帧的一个回环，并将回环帧记为loop_keyframe_

  - 检测到回环之后，进行特征点匹配(当前帧与回环帧)，如果匹配上足够的特征就进行位姿矫正

    - 这里是进行校验过程吗？即使前面我们判断是回环，但是仍然要匹配两张图片之间是否存在足够的配对点？

    - 匹配回环帧的ORB描述子与当前帧的ORB描述子LoopClosing::MatchFeatures()函数

      - ```
        cv_matcher_->match(
              loop_keyframe_->ORBDescriptors_, current_keyframe_->ORBDescriptors_, matches);
        ```

        

      - 针对匹配结果mathes进行配对点筛选，把配对成功的特征点的id对插入set_valid_feature_matches_ 这个std::set<std::pair<int, int>>**集合**中

      - 如果匹配成功的点对小于10(set_valid_feature_matches_.size() < 10)，则认为匹配失败，就不进行下一步位姿矫正

  - 如果当前帧与回环帧匹配上了，则进行位姿矫正计算

    - 进行confirm_loop_closing = ComputeCorrectPose();

      - 遍历配对好的特征点对

        - 获取回环帧的特征点关联的Landmark，将Landmark的3D点位置用loop_point3f表示，当前关键帧的关键点2D点位置用current_point2f表示
        - 如果该回环帧的特征点无关联的Landmark，则从匹配点对set_valid_feature_matches_中删去该对配对点
        - 剩下的则是存在Landmark的有效配对(LoopClosing: number of valid matches with mappoints:)

      - 如果配对好的landmarks小于10，return false

      - 如果有足够的配对好的landmarks，PnP计算Landmarks与current_point2f，这个地方需要相机内参

        ```
        cv::solvePnPRansac(
                loop_point3f, current_point2f, K, cv::Mat(), rvec, tvec, false, 100, 5.991, 0.99);
        cv::Rodrigues(rvec, R);
        cv::cv2eigen(R, Reigen);
        cv::cv2eigen(tvec, teigen);        
        corrected_current_pose_ = Sophus::SE3d(Reigen, teigen);
        ```

      - 优化当前帧的位姿 **优化投影误差**

        - dd
        - 从配对点中去除XXX，剩下的就是内点

  - 如果计算结果合理，则确定检测到了回环，即confirm_loop_closing = true

    - 进行LoopCorrect()





all_new_keyframes_ 的维护逻辑

key_frame_database_ 的维护逻辑

存不存在关键帧的特征点没有关联到任何Landmark呢？



- 在这个优化问题中，每个边（EdgeProjectionPoseOnly）都对应于一个**特征点的观测值和相机位姿之间的约束。**

- // 只会跟踪已经三角化后的点，因为**没有三角化的点是无法形成BA约束的**
- 没做重定位会怎么样？
- 左右图特征点的个数和顺序是一样的吗 在容器里面

## C++编程技巧



### Lambda表达式

```
//函数声明
auto to_vec2 = [](cv::Point2f &pt) { return Eigen::Vector2d(pt.x, pt.y); };

这是一个C++的lambda表达式，它定义了一个名为to_vec2的函数对象。这个函数接受一个cv::Point2f类型的引用作为参数，然后返回一个Eigen::Vector2d类型的对象。

cv::Point2f是OpenCV库中的一个类，用于表示2D空间中的一个点，它有两个浮点数成员变量x和y。

Eigen::Vector2d是Eigen库中的一个类，用于表示2D向量，它也有两个成员变量，通常用于表示向量或者点的坐标。

所以，这个lambda表达式的作用是将一个cv::Point2f对象转换为一个Eigen::Vector2d对象。

//函数调用
Eigen::Vector2d observe = to_vec2(feat->kp_position_.pt);
```



### 回环检测

loop_keyframe_ 维护：

last_closed_keyframe_ 维护



current_mp->GetObservations() 怎么替换？



### 修改日志

---

#### 2023.12.06

- 将前端提取GFTT角点改为提取ORB特征点
- 增加关键帧类
  - 关键帧做如下处理：
  - 普通帧只进行如下处理：



#### TODO

检查后端优化有没有起作用

把回环的代码补全











![image-20231211202245257](/home/cw/Slam/VSLAM/README.assets/image-20231211202245257.png)





![image-20231211203149368](/home/cw/Slam/VSLAM/README.assets/image-20231211203149368.png)







![image-20231211203325132](/home/cw/Slam/VSLAM/README.assets/image-20231211203325132.png)



目前存在两个问题：

- 能检测出回环，但是检测出来的回环次数很少？
- 检测出回环之后貌似也没有对位姿和地图点进行校正？
