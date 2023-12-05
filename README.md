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
