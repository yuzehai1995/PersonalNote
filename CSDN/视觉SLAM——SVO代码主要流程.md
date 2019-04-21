@[toc]
SVO的核心流程主要可以在**frame_handler_mono.cpp中的addImage**( )找到。阅读addImage( )函数：

> **addImage( )通过 stage_ 来确定当前帧状态，判断需要进行哪一步操作，包括**：   
> processFirstFrame（），processSecondFrame（），processFrame（），relocalizeFrame（）

以下是各个主要函数的具体流程：
# 1 processFirstFrame
将第一帧位姿设为单位阵，并添加为关键帧到地图中。

```c
new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero()); //将第一帧的位姿设为单位阵作为参考
klt_homography_init_.addFirstFrame(new_frame_)
new_frame_->setKeyframe();  //is_keyframe_ = true
map_.addKeyframe(new_frame_);
```

# 2 processSecondFrame
用光流法跟踪第一帧，计算单应矩阵，得到相机位姿，计算特征点深度。

```c
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_); 
```

然后进行two-frame BA ，并将第二帧加入到深度滤波器。

```c
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);
    // add frame to map
  map_.addKeyframe(new_frame_);
```

# 3 processFrame
## 3.1 sparseimg_align
**目的**：最小化光度误差来优化相机位姿。
**步骤**：
1）通过加权方法计算**参考帧**中每个fts对应的patch的光度和雅克比矩阵（每个patch要计算4X4个雅各比）。将所有的雅克比存在jacobian_cahe_中。
```c
void SparseImgAlign::precomputeReferencePatches()
```
2）用**参考帧的雅可比作为当前帧的雅可比矩阵**。构造直接法的目标函数。计算两帧中对应点的光度误差；计算H和b
```c
double SparseImgAlign::computeResiduals()  
```
3）对构造的目标函数求增量解，最后再迭代得到新位姿
```c
int SparseImgAlign::solve() 
```

## 3.2  map reprojector & feature alignment
目的：经过上一步将相机位姿优化了。现在需要优化特征点的位置了。于是将地图中的点重投影回new_frame, 并进行feature aligment
1）map reprojector：将地图中的点重投影回new_frame，并寻找相应的特征点

```c
Reprojector::reprojectMap(new_frame_, overlap_kfs_);//overlap_kfs_是有共视点的帧
```
细节：
先用map_.getcloseKeyframes来获得与frame相近的关键帧，将它们按距离由小到大排序，**得到最近的N个关键帧**。
将每个关键帧对应的fts_用reprojectPoint重投影到frame中。然后将map_中的candidates用reprojectPoint也重投影到frame中。
最后，对于frame，每个cell中有许多投影点。用reprojectCell对**每个cell选出一个点（重要）**。
2）feature aligment：优化cell包含的点的位置
方法是对每个cell的各个点进行findMatchDirect（），如果某个点满足要求则选拔出来作为这个cell的代表。注意该点的位置在findMatchDirect（）就被优化了。这样，每个cell只需优化一个特征块的位置，实现了加速。
**注意**：feature aligment是在reprojectMap函数中被调用执行的。过程：reprojectMap( )-->reprojectCell( )-->findMatchDirect ( )

```c
  /// Find a match by directly applying subpix refinement.
  /// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
  bool findMatchDirect(const Point& pt,   const Frame& frame,   Vector2d& px_cur);
```



## 3.3  pose optimization
再次优化相机位姿。方法是：最小化重投影位置误差来调整位姿（李代数），也称Motion-only BA。
此步直接调用pose_optimizer.h 中的optimizeGaussNewton（）来完成。

```c
void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs);	
```
## 3.4  structure optimization
优化空间点的位置，方法也是最小化重投影位置误差，也称structure-only BA。
调用FrameHandlerBase中的optimizeStructure( )。

```c
// 调整三维点坐标(x,y,z)优化重投影位置误差
optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
```

```c
void FrameHandlerBase::optimizeStructure(
    FramePtr frame,
    size_t max_n_pts,
    int max_iter)
{
  deque<Point*> pts;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point != NULL)
      pts.push_back((*it)->point);
  }
  max_n_pts = min(max_n_pts, pts.size());
  nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
  for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
  {
    (*it)->optimize(max_iter);
    (*it)->last_structure_optim_ = frame->id_;
  }
}
```
其中的optimize( )函数在point.h中
```c
/// Optimize point position through minimizing the reprojection error.！！！！！！！！！！ 
void optimize(const size_t n_iter);//通过最小化重投影误差来优化。用point->obs_来进行优化

```
## 3.5 判断当前帧是否为关键帧
这一步仍然是在processframe（）中
KF 的判断标准: 如果new_frame_ 跟与它相邻的所有KF之间的相对平移都超过了场景平均深度的12%

如果是，则设为关键帧  new_frame_->setKeyframe(); 再初始化深度滤波器depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

如果不是,那就直接用来更新深度值 depth_filter_->addFrame(new_frame_)

# 4 relocalizeFrame 重定位
回顾一下：VO通过 stage_ 来确定当前帧需要进行哪一步操作： processFirstFrame，processSecondFrame，processFrame，relocalizeFrame
当 stage_ == STAGE_RELOCALIZING 进行重定位。
**启动重定位操作的条件：**（参考[1]）

> 1.位姿变化过大，导致可用于跟踪的特征点数太少。
> 2.重投影以后，利用多关键帧和当前帧的匹配点进行相机位姿优化的过程中，不断丢弃重投影误差大的点。优化完以后，如果**inlier数目较少**，跟踪失败，开启重定位。


重定位函数就在frame_handler_mono.cpp中定义

```c
FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
            const SE3& T_cur_ref,
            FramePtr ref_keyframe)
    {
      SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
      if(ref_keyframe == nullptr)
      {
        SVO_INFO_STREAM("No reference keyframe.");
        return RESULT_FAILURE;
      }
      SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                               30, SparseImgAlign::GaussNewton, false, false);
      size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
      if(img_align_n_tracked > 30)
      {
        SE3 T_f_w_last = last_frame_->T_f_w_;
        last_frame_ = ref_keyframe;
        FrameHandlerMono::UpdateResult res = processFrame();
        if(res != RESULT_FAILURE)
        {
          stage_ = STAGE_DEFAULT_FRAME;
          SVO_INFO_STREAM("Relocalization successful.");
        }
        else
          new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
        return res;
      }
      return RESULT_FAILURE;
    }
```

   **通过ref_keyframe来和new_frame_进行sparse_image_alignment**  .如果返回的追踪上的点的数量小于30，则重定位失败。大于30，则将 ref_keyframe设为last_frame, 进行processFrame(), 只要返回值不是RESULT_FAILURE,就说明重定位成功。
# 5 depth_filter
前面的步骤主要是位姿估计线程的内容。而depth_filter是SVO中的mapping线程的主要内容。两线程并行。
可以将mapping线程分为深度估计和深度滤波两部分。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190421101209148.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
**流程：**
1）当出现新的关键帧 r 时，作者在r上选取若干特征点（即seed），每个特征点对应一个深度估计，其**初值为该帧的平均深度**，并被赋予极大的不确定性（图中浅绿色部分）。
2）后续帧 { Ik } 对它能观测到的seed的深度估计产生贡献。具体而言，对r上深度还没有确定的点{p,u}，根据Tk,r找到p对应的极线Lp，**在极线上搜索和u最相似的点u′**，通过**三角测量**计算得到深度及不确定性（图中深绿色部分）[\[2\]](https://www.cnblogs.com/luyb/p/5773691.html) 。

> **极线搜索匹配点u'的方法**：[\[1\]](https://blog.csdn.net/heyijia0327/article/details/51083398)
> 如果极线段很短，小于两个像素，那直接使用上面求位姿时提到的Feature Alignment光流法就可以比较准确地预测特征位置。
> 如果极线段很长，那分两步走:  (1) 在极线段上间隔采样，对采样的多个特征块一一和参考帧中的特征块匹配，用Zero mean Sum of Squared Differences 方法对各采样特征块评分，那个得分最高，说明他和参考帧中的特征块最匹配;  (2) 就是在这个得分最高点附近使用Feature Alignment得到次像素精度的特征点位置。


3）由于有多个后续帧对关键帧  r 进行估计，因此有多个深度信息，为此利用贝叶斯概率模型更新p点的估计。当p的深度估计收敛时（不确定性足够小），计算其三维坐标，并加入地图中，用于位姿估计。

> 深度滤波使用的概率模型是一个**高斯分布**加上一个设定在最小深度dmin和最大深度dmax之间的**均匀分布**。这个均匀分布的意义是假设会有一定的概率出现错误的深度估计值。[\[1\]](https://blog.csdn.net/heyijia0327/article/details/51083398)


# 参考文章
[1] https://blog.csdn.net/heyijia0327/article/details/51083398
[2] https://www.cnblogs.com/luyb/p/5773691.html
