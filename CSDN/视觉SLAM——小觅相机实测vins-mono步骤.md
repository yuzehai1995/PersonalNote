@[toc]
使用的是双目深度板小觅相机，只需要使用相机左摄像头和IMU。其他相机亦可参考。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190417155436484.png)
##  step1 相机对应驱动——ubuntu SDK源码安装
（1）下载与安装

注：下载安装前保证opencv等已安装。
```c
git clone https://github.com/slightech/MYNT-EYE-S-SDK.git
cd <sdk>  // <sdk> 是指MYNT-EYE-S-SDK路径
make init
make install  //最终，默认会安装在 /usr/local 目录
```

（2）编译样例

```c
cd <sdk>
make samples
```

（3）编译工具

```c
cd <sdk>
make tools
```


##  step2 相机测试、标定参数获取
1）测试相机（不使用ros） 

    cd <sdk>
    ./samples/_output/bin/api/camera_a

2）测试相机（使用ros） 

```c
cd <sdk>
make ros   //仅第一次需要
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch     // 打开摄像头，但默认不显示图像
roslaunch mynt_eye_ros_wrapper display.launch    //也可以运行这个节点，会打开 RViz 显示图像
```
   

3）获得当前工作设备的图像校准参数与imu校准参数：

```c
cd <sdk>
./samples/_output/bin/tutorials/get_img_params     //获取相机标定参数
./samples/_output/bin/tutorials/get_imu_params    //获取IMU标定参数
```
##  step3 安装vins并建立启动文件
1）在github下载vins源码，并在工作空间下编译安装

*vins源码：https://github.com/HKUST-Aerial-Robotics/VINS-Mono*


也可使用以下命令安装
```c
    cd ~/ws_vins/src   //your vins_workspace
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ../
    catkin_make
    source ~/ws_vins/devel/setup.bash
```
2）建立启动文件
第一步：在~/ws_vins/src/VINS-Mono/vins_estimator/launch下新建一个mynteye.launch文件。
第二步：在/home/fish/ws_vins/src/VINS-Mono/config文件下建立一个名为mynteye的文件夹，并新建mynteye_config.yaml文件。
**两个文件内容分别如下**：
mynteye.launch  ：
```c
<launch>
    <arg name="config_path" default = "$(find feature_tracker)/../config/mynteye/mynteye_config.yaml" />
	  <arg name="vins_path" default = "$(find feature_tracker)/../config/../" />
    
    <node name="feature_tracker" pkg="feature_tracker" type="feature_tracker" output="log">
        <param name="config_file" type="string" value="$(arg config_path)" />
        <param name="vins_folder" type="string" value="$(arg vins_path)" />
    </node>

    <node name="vins_estimator" pkg="vins_estimator" type="vins_estimator" output="screen">
       <param name="config_file" type="string" value="$(arg config_path)" />
       <param name="vins_folder" type="string" value="$(arg vins_path)" />
    </node>

    <node name="pose_graph" pkg="pose_graph" type="pose_graph" output="screen">
        <param name="config_file" type="string" value="$(arg config_path)" />
        <param name="visualization_shift_x" type="int" value="0" />
        <param name="visualization_shift_y" type="int" value="0" />
        <param name="skip_cnt" type="int" value="0" />
        <param name="skip_dis" type="double" value="0" />
    </node>

</launch>
```
mynteye_config.yaml  ：
**注意将部分参数更换（如topic和相机、imu参数）**

```c
%YAML:1.0

#common parameters
imu_topic: "/mynteye/imu/data_raw"  #换成你的IMU的话题
image_topic: "/mynteye/left/image_raw"  #换成你的相机的话题
output_path: "/home/fish/ws_vins/src/VINS-Mono/config/output_path/" #换成你的路径

#camera calibration 
model_type: PINHOLE
camera_name: camera
image_width: 752   #换成你的相机参数（step2中获取的参数）
image_height: 480  #换成你的相机参数
distortion_parameters:   #换成你的畸变参数
   k1: -0.266278
   k2: 0.0527945
   p1: -0.000182013
   p2: 0.000422317
projection_parameters:   #换成你的相机内参
   fx: 365.75
   fy: 373.236
   cx: 357.402
   cy: 241.418

# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 0   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                        # 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. Do some rotation movement at beginning.                        
#If you choose 0 or 1, you should write down the following matrix.
#Rotation from camera frame to imu frame, imu^R_cam
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [-0.00646620000000000, -0.99994994000000004, -0.00763565000000000, 0.99997908999999996, -0.00646566000000000, -0.00009558000000000, 0.00004620000000000, -0.00763611000000000, 0.99997084000000003]
#Translation from camera frame to imu frame, imu^T_cam
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.00533646000000000, -0.04302922000000000, 0.02303124000000000]

#feature traker paprameters
max_cnt: 150            # max feature number in feature tracking
min_dist: 30            # min distance between two features 
freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
equalize: 1             # if image is too dark or light, trun on equalize to find enough features
fisheye: 0              # if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points

#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.08          # accelerometer measurement noise standard deviation. #0.2   0.04
gyr_n: 0.004         # gyroscope measurement noise standard deviation.     #0.05  0.004
acc_w: 0.00004         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 2.0e-6       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.81007     # gravity magnitude

#loop closure parameters
loop_closure: 1                    # start loop closure
load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
fast_relocalization: 0             # useful in real-time and large project
pose_graph_save_path: "/home/fish/ws_vins/src/VINS-Mono/config/output_path/" # #换成你的路径

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.0                             # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

#rolling shutter parameters
rolling_shutter: 0                  # 0: global shutter camera, 1: rolling shutter camera
rolling_shutter_tr: 0               # unit: s. rolling shutter read out time per frame (from data sheet). 

#visualization parameters
save_image: 1                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
visualize_imu_forward: 0        # output imu forward propogation to achieve low latency and high frequence results
visualize_camera_size: 0.4      # size of camera marker in RVIZ


```

##  step4 开启相机节点并运行vins
1） 启动相机

```c
cd <sdk>
source ./wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```


2）启动vins

```c
cd ws_vins     //your vins_workspace
source devel/setup.bash
roslaunch vins_estimator mynteye.launch
```


3）启动可视化环境

```c
cd ws_vins     //your vins_workspace
source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```
完成。

