源码：https://download.csdn.net/download/qq_43145072/11083273
棋格标定板下载地址：https://download.csdn.net/download/qq_43145072/11083282
只涉及标定程序及步骤。标定原理参考其他博客。
在源码的基础上稍加修改即可实现对自己的单目相机标定。
需要修改的地方：
## 1 修改in_my_camera_data.xml
（1）角点数修改
```c
<!-- Number of inner corners per a item row and column. (square, circle) -->
<BoardSize_Width>7</BoardSize_Width>
<BoardSize_Height>6</BoardSize_Height>
```
7和6分别为横向和纵向角点个数。比如下图角点数就是7，6。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190402231143642.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
（2）棋格尺寸

```c
 <!-- The size of a square in some user defined metric system (pixel, millimeter)-->
 <Square_Size>24</Square_Size>
```
24mm是每个棋盘格的尺寸，根据实际棋格尺寸修改。
（3）图像数量
修改你的相机所拍摄的图片数量（图片放在/imges文件夹下）

```c
 <!-- How many frames to use, for calibration. -->
  <Calibrate_NrOfFrameToUse>15</Calibrate_NrOfFrameToUse>
```
（4）结果输出文件设定

```c
  <!-- The name of the output log file. -->
  <Write_outputFileName>"out_my_camera_data.yml"</Write_outputFileName>
```
out_my_camera_data.yml是你要输出结果的文件。可保持不变。

## 2 修改my_camera.xml 
my_camera.xml 用于告知图像位置和数量（要与之前设定的一致），具体代码如下：
```c
<?xml version="1.0"?>
<opencv_storage>
<images>
  imges/1.jpg
  imges/2.jpg
  imges/3.jpg
  imges/4.jpg
  imges/5.jpg
  imges/6.jpg
  imges/7.jpg
  imges/8.jpg
  imges/9.jpg
  imges/10.jpg
  imges/11.jpg
  imges/12.jpg
  imges/13.jpg
  imges/14.jpg
  imges/15.jpg

</images>
</opencv_storage>
```
## 3 运行
在Camera_calibration文件夹下依次执行

```c
cmake.
```

```c
make
```
```c
./camera_calibration
```
则开始进行标定，待图片检测完，会输出out_my_camera_data.yml文件，包含标定结果，如相机内参数、畸变系数等。如下：

```c
%YAML:1.0
---
calibration_Time: "2019年04月02日 星期二 21时14分35秒"
nrOfFrames: 15
image_Width: 640
image_Height: 480
board_Width: 7
board_Height: 6
square_Size: 24.
FixAspectRatio: 1.
# flags:  +fix_aspectRatio +fix_principal_point +zero_tangent_dist
flagValue: 14
Camera_Matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 8.2610193280047451e+02, 0., 320., 0., 8.2610193280047451e+02,
       240., 0., 0., 1. ]
Distortion_Coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ 2.3038828371953732e-01, -5.6574255412185188e+00, 0., 0.,
       3.5455164978427696e+01 ]
Avg_Reprojection_Error: 1.0114731921363718e+00
Per_View_Reprojection_Errors: !!opencv-matrix
   rows: 15
   cols: 1
   dt: f
   data: [ 7.36173391e-01, 4.50691104e-01, 6.23305082e-01,
       7.38970578e-01, 1.71988678e+00, 1.91684270e+00, 1.87989831e+00,
       8.41988862e-01, 4.85461861e-01, 5.06118178e-01, 8.46351445e-01,
       9.17777896e-01, 7.77284145e-01, 2.98195392e-01, 2.18411252e-01 ]
# a set of 6-tuples (rotation vector + translation vector) for each view
Extrinsic_Parameters: !!opencv-matrix
   rows: 15
   cols: 6
   dt: d
   data: [ -4.6724971106899821e-01, 7.6244283040321192e-01,
       2.8983092061514539e+00, 7.3664105184506710e+01,
       3.2841763752176838e+01, 3.2979956450589361e+02,
       2.7439120767793707e-01, -2.2591598536547688e-01,
		...
```

## 参考：
基于OpenCV单目相机的快速标定--源码、工程、实现过程 https://blog.csdn.net/Shawn_Zhangguang/article/details/77801833
