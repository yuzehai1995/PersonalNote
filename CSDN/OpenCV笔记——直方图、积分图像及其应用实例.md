本文主要参考《OpenCV计算机视觉编程攻略(第三版)》
完整代码已上传博客。链接：https://download.csdn.net/download/qq_43145072/11126303

@[toc]
# 1 灰度图的直方图

> **灰度直方图**是一幅图像中各像素灰度值出现次数或频数的统计结果。换句话说，直方图统计了图像每一个强度值（灰度）所具有的像素个数。一般来说，灰度图的直方图横坐标是0～255灰度值。

本例使用的图片是下图的灰度形式：（来自《OpenCV计算机视觉编程攻略(第三版)》）
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418112604459.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)

## 1.1 计算直方图
计算直方图使用的是opencv自带函数：

```c
CV_EXPORTS void calcHist( const Mat* images, int nimages,
                          const int* channels, InputArray mask,
                          OutputArray hist, int dims, const int* histSize,
                          const float** ranges, bool uniform = true, bool accumulate = false );
```
在Histogram1D类里面实现 calcHist( ) 的使用
```c
class Histogram1D 
{
  private:

    int histSize[1];         // number of bins in histogram（对于灰度图就是256）
	float hranges[2];        // range of values（范围是0-255）
    const float* ranges[1];  // pointer to the different value ranges
    int channels[1];         // channel number to be examined 

  public:

	Histogram1D() { //构造函数，用于初始化一些参数

		// Prepare default arguments for 1D histogram
		histSize[0]= 256;   // 256 bins
		hranges[0]= 0.0;    // from 0 (inclusive)
		hranges[1]= 256.0;  // to 256 (exclusive)
		ranges[0]= hranges; 
		channels[0]= 0;     // we look at channel 0
	}
	cv::Mat getHistogram(const cv::Mat &image) 
	{
		cv::Mat hist;  // 
		// Compute 1D histogram with calcHist
		cv::calcHist(&image, //输入图像
			1,			// 输入图像数量，为1
			channels,	// 使用的通道，为0
			cv::Mat(),	// no mask is used
			hist,		// 获得的直方图
			1,			// 直方图为一维
			histSize,	// number of bins
			ranges		// pixel value range
			);
		return hist;
	}
};
```
在主函数输入以下程序即可得到直方图。由于**灰度图得到直方图是一个一维数组**，遍历十分简单。

```c
	cv::Mat image= cv::imread("group.jpg",0);
	
	Histogram1D h;// The histogram object
	cv::Mat histo= h.getHistogram(image);// Compute the histogram

	for (int i=0; i<256; i++)  //遍历直方图
		cout << "Value " << i << " = " << histo.at<float>(i) << endl;  
```
## 1.2 绘制直方图
若要**绘制直方图**可在Histogram1D类里面加入如下函数，并在主函数调用即可

```c
 static cv::Mat getImageOfHistogram(const cv::Mat &hist, int zoom) {

        // Get min and max bin values
        double maxVal = 0;
        double minVal = 0;
        cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);

        // get histogram size
        int histSize = hist.rows;

        // Square image on which to display histogram
        cv::Mat histImg(histSize*zoom, histSize*zoom, CV_8U, cv::Scalar(255));

        // set highest point at 90% of nbins (i.e. image height)
        int hpt = static_cast<int>(0.9*histSize);

        // Draw vertical line for each bin
        for (int h = 0; h < histSize; h++) {

            float binVal = hist.at<float>(h);
            if (binVal>0) {
                int intensity = static_cast<int>(binVal*hpt / maxVal);
                cv::line(histImg, cv::Point(h*zoom, histSize*zoom),
                    cv::Point(h*zoom, (histSize - intensity)*zoom), cv::Scalar(0), zoom);
            }
        }

        return histImg;
    }
```
绘制出的直方图如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418110849314.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
## 1.3 补充
计算彩色图的直方图方法类似，同样是调用cv::calcHist（）函数，只是将输入参数做一定修改即可。返回的直方图是三维的cv::Mat实例。
# 2 直方图均衡化
在一个完全均衡化的直方图中，灰度分布是均匀的，意味着50%的像素的灰度小于128，25%的像素的灰度小于64......。opencv进行直方图均值化只需一个函数：

```c
    cv::equalizeHist( image_src, image_result );
```
均衡化后以及均衡化直方图如下：

![均衡化图](https://img-blog.csdnimg.cn/20190418112022191.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418112302690.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
# 3 反向投影直方图进行图像检测
**下面实例演示如何检测一幅图中可能是云彩的地方。**
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418191637341.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)

先定义一个感兴趣区域ROI

```c
cv::Mat imageROI;
imageROI = image(cv::Rect(216,33,24,30) );
```
然后调用之前定义好的类来建立ROI的直方图

```c
Histogram1D h;
cv::Mat hist= h.getHistogram(imageROI);
```
归一化直方图，得到该区域像素的灰度值概率分布。

```c
cv::normalize(hist,hist,1.0);
```
反投影

```c
cv::calcBackProject(&image,         //反投影到image
                      1,            // we only use one image at a time
                      channels,     // vector specifying what histogram dimensions belong to what image channels
                      hist,   // the histogram we are using
                      result,       // the resulting back projection image
                      ranges,       // the range of values, for each dimension
                      255.0         // the scaling factor is chosen such that a histogram value of 1 maps to 255
```
由此即可得到反投影概率分布图 —— Backprojection result （越亮则是云彩的概率越大）
以及阈值化处理后的图像——Detection Result。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418115552135.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
# 4 积分图像
**积分图像是用于计算图像某一区域各个像素对应值的累加和的手段**。
先看以下求像素累加和的**常规方法**：

```c
cv::Mat image = cv::imread("bike.bmp",0);
//定义ROI区域
int xo=97, yo=112;
int width=25,height=30;
cv::Mat roi(image,cv::Rect(xo,yo,width,height));

//计算累加值
cv::Scalar sum = cv::sum(roi);
```
再看一下**使用积分图像的方法**：

```c
//计算整个图的积分图像
cv::Mat integralImage;
cv::integral(image,integralImage,CV_32S);

//通过简单的加减计算某一区域的像素累加和
int sumInt = integralImage.at<int>(yo+height,xo+width)-
			integralImage.at<int>(yo+height,xo)-
			integralImage.at<int>(yo,xo+width)+
			integralImage.at<int>(yo,xo);
```

> 由此可知，使用积分图像方法，首先遍历整个图像的所有像素，然后通过三次加或减操作即可完成**任意区域**的像素累加。且计算的复杂度与积分区域大小无关。所以，在**需要对图像进行多个区域的像素累加操作时**（如动态窗口），积分图像发十分有优势。

下面看一下积分图像法的应用。

# 5 积分图像的应用
## 5.1 自适应阈值化来处理图像
（1）首先看一下普通的固定阈值方法：

```c
	cv::Mat binaryFixed;
	//超过阈值70的像素，灰度设为255，否则设为0;
	cv::threshold(image,binaryFixed,70,255,cv::THRESH_BINARY);
```
处理结果：
![固定阈值](https://img-blog.csdnimg.cn/20190418163152387.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
可见固定阈值会丢失一部分需要的文本。
（2）再看一下自适应阈值方法
自适应阈值方法通过求某一像素点所在正方形区域的**平均灰度值**sum，若该点的灰度比sum小于某一阈值，则置为0，否则置为255;
具体代码如下：

```c
cv::Mat iimage;   //积分图像
cv::integral(image ,iimage,CV_32S); //求得积分图像
```

```c
int blockSize = 21;//正方形区域大小21×21
int threshold = 10;//阈值

int halfSize= blockSize/2;
// for each row
for (int j = halfSize; j<nl - halfSize - 1; j++) //nl为总行数
{
	uchar* data = binary.ptr<uchar>(j);// get the address of row j
	int* idata1 = iimage.ptr<int>(j - halfSize);// get the address of row （j-halfSize）
	int* idata2 = iimage.ptr<int>(j + halfSize + 1);// get the address of row（j+halfSize+1）

	// for each pixel
	for (int i = halfSize; i<nc - halfSize - 1; i++) //nl为总行数列数
	{
		// 方形区域的灰度均值
		int sum = (idata2[i + halfSize + 1] - idata2[i - halfSize] -
			idata1[i + halfSize + 1] + idata1[i - halfSize]) / (blockSize*blockSize);

		// apply adaptive threshold
		if (data[i]<(sum - threshold))
			data[i] = 0;
		else
			data[i] = 255;
	}
}
```
得到的结果：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418164815744.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
很明显，效果比固定阈值的方法好很多。

> 另外，可以用OpenCV自带的函数进行自适应阈值处理：
> 
> ```c cv::adaptiveThreshold(image,           // input image
> 	                  binaryAdaptive,  // output binary image
> 					  255,             // max value for output
> 	                  cv::ADAPTIVE_THRESH_MEAN_C, // adaptive method
> 					  cv::THRESH_BINARY, // threshold type
> 					  blockSize,       // size of the block
> 					  threshold);      // threshold used ```


## 5.2 视觉追踪
先看以下结果：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418182725927.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)![在这里插入图片描述](https://img-blog.csdnimg.cn/20190418182741853.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMTQ1MDcy,size_16,color_FFFFFF,t_70)
**基本步骤**：
1 在initial Image上人工标出感兴趣区域，并计算原始直方图;
2 然后对New image转换为为多通道二值图（灰度只有0,1）并建立积分图像;
3.遍历计算各个块的像素灰度累加和（**对于二值图，灰度累加和即为灰度为1的像素总和**），并与原始直方图比较，最接近的即为检测到的区域。
**代码：**

```c
// convert to a multi-channel image made of binary planes
// nPlanes must be a power of 2
//将图像转换为多通道二值图
void convertToBinaryPlanes(const cv::Mat& input, cv::Mat& output, int nPlanes) {

	    // number of bits to mask out
	    int n= 8-static_cast<int>(log(static_cast<double>(nPlanes))/log(2.0));
	    // mask used to eliminate least significant bits
	    uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0

		// create a vector of 16 binary images
		std::vector<cv::Mat> planes;
		// reduce to nBins bins by eliminating least significant bits
		cv::Mat reduced= input&mask;

		// compute each binary image plane
		for (int i=0; i<nPlanes; i++) {

			// 1 for each pixel equals to i<<shift
			planes.push_back((reduced==(i<<n))&0x1);
		}

	    // create multi-channel image
		cv::merge(planes,output);
}
```

```c
//用于计算积分图像的类
template <typename T, int N>
class IntegralImage 
{
  public:
	  cv::Mat integralImage;
	  IntegralImage(cv::Mat image) 
	  {// (costly) computation of the integral image
		cv::integral(image,integralImage,cv::DataType<T>::type);
	  }

	  // compute sum over sub-regions of any size
	  cv::Vec<T,N> operator()(int xo, int yo, int width, int height) 
	  {
		  // window at (xo,yo) of size width by height
          return (integralImage.at<cv::Vec<T,N> >(yo+height,xo+width)
                  -integralImage.at<cv::Vec<T,N> >(yo+height,xo)
                  -integralImage.at<cv::Vec<T,N> >(yo,xo+width)
                  +integralImage.at<cv::Vec<T,N> >(yo,xo));
	  }
};
```
计算第一幅图的ROI的直方图refHistogram：
```c	
// Open image
	cv::Mat image= cv::imread("bike55.bmp",0);
	// define image roi
	int xo=97, yo=112;  int width=25, height=30;
	cv::Mat roi(image,cv::Rect(xo,yo,width,height));
// histogram of 16 bins
	Histogram1D h;
	h.setNBins(16);
	// compute histogram over image roi 
	cv::Mat refHistogram= h.getHistogram(roi);
```

通过实例化class IntegralImage类来计算第二副图的积分图像：

```c
// first create 16-plane binary image
cv::Mat planes;
convertToBinaryPlanes(image,planes,16);  //转换为多通道二值图
// then compute integral image
IntegralImage<float,16> intHisto(planes);  
```
遍历110～120行之间的积分图像并与原始直方图做比较：

```c
double maxSimilarity=0.0;
int xbest, ybest;
// 在110行到120行之间遍历
for (int y=110; y<120; y++) {
	for (int x=0; x<secondImage.cols-width; x++) 
	{
		// compute histogram of 16 bins using integral image
		histogram= intHistogram(x,y,width,height);
		// compute distance with reference histogram
		double distance= cv::compareHist(refHistogram,histogram, cv::HISTCMP_INTERSECT);
		// find position of most similar histogram
		if (distance>maxSimilarity) 
		{
			xbest= x;
			ybest= y;
			maxSimilarity= distance;
		}
		std::cout << "Distance(" << x << "," << y << ")=" << distance << std::endl;
	}
}

// draw a rectangle around target object
cv::rectangle(image,cv::Rect(xo,yo,width,height),0);
```





















