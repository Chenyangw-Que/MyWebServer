#include "paraType.h"
#include <vector>
#include "opencv2/opencv.hpp"
using namespace std;
/// <summary>
/// 连接图像中断裂的边缘
/// </summary>
/// <param name="SrcImage">输入轮廓图</param>
/// <param name="DesImage">输出轮廓图</param>
void ConnectEdge(const cv::Mat SrcImage, cv::Mat &DesImage);
//导出声明

/************************************************************************/
// 提取canny边缘
/************************************************************************/
extern "C" bool CannyEdge(const unsigned char *pSrcData, const int wSrc,
                          const int hSrc, const int wsSrc,
                          unsigned char *pDstData, const int wDst,
                          int const hDst, const int wsDst, const int cannyLow,
                          const int cannyHigh);
