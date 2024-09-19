#pragma once
#include "opencv2/opencv.hpp"
#include "paraType.h"

class CFilter
{
public:
  CFilter();
  ~CFilter();

  void getFilter(cv::Mat mSrcImage, int nType, std::string strParament,
                 cv::Mat &mDesImage);
  /// <summary>
  /// 高斯滤波
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="strParament"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool gaussianBlurFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage);

  /// <summary>
  /// 中值滤波
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="strParament"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool medianBlurFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage);

  /// <summary>
  /// 双边滤波-保持边缘、降噪平滑
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="strParament"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool bilateralFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage);
};