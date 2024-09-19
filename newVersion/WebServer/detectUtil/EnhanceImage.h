#pragma once
#include "opencv2/opencv.hpp"
class CEnhance {
public:
  CEnhance();
  CEnhance(bool bIsWhite);
  ~CEnhance();
  bool getEnhanceImage(const cv::Mat mSrcImage, int nAlgorithm,
                       cv::Mat &mDesImage, int nDiameter = 0,
                       int nDiameter2 = 0, int n = 1.5, double factor = 10);

  /// <summary>
  /// 傅里叶变换
  /// </summary>
  /// <param name="mSrcImage-输入图片"></param>
  /// <param name="nAlgorithm-处理算法"></param>
  /// <param name="mDesImage-输入图片"></param>
  /// <param name="nDiameter-中心区域直径"></param>
  /// <param name="width-线宽"></param>
  /// <returns></returns>
  bool FourierTransform(const cv::Mat mSrcImage, int nAlgorithm,
                        cv::Mat &mDesImage, bool bHor, int nDiameter,
                        int width);

  //***************局部增强***************
  // mSrcImage[IN]-输入图片
  // dMeanMinScalar[IN]-均值下限
  // dMeanMaxScalar[IN]-均值上限
  // dStdMinScalar[IN]-方差下限
  // dStdMaxScalar[IN]-方差上限
  // mDesImage[OUT]-输出图片
  bool getLocalEnhance(const cv::Mat mSrcImage, double dMeanMinScalar,
                       double dMeanMaxScalar, double dStdMinScalar,
                       double dStdMaxScalar, cv::Mat &mDesImage,
                       bool bIsEnchance = false);

private:
  CEnhance &operator=(const CEnhance &);
  /// <summary>
  /// 对数变换增强
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getLogImage(const cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// 伽马变换增强
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getGammaImage(const cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// 直方图均衡化
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getEqualizeHist(const cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// 高反差保留
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getHighPass(const cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// 增强图像高频对比度
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <param name="n"></param>
  /// <param name="factor"></param>
  /// <returns></returns>
  bool getEmphasize(const cv::Mat mSrcImage, cv::Mat &mDesImage, int n = 3,
                    double factor = 1);

  /// <summary>
  /// 图像整体增强
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <param name="n"></param>
  /// <param name="factor"></param>
  /// <returns></returns>
  bool EmphasizeScale(const cv::Mat mSrcImage, cv::Mat &mDesImage, int n = 1.5,
                      double factor = 10);

  /// <summary>
  /// 直方图均衡化自实现
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <param name="nMaxDiameter">上限直径</param>
  /// <param name="nMinDiameter">下限直径</param>
  /// <returns></returns>
  bool equliaze_hist(cv::Mat mSrcImage, cv::Mat &mDesImage, int nMaxDiameter,
                     int nMinDiameter);

  /// <summary>
  /// 傅里叶频谱图频率迁移
  /// </summary>
  /// <param name="plane0"></param>
  /// <param name="plane1"></param>
  void dftshift(cv::Mat &plane0, cv::Mat &plane1);

  /// <summary>
  /// 矩形区域纹理去除
  /// </summary>
  /// <param name="SrcImage"></param>
  /// <param name="bHor"></param>
  /// <param name="nDiameter"></param>
  /// <param name="width"></param>
  void rectRoi(cv::Mat &SrcImage, bool bHor, int nDiameter, int width);

private:
  bool m_bIsWhite;
};
