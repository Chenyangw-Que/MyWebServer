#pragma once
#include "DeepLabv3.h"
#include "yolov5.h"
#include "opencv2/opencv.hpp"
#include "paraType.h"
#include <vector>
#include <iostream>
#include <memory>
class CExecute
{
public:
  CExecute();
  ~CExecute();
  int MarkAutoDetection(
      const cv::Mat mSrcImage,
      std::vector<std::pair<cv::Mat, cv::Point3d>> &detBoxResult);

  int MarkAutoSegmentation(const cv::Mat SrcImage, short class_id,
                           cv::Mat &mDesImage);

  cv::Mat getDetectResult();

  int GetMaskParam(cv::Mat BinaryMask, cv::Mat ImageRoi, short class_id,
                   DetectParameterV3 &MarkParam,
                   std::vector<std::pair<cv::Mat, cv::Point>> &refinedRoi);

  int getMatrixHoleCenterAuto(
      const std::pair<cv::Mat, cv::Point3d> detbox,
      const std::vector<std::pair<cv::Mat, cv::Point>> roughPoint,
      DetectParameterV3 param, ConfigAlgorithmParam algorithmPara,
      DetectResultV3 *result);

  int getCircleCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                          DetectParameterV3 param,
                          ConfigAlgorithmParam algorithmPara,
                          DetectResultV3 *result);

  int getCrossCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                         DetectParameterV3 param,
                         ConfigAlgorithmParam algorithmPara,
                         DetectResultV3 *result);

  int getRingCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                        DetectParameterV3 param,
                        ConfigAlgorithmParam algorithmPara,
                        DetectResultV3 *result);

  int getPlumCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                        const std::vector<std::pair<cv::Mat, cv::Point>> roughPoint,
                        DetectParameterV3 param,
                        ConfigAlgorithmParam algorithmPara,
                        DetectResultV3 *result);

  int getComplexPlumCenterAuto(
      const std::pair<cv::Mat, cv::Point3d> detbox,
      const std::vector<std::pair<cv::Mat, cv::Point>> roughPoint,
      DetectParameterV3 param, ConfigAlgorithmParam algorithmPara,
      DetectResultV3 *result);

  int getCircleRingCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                              DetectParameterV3 param,
                              ConfigAlgorithmParam algorithmPara,
                              DetectResultV3 *result);

  int getRectangleCenterAuto(const std::pair<cv::Mat, cv::Point3d> detbox,
                             DetectParameterV3 param,
                             ConfigAlgorithmParam algorithmPara,
                             DetectResultV3 *result);

private:
  CExecute &operator=(const CExecute &);
  CYoloV5 *CtargetDet_;
  CDeeplabV3 *CtargetSeg_;
};

class executeHandler
{
public:
  executeHandler() {}
  ~executeHandler() {}
  static CExecute& getExecuteHandler();

private:
};