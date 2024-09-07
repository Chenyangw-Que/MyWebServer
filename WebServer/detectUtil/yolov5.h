#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <openvino/openvino.hpp>
class CYoloV5 {
public:
  CYoloV5(std::string strModelPath, std::string strLog,
          const DetectParameterV3 temParament);
  CYoloV5(std::string strModelPath, std::string strLog);
  CYoloV5(std::string strModelPath);
  ~CYoloV5();

  /// <summary>
  /// 靶标区域粗定位
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getTargetRoi(const cv::Mat mSrcImage,
                    std::vector<std::pair<cv::Mat, cv::Point>> &mROiImage);

  /// <summary>
  /// 靶标区域粗定位,返回内容中带有靶标类别信息
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool
  getTargetClassRoi(const cv::Mat mSrcImage,
                    std::vector<std::pair<cv::Mat, cv::Point3d>> &mROiImage);
  
  cv::Mat getResultImage();
private:
  CYoloV5 &operator=(const CYoloV5 &);

  /// <summary>
  /// 目标检测
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="strModelPath"></param>
  /// <param name="vecBoxes"></param>
  /// <returns></returns>
  void targetDetect(cv::Mat mSrcImage, std::vector<TargetDetect> &vecBoxes);

  /// <summary>
  /// 图像尺度变化
  /// </summary>
  /// <param name="img"></param>
  /// <param name="paddings"></param>
  /// <param name="new_shape"></param>
  /// <returns></returns>
  cv::Mat letterbox(cv::Mat &img, std::vector<float> &paddings,
                    std::vector<int> new_shape = {640, 640});

  /// <summary>
  /// 根据入参获取靶标规格范围
  /// </summary>
  void getMarkRange();

  /// <summary>
  /// 检测结果校验
  /// </summary>
  /// <param name="vecBoxes"></param>
  void scanBoxResult(std::vector<TargetDetect> &vecBoxes);

  /// <summary>
  /// 基于置信度得分和检测框长宽比阈值进行检测框过滤
  /// </summary>
  /// <param name = "vecBoxes"></param>
  void checkBoxResult(std::vector<TargetDetect> &vecBoxes);

private:
    ov::Core core_;
    //使用CPU进行模型预测
    ov::CompiledModel compiled_model_;
    //推理初始化需求
    ov::InferRequest infer_request_;


  std::string m_strLog;           //日志文档
  std::string m_strModel;         //模型路径
  int m_nMarkType;                //待检测靶标类型
  double m_dWidth[PARAM_SIZE];    //靶标宽度
  double m_dHeight[PARAM_SIZE];   //靶标高度
  double m_dDiameter[PARAM_SIZE]; //靶标直径
  int m_nWHCount;                 //宽高数量
  int m_nDiameterCount;           //直径数量
  int m_nRows;                    //列数
  int m_nCols;                    //行数
  double m_dRowspan;              //列间距
  double m_dColspan;              //行间距
  int m_nCenterDistance;          //中心点距离
  int m_nMinRange;                //范围下限
  int m_nMaxRange;                //范围下限
  int image_col;
  int image_row;

  const std::vector<std::string> m_class_names = {"circle",
                                                  "complexCrossOne",
                                                  "complexMatrixHole",
                                                  "cross",
                                                  "rectangle",
                                                  "matrixHole",
                                                  "plum",
                                                  "ring",
                                                  "complexPlum",
                                                  "circleRing",
                                                  "corner"};

  std::vector<cv::Scalar> m_colors = {
      cv::Scalar(0, 0, 255),   cv::Scalar(0, 255, 0),   cv::Scalar(255, 0, 0),
      cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255),
      cv::Scalar(0, 0, 128),   cv::Scalar(0, 128, 0),   cv::Scalar(128, 0, 0),
      cv::Scalar(128, 0, 128), cv::Scalar(128, 128, 0)};
      
  cv::Mat curResultPic_;
};