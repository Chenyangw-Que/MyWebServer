#pragma once
#include "opencv2/opencv.hpp"
#include "paraType.h"
#include "openvino/openvino.hpp"
class CDeeplabV3 {
public:
  CDeeplabV3(std::string strModelPath, std::string strLog = "./log"); // 初始化
  ~CDeeplabV3();

  bool getSegmentImage(const cv::Mat mSrcImage, cv::Mat &mDesImage);


  bool getSegmentMask(const cv::Mat mSrcImage, cv::Mat &mDesImage,
                      short classId);

private:
  void modelInit();                   //模型参数初始化
  void preprocess(cv::Mat mSrcImage); // 对输入的图片进行预处理，转为vector
  void predict();                     // 执行推理
  void postprocess(cv::Mat &mDesImage); // 对输出的结果进行处理，可视化

  void preprocessPadding(const cv::Mat mSrcImage); //先填充再resize的预处理方法
  void
  postprocessPadding(cv::Mat &mDesImage,
                     unsigned short int classnum); // 填充预处理配套的后处理

private:
  std::string m_strLog;    //日志文档
  std::string m_strModel;  //模型路径
  ov::Tensor input_tensor; //输入张量
  ov::Shape input_shape;
  ov::InferRequest infer_request;
  float *input_values;
  float *output_values;
  int image_h;
  int image_w;
  int model_size = 512;
};
