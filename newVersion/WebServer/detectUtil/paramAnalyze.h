#pragma once
#include "opencv2/opencv.hpp"
#include "paraType.h"

/// <summary>
/// 解析圆靶标mask
/// </summary>
/// <param name="mSrcImage">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetCircleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                    DetectParameterV3 &MarkParam);

/// <summary>
/// 解析梅花孔靶标mask
/// </summary>
/// <param name="mSrcImage">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetPlumParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                  DetectParameterV3 &MarkParam,
                  std::vector<std::pair<cv::Mat, cv::Point>> &refinedRoi);

/// <summary>
/// 解析孔矩阵靶标mask
/// </summary>
/// <param name="mSrcImage">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetMatrixHoleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                        DetectParameterV3 &MarkParam,
                        std::vector<std::pair<cv::Mat, cv::Point>> &refinedRoi);

/// <summary>
/// 解析矩形靶标mask
/// </summary>
/// <param name="mSrcImage">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetRectangleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                       DetectParameterV3 &MarkParam);

/// <summary>
/// 解析复合梅花孔靶标mask
/// </summary>
/// <param name="mSrcImage">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetComplexPlumParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                         DetectParameterV3 &MarkParam,
                         std::vector<std::pair<cv::Mat, cv::Point>> &refinedRoi);

/// <summary>
/// 解析圆环靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetRingParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                  DetectParameterV3 &MarkParam);

/// <summary>
/// 解析CircleRing靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetCircleRingParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                        DetectParameterV3 &MarkParam);

/// <summary>
/// 解析Cross靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetCrossParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                   DetectParameterV3 &MarkParam);

/// <summary>
/// 写入从分割mask解析到的参数
/// 跟据类别编号读取相应的靶标配置文件作为模板、正常解析模板xml
/// markparam子节点并更新相应字段、调用tinyxml接口进行xml文件写入
/// </summary>
/// <param name="class_id">靶标类别</param>
/// <param name="MarkParam">从分割mask中解析到的靶标参数</param>
/// <returns></returns>
bool MarkParamWrite(short class_id, const DetectParameterV3 MarkParam,
                    char *detectConfig);

/// <summary>
/// 计算轮廓中心对称性得分(中心旋转交并比)，仅针对单一轮廓
/// </summary>
/// <param name="contour">输入轮廓</param>
/// <param name="threshold">中心对称性阈值</param>
/// <param name="size">所处图片尺寸</param>
/// <returns></returns>
bool JudgeCentralSymmetry(std::vector<cv::Point> contour, double threshold,
                          cv::Size2i size);

/// <summary>
/// 根据输入的评判数据对轮廓进行分类，数值接近的分为一类，将一组数据分为n个段
/// </summary>
/// <param name="dataVector">输入的数据</param>
/// <param name="clusterData">分类后的数据</param>
/// <returns></returns>
bool contourCluster(
    std::vector<std::pair<double, std::vector<cv::Point>>> inputContour,
    std::vector<std::pair<std::vector<std::vector<cv::Point>>, cv::Point2f>> &clusteredContour,
    double AverageThreshold, double VarianceThreshold);

/// <summary>
/// 计算添加某值后的归一化方差
/// </summary>
/// <param name="datalist">数据列表</param>
/// <param name="average">列表均值</param>
double calNormalizationVar(std::vector<double> datalist, double average);

/// <summary>
/// 迭代圆拟合
/// </summary>
/// <param name="contour"></param>
/// <param name="fCenterX"></param>
/// <param name="fCenterY"></param>
/// <param name="fRadius"></param>
/// <returns></returns>
bool iterCircleFitting(std::vector<cv::Point> contour, float &fCenterX,
                       float &fCenterY, float fRadius);

/// <summary>
/// 判断靶标颜色
/// </summary>
/// <param name="mSrcImage">输入图像</param>
/// <param name="contour">靶标轮廓</param>
/// <param name="innerContour">可能的内层轮廓</param>
/// <returns></returns>
bool JudgeIsWhite(cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> contour,
                  std::vector<cv::Point> innerContour);
