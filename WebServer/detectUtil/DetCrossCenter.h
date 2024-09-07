#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetCross
{
public:
    /// <summary>
    /// 构造函数
    /// </summary>
    /// <param name="strLog">日志文件路径</param>
    /// <param name="param">靶标参数结构体</param>
    /// <param name="algorithmPara">算法参数结构体</param>
    CdetCross(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetCross();

    /// <summary>
    /// 通用十字检测算法
    /// </summary>
    /// <param name="mSrcImage">输入检测图片</param>
    /// <param name="bIsCalibrate">标定检测</param>
    /// <param name="pCenterPoint">检测靶标中心点</param>
    /// <param name="dScore">靶标检测得分</param>
    /// <returns></returns>
    bool getCrossCenter(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, double &dScore);

    /// <summary>
    /// 通用十字检测算法,返回中心四角点
    /// </summary>
    /// <param name="mSrcImage">输入检测图片</param>
    /// <param name="bIsCalibrate">标定检测</param>
    /// <param name="pCenterPoint">检测靶标中心点</param>
    /// <param name="dScore">靶标检测得分</param>
    /// <returns></returns>
    bool getCrossCenterFourPoint(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, std::vector<cv::Point2d> &fourPoint, double &dScore);

    /// <summary>
    /// 半导体复合十字检测算法
    /// </summary>
    /// <param name="mSrcImage">输入检测图片</param>
    /// <param name="vecCenterPoint">检测靶标中心点集合</param>
    /// <param name="dScore">靶标检测得分</param>
    /// <returns></returns>
    bool getSEMIComplexCrossCenter(const cv::Mat mSrcImage, std::vector<cv::Point2d> &vecCenterPoint, double &dScore);

private:
    CdetCross &operator=(const CdetCross &);

    /// <summary>
    /// 单直线检测算法
    /// </summary>
    /// <param name="mSrcImage">输入图像</param>
    /// <param name="pCenterPoint">直线中心点坐标</param>
    /// <returns></returns>
    bool detSingleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint, std::vector<cv::Point2d> &fourPoint);

    /// <summary>
    /// 平行双直线检测算法
    /// </summary>
    /// <param name="mSrcImage">输入图像</param>
    /// <param name="pCenterPoint">直线中心点坐标</param>
    /// <returns></returns>
    bool detDoubleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint);

    /// <summary>
    /// 单直线区域获取
    /// </summary>
    /// <param name="mSrcImage">待检测图片</param>
    /// <param name="nLength">长</param>
    /// <param name="nBorderWidth">宽</param>
    /// <returns></returns>
    std::vector<std::pair<cv::Mat, cv::Point>> getSegmentLine(const cv::Mat mSrcImage,
                                                              int nLength, int nBorderWidth);

private:
    /// <summary>
    /// 检测左竖向直线
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="vecPoint">检测到的直线坐标点</param>
    /// <returns></returns>
    bool detLiftVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// 检测右竖向直线
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="vecPoint">检测到的直线坐标点</param>
    /// <returns></returns>
    bool detRightVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// 检测上横向直线
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="vecPoint">检测到的直线坐标点</param>
    /// <returns></returns>
    bool detTopHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// 检测下横向直线
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="vecPoint">检测到的直线坐标点</param>
    /// <returns></returns>
    bool detDownHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    //获取拟合直线参数
    //通过色差获取直线分界线，效果不是很理想，无法确定最佳分界阈值
    bool getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c);

    //四点确定直线交点
    cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4);

private:
    std::string m_strLog;    //日志文档
    int m_nWidth;            //十字宽度
    int m_nHeight;           //十字高度
    int m_nDimeter;          //直径参数
    int m_nThreshold;        //误差参数
    bool m_bIsWhite;         //靶标为白色
    int m_nAlgorithm;        //识别算法
    double m_dLineCount;     //复合十字线段数
    double m_dAngle;         //旋转角度
    bool m_bIsPolarChange;   //极性检测
    bool m_bIsInnerToExtern; //由内到外筛选
};
