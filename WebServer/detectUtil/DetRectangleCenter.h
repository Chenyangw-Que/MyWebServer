#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetRectangle
{
public:
    /// <summary>
    /// 构造函数
    /// </summary>
    /// <param name="strLog">日志文件路径</param>
    /// <param name="param">靶标参数结构体</param>
    /// <param name="algorithmPara">算法参数结构体</param>
    CdetRectangle(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetRectangle();

    /// <summary>
    /// 矩形检测算法
    /// </summary>
    /// <param name="mSrcImage">输入图像</param>
    /// <param name="bIsCalibration">标定</param>
    /// <param name="nRoateAngle">旋转角度</param>
    /// <param name="pCenterPoint">检测到的矩形参数</param>
    /// <param name="dScore">检测得分</param>
    /// <returns></returns>
    bool getRectangleCenter(const cv::Mat mSrcImage, bool bIsCalibration, int nRoateAngle,
                            Cfmee_Rectangle &pCenterPoint, double &dScore);

private:
    /// <summary>
    /// 基于四四象限线拟合的
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="nRoateAngle"></param>
    /// <param name="pCenterPoint"></param>
    /// <param name="dScore"></param>
    /// <returns></returns>
    bool fourQuadrantLineFit(const cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //获取拟合直线参数
    //通过色差获取直线分界线，效果不是很理想，无法确定最佳分界阈值
    bool getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c);

    //轮廓检测算法
    bool getContoursCenter(cv::Mat mSrcImage, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //边缘线拟合检测---大角度旋转
    //bool getFitEdgeLine(cv::Mat mSrcImage, DetectParameterV3 Param, std::string strPath, Cfmee_Rectangle& pCenterPoint, double& dScore);

    //边缘线拟合检测---小角度旋转
    bool getFitEdgeLine2(cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //四顶点排序
    void sortRectanglePoint(Cfmee_Rectangle &pCenterPoint);

    // 矩形边缘直线检测
    bool getLinePara3(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop,
                      int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara4(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift,
                      int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara3Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop,
                           int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara4Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift,
                           int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    //四点确定直线交点
    cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4);

private:
    CdetRectangle &operator=(const CdetRectangle &);

private:
    std::string m_strLog; //日志文档
    int m_nWidth;         //十字宽度
    int m_nHeight;        //十字高度
    int m_nThreshold;     //误差参数
    bool m_bIsWhite;      //靶标为白色
    int m_nAlgorithm;     //识别算法
    double m_dAngle;      //旋转角度
};
