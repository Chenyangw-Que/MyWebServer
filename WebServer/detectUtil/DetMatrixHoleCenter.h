#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"
class CdetHoleMatrix
{
public:
    /// <summary>
    /// 构造函数
    /// </summary>
    /// <param name="strLog">日志文件路径</param>
    /// <param name="param">靶标参数结构体</param>
    /// <param name="algorithmPara">算法参数结构体</param>
    CdetHoleMatrix(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetHoleMatrix();

    /// <summary>
    /// 获取每一个小圆的圆心坐标
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="dAngle">旋转角度</param>
    /// <param name="vecEveryHoleMatrixPoint">检测到每一个小圆中心</param>
    /// <param name="pHoleMatrixCenter">靶标中心点</param>
    /// <returns></returns>
    bool getEveryLittleCircle(const cv::Mat mSrcImage, double dAngle, std::vector<cv::Point2d> &vecEveryAccurateLocation,
                              cv::Point2d &pHoleMatrixCenter, double &dScore);

    /// <summary>
    /// 直接定位小圆，调用计算圆心计算算法
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="dAngle">旋转角度</param>
    /// <param name="vecEveryHoleMatrixPoint">检测到每一个小圆中心</param>
    /// <param name="pHoleMatrixCenter">靶标中心点</param>
    /// <returns></returns>
    bool getEveryLittleCircleDirect(const cv::Mat mSrcImage, const std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> refinedRoi, std::vector<cv::Point2d> &vecEveryAccurateLocation,
                                    cv::Point2d &pHoleMatrixCenter, double &dScore);

    //若干个靶点计算均值
    cv::Point2d getMeanPoint(std::vector<std::vector<cv::Point2d>> one);

private:
    CdetHoleMatrix &operator=(const CdetHoleMatrix &);

    /// <summary>
    /// 通过入参确定是否存在中心圆
    /// </summary>
    /// <returns></returns>
    bool isExistCenterMark();

    /// <summary>
    /// 推测每个靶标的大致中心点位置
    /// </summary>
    /// <param name="mSrcImage">输入图像</param>
    /// <returns></returns>
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> getRoughPointLocation(cv::Mat mSrcImage);

    /// <summary>
    /// 检测每一个小圆的中心点坐标
    /// </summary>
    /// <param name="mSrcImage">输入图像</param>
    /// <param name="vecRoughLocation">粗定位坐标</param>
    /// <param name="vecAccurateLocation">精确检测坐标</param>
    /// <param name="pHoleMatrixCenter">孔矩阵中心点坐标</param>
    /// <param name="dScore">检测得分</param>
    /// <returns></returns>
    bool detEveryCircleLocation(cv::Mat mSrcImage, std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore);

    /// <summary>
    /// 大直径单圆检测
    /// </summary>
    /// <param name="vecRoughLocation">每一个单圆存储容器</param>
    /// <param name="vecCircleCenter">检测结果</param>
    /// <returns></returns>
    bool detEveryBigCircleDLPrior(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                  std::vector<std::vector<cv::Point2d>> &vecCircleCenter);

    /// <summary>
    /// 对检测成功的小圆进行筛选
    /// </summary>
    /// <param name="vecCircleCenter">检测成功小圆坐标容器</param>
    /// <param name="vecAccurateLocation">筛选后坐标容器</param>
    /// <param name="pHoleMatrixCenter">筛选后整体中心点坐标</param>
    /// <param name="dScore">检测得分</param>
    /// <returns></returns>
    bool analysisDetSuccessCircle(std::vector<std::vector<cv::Point2d>> vecCircleCenter,
                                  std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore);

private:
    //*********************************小直径单圆检测*********************************
    /// <summary>
    /// 小直径单圆检测
    /// </summary>
    /// <param name="vecRoughLocation">每一个单圆存储容器</param>
    /// <param name="vecCircleCenter">检测结果</param>
    /// <returns></returns>
    bool detEveryLittleCircle(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                              std::vector<std::vector<cv::Point2d>> &vecCircleCenter);

    /// <summary>
    /// 自适应阈值分割检测小圆算法
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="nThreshold">分割阈值</param>
    /// <param name="roughPoint">检测到的中心点坐标</param>
    /// <returns></returns>
    bool getLittleCircleAutoSeg(cv::Mat mSrcImage, int nThreshold, cv::Point2d &roughPoint);

    /// <summary>
    /// 统计阈值分割检测小圆算法
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="roughPoint">检测到的中心点坐标</param>
    /// <returns></returns>
    bool getLittleCircleStatisSeg(cv::Mat mSrcImage, cv::Point2d &roughPoint);

    /// <summary>
    /// 轮廓检测拟合检测小圆算法
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="roughPoint">检测到的中心点坐标</param>
    /// <returns></returns>
    bool getLittleCircleContourFit(cv::Mat mSrcImage, cv::Point2d &roughPoint);

    /// <summary>
    /// 象限轮廓筛选算法
    /// </summary>
    /// <param name="mSrcImage">输入图片</param>
    /// <param name="vecInput">带筛选轮廓</param>
    /// <param name="pRoughCenter">小圆粗定位中心点坐标</param>
    /// <param name="pCenter">小圆精确检测到的参数结果</param>
    /// <returns></returns>
    int scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput,
                            cv::Point2d pRoughCenter, cv::Point3d &pCenter);

    //获取大直径靶标中心
    bool getBigSingleCenter(cv::Mat mSrcImage, cv::Point2d &roughPoint);

private:
    std::string m_strLog;     //日志文档
    int m_nRows;              //列
    int m_nCols;              //行
    int m_RowSpan;            //列间距
    int m_ColSpan;            //行间距
    int m_nDiameter;          //直径参数
    int m_nThreshold;         //误差参数
    int m_nSpanRange;         //直径与间距差
    bool m_bIsWhite;          //靶标为白色
    int m_nAlgorithm;         //识别算法
    bool m_bWhiteEdge;        //白色靶标边缘
    bool m_bIsPolarChange;    //极性检测
    float m_fHoleMatrixSocre; //检测阈值参数
    bool m_bIsEnhance;        //图像增强
    double m_dAngle;          //旋转角度
};
