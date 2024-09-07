#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetPlum
{
public:
    /// <summary>
    /// 构造函数
    /// </summary>
    /// <param name="strLog">日志文件路径</param>
    /// <param name="param">靶标参数结构体</param>
    /// <param name="algorithmPara">算法参数结构体</param>
    CdetPlum(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetPlum();

    //**********************环形梅花孔检测算法********************
    /// <summary>
    /// 检测每一个小圆坐标
    /// </summary>
    /// <param name="mSrcImage">检测图片</param>
    /// <param name="vecLittleCenter">检测成功的小圆坐标</param>
    /// <param name="dScore">得分</param>
    /// <returns></returns>
    bool getLittleObjectCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// 使用分割所得先验直接定位到每个小圆
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <param name="dScore"></param>
    /// <returns></returns>
    bool getLittleObjectCenterDirect(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// 使用深度学习先验的增强圆拟合
    /// 区分大小圆半径
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="refinedRoi"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

private:
    CdetPlum &operator=(const CdetPlum &);
    //**********************环形梅花孔检测算法********************

    /// <summary>
    /// 算法一：距离推测+自动阈值拟合
    /// 此算发成功率很高，但是误检率也很高，应急使用，不推荐
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getDistancePredictCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// 算法二：此算法含定制化成分，部分情况可能不通用
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getSegmentImageCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// 算法三：增强圆拟合
    /// 默认使用此算法
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// 算法四：使用深度学习先验的增强圆拟合
    /// 自动检测下默认使用此算法
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getLittleCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// 使用深度学习先验的增强大圆圆拟合
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="refinedRoi"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getBigCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// 定制算法尝试
    /// 适用于检测中心反光的白色小圆，对于小圆直径要求较高
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool customLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

private:
    /// <summary>
    /// 粗定位小圆图片
    /// </summary>
    /// <param name="mSrcImage">待检测图片</param>
    /// <param name="pCircleCenter">参考中心点坐标</param>
    /// <param name="nType">获取小圆坐标方式</param>
    /// <param name="vecMap">每个小圆区域参数</param>
    /// <returns></returns>
    int getLittleCircleLocation(cv::Mat mSrcImage, cv::Point2d pCircleCenter, int nType, std::vector<std::pair<cv::Mat, cv::Point>> &vecMap);

    /// <summary>
    /// 轮廓点分象限删选
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecInput"></param>
    /// <param name="pRoughCenter"></param>
    /// <param name="pCenter"></param>
    /// <returns></returns>
    int scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput, cv::Point2d pRoughCenter, cv::Point3d &pCenter);

    //获取小圆区域之间的最佳间距
    int getOptimumCenterInterval();

private:
    std::string m_strLog;  //日志文档
    int m_nAlgorithm;      //算法
    int m_nLittleCount;    //小圆个数
    int m_nBigDiameter;    //外围直径
    int m_nLittleDiameter; //单圆直径
    int m_nThreshold;      //误差参数
    bool m_bIsWhite;       //靶标为白色
    double m_dAngle;       //梅花孔旋转角度
    bool m_bWhiteEdge;     //白色靶标边缘
    bool m_bIsPolarChange; //极性检测
    bool m_bIsEnhance;     //图像增强
    double m_dCircleRate;  //小圆检测比
};
