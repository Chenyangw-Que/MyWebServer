#include "Execute.h"
#include "Common.h"
#include "DeepLabv3.h"
#include "DetCircleCenter.h"
#include "DetCrossCenter.h"
#include "DetRectangleCenter.h"
#include "DetPlumCenter.h"
#include "DetMatrixHoleCenter.h"
#include "EnhanceImage.h"
#include "FilterImage.h"
#include "FitLine.h"
#include "Gradient.h"
#include "LogHelp.h"
#include "opencv2/opencv.hpp"
#include "paramAnalyze.h"
#include "yolov5.h"
#include <string>
#include <tuple>

using namespace std;
using namespace cv;
std::string g_strTargetModel = "/home/que/autoDetMark/Model/yolov5m_best_openvino_model.xml";
std::string g_strSegmentationModel =
    "/home/que/autoDetMark/Model/DeepLabV3_PLUS_Gating_refine_crop_transfer-last_512.xml";

cv::Mat CExecute::getDetectResult(){
    return CtargetDet_->getResultImage();
}
CExecute::CExecute() {
    CtargetDet_ = new CYoloV5(g_strTargetModel);
    CtargetSeg_ = new CDeeplabV3(g_strSegmentationModel); //初始化Deeplab分割对象
}
CExecute::~CExecute() {}

int CExecute::MarkAutoDetection(
    const cv::Mat mSrcImage,
    std::vector<std::pair<cv::Mat, cv::Point3d>> &detBoxResult)
{
    if (!mSrcImage.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    mSrcImage.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "auto_detection.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************开始目标检测****************");
    writeLog(strTxtPath, "开始目标检测");
    if (CtargetDet_->getTargetClassRoi(mGrayImage, detBoxResult))
    {
        writeLog(strTxtPath, "目标检测成功");
        return CFMEE_CODE_OK;
    }
    else
    {
        writeLog(strTxtPath, "ERROR:目标检测失败");
        writeLog(strTxtPath, "ERROR:"
                             "可能原因：图中没有通用十类靶标，靶标处于图片边缘被过"
                             "滤，检测置信度不足被过滤");
        return CFMEE_CODE_GETROI_ERROR;
    }
    return CFMEE_CODE_FAILED;
}

int CExecute::MarkAutoSegmentation(const cv::Mat SrcImage, short class_id,
                                   cv::Mat &mDesImage)
{
    cv::Mat mGrayImage = SrcImage;
    //由于Deeplab预处理会执行通道检验，所以这里只进行空输入判断
    if (!mGrayImage.data)
        return CFMEE_CODE_IMAGE_ERROR;
    //生成日志文件
    std::string strTxtPath = "auto_segmentation.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************开始语义分割****************");
    writeLog(strTxtPath, "开始语义分割");
    if (CtargetSeg_->getSegmentMask(mGrayImage, mDesImage, class_id))
    {
        writeLog(strTxtPath, "语义分割成功");
        return CFMEE_CODE_OK;
    }

    else
    {
        writeLog(strTxtPath, "ERROR:语义分割失败");
        return CFMEE_SEGMENT_ERROR;
    }
    return CFMEE_CODE_FAILED;
}

int CExecute::GetMaskParam(cv::Mat BinaryMask, cv::Mat ImageRoi, short class_id,
                           DetectParameterV3 &MarkParam,
                           std::vector<pair<cv::Mat, cv::Point>> &refinedRoi)
{
    cv::Mat Mask = BinaryMask;
    std::string strTxtPath = "getMarkParam.txt";
    isReadWriteFile(strTxtPath);
    // cv::cvtColor(Mask, Mask, COLOR_RGB2GRAY);
    // 中值滤波，过滤可能的细小干扰点
    cv::medianBlur(Mask, Mask, 3);
    Mask.convertTo(Mask, CV_8U);
    ImageRoi.convertTo(ImageRoi, CV_8U);
    int nError = 0;
    switch (class_id)
    {
    case 0:
        nError = GetCircleParam(Mask, ImageRoi, MarkParam);
        break;
    case 3:
        nError = GetCrossParam(Mask, ImageRoi, MarkParam);
        break;
    case 4:
        nError = GetRectangleParam(Mask, ImageRoi, MarkParam);
        break;
    case 5:
        nError = GetMatrixHoleParam(Mask, ImageRoi, MarkParam, refinedRoi);
        break;
    case 6:
        nError = GetPlumParam(Mask, ImageRoi, MarkParam, refinedRoi);
        break;
    case 7:
        nError = GetRingParam(Mask, ImageRoi, MarkParam);
        break;
    case 8:
        nError = GetComplexPlumParam(Mask, ImageRoi, MarkParam, refinedRoi);
        break;
    case 9:
        nError = GetCircleRingParam(Mask, ImageRoi, MarkParam);
        break;
    default:
        nError = CFMEE_CODE_PARAM;
        writeLog(strTxtPath, "检测分类错误或者输入靶标目前自动检测不支持");
        break;
    }
    return nError;
}

int CExecute::getCircleCenterAuto(const pair<cv::Mat, cv::Point3d> detbox,
                                  DetectParameterV3 param,
                                  ConfigAlgorithmParam algorithmPara,
                                  DetectResultV3 *result)
{
    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoCircleMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************圆靶标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入直径：" + to_string(param.Diameter[0]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    CdetCircle getCenter(strTxtPath, param, algorithmPara);
    cv::Point3d CenterCirclePoint(0, 0, 0); //检测圆中心点
    int nSuccessNum = 0;
    double dScore = 60;
    //精确计算

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    if (getCenter.getCircleCenter(mGrayImage, false, CenterCirclePoint, dScore))
    {
        result->DetEveryResult[0].PartResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].PartResult[0].CenterX =
            CenterCirclePoint.x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].CenterY =
            CenterCirclePoint.y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].Diameter = CenterCirclePoint.z * 2;
        result->DetEveryResult[0].PartResult[0].DetScore =
            dScore > 100 ? 100 : dScore;

        result->DetEveryResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].CenterX = CenterCirclePoint.x + detbox.second.x;
        result->DetEveryResult[0].CenterY = CenterCirclePoint.y + detbox.second.y;
        result->DetEveryResult[0].Diameter = CenterCirclePoint.z * 2;
        result->DetEveryResult[0].PartCount = 1;
        result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;
    }
    else
    {
        writeLog(strTxtPath, "拟合失败");
        return CFMEE_FITTING_ERROR;
    }
    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_CODE_FAILED;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        if (result->CircleCount > 1)
        {
            for (int i = 0; i < result->CircleCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].CenterX,
                             result->DetEveryResult[i].CenterY),
                       param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX - 30,
                           result->DetEveryResult[i].CenterY),
                     Point(result->DetEveryResult[i].CenterX + 30,
                           result->DetEveryResult[i].CenterY),
                     Scalar(255, 0, 0), 2, 8);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY - 30),
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY + 30),
                     Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

int CExecute::getMatrixHoleCenterAuto(
    const pair<cv::Mat, cv::Point3d> detbox,
    const std::vector<pair<cv::Mat, cv::Point>> roughPoint,
    DetectParameterV3 param, ConfigAlgorithmParam algorithmPara,
    DetectResultV3 *result)
{
    // if (!detbox.first.data)
    //	return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoMatrixHoleMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************开始检测定位****************");
    writeLog(strTxtPath, "孔矩阵抓取");
    writeLog(strTxtPath, "输入行数：" + to_string(param.Cols) + "，输入列数：" +
                             to_string(param.Rows) + "，输入行间距：" +
                             to_string(param.Colspan) + "，输入列间距：" +
                             to_string(param.Rowspan) + "，输入直径" +
                             to_string(param.Diameter[0]) + "，输入误差" +
                             to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");
    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }
    CdetHoleMatrix getCenter(strTxtPath, param, algorithmPara);
    std::vector<cv::Point2d> vecCenterHoleMatrixPoint; //检测孔矩阵小圆中心点容器
    cv::Point2d CenterHoleMatrixPoint(0, 0);           //检测孔矩阵中心点
    double dScore = 60;
    //精确计算
    writeLog(strTxtPath, "启用精确计算");
    if (getCenter.getEveryLittleCircleDirect(mGrayImage, {roughPoint},
                                             vecCenterHoleMatrixPoint,
                                             CenterHoleMatrixPoint, dScore))
    {
        result->DetEveryResult[0].PartCount = vecCenterHoleMatrixPoint.size();
        // 填充小圆参数
        for (int i = 0; i < vecCenterHoleMatrixPoint.size(); i++)
        {
#ifdef _DEBUG
            // cv::Mat showImage;
            // mGrayImage.copyTo(showImage);
            // circle(showImage, cv::Point(vecCenterHoleMatrixPoint[i].x,
            // vecCenterHoleMatrixPoint[i].y), 10, 255, 1); CvWaitShowImage(showImage,
            // "孔矩阵结果显示");
#endif // _DEBUG

            result->DetEveryResult[0].PartResult[i].MarkType = MT_CIRCLE;
            result->DetEveryResult[0].PartResult[i].CenterX =
                vecCenterHoleMatrixPoint[i].x + detbox.second.x;
            result->DetEveryResult[0].PartResult[i].CenterY =
                vecCenterHoleMatrixPoint[i].y + detbox.second.y;
            result->DetEveryResult[0].PartResult[i].Diameter = param.Diameter[0];
            result->DetEveryResult[0].PartResult[i].DetScore =
                dScore > 100 ? 100 : dScore;
        }
        result->DetEveryResult[0].MarkType = MT_MATRIX;
        result->DetEveryResult[0].PartCount = vecCenterHoleMatrixPoint.size();
        result->DetEveryResult[0].CenterX =
            CenterHoleMatrixPoint.x + detbox.second.x;
        result->DetEveryResult[0].CenterY =
            CenterHoleMatrixPoint.y + detbox.second.y;
        result->DetEveryResult[0].Diameter = param.Diameter[0];
        result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;
    }
    else
    {
        writeLog(strTxtPath, "孔矩阵精确计算失败");
        return CFMEE_CODE_FAILED;
    }

    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(detbox.first, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage, Point(result->CenterX, result->CenterY),
               param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage, Point(result->CenterX - 30, result->CenterY),
             Point(result->CenterX + 30, result->CenterY), Scalar(255, 0, 0), 2, 8);
        line(mShowImage, Point(result->CenterX, result->CenterY - 30),
             Point(result->CenterX, result->CenterY + 30), Scalar(255, 0, 0), 2, 8);
        if (result->CircleCount > 1)
        {
            for (int i = 0; i < result->CircleCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].CenterX,
                             result->DetEveryResult[i].CenterY),
                       param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX - 30,
                           result->DetEveryResult[i].CenterY),
                     Point(result->DetEveryResult[i].CenterX + 30,
                           result->DetEveryResult[i].CenterY),
                     Scalar(255, 0, 0), 2, 8);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY - 30),
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY + 30),
                     Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

int CExecute::getCrossCenterAuto(const pair<cv::Mat, cv::Point3d> detbox,
                                 DetectParameterV3 param,
                                 ConfigAlgorithmParam algorithmPara,
                                 DetectResultV3 *result)
{
    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoCrossMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************开始检测定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入线宽：" + to_string(param.Diameter[0]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    CdetCross getCenter(strTxtPath, param, algorithmPara);
    cv::Point2d CenterPoint(0, 0);
    vector<cv::Point2d> fourPoint;
    int nSuccessNum = 0;
    double dScore = 60;
    algorithmPara.IsPolarChange = true;
#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    if (getCenter.getCrossCenterFourPoint(detbox.first, false, CenterPoint,
                                          fourPoint, dScore))
    {
        sort(fourPoint.begin(), fourPoint.end(),
             [](cv::Point2d a, cv::Point2d b) { return a.y > b.y; });
        cv::Point2d temp;
        if (fourPoint[0].x > fourPoint[1].x)
        {
            temp = fourPoint[1];
            fourPoint[1] = fourPoint[0];
            fourPoint[0] = temp;
        }
        if (fourPoint[2].x < fourPoint[3].x)
        {
            temp = fourPoint[3];
            fourPoint[3] = fourPoint[2];
            fourPoint[2] = temp;
        }

        vector<cv::Point2f> fourPointsf;
        for (auto p : fourPoint)
            fourPointsf.push_back(cv::Point2f(p));
        // 计算矩形得分
        cv::RotatedRect minRect = cv::minAreaRect(fourPointsf);
        double minArea = minRect.size.height * minRect.size.width;
        double conArea = cv::contourArea(fourPointsf);
        dScore += (conArea / minArea) * 40;
        result->DetEveryResult[0].PartResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].PartResult[0].CenterX =
            CenterPoint.x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].CenterY =
            CenterPoint.y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].Width = param.Width[0];
        result->DetEveryResult[0].PartResult[0].Height = param.Height[0];
        result->DetEveryResult[0].PartResult[0].Diameter = param.Diameter[0];
        result->DetEveryResult[0].PartResult[0].DetScore =
            dScore > 100 ? 100 : dScore;

        // 存储四点
        result->DetEveryResult[0].PartResult[0].ReserveCount = 8;
        result->DetEveryResult[0].PartResult[0].ReserveModel[0] =
            fourPoint[0].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[1] =
            fourPoint[0].y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].ReserveModel[2] =
            fourPoint[1].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[3] =
            fourPoint[1].y + detbox.second.y;

        result->DetEveryResult[0].PartResult[0].ReserveModel[4] =
            fourPoint[2].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[5] =
            fourPoint[2].y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].ReserveModel[6] =
            fourPoint[3].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[7] =
            fourPoint[3].y + detbox.second.y;

        result->DetEveryResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].CenterX = CenterPoint.x + detbox.second.x;
        result->DetEveryResult[0].CenterY = CenterPoint.y + detbox.second.y;
        result->DetEveryResult[0].Width = param.Width[0];
        result->DetEveryResult[0].Height = param.Height[0];
        result->DetEveryResult[0].Diameter = param.Diameter[0];

        result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;

        result->DetEveryResult[0].PartCount = 1;
    }
    //记录检测成功数
    result->CircleCount = 1;
    getLastCenter(mGrayImage, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage, Point(result->CenterX - 30, result->CenterY),
             Point(result->CenterX + 30, result->CenterY), Scalar(255, 0, 0), 2, 8);
        line(mShowImage, Point(result->CenterX, result->CenterY - 30),
             Point(result->CenterX, result->CenterY + 30), Scalar(255, 0, 0), 2, 8);
        if (result->CircleCount > 1)
        {
            for (int i = 0; i < result->CircleCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].CenterX,
                             result->DetEveryResult[i].CenterY),
                       param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX - 30,
                           result->DetEveryResult[i].CenterY),
                     Point(result->DetEveryResult[i].CenterX + 30,
                           result->DetEveryResult[i].CenterY),
                     Scalar(255, 0, 0), 2, 8);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY - 30),
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY + 30),
                     Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

/// <summary>
/// 圆环靶标精计算
/// </summary>
/// <param name="detbox"></param>
/// <param name="strResultPath"></param>
/// <param name="param"></param>
/// <param name="algorithmPara"></param>
/// <param name="result"></param>
/// <returns></returns>
int CExecute::getRingCenterAuto(const pair<cv::Mat, cv::Point3d> detbox,
                                DetectParameterV3 param,
                                ConfigAlgorithmParam algorithmPara,
                                DetectResultV3 *result)
{
    param.Threshold = 5;
    // algorithmPara.nInnerRing = 0;

    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoRingMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************圆环靶标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入外直径：" + to_string(param.Diameter[0]) +
                             "输入内直径：" + to_string(param.Diameter[1]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    cv::Point3d CenterCirclePoint(0, 0, 0); //检测圆中心点
    int nSuccessNum = 0;
    double dScore = 60;
    double dScore2 = 60;
    result->MarkType = MT_CIRCLE;
    //精确计算

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    if (algorithmPara.nInnerRing == 0)
    {
        DetectParameterV3 tempParam = param;
        // 外圆检测器
        CdetCircle getCenterOuter(strTxtPath, tempParam, algorithmPara);
        tempParam.Diameter[0] = tempParam.Diameter[1];
        // 内圆检测器
        CdetCircle getCenterInner(strTxtPath, tempParam, algorithmPara);
        writeLog(strTxtPath, "进行圆环内外环检测");
        writeLog(strTxtPath, "首先进行圆环外环检测");
        if (getCenterOuter.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore))
        {
            result->DetEveryResult[0].PartResult[0].MarkType = 0;
            result->DetEveryResult[0].PartResult[0].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[0].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[0].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[0].DetScore =
                dScore > 100 ? 100 : dScore;
            dScore = dScore > 100 ? 100 : dScore;
        }
        else
        {
            writeLog(strTxtPath, "外环检测失败");
            return CFMEE_FITTING_ERROR;
        }
        writeLog(strTxtPath, "首先进行圆环内环检测");
        if (getCenterInner.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore2))
        {
            result->DetEveryResult[0].PartResult[1].MarkType = 0;
            result->DetEveryResult[0].PartResult[1].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[1].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[1].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[1].DetScore =
                dScore2 > 100 ? 100 : dScore2;
            dScore2 = dScore2 > 100 ? 100 : dScore2;
        }
        else
        {
            writeLog(strTxtPath, "内环检测失败");
            return CFMEE_FITTING_ERROR;
        }
        // 完成内外环检测，进行参数整合
        param.dCenterWeight = param.dCenterWeight == 0 ? 0.5 : param.dCenterWeight;
        result->DetEveryResult[0].PartCount = 2;
        result->DetEveryResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].CenterX =
            result->DetEveryResult[0].PartResult[0].CenterX * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[1].CenterX *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].CenterY =
            result->DetEveryResult[0].PartResult[0].CenterY * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[1].CenterY *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].Diameter =
            result->DetEveryResult[0].PartResult[0].Diameter * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[1].Diameter *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].DetScore =
            dScore * param.dCenterWeight + dScore2 * (1 - param.dCenterWeight);

        writeLog(strTxtPath, "内外环检测成功");
    }
    else if (algorithmPara.nInnerRing == 1)
    {
        // 内圆检测器
        param.Diameter[0] = param.Diameter[1];
        CdetCircle getCenterInner(strTxtPath, param, algorithmPara);
        writeLog(strTxtPath, "进行圆环内环检测");
        if (getCenterInner.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore2))
        {
            result->DetEveryResult[0].MarkType = param.MarkType;
            result->DetEveryResult[0].CenterX = CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].CenterY = CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].Diameter = CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[0].DetScore =
                dScore2 > 100 ? 100 : dScore2;

            result->DetEveryResult[0].PartResult[0].MarkType = 0;
            result->DetEveryResult[0].PartResult[0].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[0].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[0].Diameter =
                CenterCirclePoint.z * 2;

            result->DetEveryResult[0].DetScore = dScore2 > 100 ? 100 : dScore2;
            result->DetEveryResult[0].PartCount = 1;
        }
        else
        {
            writeLog(strTxtPath, "内环检测失败");
            return CFMEE_FITTING_ERROR;
        }
    }
    else if (algorithmPara.nInnerRing == 2)
    {
        // 外圆检测器
        CdetCircle getCenterOuter(strTxtPath, param, algorithmPara);
        writeLog(strTxtPath, "进行圆环外环检测");
        if (getCenterOuter.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore))
        {
            result->DetEveryResult[0].MarkType = param.MarkType;
            result->DetEveryResult[0].CenterX = CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].CenterY = CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].Diameter = CenterCirclePoint.z * 2;

            result->DetEveryResult[0].PartResult[0].MarkType = 0;
            result->DetEveryResult[0].PartResult[0].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[0].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[0].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[0].DetScore =
                dScore > 100 ? 100 : dScore;

            result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;
            result->DetEveryResult[0].PartCount = 1;
        }
        else
        {
            writeLog(strTxtPath, "外环检测失败");
            return CFMEE_FITTING_ERROR;
        }
    }
    else
    {
        writeLog(strTxtPath, "检测类型填写错误，请填写0到2");
        return CFMEE_CODE_FAILED;
    }

    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               result->Diameter / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        if (result->CircleCount > 1)
        {
            for (int i = 0; i < result->CircleCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].CenterX,
                             result->DetEveryResult[i].CenterY),
                       param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX - 30,
                           result->DetEveryResult[i].CenterY),
                     Point(result->DetEveryResult[i].CenterX + 30,
                           result->DetEveryResult[i].CenterY),
                     Scalar(255, 0, 0), 2, 8);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY - 30),
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY + 30),
                     Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

/// <summary>
/// 梅花孔靶标精计算
/// </summary>
/// <param name="detbox"></param>
/// <param name="param"></param>
/// <param name="algorithmPara"></param>
/// <param name="result"></param>
/// <returns></returns>
int CExecute::getPlumCenterAuto(
    const pair<cv::Mat, cv::Point3d> detbox,
    const std::vector<pair<cv::Mat, cv::Point>> roughPoint,
    DetectParameterV3 param, ConfigAlgorithmParam algorithmPara,
    DetectResultV3 *result)
{
    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoPlumMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************梅花孔标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入直径：" + to_string(param.Diameter[0]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }
    // algorithmPara.nMarkAlgorithm = 3;
    CdetPlum getCenter(strTxtPath, param, algorithmPara);
    std::vector<cv::Point3d> vecLittleCenter;
    int nSuccessNum = 0;
    double dScore = 60;
    //精确计算
    writeLog(strTxtPath, "使用增强圆拟合进行梅花孔精计算");

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    // if (getCenter.getLittleObjectCenterDirect(mGrayImage, roughPoint,
    // vecLittleCenter, dScore))
    if (getCenter.getCircleCenterDLPrior(mGrayImage, roughPoint, vecLittleCenter,
                                         dScore))
    {
        result->DetEveryResult[0].DetScore = dScore;
        result->DetEveryResult[0].PartCount = vecLittleCenter.size();
        vector<ContoursPoint> circlePoints;
        // 填充小组成部分字段
        for (int i = 0; i < vecLittleCenter.size(); i++)
        {
            result->DetEveryResult[0].PartResult[i].MarkType = 0;
            result->DetEveryResult[0].PartResult[i].CenterX =
                vecLittleCenter[i].x + detbox.second.x;
            result->DetEveryResult[0].PartResult[i].CenterY =
                vecLittleCenter[i].y + detbox.second.y;
            result->DetEveryResult[0].PartResult[i].Diameter =
                vecLittleCenter[i].z * 2;
            circlePoints.push_back(ContoursPoint(
                result->DetEveryResult[0].PartResult[i].CenterX,
                result->DetEveryResult[0].PartResult[i].CenterY, 0, 0, 0, 255));
        }

        // 进行整体圆拟合
        CdetCircle *fiter = new CdetCircle();
        cv::Point3d circleParam;
        if (fiter->iterFitting(circlePoints, circleParam, false))
        {
            // 将整体plum的参数填入
            result->DetEveryResult[0].MarkType = param.MarkType;
            result->DetEveryResult[0].CenterX = circleParam.x;
            result->DetEveryResult[0].CenterY = circleParam.y;
            result->DetEveryResult[0].Diameter = circleParam.z * 2;
        }
    }
    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        if (result->DetEveryResult[0].PartCount > 1)
        {
            for (int i = 0; i < result->DetEveryResult[0].PartCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].PartResult[i].CenterX,
                             result->DetEveryResult[i].PartResult[i].CenterY),
                       result->DetEveryResult[i].PartResult[i].Diameter,
                       cv::Scalar(0, 0, 255), 2, 8, 0);
                // line(mShowImage, Point(result->DetEveryResult[i].CenterX - 30,
                // result->DetEveryResult[i].CenterY),
                // Point(result->DetEveryResult[i].CenterX + 30,
                // result->DetEveryResult[i].CenterY), Scalar(255, 0, 0), 2, 8);
                // line(mShowImage, Point(result->DetEveryResult[i].CenterX,
                // result->DetEveryResult[i].CenterY - 30),
                // Point(result->DetEveryResult[i].CenterX,
                // result->DetEveryResult[i].CenterY + 30), Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

/// <summary>
/// 复合梅花孔靶标精计算
/// </summary>
/// <param name="detbox"></param>
/// <param name="param"></param>
/// <param name="algorithmPara"></param>
/// <param name="result"></param>
/// <returns></returns>
int CExecute::getComplexPlumCenterAuto(
    const pair<cv::Mat, cv::Point3d> detbox,
    const std::vector<pair<cv::Mat, cv::Point>> roughPoint,
    DetectParameterV3 param, ConfigAlgorithmParam algorithmPara,
    DetectResultV3 *result)
{
    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoComplexPlumMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath,
             "****************复合梅花孔靶标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入直径：" + to_string(param.Diameter[0]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    DetectParameterV3 circleParam;
    ConfigAlgorithmParam circleAlgori;
    circleParam.Diameter[0] = param.Diameter[0];
    circleParam.IsWhite = false;
    circleParam.Threshold = 15;
    CdetCircle getCenterCircle(strTxtPath, circleParam, circleAlgori);
    cv::Point3d circleCenter(0, 0, 0);
    std::vector<cv::Point3d> circleCenterVec;
    DetectParameterV3 plumParam = param;
    algorithmPara.nMarkAlgorithm = 3;
    plumParam.Diameter[0] = param.nCenterDistance;
    // plumParam.nLittleCount = param.nLittleCount - 1;
    cout << plumParam.nLittleCount << "小圆个数" << endl;
    CdetPlum getCenter(strTxtPath, plumParam, algorithmPara);
    std::vector<cv::Point3d> vecLittleCenter;
    double dScoreCircle = 60;
    double dScore = 60;
    //精确计算
    writeLog(strTxtPath, "使用增强圆拟合进行梅花孔精计算");

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    writeLog(strTxtPath, "进行中心大圆精计算");
    // 基于complexPlum配置文件创建圆配置文件
    if (circleParam.Diameter[0] > 50)
    // if(false)
    {
        if (getCenterCircle.getCircleCenter(mGrayImage, false, circleCenter,
                                            dScoreCircle))
        {
            result->DetEveryResult[0].PartResult[0].MarkType = 0;
            result->DetEveryResult[0].PartResult[0].CenterX =
                circleCenter.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[0].CenterY =
                circleCenter.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[0].Diameter = circleCenter.z * 2;
            result->DetEveryResult[0].PartCount = 1;
        }
        else
        {
            writeLog(strTxtPath, "圆精计算失败");
            return CFMEE_FITTING_ERROR;
        }
    }
    else
    {
        // 启动小圆拟合
        if (getCenterCircle.getLittleCircleCenter(
                {roughPoint[roughPoint.size() - 1]}, circleCenterVec))
        {

            circleCenter.x = circleCenterVec[0].x;
            circleCenter.y = circleCenterVec[0].y;
            circleCenter.z = circleCenterVec[0].z;
            result->DetEveryResult[0].PartResult[0].MarkType = 0;
            result->DetEveryResult[0].PartResult[0].CenterX =
                circleCenter.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[0].CenterY =
                circleCenter.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[0].Diameter = circleCenter.z * 2;
            result->DetEveryResult[0].PartCount = 1;

            //// 使用语义分割先验进行填充
            // circleCenter.x = roughPoint[roughPoint.size() - 1].first.cols / 2;
            // circleCenter.y = roughPoint[roughPoint.size() - 1].first.rows / 2;
            // circleCenter.z = circleParam.Diameter[0] / 2;
            // result->DetEveryResult[0].PartResult[0].MarkType = 0;
            // result->DetEveryResult[0].PartResult[0].CenterX = circleCenter.x +
            // detbox.second.x; result->DetEveryResult[0].PartResult[0].CenterY =
            // circleCenter.y + detbox.second.y;
            // result->DetEveryResult[0].PartResult[0].Diameter = circleCenter.z * 2;
            // result->DetEveryResult[0].PartCount = 1;
        }
        else
        {
            writeLog(strTxtPath, "圆精计算失败");
            return CFMEE_FITTING_ERROR;
        }
    }
    cout << roughPoint.size() << endl;
    cout << "小圆个数" << param.nLittleCount << endl;
    vector<pair<cv::Mat, cv::Point>> rp(roughPoint.begin(), roughPoint.end() - 1);
    cout << rp.size() << endl;
    if (getCenter.getCircleCenterDLPrior(mGrayImage, rp, vecLittleCenter,
                                         dScore))
    {
        result->DetEveryResult[0].DetScore = dScore;
        // result->DetEveryResult[0].PartCount = vecLittleCenter.size() + 1;
        vector<ContoursPoint> circlePoints;
        // 填充小组成部分字段
        for (int i = 0; i < vecLittleCenter.size(); i++)
        {
            result->DetEveryResult[0].PartResult[i + 1].MarkType = 0;
            result->DetEveryResult[0].PartResult[i + 1].CenterX =
                vecLittleCenter[i].x + detbox.second.x;
            result->DetEveryResult[0].PartResult[i + 1].CenterY =
                vecLittleCenter[i].y + detbox.second.y;
            result->DetEveryResult[0].PartResult[i + 1].Diameter =
                vecLittleCenter[i].z * 2;
            circlePoints.push_back(ContoursPoint(
                result->DetEveryResult[0].PartResult[i + 1].CenterX,
                result->DetEveryResult[0].PartResult[i + 1].CenterY, 0, 0, 0, 255));
        }
        result->DetEveryResult[0].PartCount += vecLittleCenter.size();
        // 进行整体圆拟合
        CdetCircle *fiter = new CdetCircle();
        cv::Point3d circleResult;
        if (fiter->iterFitting(circlePoints, circleResult, false))
        {
            circleResult.z *= 2;
            // 将整体plum的参数填入
            // 建立加权参数
            param.dCenterWeight =
                param.dCenterWeight == 0 ? 0.5 : param.dCenterWeight;
            result->DetEveryResult[0].MarkType = param.MarkType;
            result->DetEveryResult[0].CenterX =
                circleResult.x * param.dCenterWeight +
                (result->DetEveryResult[0].PartResult[0].CenterX) *
                    (1 - param.dCenterWeight);
            result->DetEveryResult[0].CenterY =
                circleResult.y * param.dCenterWeight +
                (result->DetEveryResult[0].PartResult[0].CenterY) *
                    (1 - param.dCenterWeight);
            result->DetEveryResult[0].Diameter =
                circleResult.z * param.dCenterWeight +
                2 * circleCenter.z * (1 - param.dCenterWeight);
            result->DetEveryResult[0].DetScore =
                dScore * param.dCenterWeight +
                dScoreCircle * (1 - param.dCenterWeight);
        }
        else
        {
            writeLog(strTxtPath, "整体外圈圆拟合失败");
            return CFMEE_FITTING_ERROR;
        }
    }
    else
    {
        writeLog(strTxtPath, "小圆精计算失败");
        return CFMEE_FITTING_ERROR;
    }
    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        if (result->DetEveryResult[0].PartCount > 1)
        {
            for (int i = 0; i < result->DetEveryResult[0].PartCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].PartResult[i].CenterX -
                                 detbox.second.x,
                             result->DetEveryResult[i].PartResult[i].CenterY -
                                 detbox.second.y),
                       result->DetEveryResult[i].PartResult[i].Diameter / 2,
                       cv::Scalar(0, 0, 255), 2, 8, 0);
                // line(mShowImage, Point(result->DetEveryResult[i].CenterX - 30,
                // result->DetEveryResult[i].CenterY),
                // Point(result->DetEveryResult[i].CenterX + 30,
                // result->DetEveryResult[i].CenterY), Scalar(255, 0, 0), 2, 8);
                // line(mShowImage, Point(result->DetEveryResult[i].CenterX,
                // result->DetEveryResult[i].CenterY - 30),
                // Point(result->DetEveryResult[i].CenterX,
                // result->DetEveryResult[i].CenterY + 30), Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

/// <summary>
/// circleRing靶标精计算
/// </summary>
/// <param name="detbox"></param>
/// <param name="param"></param>
/// <param name="algorithmPara"></param>
/// <param name="result"></param>
/// <returns></returns>
int CExecute::getCircleRingCenterAuto(const pair<cv::Mat, cv::Point3d> detbox,
                                      DetectParameterV3 param,
                                      ConfigAlgorithmParam algorithmPara,
                                      DetectResultV3 *result)
{

    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoCircleRingMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath,
             "****************circleRing靶标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入外直径：" + to_string(param.Diameter[0]) +
                             "输入内直径：" + to_string(param.Diameter[1]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    cv::Point3d CenterCirclePoint(0, 0, 0); //检测圆中心点
    int nSuccessNum = 0;
    double dScore = 60;
    double dScore2 = 60;
    double dScoreCircle = 60;

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    // 首先检测最内层圆

    DetectParameterV3 circleParam = param;
    circleParam.IsWhite = false;
    circleParam.Diameter[0] = circleParam.Diameter[2];
    CdetCircle getCenterCircle(strTxtPath, circleParam, algorithmPara);
    writeLog(strTxtPath, "进行最内层圆检测");
    if (getCenterCircle.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                        dScoreCircle))
    {
        result->DetEveryResult[0].PartResult[0].MarkType = 0;
        result->DetEveryResult[0].PartResult[0].CenterX =
            CenterCirclePoint.x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].CenterY =
            CenterCirclePoint.y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].Diameter = CenterCirclePoint.z * 2;
        result->DetEveryResult[0].PartResult[0].DetScore =
            dScoreCircle > 100 ? 100 : dScoreCircle;
        dScoreCircle = dScoreCircle > 100 ? 100 : dScoreCircle;
    }
    else
    {
        writeLog(strTxtPath, "内层圆检测失败");
        return CFMEE_CODE_FAILED;
    }
    writeLog(strTxtPath, "进行圆环检测");
    if (algorithmPara.nInnerRing == 0)
    {
        DetectParameterV3 tempParam = param;
        // 外圆检测器
        CdetCircle getCenterOuter(strTxtPath, tempParam, algorithmPara);
        tempParam.Diameter[0] = tempParam.Diameter[1];
        // 内圆检测器
        CdetCircle getCenterInner(strTxtPath, tempParam, algorithmPara);
        writeLog(strTxtPath, "进行圆环内外环检测");
        writeLog(strTxtPath, "首先进行圆环外环检测");
        if (getCenterOuter.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore))
        {
            result->DetEveryResult[0].PartResult[1].MarkType = 0;
            result->DetEveryResult[0].PartResult[1].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[1].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[1].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[1].DetScore =
                dScore > 100 ? 100 : dScore;
            dScore = dScore > 100 ? 100 : dScore;
        }
        else
        {
            writeLog(strTxtPath, "外环检测失败");
            return CFMEE_CODE_FAILED;
        }
        writeLog(strTxtPath, "首先进行圆环内环检测");
        if (getCenterInner.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore2))
        {
            result->DetEveryResult[0].PartResult[2].MarkType = 0;
            result->DetEveryResult[0].PartResult[2].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[2].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[2].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[2].DetScore =
                dScore2 > 100 ? 100 : dScore2;
            dScore2 = dScore2 > 100 ? 100 : dScore2;
        }
        else
        {
            writeLog(strTxtPath, "内环检测失败");
            return CFMEE_CODE_FAILED;
        }
        // 完成内外环检测，进行参数整合
        param.dCenterWeight = param.dCenterWeight == 0 ? 0.5 : param.dCenterWeight;
        result->DetEveryResult[0].PartCount = 3;
        result->DetEveryResult[0].CenterX =
            result->DetEveryResult[0].PartResult[1].CenterX * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[2].CenterX *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].CenterY =
            result->DetEveryResult[0].PartResult[1].CenterY * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[2].CenterY *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].Diameter =
            result->DetEveryResult[0].PartResult[1].Diameter * param.dCenterWeight +
            result->DetEveryResult[0].PartResult[2].Diameter *
                (1 - param.dCenterWeight);
        result->DetEveryResult[0].DetScore =
            dScore * param.dCenterWeight + dScore2 * (1 - param.dCenterWeight);

        writeLog(strTxtPath, "内外环检测成功");
    }
    else if (algorithmPara.nInnerRing == 1)
    {
        // 内圆检测器
        param.Diameter[0] = param.Diameter[1];
        CdetCircle getCenterInner(strTxtPath, param, algorithmPara);
        writeLog(strTxtPath, "进行圆环内环检测");
        if (getCenterInner.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore2))
        {
            result->DetEveryResult[0].PartCount = 2;
            result->DetEveryResult[0].PartResult[1].MarkType = 0;
            result->DetEveryResult[0].PartResult[1].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[1].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[1].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[1].DetScore =
                dScore2 > 100 ? 100 : dScore2;
            result->DetEveryResult[0].CenterX = CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].CenterY = CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].Diameter = CenterCirclePoint.z * 2;
            result->DetEveryResult[0].DetScore = dScore2 > 100 ? 100 : dScore2;
            writeLog(strTxtPath, "内环检测成功");
        }
        else
        {
            writeLog(strTxtPath, "内环检测失败");
            return CFMEE_CODE_FAILED;
        }
    }
    else if (algorithmPara.nInnerRing == 2)
    {
        // 外圆检测器
        CdetCircle getCenterOuter(strTxtPath, param, algorithmPara);
        writeLog(strTxtPath, "进行圆环外环检测");
        if (getCenterOuter.getCircleCenter(mGrayImage, false, CenterCirclePoint,
                                           dScore))
        {
            result->DetEveryResult[0].PartCount = 2;
            result->DetEveryResult[0].PartResult[1].MarkType = 0;
            result->DetEveryResult[0].PartResult[1].CenterX =
                CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].PartResult[1].CenterY =
                CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].PartResult[1].Diameter =
                CenterCirclePoint.z * 2;
            result->DetEveryResult[0].PartResult[1].DetScore =
                dScore > 100 ? 100 : dScore;

            result->DetEveryResult[0].CenterX = CenterCirclePoint.x + detbox.second.x;
            result->DetEveryResult[0].CenterY = CenterCirclePoint.y + detbox.second.y;
            result->DetEveryResult[0].Diameter = CenterCirclePoint.z * 2;
            result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;
        }
        else
        {
            writeLog(strTxtPath, "外环检测失败");
            return CFMEE_CODE_FAILED;
        }
    }
    else
    {
        writeLog(strTxtPath, "检测类型填写错误，请填写0到2");
        return CFMEE_CODE_FAILED;
    }
    // 进行完整的参数加权
    param.dCenterWeight = param.dCenterWeight == 0 ? 0.5 : param.dCenterWeight;
    result->DetEveryResult[0].MarkType = param.MarkType;
    result->DetEveryResult[0].CenterX =
        result->DetEveryResult[0].PartResult[0].CenterX * param.dCenterWeight +
        result->DetEveryResult[0].CenterX * (1 - param.dCenterWeight);
    result->DetEveryResult[0].CenterY =
        result->DetEveryResult[0].PartResult[0].CenterY * param.dCenterWeight +
        result->DetEveryResult[0].CenterY * (1 - param.dCenterWeight);
    result->DetEveryResult[0].Diameter =
        result->DetEveryResult[0].PartResult[0].Diameter * param.dCenterWeight +
        result->DetEveryResult[0].Diameter * (1 - param.dCenterWeight);
    result->DetEveryResult[0].DetScore =
        dScoreCircle * param.dCenterWeight +
        result->DetEveryResult[0].DetScore * (1 - param.dCenterWeight);
    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    // saveResultImage(strResultPath + ".jpg", detbox.first, result);
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_CODE_FAILED;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        circle(mShowImage,
               Point(result->CenterX - detbox.second.x,
                     result->CenterY - detbox.second.y),
               result->Diameter / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        if (result->CircleCount > 1)
        {
            for (int i = 0; i < result->CircleCount; ++i)
            {
                circle(mShowImage,
                       Point(result->DetEveryResult[i].CenterX,
                             result->DetEveryResult[i].CenterY),
                       param.Diameter[0] / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX - 30,
                           result->DetEveryResult[i].CenterY),
                     Point(result->DetEveryResult[i].CenterX + 30,
                           result->DetEveryResult[i].CenterY),
                     Scalar(255, 0, 0), 2, 8);
                line(mShowImage,
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY - 30),
                     Point(result->DetEveryResult[i].CenterX,
                           result->DetEveryResult[i].CenterY + 30),
                     Scalar(255, 0, 0), 2, 8);
            }
        }
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

int CExecute::getRectangleCenterAuto(const pair<cv::Mat, cv::Point3d> detbox,
                                     DetectParameterV3 param,
                                     ConfigAlgorithmParam algorithmPara,
                                     DetectResultV3 *result)
{
    if (!detbox.first.data)
        return CFMEE_CODE_IMAGE_ERROR;
    cv::Mat mGrayImage;
    detbox.first.copyTo(mGrayImage);
    //生成日志文件
    std::string strTxtPath = "check_AutoRectangleMark.txt";
    isReadWriteFile(strTxtPath);

    writeLog(strTxtPath, "****************矩形靶标自动精定位****************");
    writeLog(strTxtPath, "单孔抓取");
    writeLog(strTxtPath, "输入直径：" + to_string(param.Diameter[0]) +
                             ",输入误差" + to_string(param.Threshold));
    writeLog(strTxtPath, "加载配置文件");

    //输入图像校验
    if (isPureColorImage(mGrayImage) != 0)
    {
        writeLog(strTxtPath, "ERROR:输入图像错误，CCD采图可能存在错误");
        return CFMEE_CODE_IMAGE_ERROR;
    }

    algorithmPara.nMarkAlgorithm = 0;
    CdetRectangle getCenter(strTxtPath, param, algorithmPara);
    Cfmee_Rectangle pCenterPoint; //检测孔矩阵中心点
    double dScore = 60;
    //精确计算

#ifdef _SHOW_PROCESS_
    CvWaitShowImage(detbox.first, "待检测图片");
#endif
    writeLog(strTxtPath, "启用精确计算");
    if (getCenter.getRectangleCenter(mGrayImage, false, 0, pCenterPoint,
                                     dScore))
    {
        // 对顶点排序
        vector<cv::Point2d> rectPoint(4);
        rectPoint[0] =
            cv::Point2d(pCenterPoint._Point[0].dX, pCenterPoint._Point[0].dY);
        rectPoint[1] =
            cv::Point2d(pCenterPoint._Point[1].dX, pCenterPoint._Point[1].dY);
        rectPoint[2] =
            cv::Point2d(pCenterPoint._Point[2].dX, pCenterPoint._Point[2].dY);
        rectPoint[3] =
            cv::Point2d(pCenterPoint._Point[3].dX, pCenterPoint._Point[3].dY);
        cout << rectPoint[0] << endl;
        sort(rectPoint.begin(), rectPoint.end(),
             [](cv::Point2d a, cv::Point2d b) { return a.y > b.y; });
        cv::Point2d temp;
        if (rectPoint[0].x > rectPoint[1].x)
        {
            temp = rectPoint[1];
            rectPoint[1] = rectPoint[0];
            rectPoint[0] = temp;
        }
        if (rectPoint[2].x < rectPoint[3].x)
        {
            temp = rectPoint[3];
            rectPoint[3] = rectPoint[2];
            rectPoint[2] = temp;
        }

        vector<cv::Point2f> fourPointsf;
        for (auto p : rectPoint)
            fourPointsf.push_back(cv::Point2f(p));
        // 计算矩形得分
        cv::RotatedRect minRect = cv::minAreaRect(fourPointsf);
        double minArea = minRect.size.height * minRect.size.width;
        double conArea = cv::contourArea(fourPointsf);
        dScore += (conArea / minArea) * 40;

        double dist1 = sqrt(
            (rectPoint[0].x - rectPoint[1].x) * (rectPoint[0].x - rectPoint[1].x) +
            (rectPoint[0].y - rectPoint[1].y) * (rectPoint[0].y - rectPoint[1].y));
        double dist2 = sqrt(
            (rectPoint[2].x - rectPoint[3].x) * (rectPoint[2].x - rectPoint[3].x) +
            (rectPoint[2].y - rectPoint[3].y) * (rectPoint[2].y - rectPoint[3].y));
        double width = dist1 > dist2 ? dist1 : dist2;
        double height = dist1 < dist2 ? dist1 : dist2;

        result->DetEveryResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].CenterX = pCenterPoint.dCenterX + detbox.second.x;
        result->DetEveryResult[0].CenterY = pCenterPoint.dCenterY + detbox.second.y;
        result->DetEveryResult[0].Width = width;
        result->DetEveryResult[0].Height = height;
        // cout << result->DetEveryResult[0].Width << endl;
        result->DetEveryResult[0].PartResult[0].MarkType = param.MarkType;
        result->DetEveryResult[0].PartResult[0].Width = width;
        result->DetEveryResult[0].PartResult[0].Height = height;
        result->DetEveryResult[0].PartResult[0].CenterX =
            pCenterPoint.dCenterX + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].CenterY =
            pCenterPoint.dCenterY + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].DetScore =
            dScore > 100 ? 100 : dScore;

        // 存放四个顶点
        result->DetEveryResult[0].PartResult[0].ReserveCount = 8;
        result->DetEveryResult[0].PartResult[0].ReserveModel[0] =
            rectPoint[0].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[1] =
            rectPoint[0].y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].ReserveModel[2] =
            rectPoint[1].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[3] =
            rectPoint[1].y + detbox.second.y;

        result->DetEveryResult[0].PartResult[0].ReserveModel[4] =
            rectPoint[2].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[5] =
            rectPoint[2].y + detbox.second.y;
        result->DetEveryResult[0].PartResult[0].ReserveModel[6] =
            rectPoint[3].x + detbox.second.x;
        result->DetEveryResult[0].PartResult[0].ReserveModel[7] =
            rectPoint[3].y + detbox.second.y;
        result->DetEveryResult[0].DetScore = dScore > 100 ? 100 : dScore;
        result->DetEveryResult[0].PartCount = 1;
    }
    //记录检测成功数
    result->CircleCount = 1;
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    //检测结果保存
    if (result->CenterX == 0 && result->CenterY == 0)
    {
        writeLog(strTxtPath, "ERROR:检测抓取失败");
        writeLog(strTxtPath, "检测结束");
        return CFMEE_FITTING_ERROR;
    }
    writeLog(strTxtPath, "检测结束");
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mGrayImage);
    if (result->CenterX != 0)
    {
        line(mShowImage,
             Point(pCenterPoint._Point[0].dX, pCenterPoint._Point[0].dY),
             Point(pCenterPoint._Point[1].dX, pCenterPoint._Point[1].dY),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(pCenterPoint._Point[1].dX, pCenterPoint._Point[1].dY),
             Point(pCenterPoint._Point[2].dX, pCenterPoint._Point[2].dY),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(pCenterPoint._Point[2].dX, pCenterPoint._Point[2].dY),
             Point(pCenterPoint._Point[3].dX, pCenterPoint._Point[3].dY),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(pCenterPoint._Point[3].dX, pCenterPoint._Point[3].dY),
             Point(pCenterPoint._Point[0].dX, pCenterPoint._Point[0].dY),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(pCenterPoint._Point[0].dX, pCenterPoint._Point[0].dY),
             Point(pCenterPoint._Point[1].dX, pCenterPoint._Point[1].dY),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Point(result->CenterX + 30 - detbox.second.x,
                   result->CenterY - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        line(mShowImage,
             Point(result->CenterX - detbox.second.x,
                   result->CenterY - 30 - detbox.second.y),
             Point(result->CenterX - detbox.second.x,
                   result->CenterY + 30 - detbox.second.y),
             Scalar(255, 0, 0), 2, 8);
        CvWaitShowImage(mShowImage, "检测结果图");
    }
#endif
    return CFMEE_CODE_OK;
}

CExecute &CExecute::operator=(const CExecute &)
{
    // TODO: 在此处插入 return 语句
    return *this;
}


CExecute& executeHandler::getExecuteHandler(){
    static CExecute executeHandler_;
    return executeHandler_;
}