#include "DetectMark.h"
#include "Common.h"
#include "DeepLabv3.h"
#include "DetCircleCenter.h"
#include "Execute.h"
#include "LogHelp.h"
#include "paramAnalyze.h"
#include <memory>
#include <time.h>
#include <string>
using namespace std;
using namespace cv;
class executeHandler;

void drawRes(cv::Mat &inputImg, const DetectResultV3 *result)
{
    for (int i = 0; i < result->CircleCount; ++i)
    {
        for (int j = 0; j < result->DetEveryResult[i].PartCount; j++)
        {
            if (result->DetEveryResult[i].PartResult[j].ReserveCount)
            {
                for (int k = 0; k < (result->DetEveryResult[i].PartResult[j].ReserveCount - 1) / 2; k++)
                {
                    cv::line(inputImg, cv::Point(result->DetEveryResult[i].PartResult[j].ReserveModel[2 * k], result->DetEveryResult[i].PartResult[j].ReserveModel[2 * k + 1]),
                             cv::Point(result->DetEveryResult[i].PartResult[j].ReserveModel[2 * k + 2], result->DetEveryResult[i].PartResult[j].ReserveModel[2 * k + 3]), cv::Scalar(128, 0, 0), 2, 8);
                }
            }
            // if (!result->DetEveryResult[i].PartResult[j].Diameter)
            //     result->DetEveryResult[i].PartResult[j].Diameter = 10;
            circle(inputImg, Point(result->DetEveryResult[i].PartResult[j].CenterX, result->DetEveryResult[i].PartResult[j].CenterY), result->DetEveryResult[i].PartResult[j].Diameter / 2, 255, -1);
        }
        circle(inputImg, Point(result->DetEveryResult[i].CenterX, result->DetEveryResult[i].CenterY), result->DetEveryResult[i].Diameter / 2, 255, 1);
    }
}

int AutoGetParam(const cv::Mat &imgData, DetectResultV3 *result)
{
    cv::Mat mGrayImage;
    imgData.copyTo(mGrayImage);
    if (!mGrayImage.data)
        return CFMEE_CODE_IMAGE_ERROR;
    if (mGrayImage.channels() > 1)
        cvtColor(mGrayImage, mGrayImage, COLOR_BGR2GRAY);
    std::vector<std::pair<cv::Mat, cv::Point3d>> detBoxResult;
    std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi;
    int nError = -10;
    try
    {
        int i = 0;
        executeHandler::getExecuteHandler().MarkAutoDetection(
            mGrayImage, detBoxResult); //获取到检测结果(roi,检测框坐标，类别编号)
        // 使用分割网络获取roi mask这里先使用for推动这个过程
        for (auto detbox : detBoxResult)
        {
            cv::Mat segMask;                // 用于存储分割结果
            int class_id = detbox.second.z; // 检测到的类别编号
            cv::Mat detRoi = detbox.first;  // 检测截取的Roi
            executeHandler::getExecuteHandler().MarkAutoSegmentation(detRoi, class_id,
                                                                     segMask); // 获取二值图mask
            DetectParameterV3 MarkParam;                                       // 存储参数解析结果
            // 解析分割结果获取靶标参数
            MarkParam.MarkType = class_id;
            executeHandler::getExecuteHandler().GetMaskParam(segMask, detRoi, class_id, MarkParam,
                                                             refinedRoi);
            i++;
        }
    }
    catch (...)
    {
        return -100;
    }
    return nError;
}

int AutoDetMark(const cv::Mat &imgData, DetectResultV3 *result, vector<cv::Mat> &resultBox)
{
    // 自动参数解析
    cv::Mat mGrayImage = imgData;
    if (!mGrayImage.data)
        return CFMEE_CODE_IMAGE_ERROR;
    if (mGrayImage.channels() > 1)
        cvtColor(mGrayImage, mGrayImage, COLOR_BGR2GRAY);
    std::vector<std::pair<cv::Mat, cv::Point3d>> detBoxResult;
    int nError = -10;
    // 进行目标检测、语义分割流程获取mask
    // 对mask进行解析，获取靶标参数并写入
    int nSuccessNum = 0;
    vector<DetectParameterV3> tempParam;
    int possibleError = 0;
    try
    {
        int i = 0;
        if (executeHandler::getExecuteHandler().MarkAutoDetection(
                mGrayImage,
                detBoxResult)) //获取到检测结果(roi,检测框坐标，类别编号)
            return CFMEE_CODE_GETROI_ERROR;
        // 使用分割网络获取roi mask这里先使用for推动这个过程
        //resultBox.emplace_back(executeHandler::getExecuteHandler().getDetectResult());
        for (auto detbox : detBoxResult)
        {

            cv::Mat segMask;                // 用于存储分割结果
            int class_id = detbox.second.z; // 检测到的类别编号
            cv::Mat detRoi = detbox.first;  // 检测截取的Roi
            //writeLog(strTxtPath, "开始对检测ROI进行语义分割");
            if (executeHandler::getExecuteHandler().MarkAutoSegmentation(detRoi, class_id, segMask)) // 获取二值图mask
                return CFMEE_SEGMENT_ERROR;
            DetectParameterV3 MarkParam; // 存储参数解析结果
            ConfigAlgorithmParam AlgorithmParam;
            // 解析分割结果获取靶标参数
            MarkParam.MarkType = class_id;
            string resultPath = "";
            DetectResultV3 tempResult;
            std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi;

            if (executeHandler::getExecuteHandler().GetMaskParam(segMask, detRoi, class_id, MarkParam,
                                                                 refinedRoi))
            {
                possibleError = CFMEE_PARAMANALYZE_ERROR;
                continue;
            }
            //writeLog(strTxtPath, "参数自动解析完成");
            switch (class_id)
            {
            case 0:
                nError = executeHandler::getExecuteHandler().getCircleCenterAuto(detbox, MarkParam,
                                                                                 AlgorithmParam, &tempResult);
                break;
            case 3:
                nError = executeHandler::getExecuteHandler().getCrossCenterAuto(detbox, MarkParam,
                                                                                AlgorithmParam, &tempResult);
                break;
            case 4:
                nError = executeHandler::getExecuteHandler().getRectangleCenterAuto(
                    detbox, MarkParam, AlgorithmParam, &tempResult);
                break;
            case 5:
                nError = executeHandler::getExecuteHandler().getMatrixHoleCenterAuto(
                    detbox, refinedRoi, MarkParam, AlgorithmParam, &tempResult);
                break;
            case 6:
                nError = executeHandler::getExecuteHandler().getPlumCenterAuto(detbox, refinedRoi, MarkParam,
                                                                               AlgorithmParam, &tempResult);
                break;
            case 7:
                nError = executeHandler::getExecuteHandler().getRingCenterAuto(detbox, MarkParam,
                                                                               AlgorithmParam, &tempResult);
                break;
            case 8:
                nError = executeHandler::getExecuteHandler().getComplexPlumCenterAuto(
                    detbox, refinedRoi, MarkParam, AlgorithmParam, &tempResult);
                break;
            case 9:
                nError = executeHandler::getExecuteHandler().getCircleRingCenterAuto(
                    detbox, MarkParam, AlgorithmParam, &tempResult);
                break;
            default:
                break;
            }
            if (!nError)
            {
                // 将自动检测获取到的靶标参数存入整体结果结构体
                result->DetEveryResult[nSuccessNum].MarkType =
                    tempResult.DetEveryResult[0].MarkType;
                result->DetEveryResult[nSuccessNum].CenterX =
                    tempResult.DetEveryResult[0].CenterX;
                result->DetEveryResult[nSuccessNum].CenterY =
                    tempResult.DetEveryResult[0].CenterY;
                result->DetEveryResult[nSuccessNum].Diameter =
                    tempResult.DetEveryResult[0].Diameter;
                result->DetEveryResult[nSuccessNum].DetScore =
                    tempResult.DetEveryResult[0].DetScore;
                result->DetEveryResult[nSuccessNum].Width =
                    tempResult.DetEveryResult[0].Width;
                result->DetEveryResult[nSuccessNum].Height =
                    tempResult.DetEveryResult[0].Height;
                result->DetEveryResult[nSuccessNum].PartCount =
                    tempResult.DetEveryResult[0].PartCount;
                for (int i = 0; i < tempResult.DetEveryResult[0].PartCount; i++)
                {
                    result->DetEveryResult[nSuccessNum].PartResult[i].MarkType =
                        tempResult.DetEveryResult[0].PartResult[i].MarkType;

                    result->DetEveryResult[nSuccessNum].PartResult[i].CenterX =
                        tempResult.DetEveryResult[0].PartResult[i].CenterX;
                    result->DetEveryResult[nSuccessNum].PartResult[i].CenterY =
                        tempResult.DetEveryResult[0].PartResult[i].CenterY;
                    result->DetEveryResult[nSuccessNum].PartResult[i].Width =
                        tempResult.DetEveryResult[0].PartResult[i].Width;
                    result->DetEveryResult[nSuccessNum].PartResult[i].Height =
                        tempResult.DetEveryResult[0].PartResult[i].Height;
                    result->DetEveryResult[nSuccessNum].PartResult[i].Diameter =
                        tempResult.DetEveryResult[0].PartResult[i].Diameter;
                    result->DetEveryResult[nSuccessNum].PartResult[i].DetScore =
                        tempResult.DetEveryResult[0].PartResult[i].DetScore;
                    result->DetEveryResult[nSuccessNum].PartResult[i].ReserveCount =
                        tempResult.DetEveryResult[0].PartResult[i].ReserveCount;
                    for (int reserve = 0;
                         reserve <
                         tempResult.DetEveryResult[0].PartResult[i].ReserveCount;
                         reserve++)
                    {
                        result->DetEveryResult[nSuccessNum]
                            .PartResult[i]
                            .ReserveModel[reserve] = tempResult.DetEveryResult[0]
                                                         .PartResult[i]
                                                         .ReserveModel[reserve];
                    }
                }
                tempParam.push_back(MarkParam);
                nSuccessNum++;
            }
            possibleError = nError;
        }
    }
    catch (...)
    {
        return -100;
    }

    result->CircleCount = nSuccessNum;
    if (!result->CircleCount)
    {
        if (possibleError)
        {
            return possibleError;
        }
        return CFMEE_AUTODET_ERROR;
    }
    //返回离图像中心最近的中心点坐标
    getLastCenter(mGrayImage, result);
    drawRes(mGrayImage, result);
    resultBox.emplace_back(mGrayImage);
    return nError;
}