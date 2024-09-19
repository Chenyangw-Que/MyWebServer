#include "DetCircleCenter.h"
#include "Common.h"
#include "Edge.h"
#include "EnhanceImage.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "ListSort.h"
#include "LogHelp.h"
#include "paraType.h"
#include <vector>
#include "FitCircleT.h"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
/// <summary>
/// 构造函数
/// </summary>
/// <param name="strLog">日志文件路径</param>
/// <param name="param">靶标参数结构体</param>
/// <param name="algorithmPara">算法参数结构体</param>
CdetCircle::CdetCircle(std::string strLog, DetectParameterV3 param,
                       ConfigAlgorithmParam algorithmPara)
{
  m_strLog = strLog;
  m_nDimeter = param.Diameter[0];
  m_nThreshold = param.Threshold;
  m_bIsWhite = param.IsWhite;
  m_nAlgorithm = algorithmPara.nMarkAlgorithm;
  m_bWhiteEdge = algorithmPara.nWhiteEdge;
  m_bIsPolarChange = algorithmPara.IsPolarChange;
  m_fCircleSocre = algorithmPara.dSocreThreshold;
  m_bIsEllipse = algorithmPara.nEllipseDetection;
  m_bIsEnhance = algorithmPara.IsEnhanceMap;
}

CdetCircle::CdetCircle()
{
  m_nDimeter = 0;
  m_nThreshold = 0;
}

CdetCircle::~CdetCircle() {}

/// <summary>
/// 圆检测算法
/// </summary>
/// <param name="mSrcImage">输入检测图片</param>
/// <param name="bIsInner">内层Imark检测</param>
/// <param name="pCenterPoint">检测靶标中心点</param>
/// <param name="dScore">靶标检测得分</param>
/// <returns></returns>
bool CdetCircle::getCircleCenter(const cv::Mat mSrcImage, bool bIsInner,
                                 cv::Point3d &pCenterPoint, double &dScore)
{
  if (!mSrcImage.data)
    return false;
  cv::Mat mDesImage;
  mSrcImage.copyTo(mDesImage);
  cv::Point3d pRoughCenterPoint(0, 0, 0);
  m_bIsInner = bIsInner;
  if (bIsInner)
  {
    writeLog(m_strLog, "内层对准检测");
    if (getCircleCenterByScanContours(mDesImage, pRoughCenterPoint, dScore))
    {
      pCenterPoint = pRoughCenterPoint;
      return true;
    }
  }
  else
  {
    writeLog(m_strLog, "外层对准检测");
    switch (m_nAlgorithm)
    {
    case 0:
      writeLog(m_strLog, "使用最大色差检测");
      if (getCircleCenterByScanContours(mDesImage, pRoughCenterPoint, dScore))
      {
        pCenterPoint = pRoughCenterPoint;
        return true;
      }
      break;
    default:
      writeLog(m_strLog, "ERROR:此算法不存在");
      break;
    }
  }
  return false;
}

/// <summary>
/// 小圆检测算法
/// </summary>
/// <param name="mSrcImage">输入检测图片</param>
/// <param name="bIsInner">内层Imark检测</param>
/// <param name="pCenterPoint">检测靶标中心点</param>
/// <param name="dScore">靶标检测得分</param>
/// <returns></returns>
bool CdetCircle::getLittleCircleCenter(
    const std::vector<pair<cv::Mat, cv::Point>> refinedRoi,
    std::vector<cv::Point3d> &vecLittleCenter)
{
  cv::Point pLittleCircleLocation;
  cv::Mat mFilterImage, mEnhanceImage, mGradientImage, mGradientImage2;
  // std::vector<std::pair<cv::Mat, cv::Point>>vecImage;
  std::vector<std::pair<cv::Mat, cv::Point>> vecImage2;
  vector<vector<Point>> opencvContours;
  vector<Vec4i> opencvHierarchy;
  Point2f pContour(0, 0);            //最小外接圆圆心
  cv::Point3d circleCenter(0, 0, 0); //拟合后的圆中心坐标
  float fRedius = 0;
  //滤波初始化
  CFilter filter;
  //梯度检测初始化
  CGradient getGradient;
  //增强初始化
  CEnhance enhance(m_bIsWhite);
  double dRows = 0, dCols = 0;
  std::vector<cv::Point2d> vecContours, vecOut;

  double dDisRediusOne, dStepXOne, dStepYOne, dMaxDiffOne;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  for (int i = 0; i < refinedRoi.size(); ++i)
  {
    filter.getFilter(refinedRoi[i].first, 3, "2", mFilterImage);
    filter.getFilter(mFilterImage, 4, "3", mFilterImage);

    std::vector<cv::Point2d> vecGradient;
    mGradientImage =
        cv::Mat::zeros(mFilterImage.rows, mFilterImage.cols, CV_8UC1);
    getGradient.getGradientImage(mFilterImage, mGradientImage, 0);
    //轮廓连接
    ConnectEdge(mGradientImage, mGradientImage2);
    findContours(mGradientImage2, contours, hierarchy, RETR_EXTERNAL,
                 CHAIN_APPROX_NONE);
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(refinedRoi[i].first);
    for (int x = 0; x < contours.size(); ++x)
    {
      Scalar color(rand() & 255, rand() & 255, rand() & 255);
      drawContours(mShowImage, contours, x, color, 2, 8, hierarchy);
    }
    CvWaitShowImage(mShowImage, "轮廓图");
#endif
    if (contours.size() <= 0)
    {
      writeLog(m_strLog, "未检测到满足条件的区域");
      continue;
    }
    else
    {
      sort(contours.begin(), contours.end(),
           [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) {
             return a.size() > b.size();
           });

      //根据外接圆面积筛选轮廓
      for (int x = 0; x < contours.size(); ++x)
      {
        if (contours[x].size() < 2)
          continue;
        //计算轮廓重心
        Moments moment;            //矩
        cv::Mat temp(contours[x]); //第一个轮廓
        moment = moments(temp, false);
        cv::Point2d pt1;
        if (moment.m00 != 0) //除数不能为0
        {
          pt1.x = cvRound(moment.m10 / moment.m00); //计算重心横坐标
          pt1.y = cvRound(moment.m01 / moment.m00); //计算重心纵坐标
        }

        /*if (abs(vecImage[i].first.cols/2- pt1.x)>10 ||
                abs(vecImage[i].first.rows / 2 - pt1.y)>10)
                continue;*/
        //最小外接圆
        minEnclosingCircle(contours[x], pContour, fRedius);
        // float fLength = arcLength(contours[i], true);
        float fArea = contourArea(contours[x]);
        if (fRedius >= m_nDimeter / 2 - m_nThreshold &&
            fRedius <= m_nDimeter / 2 + m_nThreshold
            //&&fArea > m_nDimeter / 4 * m_nDimeter / 4 * CV_PI && fArea <
            // m_nDimeter * m_nDimeter * CV_PI
        )
        {
#if _DEBUG
          cv::Mat mShowImage2 = generateColorImage(refinedRoi[i].first);
          drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8,
                       hierarchy);
          CvWaitShowImage(mShowImage2, "轮廓图2");
#endif
          //精确计算
          // if (m_nDeepCalculation)
          {
            writeLog(m_strLog, "启用精确计算");
            //满足条件的轮廓按照不同象限进行分类处理
            scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
            //存储小圆中心点坐标
            vecLittleCenter.push_back(
                cv::Point3d(circleCenter.x + refinedRoi[i].second.x,
                            circleCenter.y + refinedRoi[i].second.y, fRedius));
          }
          /*else
                  vecLittleCenter.push_back(cv::Point3d(pContour.x +
             vecImage[i].second.x, pContour.y + vecImage[i].second.y,
             fRedius));*/
          break;
        }
      }
    }
  }
  if (vecLittleCenter.size() >= 1)
    return true;
  return false;
}

CdetCircle &CdetCircle::operator=(const CdetCircle &)
{
  // TODO: 在此处插入 return 语句
  return *this;
}

/// <summary>
/// 最大色差拟合检测---非常依赖模板匹配的准确性
/// </summary>
/// <param name="mSrcImage">输入检测图片</param>
/// <param name="pCenterPoint">检测靶标中心点</param>
/// <param name="dScore">靶标检测得分</param>
/// <returns></returns>
bool CdetCircle::getCircleCenterByScanContours(cv::Mat mSrcImage,
                                               cv::Point3d &pCenterPoint,
                                               double &dScore)
{
  if (!mSrcImage.data)
    return false;
  CFilter filter;
  CGradient gradient;
  CEnhance enhance(m_bIsWhite);
  cv::Mat mFilterImage, mGradientImage, mEnhanceImage, mDesImage;
  mSrcImage.copyTo(mDesImage);
  //内层Imark进行局部图像增强
  if (m_bIsInner)
  {
    writeLog(m_strLog, "内层Imark检测");
    if (m_bIsEnhance)
    {
      writeLog(m_strLog, "使用图像增强");
      /*if (enhance.FourierTransform(mDesImage, 4, mFilterImage, true, 1, 5))
              writeLog(m_strLog, "使用傅里叶变换");
      double dMeanThreshold = getEnhanceThreshold(mFilterImage);*/
      //图像增强
      // enhance.getLocalEnhance(mFilterImage, dMeanThreshold, 1, 0, 1,
      // mFilterImage, true);
      enhance.getEnhanceImage(mDesImage, 4, mFilterImage, 0, 0, 400, 10);
#ifdef _SHOW_PROCESS_
      CvWaitShowImage(mFilterImage, "增强图片");
#endif
      filter.getFilter(mFilterImage, 3, "11", mFilterImage);
    }
    else
    {
      //滤波
      filter.getFilter(mDesImage, 3, "11", mFilterImage);
    }
    filter.getFilter(mFilterImage, 4, "7", mFilterImage);
  }
  else //外层只滤波，不增强
  {
    writeLog(m_strLog, "外层靶标检测");
    if (m_bIsEnhance)
    {
      writeLog(m_strLog, "使用图像增强");
      double dMeanThreshold = getEnhanceThreshold(mDesImage);
      //图像增强
      // enhance.getEnhanceImage(mDesImage, 3, mFilterImage);
      enhance.getEnhanceImage(mDesImage, 4, mFilterImage, 0, 0, 100, 3);
#ifdef _SHOW_PROCESS_
      CvWaitShowImage(mFilterImage, "增强图片");
#endif
      //滤波
      filter.getFilter(mFilterImage, 3, "11", mFilterImage);
    }
    else
    {
      //滤波
      filter.getFilter(mDesImage, 3, "11", mFilterImage);
    }
    // filter.getFilter(mFilterImage, 4, "7", mFilterImage);
  }
  //获取最大色差轮廓--细节处待优化
  std::vector<ContoursPoint> vecRoughContours;
  // std::vector<std::pair<double, cv::Point2d>> vecRoughContours;
  std::vector<cv::Point2d> vecLastContours;
  std::vector<std::vector<cv::Point>> vecRough, vecRough1;
  //待检测区域获取，并按照象限分成四部分
  getRoughContours(mFilterImage, vecRough);
  if (vecRough.empty())
  {
    writeLog(m_strLog, "ERROR：轮廓粗分类失败");
    return false;
  }
#ifdef _SHOW_PROCESS_
  cv::Mat mQuadrantImage = fourQuadrantMap(mDesImage, vecRough);
  CvWaitShowImage(mQuadrantImage, "四象限轮廓");
#endif
  getFitContours(vecRough, mFilterImage, vecRoughContours);
  //理想情况下一个角度会有一个轮廓点，即共有360个轮廓点,事实上受限于计算斜率方式最佳大约有300个轮廓点左右
  if (vecRoughContours.size() < 180)
  {
    writeLog(m_strLog, "ERROR：图像质量不佳，可使用轮廓点数过少");
    writeLog(m_strLog, "PROMPT：输入直径参数可能不准确");
    return false;
  }
#ifdef _SHOW_PROCESS_
  cv::Mat mShowImage = drawColorImage(mDesImage, vecRoughContours);
  CvWaitShowImage(mShowImage, "对比最强烈区域");
#endif
  double dThreshold = vecRoughContours.size();
  //椭圆拟合
  if (m_bIsEllipse)
  {
  }
  else
  {
    writeLog(m_strLog, "圆拟合检测");
    if (iterFitting(vecRoughContours, pCenterPoint, true) &&
        vecRoughContours.size() >= dThreshold * m_fCircleSocre &&
        pCenterPoint.x != 0 && pCenterPoint.y != 0)
    {
      if (pCenterPoint.z >= (m_nDimeter - m_nThreshold * 2) / 2 &&
          pCenterPoint.z <= (m_nDimeter + m_nThreshold * 2) / 2)
      {
        //对删选后的轮廓点分布进行校验，至少分布在三个象限认为可信
        if (isRightPick(mFilterImage, vecRoughContours))
        {
          if (m_bIsInner)
          {
          }
          else
          {
            dScore = dScore + 20 + (vecRoughContours.size() / dThreshold * 20);
#ifdef _SHOW_PROCESS_
            cv::Mat mShowImageT = drawColorImage(mDesImage, vecRoughContours);
            line(mShowImageT, Point(pCenterPoint.x - 30, pCenterPoint.y),
                 Point(pCenterPoint.x + 30, pCenterPoint.y), Scalar(255, 0, 0),
                 2, 8);
            line(mShowImageT, Point(pCenterPoint.x, pCenterPoint.y - 30),
                 Point(pCenterPoint.x, pCenterPoint.y + 30), Scalar(255, 0, 0),
                 2, 8);
            CvWaitShowImage(mShowImageT, "最终删选区域");
#endif
          }
        }
        else
        {
          writeLog(m_strLog, "ERROR：轮廓点象限校验未通过，检测结果置信度低,"
                             "轮廓未分布在三象限内");
          return false;
        }
      }
      else
      {
        writeLog(m_strLog, "ERROR：检测结果直径不符合合理范围，检测直径：" +
                               to_string(double(pCenterPoint.z * 2)));
        return false;
      }
    }
    else
    {
      writeLog(
          m_strLog,
          "ERROR：边缘点离散度过高，检测结果置信度低,满足条件的轮廓点占比:" +
              to_string(double(vecLastContours.size() / dThreshold)));
      writeLog(m_strLog,
               "PROMPT：可以根据实际情况调正配置文件中CircleSocreThreshold值");
      return false;
    }
  }
  return true;
}

int CdetCircle::scanQuadrantContour(cv::Mat mSrcImage,
                                    std::vector<cv::Point> vecInput,
                                    cv::Point2d pRoughCenter,
                                    cv::Point3d &pCenter)
{
  int nQuadrant = 0;
  std::vector<cv::Point> vecOne, vecTwo, vecThree, vecFour;
  std::vector<std::vector<cv::Point>> vecRough;
  std::vector<ContoursPoint> vecContours;
  double dK = 0;
  double dB = 0;
  double dK2 = 0;
  double dB2 = 0; //圆周上边缘点到圆心直线参数
  double dDisRediusOne = 0;
  double dDisRediusTwo = 0; //轮廓点到圆心距离
  double dCheckDistance = 0;
  //将轮廓点按照四象限进行分离
  for (int i = 0; i < vecInput.size(); ++i)
  {
    nQuadrant =
        isQuadrant(cv::Point(vecInput[i].x, vecInput[i].y), pRoughCenter);
    switch (nQuadrant)
    {
    case 1:
      vecOne.push_back(cv::Point(vecInput[i].x, vecInput[i].y));
      break;
    case 2:
      vecTwo.push_back(cv::Point(vecInput[i].x, vecInput[i].y));
      break;
    case 3:
      vecThree.push_back(cv::Point(vecInput[i].x, vecInput[i].y));
      break;
    case 4:
      vecFour.push_back(cv::Point(vecInput[i].x, vecInput[i].y));
      break;
    default:
      break;
    }
  }
  vecRough.push_back(vecOne);
  vecRough.push_back(vecTwo);
  vecRough.push_back(vecThree);
  vecRough.push_back(vecFour);
#if _DEBUG
  cv::Mat mQuadrantImage = fourQuadrantMap(mSrcImage, vecRough);
  CvWaitShowImage(mQuadrantImage, "四象限轮廓");
#endif
  for (int i = 0; i < vecRough.size(); ++i)
    if (vecRough[i].size() > 1)
    {
      for (int j = 0; j < vecRough[i].size() - 1; ++j)
      {
        cv::Point pContours(0, 0);
        if (vecRough[i][j].x - pRoughCenter.x != 0)
        {
          dK = double(vecRough[i][j].y - pRoughCenter.y) /
               double(vecRough[i][j].x - pRoughCenter.x);
          dB = double(pRoughCenter.y * vecRough[i][j].x -
                      vecRough[i][j].y * pRoughCenter.x) /
               double(vecRough[i][j].x - pRoughCenter.x);
        }
        else
        {
          dK = 10e9;
          dB = 0;
        }
        dDisRediusOne = sqrt(pow(double(vecRough[i][j].x - pRoughCenter.x), 2) +
                             pow(double(vecRough[i][j].y - pRoughCenter.y), 2));
        dCheckDistance = dDisRediusOne;
        pContours = vecRough[i][j];
        for (int x = j + 1; x < vecRough[i].size(); ++x)
        {
          if (vecRough[i][x].x - pRoughCenter.x != 0)
          {
            dK2 = double(vecRough[i][x].y - pRoughCenter.y) /
                  double(vecRough[i][x].x - pRoughCenter.x);
            dB2 = double(pRoughCenter.y * vecRough[i][x].x -
                         vecRough[i][x].y * pRoughCenter.x) /
                  double(vecRough[i][x].x - pRoughCenter.x);
          }
          else
          {
            dK2 = 10e9;
            dB2 = 0;
          }
          //每2弧度选取一个轮廓点
          if (abs(atan(dK) - atan(dK2)) <= double(CV_PI / 90))
          {
            dDisRediusTwo =
                sqrt(pow(double(vecRough[i][x].x - pRoughCenter.x), 2) +
                     pow(double(vecRough[i][x].y - pRoughCenter.y), 2));
            if (dCheckDistance > dDisRediusTwo)
            {
              pContours = vecRough[i][x];
              dCheckDistance = dDisRediusTwo;
            }
            vecRough[i].erase(vecRough[i].begin() + x);
            x--;
          }
        }
        ContoursPoint cPoint(pContours.x, pContours.y, 0, 0, 1, 0);
        vecContours.push_back(cPoint);
      }
    }

#if _DEBUG
  cv::Mat mShowImage = drawColorImage(mSrcImage, vecContours);
  CvWaitShowImage(mShowImage, "删选后的轮廓点");
#endif
  cv::Point3f tempCircle = leastSquareFittingCircle(vecContours);
  if (tempCircle.x != 0 && tempCircle.y != 0)
  {
    pCenter.x = tempCircle.x;
    pCenter.y = tempCircle.y;
    pCenter.z = tempCircle.z;
  }
  return 0;
}
