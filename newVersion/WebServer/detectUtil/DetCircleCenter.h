#include "opencv2/opencv.hpp"
#include "paraType.h"
#include <string>
#include "Common.h"
#include "LeastSquare.h"

class CdetCircle
{
public:
  CdetCircle(std::string strLog, DetectParameterV3 param,
             ConfigAlgorithmParam algorithmPara);
  CdetCircle();
  ~CdetCircle();

  bool getCircleCenter(const cv::Mat mSrcImage, bool bIsInner,
                       cv::Point3d &pCenterPoint, double &dScore);

  bool
  getLittleCircleCenter(const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi,
                        std::vector<cv::Point3d> &vecLittleCenter);
  template <class inTypePoint, class outTypePoint>
  bool iterFitting(inTypePoint &T, outTypePoint &pCenterPoint, bool isCorrect)
  {
    if (T.size() < 3)
      return false;
    double dSigma = getVariance(T).second;
    double dMeanDis = getVariance(T).first;
    double dDis = 0;
    outTypePoint tempCircle(0, 0, 0);
    tempCircle = leastSquareFittingCircle(T);

    while (dSigma > 1)
    {
      if (T.size() < 3)
        return false;
      tempCircle = leastSquareFittingCircle(T);
      for (int j = 0; j < T.size(); ++j)
      {
        dDis = abs(sqrt(pow((double)T[j].dX - tempCircle.x, 2) +
                        pow((double)T[j].dY - tempCircle.y, 2)) -
                   dMeanDis);
        if (dDis > dSigma)
        {
          T.erase(T.begin() + j);
          j--;
        }
      }
      dMeanDis = getVariance(T).first;
      dSigma = getVariance(T).second;
    }

    if (tempCircle.x != 0 && tempCircle.y != 0)
    {
      if (isCorrect && tempCircle.z >= m_nDimeter / 2 - m_nThreshold &&
          tempCircle.z <= m_nDimeter / 2 + m_nThreshold)
      {
        pCenterPoint.x = tempCircle.x;
        pCenterPoint.y = tempCircle.y;
        pCenterPoint.z = tempCircle.z;
        return true;
      }
      else
      {
        pCenterPoint.x = tempCircle.x;
        pCenterPoint.y = tempCircle.y;
        pCenterPoint.z = tempCircle.z;
        return true;
      }
    }
    else
      return false;
  }

private:
  CdetCircle &operator=(const CdetCircle &);

  /// <summary>
  /// 最大色差拟合检测---非常依赖模板匹配的准确性
  /// </summary>

  bool getCircleCenterByScanContours(cv::Mat mSrcImage,
                                     cv::Point3d &pCenterPoint, double &dScore);

  /// <summary>
  /// 轮廓点分象限删选
  /// </summary>
  int scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput,
                          cv::Point2d pRoughCenter, cv::Point3d &pCenter);
  /// <summary>
  /// 获取待拟合区域,并保存坐标值
  /// </summary>
  /// <typeparam name="TypePoint"></typeparam>
  /// <param name="mSrcImage"></param>
  /// <param name="T"></param>
  template <class TypePoint>
  void getRoughContours(cv::Mat mSrcImage,
                        std::vector<std::vector<TypePoint>> &T)
  {
    cv::Point pp(tRound(mSrcImage.cols / 2), tRound(mSrcImage.rows / 2));
    int i;
    int j;
    int nMaxX = mSrcImage.cols / 2 + (m_nDimeter / 2 + m_nThreshold - 2) >
                        mSrcImage.cols - 2
                    ? mSrcImage.cols - 2
                    : mSrcImage.cols / 2 + (m_nDimeter / 2 + m_nThreshold - 2);
    int nMaxY = mSrcImage.rows / 2 + (m_nDimeter / 2 + m_nThreshold - 2) >
                        mSrcImage.rows - 2
                    ? mSrcImage.rows - 2
                    : mSrcImage.rows / 2 + (m_nDimeter / 2 + m_nThreshold - 2);
    int nStartX = mSrcImage.cols / 2 - (m_nDimeter / 2 + m_nThreshold - 2) > 2
                      ? mSrcImage.cols / 2 - (m_nDimeter / 2 + m_nThreshold - 2)
                      : 2;
    int nStartY = mSrcImage.rows / 2 - (m_nDimeter / 2 + m_nThreshold - 2) > 2
                      ? mSrcImage.rows / 2 - (m_nDimeter / 2 + m_nThreshold - 2)
                      : 2;
    double dRedius = 0;
    double dK = 0;
    double dB = 0;
    double dDisRediusOne = 0;
    int dStepXOne = 0, dStepYOne = 0;
    double dMaxDiffOne = 0;
    //四象限点集容器
    std::vector<TypePoint> vecOne, vecTwo, vecThree, vecFour;
    int nQuadrant = 0;
    for (i = nStartX; i < nMaxX; ++i)
    {
      for (j = nStartY; j < nMaxY; ++j)
      {
        dRedius = sqrt(pow(double(i - pp.x), 2) + pow(double(j - pp.y), 2));
        if (dRedius > m_nDimeter / 2 - m_nThreshold + 2 &&
            dRedius < m_nDimeter / 2 + m_nThreshold - 2)
        {
          dDisRediusOne =
              sqrt(pow(double(i - pp.x), 2) + pow(double(j - pp.y), 2));
          dStepXOne = tRound(double((i - pp.x) / dDisRediusOne));
          dStepYOne = tRound(double((j - pp.y) / dDisRediusOne));
          //获取直径范围内指向中心点方向8领域最大色差
          dMaxDiffOne =
              mSrcImage.at<uchar>(j - 1 * dStepYOne, i - 1 * dStepXOne) +
              mSrcImage.at<uchar>(j - 2 * dStepYOne, i - 2 * dStepXOne) -
              mSrcImage.at<uchar>(j + 1 * dStepYOne, i + 1 * dStepXOne) -
              mSrcImage.at<uchar>(j + 2 * dStepYOne, i + 2 * dStepXOne);
          if (m_bIsPolarChange || m_bIsInner) //存在极性变换删选轮廓
          {
            if (!m_bIsWhite && dMaxDiffOne < DIFF_PIXEL2) //黑色
            {
              nQuadrant = isQuadrant(cv::Point(i, j), pp);
              switch (nQuadrant)
              {
              case 1:
                vecOne.push_back(cv::Point(i, j));
                break;
              case 2:
                vecTwo.push_back(cv::Point(i, j));
                break;
              case 3:
                vecThree.push_back(cv::Point(i, j));
                break;
              case 4:
                vecFour.push_back(cv::Point(i, j));
                break;
              default:
                break;
              }
            }
            else if (m_bIsWhite && dMaxDiffOne > DIFF_PIXEL) //白色
            {
              nQuadrant = isQuadrant(cv::Point(i, j), pp);
              switch (nQuadrant)
              {
              case 1:
                vecOne.push_back(cv::Point(i, j));
                break;
              case 2:
                vecTwo.push_back(cv::Point(i, j));
                break;
              case 3:
                vecThree.push_back(cv::Point(i, j));
                break;
              case 4:
                vecFour.push_back(cv::Point(i, j));
                break;
              default:
                break;
              }
            }
          }
          else if (abs(dMaxDiffOne) >= DIFF_PIXEL)
          {
            nQuadrant = isQuadrant(cv::Point(i, j), pp);
            switch (nQuadrant)
            {
            case 1:
              vecOne.push_back(cv::Point(i, j));
              break;
            case 2:
              vecTwo.push_back(cv::Point(i, j));
              break;
            case 3:
              vecThree.push_back(cv::Point(i, j));
              break;
            case 4:
              vecFour.push_back(cv::Point(i, j));
              break;
            default:
              break;
            }
          }
        }
      }
    }
    T.push_back(vecOne);
    T.push_back(vecTwo);
    T.push_back(vecThree);
    T.push_back(vecFour);
  }

  /// <summary>
  /// 获取待拟合轮廓--通过色差获取待拟合轮廓点
  /// </summary>
  /// <typeparam name="TypePoint"></typeparam>
  /// <param name="mSrcImage"></param>
  /// <param name="out"></param>
  template <class inTypePoint, class outTypePoint>
  void getFitContours(inTypePoint &in, cv::Mat mSrcImage, outTypePoint &out)
  {
    cv::Point pp(tRound(mSrcImage.cols / 2), tRound(mSrcImage.rows / 2));
    double dMaxDis = 0;
    double dCheckDistance = 0;
    // y=kx+b
    double dK = 0;
    double dB = 0;
    double dK2 = 0;
    double dB2 = 0;
    double dDisRediusOne = 0;
    int dStepXOne = 0, dStepYOne = 0;
    double dMaxDiffOne = 0;
    double dDisRediusTwo = 0;
    int dStepXTwo = 0, dStepYTwo = 0;
    double dMaxDiffTwo = 0;

    int x = 0, y = x + 1;
    cv::Point pMaxPoint(0, 0);
    //精删选
    for (int i = 0; i < in.size(); ++i)
    {
      if (in[i].size() < 2)
        continue;
      for (x = 0; x < in[i].size() - 1; ++x)
      {
        if (in[i][x].x - pp.x != 0)
        {
          dK = double(in[i][x].y - pp.y) / double(in[i][x].x - pp.x);
          dB = double(pp.y * in[i][x].x - in[i][x].y * pp.x) /
               double(in[i][x].x - pp.x);
        }
        else
        {
          dK = 10e9;
          dB = 0;
        }
        dDisRediusOne = sqrt(pow(double(in[i][x].x - pp.x), 2) +
                             pow(double(in[i][x].y - pp.y), 2));
        dStepXOne = tRound(
            double((in[i][x].x - pp.x) / (dDisRediusOne + 0.00000001))); // cos
        dStepYOne = tRound(
            double((in[i][x].y - pp.y) / (dDisRediusOne + 0.00000001))); // sin
        //内层或使用极性检测
        if (m_bIsPolarChange || m_bIsInner)
          dMaxDiffOne = mSrcImage.at<uchar>(in[i][x].y - 1 * dStepYOne,
                                            in[i][x].x - 1 * dStepXOne) +
                        mSrcImage.at<uchar>(in[i][x].y - 2 * dStepYOne,
                                            in[i][x].x - 2 * dStepXOne) -
                        mSrcImage.at<uchar>(in[i][x].y + 1 * dStepYOne,
                                            in[i][x].x + 1 * dStepXOne) -
                        mSrcImage.at<uchar>(in[i][x].y + 2 * dStepYOne,
                                            in[i][x].x + 2 * dStepXOne);
        else
          dMaxDiffOne = abs(mSrcImage.at<uchar>(in[i][x].y - 1 * dStepYOne,
                                                in[i][x].x - 1 * dStepXOne) +
                            mSrcImage.at<uchar>(in[i][x].y - 2 * dStepYOne,
                                                in[i][x].x - 2 * dStepXOne) -
                            mSrcImage.at<uchar>(in[i][x].y + 1 * dStepYOne,
                                                in[i][x].x + 1 * dStepXOne) -
                            mSrcImage.at<uchar>(in[i][x].y + 2 * dStepYOne,
                                                in[i][x].x + 2 * dStepXOne));
        dMaxDis = dMaxDiffOne;
        dCheckDistance = dDisRediusOne;
        //靶标边缘颜色选择
        if (m_bIsInner) //内层靶标默认为黑色
        {
          pMaxPoint = getMinScale(
              mSrcImage, in[i][x],
              cv::Point(in[i][x].x - 1 * dStepXOne, in[i][x].y - 1 * dStepYOne),
              cv::Point(in[i][x].x + 1 * dStepXOne, in[i][x].y + 1 * dStepYOne),
              cv::Point(in[i][x].x - 2 * dStepXOne, in[i][x].y - 2 * dStepYOne),
              cv::Point(in[i][x].x + 2 * dStepXOne, in[i][x].y + 2 * dStepYOne));
        }
        else if (m_bWhiteEdge) //白色
        {
          pMaxPoint = getMaxScale(
              mSrcImage, in[i][x],
              cv::Point(in[i][x].x - 1 * dStepXOne, in[i][x].y - 1 * dStepYOne),
              cv::Point(in[i][x].x + 1 * dStepXOne, in[i][x].y + 1 * dStepYOne),
              cv::Point(in[i][x].x - 2 * dStepXOne, in[i][x].y - 2 * dStepYOne),
              cv::Point(in[i][x].x + 2 * dStepXOne, in[i][x].y + 2 * dStepYOne));
        }
        else //黑色
        {
          pMaxPoint = getMinScale(
              mSrcImage, in[i][x],
              cv::Point(in[i][x].x - 1 * dStepXOne, in[i][x].y - 1 * dStepYOne),
              cv::Point(in[i][x].x + 1 * dStepXOne, in[i][x].y + 1 * dStepYOne),
              cv::Point(in[i][x].x - 2 * dStepXOne, in[i][x].y - 2 * dStepYOne),
              cv::Point(in[i][x].x + 2 * dStepXOne, in[i][x].y + 2 * dStepYOne));
        }
        for (y = x + 1; y < in[i].size(); ++y)
        {
          if (in[i][y].x - pp.x != 0)
          {
            dK2 = double(in[i][y].y - pp.y) / double(in[i][y].x - pp.x);
            dB2 = double(pp.y * in[i][y].x - in[i][y].y * pp.x) /
                  double(in[i][y].x - pp.x);
          }
          else
          {
            dK2 = 10e9;
            dB2 = 0;
          }
          if (abs(atan(dK) - atan(dK2)) <= double(CV_PI / 180))
          {
            dDisRediusTwo = sqrt(pow(double(in[i][y].x - pp.x), 2) +
                                 pow(double(in[i][y].y - pp.y), 2));
            dStepXTwo = tRound(
                double((in[i][y].x - pp.x) / (dDisRediusTwo + 0.00000001)));
            dStepYTwo = tRound(
                double((in[i][y].y - pp.y) / (dDisRediusTwo + 0.00000001)));
            if (m_bIsPolarChange || m_bIsInner)
              dMaxDiffTwo = mSrcImage.at<uchar>(in[i][y].y - 1 * dStepYTwo,
                                                in[i][y].x - 1 * dStepXTwo) +
                            mSrcImage.at<uchar>(in[i][y].y - 2 * dStepYTwo,
                                                in[i][y].x - 2 * dStepXTwo) -
                            mSrcImage.at<uchar>(in[i][y].y + 1 * dStepYTwo,
                                                in[i][y].x + 1 * dStepXTwo) -
                            mSrcImage.at<uchar>(in[i][y].y + 2 * dStepYTwo,
                                                in[i][y].x + 2 * dStepXTwo);
            else
              dMaxDiffTwo = abs(mSrcImage.at<uchar>(in[i][y].y - 1 * dStepYTwo,
                                                    in[i][y].x - 1 * dStepXTwo) +
                                mSrcImage.at<uchar>(in[i][y].y - 2 * dStepYTwo,
                                                    in[i][y].x - 2 * dStepXTwo) -
                                mSrcImage.at<uchar>(in[i][y].y + 1 * dStepYTwo,
                                                    in[i][y].x + 1 * dStepXTwo) -
                                mSrcImage.at<uchar>(in[i][y].y + 2 * dStepYTwo,
                                                    in[i][y].x + 2 * dStepXTwo));
            if (m_bIsPolarChange)
            {
              if (!m_bIsWhite || m_bIsInner) //黑色靶标
              {
                if (dMaxDiffTwo < dMaxDis && dMaxDiffTwo < DIFF_PIXEL2)
                {
                  //获取像素值最低的像素点作为坐标点返回
                  pMaxPoint = getMinScale(mSrcImage, in[i][y],
                                          cv::Point(in[i][y].x - 1 * dStepXTwo,
                                                    in[i][y].y - 1 * dStepYTwo),
                                          cv::Point(in[i][y].x + 1 * dStepXTwo,
                                                    in[i][y].y + 1 * dStepYTwo),
                                          cv::Point(in[i][y].x - 2 * dStepXTwo,
                                                    in[i][y].y - 2 * dStepYTwo),
                                          cv::Point(in[i][y].x + 2 * dStepXTwo,
                                                    in[i][y].y + 2 * dStepYTwo));
                  dMaxDis = dMaxDiffTwo;
                }
              }
              else //白色靶标
              {
                if (dMaxDiffTwo > dMaxDis && dMaxDiffTwo > DIFF_PIXEL)
                {
                  if (m_bWhiteEdge) //白色
                  {
                    //获取像素值最高的像素点作为坐标点返回
                    pMaxPoint =
                        getMaxScale(mSrcImage, in[i][y],
                                    cv::Point(in[i][y].x - 1 * dStepXTwo,
                                              in[i][y].y - 1 * dStepYTwo),
                                    cv::Point(in[i][y].x + 1 * dStepXTwo,
                                              in[i][y].y + 1 * dStepYTwo),
                                    cv::Point(in[i][y].x - 2 * dStepXTwo,
                                              in[i][y].y - 2 * dStepYTwo),
                                    cv::Point(in[i][y].x + 2 * dStepXTwo,
                                              in[i][y].y + 2 * dStepYTwo));
                  }
                  else //黑色
                  {
                    pMaxPoint =
                        getMinScale(mSrcImage, in[i][y],
                                    cv::Point(in[i][y].x - 1 * dStepXTwo,
                                              in[i][y].y - 1 * dStepYTwo),
                                    cv::Point(in[i][y].x + 1 * dStepXTwo,
                                              in[i][y].y + 1 * dStepYTwo),
                                    cv::Point(in[i][y].x - 2 * dStepXTwo,
                                              in[i][y].y - 2 * dStepYTwo),
                                    cv::Point(in[i][y].x + 2 * dStepXTwo,
                                              in[i][y].y + 2 * dStepYTwo));
                  }
                  dMaxDis = dMaxDiffTwo;
                }
              }
            }
            else
            {
              if (dMaxDiffTwo > dMaxDis)
              {
                if (m_bWhiteEdge) //白色
                {
                  pMaxPoint = getMaxScale(mSrcImage, in[i][y],
                                          cv::Point(in[i][y].x - 1 * dStepXTwo,
                                                    in[i][y].y - 1 * dStepYTwo),
                                          cv::Point(in[i][y].x + 1 * dStepXTwo,
                                                    in[i][y].y + 1 * dStepYTwo),
                                          cv::Point(in[i][y].x - 2 * dStepXTwo,
                                                    in[i][y].y - 2 * dStepYTwo),
                                          cv::Point(in[i][y].x + 2 * dStepXTwo,
                                                    in[i][y].y + 2 * dStepYTwo));
                }
                else //黑色
                {
                  pMaxPoint = getMinScale(mSrcImage, in[i][y],
                                          cv::Point(in[i][y].x - 1 * dStepXTwo,
                                                    in[i][y].y - 1 * dStepYTwo),
                                          cv::Point(in[i][y].x + 1 * dStepXTwo,
                                                    in[i][y].y + 1 * dStepYTwo),
                                          cv::Point(in[i][y].x - 2 * dStepXTwo,
                                                    in[i][y].y - 2 * dStepYTwo),
                                          cv::Point(in[i][y].x + 2 * dStepXTwo,
                                                    in[i][y].y + 2 * dStepYTwo));
                }
                dMaxDis = dMaxDiffTwo;
              }
            }
            in[i].erase(in[i].begin() + y);
            y--;
          }
        }
        ContoursPoint cPoint(pMaxPoint.x, pMaxPoint.y, 0, 0, 1, dMaxDis);
        out.push_back(cPoint);
      }
    }
  }

  /// <summary>
  /// 迭代加权拟合
  /// </summary>
  /// <typeparam name="inTypePoint"></typeparam>
  /// <typeparam name="weightType"></typeparam>
  /// <typeparam name="outTypePoint"></typeparam>
  /// <param name="T"></param>
  /// <param name="pCenterPoint"></param>
  /// <returns></returns>
  template <class inTypePoint, class outTypePoint>
  bool iterWightFitting(inTypePoint &T, outTypePoint &pCenterPoint)
  {
    if (T.size() < 3)
      return false;
    int nIterCount = 10;
    outTypePoint tempCircle(0, 0, 0);
    tempCircle = leastSquareWeightFittingCircle(T);
    for (int i = 0; i < nIterCount; ++i)
    {
      std::vector<double> vDist;
      for (int x = 0; x < T.size(); ++x)
      {
        double dDist =
            abs(sqrt((T[x].dX - tempCircle.x) * (T[x].dX - tempCircle.x) +
                     (T[x].dY - tempCircle.y) * (T[x].dY - tempCircle.y)) -
                tempCircle.z);
        vDist.push_back(dDist);
      }
      std::vector<double> vDistCopy;
      vDistCopy.assign(vDist.begin(), vDist.end());

      sort(vDistCopy.begin(), vDistCopy.end());
      double sigma = vDistCopy[vDistCopy.size() / 2] / 0.075;
      sigma *= 2;
      for (int j = 0; j < vDist.size(); ++j)
      {
        // Tukey
        if (vDist[j] <= sigma)
        {
          double rate = vDist[j] / sigma;
          T[j].dWeight = pow(double(1 - rate * rate), 2);
        }
        else
        {
          T[j].dWeight = 0;
        }
      }
    }
    tempCircle = leastSquareWeightFittingCircle(T);
    if (tempCircle.x != 0 && tempCircle.y != 0 &&
        tempCircle.z >= m_nDimeter / 2 - m_nThreshold &&
        tempCircle.z <= m_nDimeter / 2 + m_nThreshold)
    {
      pCenterPoint.x = tempCircle.x;
      pCenterPoint.y = tempCircle.y;
      pCenterPoint.z = tempCircle.z;
      return true;
    }
    else
      return false;
  }

  double getEnhanceThreshold(cv::Mat mSrcImage)
  {
    cv::Mat mat_mean, mat_stddev;
    //全局均值和方差
    meanStdDev(mSrcImage, mat_mean, mat_stddev);
    double m1, m2;
    m1 = mat_mean.at<double>(0, 0);
    double dRediusRoi = m_nDimeter / 2 * sin(0.69);
    cv::Mat mCenterRoi = mSrcImage(cv::Rect(mSrcImage.cols / 2 - dRediusRoi, mSrcImage.rows / 2 - dRediusRoi,
                                            dRediusRoi * 2, dRediusRoi * 2));
    meanStdDev(mCenterRoi, mat_mean, mat_stddev);
    m2 = mat_mean.at<double>(0, 0);
    //考虑到光照不均的情况，阈值+0.1
    return m2 / m1 + 0.1 > 1 ? 1 : m2 / m1 + 0.1;
  }

private:
  std::string m_strLog;  //日志文档
  int m_nDimeter;        //直径参数
  int m_nThreshold;      //误差参数
  bool m_bIsWhite;       //靶标为白色
  int m_nAlgorithm;      //识别算法
  bool m_bWhiteEdge;     //白色靶标边缘
  bool m_bIsPolarChange; //极性检测
  float m_fCircleSocre;  //检测阈值参数
  bool m_bIsEllipse;     //椭圆检测
  bool m_bIsEnhance;     //图像增强
  bool m_bIsInner;       //内层检测
};