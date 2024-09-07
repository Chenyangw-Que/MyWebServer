#pragma once
#include "FitLine.h"
#include "opencv2/opencv.hpp"
#include "paraType.h"
#include "ListSort.h"
/// <summary>
/// 获取当前时间
/// </summary>
/// <returns></returns>
std::string getNowTime();


int isPureColorImage(const cv::Mat mSrcImage);



/// <summary>
/// 图片画点
/// </summary>
/// <param name="pPoints"></param>
/// <param name="nPointsCnt"></param>
template <class TypePoint>
cv::Mat WritePointsOnImage(TypePoint *pPoints, int nPointsCnt, int width = 1280,
                           int height = 960)
{
  cv::Mat mDesImage = cv::Mat::zeros(height, width, CV_8UC1);
  for (int i = 0; i < nPointsCnt; ++i)
  {
    if (pPoints[i].seq != 0)
    {
      if (pPoints[i].x >= 0 && pPoints[i].x < width && pPoints[i].y >= 0 &&
          pPoints[i].y < height)
      {
        mDesImage.at<uchar>(pPoints[i].y, pPoints[i].x) = 255;
      }
    }
  }
  return mDesImage;
}

/// <summary>
/// 将GrayBitmap灰度图转换成彩色Mat图
/// </summary>
/// <param name="gBitmap"></param>
/// <returns></returns>
cv::Mat GrayBitmapToMatColor(GrayBitmap *gBitmap);

/// <summary>
/// 图像上画点
/// </summary>
/// <param name="mSrcImage">待算法图像</param>
/// <param name="color">颜色选择</param>
/// <param name="x">横坐标</param>
/// <param name="y">纵坐标</param>
/// <returns></returns>
void CvDrawPointOnImage(cv::Mat &mSrcImage, cv::Scalar color, int x, int y);

/// <summary>
/// 保存图片名称
/// </summary>
/// <param name="nDiameter"></param>
/// <param name="nThreshold"></param>
/// <param name="strImagePath"></param>
void saveImage(std::vector<double> vecParam, std::string &strImagePath);

/// <summary>
/// 检测结果绘图保存
/// </summary>
/// <param name="strResultImagePath"></param>
/// <param name="mSrcImage"></param>
/// <param name="result"></param>
void saveResultImage(std::string strResultImagePath, cv::Mat mSrcImage,
                     DetectResultV3 *result);

/// <summary>
/// 判断模板背景颜色
/// </summary>
/// <param name="mSrcImage"></param>
/// <returns></returns>
bool isWhiteBackground(cv::Mat mSrcImage);

/// <summary>
/// 模板图像按中心点旋转
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="fAngle"></param>
/// <returns></returns>
cv::Mat rotateImageCenter(cv::Mat mSrcImage, float fAngle);

/// <summary>
/// 多模板匹配时最终返回结果
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="result"></param>
void getLastCenter(const cv::Mat mSrcImage, DetectResultV3 *result);

/// <summary>
/// 判断点处于哪一个象限
/// </summary>
/// <typeparam name="TypePointone"></typeparam>
/// <typeparam name="TypePointthree"></typeparam>
/// <param name="one"></param>
/// <param name="center"></param>
/// <returns></returns>
template <class TypePointone, class TypePointthree>
inline int isQuadrant(TypePointone one, TypePointthree center)
{
  if (one.x >= center.x && one.y < center.y)
    return 1;
  else if (one.x > center.x && one.y >= center.y)
    return 2;
  else if (one.x <= center.x && one.y > center.y)
    return 3;
  else
    return 4;
}

/// <summary>
/// 四舍五入
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="r"></param>
/// <returns></returns>
template <class T>
inline int tRound(T r)
{
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

/// <summary>
/// 创建彩色图
/// </summary>
/// <param name="mGrayImage"></param>
/// <returns></returns>
cv::Mat generateColorImage(cv::Mat mGrayImage);

/// <summary>
/// 四象限轮廓点绘图
/// </summary>
/// <typeparam name="TypeVecFourPoint"></typeparam>
/// <param name="mGrayImage"></param>
/// <param name="vec"></param>
/// <returns></returns>
template <class TypeVecFourPoint>
cv::Mat fourQuadrantMap(cv::Mat mGrayImage, TypeVecFourPoint &vec)
{
  cv::Mat mShowMap = generateColorImage(mGrayImage);
  for (int i = 0; i < vec.size(); ++i)
  {
    switch (i)
    {
    case 0:
      for (int j = 0; j < vec[i].size(); ++j)
      {
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[0] = 0;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[1] = 0;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[2] = 255;
      }
      break;
    case 1:
      for (int j = 0; j < vec[i].size(); ++j)
      {
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[0] = 255;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[1] = 0;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[2] = 0;
      }
      break;
    case 2:
      for (int j = 0; j < vec[i].size(); ++j)
      {
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[0] = 0;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[1] = 255;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[2] = 0;
      }
      break;
    case 3:
      for (int j = 0; j < vec[i].size(); ++j)
      {
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[0] = 0;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[1] = 255;
        mShowMap.at<cv::Vec3b>(vec[i][j].y, vec[i][j].x)[2] = 255;
      }
      break;
    }
  }
  return mShowMap;
}

/// <summary>
/// 轮廓点画彩色图
/// </summary>
/// <typeparam name="TypePoint"></typeparam>
/// <param name="mGrayImage"></param>
/// <param name="pContoursPoint"></param>
/// <returns></returns>
template <class TypePoint>
cv::Mat drawColorImage(cv::Mat mGrayImage, TypePoint pContoursPoint)
{
  cv::Mat mColorImage = generateColorImage(mGrayImage);
  for (int n = 0; n < pContoursPoint.size(); ++n)
  {
    mColorImage.at<cv::Vec3b>(pContoursPoint[n].dY, pContoursPoint[n].dX)[0] = 0;
    mColorImage.at<cv::Vec3b>(pContoursPoint[n].dY, pContoursPoint[n].dX)[1] = 0;
    mColorImage.at<cv::Vec3b>(pContoursPoint[n].dY, pContoursPoint[n].dX)[2] = 255;
  }
  return mColorImage;
}

/// <summary>
/// 获取最小像素值的坐标点
/// </summary>
/// <typeparam name="TypePointone"></typeparam>
/// <typeparam name="TypePointtwo"></typeparam>
/// <typeparam name="TypePointthree"></typeparam>
/// <typeparam name="TypePointfour"></typeparam>
/// <typeparam name="TypePointfive"></typeparam>
/// <param name="mSrcImage"></param>
/// <param name="one"></param>
/// <param name="two"></param>
/// <param name="three"></param>
/// <param name="four"></param>
/// <param name="five"></param>
/// <returns></returns>
template <class TypePointone, class TypePointtwo, class TypePointthree,
          class TypePointfour, class TypePointfive>
inline cv::Point getMinScale(const cv::Mat mSrcImage, TypePointone one,
                             TypePointtwo two, TypePointthree three,
                             TypePointfour four, TypePointfive five)
{
  cv::Point pp;
  std::vector<std::pair<int, cv::Point>> vec;
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(one.y, one.x), one));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(two.y, two.x), two));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(three.y, three.x), three));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(four.y, four.x), four));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(five.y, five.x), five));
  sort(vec.begin(), vec.end(), sortList);
  return vec[0].second;
}

/// <summary>
/// 获取最大像素值的坐标点
/// </summary>
/// <typeparam name="TypePointone"></typeparam>
/// <typeparam name="TypePointtwo"></typeparam>
/// <typeparam name="TypePointthree"></typeparam>
/// <typeparam name="TypePointfour"></typeparam>
/// <typeparam name="TypePointfive"></typeparam>
/// <param name="mSrcImage"></param>
/// <param name="one"></param>
/// <param name="two"></param>
/// <param name="three"></param>
/// <param name="four"></param>
/// <param name="five"></param>
/// <returns></returns>
template <class TypePointone, class TypePointtwo, class TypePointthree,
          class TypePointfour, class TypePointfive>
inline cv::Point getMaxScale(const cv::Mat mSrcImage, TypePointone one,
                             TypePointtwo two, TypePointthree three,
                             TypePointfour four, TypePointfive five)
{
  cv::Point pp;
  std::vector<std::pair<int, cv::Point>> vec;
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(one.y, one.x), one));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(two.y, two.x), two));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(three.y, three.x), three));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(four.y, four.x), four));
  vec.push_back(std::make_pair(mSrcImage.at<uchar>(five.y, five.x), five));
  sort(vec.begin(), vec.end(), sortList);
  return vec[4].second;
}

/// <summary>
/// 获取圆轮廓容器方差
/// </summary>
/// <typeparam name="inTypePoint"></typeparam>
/// <param name="T"></param>
/// <returns></returns>
template <class inTypePoint>
std::pair<double, double> getVariance(inTypePoint T)
{
  cv::Point3f tempCircle = leastSquareFittingCircle(T);
  double dSigma = 0;
  double dDis = 0;
  for (int i = 0; i < T.size(); ++i)
  {
    dDis += sqrt(pow((double)T[i].dX - tempCircle.x, 2) +
                 pow((double)T[i].dY - tempCircle.y, 2));
  }
  dDis = dDis / T.size();
  for (int j = 0; j < T.size(); ++j)
  {
    dSigma += pow(sqrt(pow((double)T[j].dX - tempCircle.x, 2) +
                       pow((double)T[j].dY - tempCircle.y, 2)) -
                      dDis,
                  2);
  }
  dSigma = sqrt(dSigma / T.size());
  std::pair<double, double> pairDouble(dDis, dSigma);
  return pairDouble;
}

template <class inTypePoint>
std::pair<double, double> getVarianceCVPoint(inTypePoint T)
{
  cv::Point3f tempCircle = leastSquareFittingCircle(T);
  double dSigma = 0;
  double dDis = 0;
  for (int i = 0; i < T.size(); ++i)
  {
    dDis += sqrt(pow((double)T[i].x - tempCircle.x, 2) +
                 pow((double)T[i].y - tempCircle.y, 2));
  }
  dDis = dDis / T.size();
  for (int j = 0; j < T.size(); ++j)
  {
    dSigma += pow(sqrt(pow((double)T[j].x - tempCircle.x, 2) +
                       pow((double)T[j].x - tempCircle.y, 2)) -
                      dDis,
                  2);
  }
  dSigma = sqrt(dSigma / T.size());
  std::pair<double, double> pairDouble(dDis, dSigma);
  return pairDouble;
}

/// <summary>
/// 最大色差获取轮廓校验--删选后四象限均存在复合要求的点
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="mSrcImage"></param>
/// <param name="vec"></param>
/// <returns></returns>
template <class T>
bool isRightPick(const cv::Mat mSrcImage, T vec)
{
  bool isOne = false, isTwo = false, isThree = false, isFour = false;
  cv::Point pp(tRound(mSrcImage.cols / 2), tRound(mSrcImage.rows / 2));
  for (int i = 0; i < vec.size(); ++i)
  {
    int nQuadrant = isQuadrant(cv::Point(vec[i].dX, vec[i].dY), pp);
    switch (nQuadrant)
    {
    case 1:
      isOne = true;
      break;
    case 2:
      isTwo = true;
      break;
    case 3:
      isThree = true;
      break;
    case 4:
      isFour = true;
      break;
    default:
      break;
    }
  }
  if ((isOne && isTwo && isThree && isFour) || (isOne && isThree && isFour) ||
      (isOne && isTwo && isFour) || (isOne && isTwo && isThree) ||
      (isTwo && isThree && isFour))
    return true;
  else
    return false;
}

template <class T>
T getMaxParam(T *Para, int nNum)
{
  T out = 0;
  if (nNum < 0)
    return out;
  for (int i = 0; i < nNum; ++i)
  {
    if (out < Para[i])
      out = Para[i];
  }
  return out;
}

/// <summary>
/// 获取文件夹中的图片个数
/// </summary>
/// <param name="strPath"></param>
/// <returns></returns>
int getFileNumber(std::string strPath);

/// <summary>
/// 清空图片保存文件
/// </summary>
/// <param name="strPath"></param>
/// <param name="nFileNumThre"></param>
void clearFile(std::string strPath, int nFileNumThre);

/// <summary>
/// 删除指定文件下的所有文件
/// </summary>
/// <param name="dir"></param>
void deleteFiles(std::string dir);

/// <summary>
/// 绘制粗定位区域
/// </summary>
/// <typeparam name="TypePoint"></typeparam>
/// <typeparam name="TypeDistance"></typeparam>
/// <param name="mGrayImage"></param>
/// <param name="pContoursPoint"></param>
/// <param name="redius"></param>
/// <returns></returns>
template <class TypePoint, class TypeDistance>
cv::Mat drawRoughLocation(cv::Mat mGrayImage, TypePoint pContoursPoint,
                          TypeDistance redius)
{
  cv::Mat mColorImage = generateColorImage(mGrayImage);
  for (int i = 0; i < pContoursPoint.size(); ++i)
  {
    for (int j = 0; j < pContoursPoint[i].size(); ++j)
    {
      circle(
          mColorImage,
          Point(pContoursPoint[i][j].second.x, pContoursPoint[i][j].second.y),
          redius / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
  }
  return mColorImage;
}
//图像转点集
template <class vecOutPoint>
void MattoPoint(const cv::Mat &mSrcImage, bool isContoursMap, double dDimater,
                double dThreshold, vecOutPoint &T)
{
  double dDis = 0;
  for (int i = 0; i < mSrcImage.rows; i++)
  {
    for (int j = 0; j < mSrcImage.cols; j++)
    {
      dDis = sqrt(pow(double(i - mSrcImage.rows / 2), 2) +
                  pow(double(j - mSrcImage.cols / 2), 2));
      if (isContoursMap)
      {
        if (mSrcImage.at<uchar>(i, j) >= 220 &&
            dDis >= dDimater / 2 - dThreshold / 2 &&
            dDis <= dDimater / 2 + dThreshold / 2)
          T.push_back(cv::Point(j, i));
      }
      else
      {
        if (dDis >= dDimater / 2 - dThreshold / 2 &&
            dDis <= dDimater / 2 + dThreshold / 2)
          T.push_back(cv::Point(j, i));
      }
    }
  }
}
////迭代拟合直线检测
// template<class inTypePoint>
// void iteaLineFit(inTypePoint& T)
//{
//	double dSigma = getLineVariance(T).second;
//	double dMeanDis = getLineVariance(T).first;
//	double dA = 0, dB = 0, dC = 0;
//	double dDis = 0;
//	if (lineFit(T, dA, dB, dC))
//	{
//		while (dSigma > 3)
//		{
//			if (T.size() < 2)
//				break;
//			if (lineFit(T, dA, dB, dC))
//			{
//				for (int i = 0; i < T.size(); ++i)
//				{
//					dDis = sqrt(pow(getPointToLineDistance(T[i], dA,
//dB, dC) - dMeanDis, 2)); 					if (dDis > dSigma)
//					{
//						T.erase(T.begin() + i);
//						i--;
//					}
//				}
//				dSigma = getLineVariance(T).second;
//				dMeanDis = getLineVariance(T).first;
//			}
//		}
//	}
//}

/// <summary>
/// 获取最小值
/// </summary>
/// <typeparam name="T"></typeparam>
/// <typeparam name="N"></typeparam>
/// <param name="intput">输入数组</param>
/// <param name="num">数组维度</param>
/// <returns></returns>
template <class T, class N>
int getMinValue(T *intput, N num)
{
  int nMinValue = 10000;
  for (int i = 0; i < num; ++i)
  {
    if (intput[i] < nMinValue)
    {
      nMinValue = intput[i];
    }
  }
  return nMinValue;
}

/************************************************************************/
//求一个点在四象限中的角度（弧度），0 ~ 2*pi
/************************************************************************/
template <class T1, class T2>
int AngleofPoint361(T1 x, T1 y, T2 &angleArc)
{
  if (x >= 0) // 1，4象限
  {
    if (y >= 0) // 1  x>0 y>0
    {
      if (x != 0)
      {
        angleArc = (T2)atan(y / x);
      }
      else // x==0
      {
        angleArc = CV_PI / 2;
      }
    }
    else // 4  x>0  y<0
    {
      if (x != 0)
      {
        angleArc = 2 * CV_PI + (T2)atan(y / x);
      }
      else // x==0
      {
        angleArc = 3 * CV_PI / 2;
      }
    }
  }
  else // 2，3象限
  {
    if (y >= 0) // 2
    {
      if (x != 0) // x<0 y>0
      {
        angleArc = CV_PI + (T2)atan(y / x);
      }
    }
    else // 3  x<0  y<0
    {
      if (x != 0)
      {
        angleArc = CV_PI + (T2)atan(y / x);
      }
    }
  }
  return 1;
}

/// <summary>
/// 容器合并
/// </summary>
/// <typeparam name="vecInPoint">输入容器</typeparam>
/// <typeparam name="vecOutPoint">输出容器</typeparam>
/// <param name="vecInput"></param>
/// <param name="vecOutput"></param>
/// <param name="nThreshold"></param>
template <class vecInPoint, class vecOutPoint>
void mergeVector(vecInPoint vecInput, vecOutPoint &vecOutput, int nThreshold)
{
  for (int i = 0; i < vecInput.size(); ++i)
  {
    if (vecInput[i].size() > nThreshold)
      vecOutput.insert(vecOutput.end(), vecInput[i].begin(), vecInput[i].end());
  }
}

/// <summary>
/// 计算点到直线距离
/// </summary>
/// <typeparam name="inPoint"></typeparam>
/// <param name="pPointP"></param>
/// <param name="a"></param>
/// <param name="b"></param>
/// <param name="c"></param>
/// <returns></returns>
template <class inPoint>
double getPointToLineDistance(inPoint pPointP, double a, double b, double c)
{
  double distance = 0;
  distance = ((double)abs(a * pPointP.x + b * pPointP.y + c)) /
             ((float)sqrtf(a * a + b * b));
  return distance;
}

/// <summary>
/// 自定义直线参数获取两点坐标
/// </summary>
/// <param name="mSrcImage">输入图像</param>
/// <param name="a"></param>
/// <param name="b"></param>
/// <param name="c"></param>
/// <returns></returns>
std::vector<cv::Point2d> transLinePara(cv::Mat mSrcImage, double a, double b,
                                       double c);

/// <summary>
/// 根据图像中心点筛选查找到的轮廓
/// </summary>
/// <typeparam name="vecInPoint"></typeparam>
/// <typeparam name="vecOutPoint"></typeparam>
/// <param name="opencvContours"></param>
/// <param name="pImageCenter"></param>
/// <param name="nMaxRadius"></param>
/// <param name="outT"></param>
template <class vecInPoint, class vecOutPoint>
void mergeContours(vecInPoint opencvContours, cv::Point pImageCenter,
                   int nMaxRadius, vecOutPoint &outT)
{
  cv::Point2f pContour(0, 0);
  float fRedius = 0;
  for (int i = 0; i < opencvContours.size(); ++i)
  {
    minEnclosingCircle(opencvContours[i], pContour, fRedius);
    double dDisMax = sqrt(pow(pContour.x - pImageCenter.x, 2) +
                          pow(pContour.y - pImageCenter.y, 2));

    if (dDisMax < nMaxRadius)
      outT.insert(outT.end(), opencvContours[i].begin(),
                  opencvContours[i].end());
  }
}

//检测圆缺损度校验
template <class TypePoint>
int checkDefectRoundness(TypePoint vec, cv::Point3d center, float &roundness)
{
  if (vec.size() < 3)
    return 0;
  // 计算圆形度
  float fAngleIntervalT = 2 * asin(0.5f / center.z) * 3;
  fAngleIntervalT *= (180.0 / CV_PI); // 转成度
  fAngleIntervalT = 2 > fAngleIntervalT ? 2 : fAngleIntervalT;
  roundness = 0;

  if (center.z <= 1e-6f || vec.size() <= 0)
    return false;

  std::vector<float> vecAngle;

  for (int i = 0; i < vec.size(); i++)
  {
    float fDist = 0;
    float fEx = vec[i].x - center.x;
    float fEy = vec[i].y - center.y;

    fDist = abs(sqrt(fEx * fEx + fEy * fEy) - center.z);

    // 如果在圆带内
    if (fDist < 20)
    {
      // 角度
      float fAngleArc = 0;
      AngleofPoint361(vec[i].x - center.x, vec[i].y - center.y, fAngleArc);

      vecAngle.push_back(fAngleArc);
    }
  }

  // 排序
  std::sort(vecAngle.begin(), vecAngle.end());

  int ptAngleNum = vecAngle.size();

  float fAngleIntervalArcT = fAngleIntervalT * CV_PI / 180; // 转成弧度

  //
  float fAccInterval = 0;
  for (int i = 1; i < ptAngleNum; i++)
  {
    float angleInterval = abs(vecAngle[i] - vecAngle[i - 1]);

    if (angleInterval > fAngleIntervalArcT)
    {
      fAccInterval += angleInterval;
    }
  }
  //
  fAccInterval += abs(abs(vecAngle[ptAngleNum - 1] - vecAngle[0]) - CV_PI * 2);

  roundness = 1 - fAccInterval / (2 * CV_PI);

  return true;
}

//根据直线参数获取两点坐标
std::vector<cv::Point2d> transCVLinePara(cv::Mat mSrcImage,
                                         cv::Vec4f line_para);

//获取容器参数点到直线距离方差
template <class TypePointIn>
double getSigmaDis(TypePointIn vec)
{
  if (vec.size() == 0)
    return DBL_MAX;
  double dSigma = 0;
  double dA = 0, dB = 0, dC = 0;
  TypePointIn intputVec = vec;
  lineFit(intputVec, dA, dB, dC);
  double dMean = 0;
  for (int i = 0; i < vec.size(); ++i)
  {
    dMean += getPointToLineDistance(vec[i], dA, dB, dC);
  }
  dMean /= vec.size();
  if (dMean == 0)
    return dSigma;
  for (int i = 0; i < vec.size(); ++i)
  {
    dSigma += pow(getPointToLineDistance(vec[i], dA, dB, dC) - dMean, 2);
  }
  dSigma /= vec.size();
  return dSigma;
}

//四点确定直线交点
cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2,
                       const cv::Point2d line3, const cv::Point2d line4);

template <class T1, class T2>
int AngleofPoint360(T1 x, T1 y, T2 &angleArc)
{
  if (x >= 0) // 1，4象限
  {
    if (y >= 0) // 1  x>0 y>0
    {
      if (x != 0)
      {
        angleArc = (T2)atan(y / x);
      }
      else //x==0
      {
        angleArc = PI_32F / 2;
      }
    }
    else // 4  x>0  y<0
    {
      if (x != 0)
      {
        angleArc = 2 * PI_32F + (T2)atan(y / x);
      }
      else //x==0
      {
        angleArc = 3 * PI_32F / 2;
      }
    }
  }
  else // 2，3象限
  {
    if (y >= 0) // 2
    {
      if (x != 0) // x<0 y>0
      {
        angleArc = PI_32F + (T2)atan(y / x);
      }
    }
    else //3  x<0  y<0
    {
      if (x != 0)
      {
        angleArc = PI_32F + (T2)atan(y / x);
      }
    }
  }
  return 1;
}

void DistanceOfPointToCircle(float fPtX, float fPtY,
                             float fCenterX, float fCenterY, float fRadius,
                             float &fDist);