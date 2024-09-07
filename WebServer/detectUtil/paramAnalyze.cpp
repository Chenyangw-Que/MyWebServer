#include "paramAnalyze.h"
#include "Common.h"
#include "DetCircleCenter.h"
#include "FitCircleT.h"
#include "FitLine.h"
#include "LogHelp.h"
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;

//#define WRITE_RESULT
/// <summary>
/// 解析圆靶标mask
/// </summary>
/// <param
/// name="mSrcImage">分割输出二值mask<靶标示例ROI,(检测框左上角横坐标，左上角纵坐标，类别id)></param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
bool GetCircleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                    DetectParameterV3 &MarkParam) {
  std::string strTxtPath = "circle_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始圆参数解析");
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  // 如果没有分割到
  if (!contours.size()) {
    writeLog(strTxtPath, "分割失败，没有分割到轮廓");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  vector<cv::Point> contour; // 用于存储选定的轮廓
  // 单实例靶标轮廓点确定, 对于单实例靶标选取轮廓面积最大的轮廓
  double Area = 0;
  for (auto con : contours) {
    if (cv::contourArea(con) >= Area) {
      contour = con;
      Area = cv::contourArea(con);
    }
  }
  // 如果中心对称性指标不够
  if (!JudgeCentralSymmetry(contour, 0.92, BinaryMask.size())) {
    writeLog(strTxtPath, "中心对称性低于阈值分割质量不够");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  // 如果面积过小(分割图中只有离散的小点的情况)，考虑分割失败
  if (Area < (0.1 * BinaryMask.size[0] * BinaryMask.size[1])) {
    writeLog(strTxtPath, "分割结果面积过小，解析失败");
    return CFMEE_PARAMANALYZE_ERROR;
  }

  // 圆拟合
  CdetCircle *fiter = new CdetCircle();
  vector<ContoursPoint> circlePoints;
  for (auto point : contour)
    circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
  cv::Point3d circleParam;
  if (fiter->iterFitting(circlePoints, circleParam, false)) {
    MarkParam.Diameter[0] = circleParam.z * 2;
    MarkParam.IsWhite = JudgeIsWhite(
        ImageRoi, vector<vector<cv::Point>>{contour}, vector<cv::Point>{});
    MarkParam.Threshold = 15;
#ifdef _DEBUG
    cv::Mat result;
    ImageRoi.copyTo(result);
    cv::circle(result, cv::Point(circleParam.x, circleParam.y),
               MarkParam.Diameter[0] / 2, 0, 2, 8, 0);
    if (MarkParam.IsWhite) {
      cv::putText(result, "white",
                  cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
      cv::putText(result, "circle",
                  cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    } else {
      cv::putText(result, "black",
                  cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
      cv::putText(result, "circle",
                  cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    }
    CvWaitShowImage(result, "圆参数解析结果");

#endif
    cout << "图像尺寸参考" << BinaryMask.size() << endl;
    cout << "圆直径" << MarkParam.Diameter[0] << endl;
    cout << "白靶标？" << MarkParam.IsWhite << endl;
    writeLog(strTxtPath, "参数解析成功");
    return CFMEE_CODE_OK;
  }
  writeLog(strTxtPath, "圆拟合失败，参数解析失败");
  return CFMEE_PARAMANALYZE_ERROR;
}

/// <summary>
/// 解析矩形靶标mask
/// </summary>
/// <param
/// name="mSrcImage">分割输出二值mask<靶标示例ROI,(检测框左上角横坐标，左上角纵坐标，类别id)></param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
bool GetRectangleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                       DetectParameterV3 &MarkParam) {
  std::string strTxtPath = "rectangle_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始矩形参数解析");
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  vector<cv::Point> contour; // 存储选定的轮廓
  // 如果没有分割到
  if (!contours.size()) {
    return CFMEE_PARAMANALYZE_ERROR;
  }
  // 单实例靶标轮廓点确定, 对于单实例靶标选取轮廓面积最大的轮廓
  double Area = 0;
  for (auto con : contours) {
    if (cv::contourArea(con) >= Area) {
      contour = con;
      Area = cv::contourArea(con);
    }
  }
  // 如果中心对称性指标不够，考虑分割失败
  if (!JudgeCentralSymmetry(contour, 0.80, BinaryMask.size())) {
    writeLog(strTxtPath, "中心对称性低于阈值分割质量不够");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  // 如果面积过小(分割图中只有离散的小点)，考虑分割失败
  if ((Area / BinaryMask.size[0] * BinaryMask.size[1]) < 0.1) {
    writeLog(strTxtPath, "分割失败");
    return CFMEE_PARAMANALYZE_ERROR;
  }

  cv::RotatedRect RectangleRect = cv::minAreaRect(
      contour); // 由于Rectangle的外部形状并不是旋转无关的，所以求最小外接旋转矩
  MarkParam.Width[0] = RectangleRect.size.width;
  MarkParam.Height[0] = RectangleRect.size.height;
  MarkParam.IsWhite = JudgeIsWhite(ImageRoi, vector<vector<cv::Point>>{contour},
                                   vector<cv::Point>{});
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::Point2f points[4];
  RectangleRect.points(points);
  line(result, points[0], points[1], 0, 5, 8);
  line(result, points[1], points[2], 0, 5, 8);
  line(result, points[2], points[3], 0, 5, 8);
  line(result, points[3], points[0], 0, 5, 8);
  string rowcoltext =
      to_string(MarkParam.Cols) + "-" + to_string(MarkParam.Rows);
  if (MarkParam.IsWhite) {
    circle(result, points[0], 5, 255, -1);
    circle(result, points[1], 5, 255, -1);
    circle(result, points[2], 5, 255, -1);
    circle(result, points[3], 5, 255, -1);
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 2);
  } else {
    circle(result, points[0], 5, 0, -1);
    circle(result, points[1], 5, 0, -1);
    circle(result, points[2], 5, 0, -1);
    circle(result, points[3], 5, 0, -1);
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 2);
  }
  CvWaitShowImage(result, "矩形参数解析结果");

#endif

  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "矩形长宽" << MarkParam.Height[0] << '\t' << MarkParam.Width[0]
       << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  return CFMEE_CODE_OK;
}

/// <summary>
/// 解析梅花孔靶标mask
/// </summary>
/// <param
/// name="mSrcImage">分割输出二值mask<靶标示例ROI,(检测框左上角横坐标，左上角纵坐标，类别id)></param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
bool GetPlumParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                  DetectParameterV3 &MarkParam,
                  std::vector<pair<cv::Mat, cv::Point>> &refinedRoi) {
  std::string strTxtPath = "plum_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始梅花孔参数解析");
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  // 分割结果验证与干扰点过滤
  // 滤除干扰点：滤除不是靶标的干扰点、滤除没有分割好的小圆干扰点
  vector<vector<cv::Point>> tempContour;
  for (auto con : contours) {
    // 把符合中心对称性阈值的点先拿进来
    if (JudgeCentralSymmetry(con, 0.75, BinaryMask.size()))
      tempContour.push_back(con);
  }
  // 可以通过面积值的方式卡掉一些点,将面积值作为评判依据，将轮廓进行分类：比如轮廓面积:1,1.1,0.9,5,10就分类为{1,
  // 1.1, 0.9} {5} {10}
  vector<pair<double, vector<cv::Point>>> filtContour;
  vector<pair<vector<vector<cv::Point>>, cv::Point2f>> clusteredContour;
  for (auto con : tempContour) {
    filtContour.push_back(make_pair(cv::contourArea(con), con));
  }
  contourCluster(filtContour, clusteredContour, 10, 0.1);
  // 找到实例数最多的聚类，把其中的轮廓拿出来
  contours.clear();
  int index = -1;
  int num = 1;
  for (int i = 0; i < clusteredContour.size(); i++) {
    if (clusteredContour[i].first.size() > num) {
      index = i;
      num = clusteredContour[i].first.size();
    }
  }

  if (index == -1) {
    writeLog(strTxtPath, "面积聚类混乱，分割失败");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  contours = clusteredContour[index].first;

  // 经过中心对称性+面积分类过滤，剩余的轮廓不论是靶标点还是干扰点都有相近的形状，可能在位置上有差异
  // 这里对所有剩余轮廓进行圆拟合得到小圆直径，接着直接对现有轮廓中心点进行圆拟合得到小圆所处圆周，使用大圆周直径+小圆直径得到外侧直径

  CFitCircle *fiter = new CFitCircle();
  float total_radius = 0;
  vector<cv::Point2f> littleCircleCenter;
  float fCenterX = 0, fCenterY = 0, fRadius = 0;
  int expandRange = 5;
  for (auto contour : contours) {
    cv::Rect rect = boundingRect(contour);
    fCenterX = rect.tl().x + rect.width / 2;
    fCenterY = rect.tl().y + rect.height / 2;
    fRadius = (rect.width + rect.height) / 4;
    littleCircleCenter.push_back(cv::Point2f(fCenterX, fCenterY));

    double tlx = rect.tl().x - expandRange < 0 ? 0 : rect.tl().x - expandRange;
    double tly = rect.tl().y - expandRange < 0 ? 0 : rect.tl().y - expandRange;
    double brx = rect.br().x + expandRange > ImageRoi.cols - 1
                     ? ImageRoi.cols - 1
                     : rect.br().x + expandRange;
    double bry = rect.br().y + expandRange > ImageRoi.rows - 1
                     ? ImageRoi.rows - 1
                     : rect.br().y + expandRange;
    refinedRoi.push_back(
        make_pair(ImageRoi(cv::Rect(cv::Point(tlx, tly), cv::Point(brx, bry))),
                  cv::Point(tlx, tly)));
    total_radius += fRadius;
  }
  // 拟合plum圆环
  float fCenterX_Outer = 0, fCenterY_Outer = 0, fRadius_Outer = 0;
  vector<cv::Point2f> buf(5);
  if (!fiter->LeastSquaresFitting2(5, littleCircleCenter.data(), buf.data(),
                                   fCenterX_Outer, fCenterY_Outer,
                                   fRadius_Outer)) {
    writeLog(strTxtPath, "外圈大圆周拟合失败");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  MarkParam.nLittleCount = contours.size();
  MarkParam.Diameter[0] =
      2 * fRadius_Outer; //使用外层小圆圆心拟合圆直径+小圆直径
  MarkParam.Diameter[1] = 2 * total_radius / contours.size(); // 小圆直径平均
  MarkParam.IsWhite = JudgeIsWhite(ImageRoi, contours, vector<cv::Point>{});
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, cv::Point(fCenterX_Outer, fCenterY_Outer),
             MarkParam.Diameter[0] / 2, 0, 2, 8, 0);
  cv::circle(result, cv::Point(fCenterX, fCenterY), MarkParam.Diameter[1] / 2,
             0, 2, 8, 0);
  string rowcoltext = "circleNum" + to_string(MarkParam.nLittleCount);
  if (MarkParam.IsWhite) {
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2 - 40, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  } else {
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2 - 40, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  }
  CvWaitShowImage(result, "梅花孔参数解析结果");
#endif
  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "外直径" << MarkParam.Diameter[0] << endl;
  cout << "小圆直径" << MarkParam.Diameter[1] << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "梅花孔参数解析成功");
  return CFMEE_CODE_OK;
}

/// <summary>
/// 解析孔矩阵靶标mask
/// </summary>
/// <param
/// name="mSrcImage">分割输出二值mask<靶标示例ROI,(检测框左上角横坐标，左上角纵坐标，类别id)></param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <param
/// name="refinedRoi">可以带出来一组roi，比如孔矩阵中每个小圆的roi</param>
bool GetMatrixHoleParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                        DetectParameterV3 &MarkParam,
                        std::vector<pair<cv::Mat, cv::Point>> &refinedRoi) {
  std::string strTxtPath = "matrixHole_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始矩形参数解析");
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  CFitCircle *fiter = new CFitCircle();

  float total_radius = 0;
  vector<cv::Point> totalContours;
  // 使用中心对称性卡掉一些点
  vector<vector<cv::Point>> tempContour;
  for (auto con : contours) {
    // 把符合中心对称性阈值的点先拿进来
    if (JudgeCentralSymmetry(con, 0.75, BinaryMask.size()))
      tempContour.push_back(con);
  }
  // 可以通过面积值的方式卡掉一些点,将面积值作为评判依据，将轮廓进行分类：比如轮廓面积:1,1.1,0.9,5,10就分类为{1,
  // 1.1, 0.9} {5} {10}
  vector<pair<double, vector<cv::Point>>> filtContour;
  vector<pair<vector<vector<cv::Point>>, cv::Point2f>> clusteredContour;
  for (auto con : tempContour) {
    filtContour.push_back(make_pair(cv::contourArea(con), con));
  }
  contourCluster(filtContour, clusteredContour, 10, 0.1);
  // 找到实例数最多的类，把其中的轮廓拿出来
  contours.clear();
  int index = -1;
  int num = 1;
  for (int i = 0; i < clusteredContour.size(); i++) {
    if (clusteredContour[i].first.size() > num) {
      index = i;
      num = clusteredContour[i].first.size();
    }
  }
  if (index == -1) {
    writeLog(strTxtPath, "面积聚类混乱，分割失败");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  contours = clusteredContour[index].first;

  // 对于过滤后的这一组轮廓，遍历所有轮廓，进行圆拟合、最小外接矩形提取
  vector<cv::Point2f> centerVec; //存储圆心坐标用于之后行间距判定、行列判定
  float fCenterX = 0, fCenterY = 0, fRadius = 0;
  int expandRange = 5;
  for (auto contour : contours) {
    totalContours.insert(totalContours.end(), contour.begin(), contour.end());
    cv::Rect rect = boundingRect(contour);
    // 将相关roi加入到refined rect中
    // 由于直接使用外接矩作为roi可能导致圆拟合失败，故这里将roi外扩几个像素
    fCenterX = rect.tl().x + rect.width / 2;
    fCenterY = rect.tl().y + rect.height / 2;
    fRadius = (rect.width + rect.height) / 4;
    centerVec.push_back(cv::Point2f(fCenterX, fCenterY));

    double tlx = rect.tl().x - expandRange < 0 ? 0 : rect.tl().x - expandRange;
    double tly = rect.tl().y - expandRange < 0 ? 0 : rect.tl().y - expandRange;
    double brx = rect.br().x + expandRange > ImageRoi.cols - 1
                     ? ImageRoi.cols - 1
                     : rect.br().x + expandRange;
    double bry = rect.br().y + expandRange > ImageRoi.rows - 1
                     ? ImageRoi.rows - 1
                     : rect.br().y + expandRange;
    refinedRoi.push_back(
        make_pair(ImageRoi(cv::Rect(cv::Point(tlx, tly), cv::Point(brx, bry))),
                  cv::Point(tlx, tly)));
    total_radius += fRadius;
  }
  // cout << "centerVec:" << centerVec.size() << endl;
  if (centerVec.size() < 4) {
    writeLog(strTxtPath, "圆拟合失效过多");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  // 防止旋转角度不大，圆拟合不佳导致的间距计算错误
  // 将现有点集旋转30度
  for (int i = 0; i < centerVec.size(); i++) {
    double srcx = centerVec[i].x;
    double srcy = centerVec[i].y;
    centerVec[i].x = 0.866 * srcx + 0.5 * srcy;
    centerVec[i].y = 0.866 * srcy - 0.5 * srcx;
  }

  // 找到孔矩阵中行列相邻点，辅助计算行间距、行列数
  // 拿到一组圆心坐标点,按照横坐标从小到大排序，前两的点在矩阵中必然是相邻点
  sort(centerVec.begin(), centerVec.end(),
       [](cv::Point2f a, cv::Point2f b) { return a.x < b.x; });
  //计算两点间距
  double dist1 = sqrt(
      (centerVec[0].x - centerVec[1].x) * (centerVec[0].x - centerVec[1].x) +
      (centerVec[0].y - centerVec[1].y) * (centerVec[0].y - centerVec[1].y));
  // 以纵坐标为排序指标再来一次
  sort(centerVec.begin(), centerVec.end(),
       [](cv::Point2f a, cv::Point2f b) { return a.y > b.y; });
  double dist2 = sqrt(
      (centerVec[0].x - centerVec[1].x) * (centerVec[0].x - centerVec[1].x) +
      (centerVec[0].y - centerVec[1].y) * (centerVec[0].y - centerVec[1].y));
  cv::RotatedRect MatrixHoleRect = cv::minAreaRect(
      totalContours); // 由于matrixHole的外部形状并不是旋转无关的，所以求最小外接旋转矩
  cv::Size2f recSize =
      MatrixHoleRect.size.width > MatrixHoleRect.size.height
          ? cv::Size2f(MatrixHoleRect.size.width, MatrixHoleRect.size.height)
          : cv::Size2f(MatrixHoleRect.size.height, MatrixHoleRect.size.width);
  cv::Size2f rectDist =
      dist1 > dist2 ? cv::Size2f(dist1, dist2) : cv::Size2f(dist2, dist1);

  // 基于最小外接矩的行列数、行列间距计算
  MarkParam.Diameter[0] = 2 * total_radius / contours.size();
  MarkParam.Colspan = rectDist.width - MarkParam.Diameter[0];
  MarkParam.Rowspan = rectDist.height - MarkParam.Diameter[0];

  MarkParam.Threshold = 15;
  MarkParam.Cols = round((recSize.width + MarkParam.Colspan) /
                         (MarkParam.Colspan + MarkParam.Diameter[0]));
  MarkParam.Rows = round((recSize.height + MarkParam.Rowspan) /
                         (MarkParam.Rowspan + MarkParam.Diameter[0]));
  // MarkParam.Colspan = (MarkParam.Cols * MarkParam.Diameter[0] +
  // (MarkParam.Cols - 1) * MarkParam.Colspan) / MarkParam.Cols;
  // MarkParam.Rowspan = (MarkParam.Rows * MarkParam.Diameter[0] +
  // (MarkParam.Rows - 1) * MarkParam.Rowspan) / MarkParam.Rows;
  MarkParam.Colspan = rectDist.width;
  MarkParam.Rowspan = rectDist.height;
  MarkParam.IsWhite = JudgeIsWhite(ImageRoi, contours, vector<cv::Point>{});
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, cv::Point(fCenterX, fCenterY), MarkParam.Diameter[0] / 2,
             0, 2, 8, 0);
  string rowcoltext =
      to_string(MarkParam.Cols) + "-" + to_string(MarkParam.Rows);
  string littleCircle = "circleNum" + to_string(centerVec.size());
  if (MarkParam.IsWhite) {
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, littleCircle,
                cv::Point(ImageRoi.rows / 2 - 80, 80 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  } else {
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, littleCircle,
                cv::Point(ImageRoi.rows / 2 - 80, 80 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  }
  CvWaitShowImage(result, "孔矩阵参数解析结果");
#endif

  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "小圆直径" << MarkParam.Diameter[0] << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "孔矩阵解析成功");
  return CFMEE_CODE_OK;
}

double calDistance(cv::Point p1, cv::Point p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

/// <summary>
/// 解析复合梅花孔靶标mask
/// </summary>
/// <param
/// name="mSrcImage">分割输出二值mask<靶标示例ROI,(检测框左上角横坐标，左上角纵坐标，类别id)></param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
bool GetComplexPlumParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                         DetectParameterV3 &MarkParam,
                         std::vector<pair<cv::Mat, cv::Point>> &refinedRoi) {
  std::string strTxtPath = "complexPlum_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始复合梅花孔参数解析");
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // 分割结果验证与干扰点过滤
  // 滤除干扰点：滤除不是靶标的干扰点、滤除没有分割好的小圆干扰点
  cv::Point p(round(ImageRoi.size().width / 2),
              round(ImageRoi.size().height / 2));
  vector<cv::Point> centerContour;
  double dis = 1000000000000000000;
  int conIndex = -1;
  for (int i = 0; i < contours.size(); i++) {
    cv::Rect rec = cv::boundingRect(contours[i]);
    if (dis > calDistance(p, cv::Point(rec.x + round(rec.width / 2),
                                       rec.y + round(rec.height / 2)))) {
      dis = calDistance(p, cv::Point(rec.x + round(rec.width / 2),
                                     rec.y + round(rec.height / 2)));
      centerContour = contours[i];
      conIndex = i;
    }
  }
  if (conIndex != -1) {
    // 从原轮廓中移除指定轮廓
    contours.erase(contours.begin() + conIndex);
  } else {
    writeLog(strTxtPath, "轮廓为空");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  if (!contours.size()) {
    writeLog(strTxtPath, "实例不足");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  vector<vector<cv::Point>> tempContour;
  for (auto con : contours) {
    // 把符合中心对称性阈值的点先拿进来
    if (JudgeCentralSymmetry(con, 0.75, BinaryMask.size()))
      tempContour.push_back(con);
  }
  // 通过面积值区分出中间的大圆 并过滤外圈的小圆
  vector<pair<double, vector<cv::Point>>> filtContour;
  vector<pair<vector<vector<cv::Point>>, cv::Point2f>> clusteredContour;
  for (auto con : tempContour) {
    filtContour.push_back(make_pair(cv::contourArea(con), con));
  }
  contourCluster(filtContour, clusteredContour, 10, 0.1);

  // 找到实例数最多的聚类，把其中的轮廓拿出来(外层小圆)
  contours.clear();
  int index = -1;
  int num = 1;
  double area = 0;
  for (int i = 0; i < clusteredContour.size(); i++) {
    if (clusteredContour[i].first.size() > num) {
      index = i;
      num = clusteredContour[i].first.size();
    }
  }
  if (index == -1) {
    writeLog(strTxtPath, "面积混乱");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  contours = clusteredContour[index].first;
  CFitCircle *fiter = new CFitCircle();
  float total_radius = 0;
  vector<cv::Point> totalContours;
  vector<cv::Point2f> littleCircleCenter;
  float fCenterX = 0, fCenterY = 0, fRadius = 0;
  // 拟合外层小圆
  float fCenterX_Center = 0, fCenterY_Center = 0, fRadius_Center = 0;
  int expandRange = 5;
  int littleCircleNum = 0;
  for (auto contour : contours) {
    totalContours.insert(totalContours.end(), contour.begin(), contour.end());
    cv::Rect rect = boundingRect(contour);
    fCenterX = rect.tl().x + rect.width / 2;
    fCenterY = rect.tl().y + rect.height / 2;
    fRadius = (rect.width + rect.height) / 4;
    littleCircleCenter.push_back(cv::Point2f(fCenterX, fCenterY));

    double tlx = rect.tl().x - expandRange < 0 ? 0 : rect.tl().x - expandRange;
    double tly = rect.tl().y - expandRange < 0 ? 0 : rect.tl().y - expandRange;
    double brx = rect.br().x + expandRange > ImageRoi.cols - 1
                     ? ImageRoi.cols - 1
                     : rect.br().x + expandRange;
    double bry = rect.br().y + expandRange > ImageRoi.rows - 1
                     ? ImageRoi.rows - 1
                     : rect.br().y + expandRange;
    refinedRoi.push_back(
        make_pair(ImageRoi(cv::Rect(cv::Point(tlx, tly), cv::Point(brx, bry))),
                  cv::Point(tlx, tly)));
    total_radius += fRadius;
    littleCircleNum++;
  }

  CdetCircle *centerFiter = new CdetCircle();
  vector<ContoursPoint> circlePoints;
  for (auto point : centerContour)
    circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
  cv::Point3d circleParam;

  cv::Rect rect = boundingRect(centerContour);
  // 中心圆roi也放进去
  double tlx = rect.tl().x - expandRange < 0 ? 0 : rect.tl().x - expandRange;
  double tly = rect.tl().y - expandRange < 0 ? 0 : rect.tl().y - expandRange;
  double brx = rect.br().x + expandRange > ImageRoi.cols - 1
                   ? ImageRoi.cols - 1
                   : rect.br().x + expandRange;
  double bry = rect.br().y + expandRange > ImageRoi.rows - 1
                   ? ImageRoi.rows - 1
                   : rect.br().y + expandRange;
  refinedRoi.push_back(
      make_pair(ImageRoi(cv::Rect(cv::Point(tlx, tly), cv::Point(brx, bry))),
                cv::Point(tlx, tly)));
  if (cv::contourArea(centerContour) < 10 * cv::contourArea(contours[0])) {
    float fCenterX, fCenterY, fRadius;
    cv::Rect rect = boundingRect(centerContour);
    if (!fiter->HoughArc(
            centerContour.data(), round((rect.width + rect.height) / 8),
            (rect.width + rect.height) / 4, fCenterX, fCenterY, fRadius)) {
      writeLog(strTxtPath, "内层拟合失败");
      return CFMEE_PARAMANALYZE_ERROR;
    }
    circleParam.x = fCenterX;
    circleParam.y = fCenterY;
    circleParam.z = fRadius;
  } else {
    // 最小二乘圆拟合
    if (!centerFiter->iterFitting(circlePoints, circleParam, false)) {
      writeLog(strTxtPath, "内存拟合失败");
      return CFMEE_PARAMANALYZE_ERROR;
    }
  }

  MarkParam.nLittleCount = littleCircleNum;
  MarkParam.Diameter[0] = 2 * circleParam.z;                  //内圆直径
  MarkParam.Diameter[1] = 2 * total_radius / littleCircleNum; // 小圆直径平均
  double centerDist = 0;
  for (auto point : littleCircleCenter) {
    centerDist += sqrt((point.x - circleParam.x) * (point.x - circleParam.x) +
                       (point.y - circleParam.y) * (point.y - circleParam.y));
  }
  MarkParam.nCenterDistance = 2 * centerDist / littleCircleCenter.size();
  MarkParam.IsWhite = JudgeIsWhite(ImageRoi, contours, vector<cv::Point>{});
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, cv::Point(fCenterX, fCenterY), MarkParam.Diameter[1] / 2,
             0, 2, 8, 0);
  cv::circle(result, cv::Point(circleParam.x, circleParam.y),
             MarkParam.Diameter[0] / 2, 0, 2, 8, 0);
  cv::circle(result, cv::Point(circleParam.x, circleParam.y),
             MarkParam.nCenterDistance / 2, 0, 2, 8, 0);
  string rowcoltext = "circleNum" + to_string(MarkParam.nLittleCount);
  if (MarkParam.IsWhite) {
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2 - 40, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  } else {
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    cv::putText(result, rowcoltext,
                cv::Point(ImageRoi.rows / 2 - 40, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  }
  CvWaitShowImage(result, "复合梅花孔参数解析结果");

#endif
  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "内圆直径" << MarkParam.Diameter[0] << endl;
  cout << "小圆直径" << MarkParam.Diameter[1] << endl;
  cout << "中心距离" << MarkParam.nCenterDistance << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "复合梅花孔解析失败");
  return CFMEE_CODE_OK;
}

/// <summary>
/// 解析圆环靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetRingParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                  DetectParameterV3 &MarkParam) {
  std::string strTxtPath = "ring_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始圆环参数解析");
  // 进行轮廓提取，找到多级轮廓
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);
  if (!contours.size()) {
    cout << "分割失败，mask中无轮廓" << endl;
    return CFMEE_PARAMANALYZE_ERROR;
  }
  // 首先找到面积最大的父轮廓
  double area = 0;
  double ContourIndex = 0;
  // 遍历轮廓层次关系，看哪个外层轮廓的面积最大
  for (int i = 0; i < hierarchy.size(); i++) {
    if (hierarchy[i][3] == -1 && cv::contourArea(contours[i]) > area) {
      ContourIndex = i;
      area = cv::contourArea(contours[i]);
    }
  }
  if (hierarchy[ContourIndex][2] == -1 ||
      cv::contourArea(contours[ContourIndex]) /
              (BinaryMask.rows * BinaryMask.cols) <
          0.1) {
    // 若所选轮廓无内层结构或所选外层轮廓面积过小，分割失败
    cout << "seg failed" << endl;
    return CFMEE_PARAMANALYZE_ERROR;
  }
  vector<cv::Point> outerContour = contours[ContourIndex]; //所选最外层轮廓
  vector<cv::Point> innerContour =
      contours[hierarchy[ContourIndex][2]]; // 对应内层轮廓
  // CFitCircle* fiter = new CFitCircle();
  float fCenterX_Outer = 0, fCenterY_Outer = 0, fRadius_Outer = 0;
  float fCenterX_Inner = 0, fCenterY_Inner = 0, fRadius_Inner = 0;
  CdetCircle *centerFiter = new CdetCircle();
  // 检验中心对称性
  if (JudgeCentralSymmetry(outerContour, 0.90, BinaryMask.size()) &&
      JudgeCentralSymmetry(innerContour, 0.90, BinaryMask.size())) {
    vector<ContoursPoint> circlePoints;
    cv::Point3d circleParam;
    for (auto point : outerContour)
      circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
    // 外层圆拟合
    if (centerFiter->iterFitting(circlePoints, circleParam, false)) {
      fCenterX_Outer = circleParam.x;
      fCenterY_Outer = circleParam.y;
      fRadius_Outer = circleParam.z;
    } else {
      writeLog(strTxtPath, "外层拟合失败");

      return CFMEE_PARAMANALYZE_ERROR;
    }

    circlePoints.clear();
    for (auto point : innerContour)
      circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
    if (centerFiter->iterFitting(circlePoints, circleParam, false)) {
      fCenterX_Inner = circleParam.x;
      fCenterY_Inner = circleParam.y;
      fRadius_Inner = circleParam.z;
    } else {
      writeLog(strTxtPath, "内层拟合失败");

      return CFMEE_PARAMANALYZE_ERROR;
    }
  } else {
    writeLog(strTxtPath, "中心对称性不足");

    return CFMEE_PARAMANALYZE_ERROR;
  }
  MarkParam.Diameter[0] = fRadius_Outer * 2;
  MarkParam.Diameter[1] = fRadius_Inner * 2;
  MarkParam.IsWhite = JudgeIsWhite(
      ImageRoi, vector<vector<cv::Point>>{outerContour}, innerContour);
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, cv::Point(fCenterX_Outer, fCenterY_Outer),
             MarkParam.Diameter[0] / 2, 0, 2, 8, 0);
  cv::circle(result, cv::Point(fCenterX_Inner, fCenterY_Inner),
             MarkParam.Diameter[1] / 2, 0, 2, 8, 0);
  if (MarkParam.IsWhite)
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  else
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
  CvWaitShowImage(result, "圆环参数解析结果");
#endif // WRITE_RESULT
  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "外层直径" << MarkParam.Diameter[0] << endl;
  cout << "内层直径" << MarkParam.Diameter[1] << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "圆环成功");

  return CFMEE_CODE_OK;
}

/// <summary>
/// 解析CircleRing靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetCircleRingParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                        DetectParameterV3 &MarkParam) {
  std::string strTxtPath = "circleRing_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始circleRing参数解析");
  // 进行轮廓提取，找到多级轮廓
  // find contour， 获取外层contour和实例数
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);
  // 首先找到面积最大的父轮廓
  double area = 0;
  double ContourIndex = -1;
  double preContourIndex = -1;
  // 遍历轮廓层次关系，找出最大和次大的外层轮廓(外层圆环，内层圆)
  for (int i = 0; i < hierarchy.size(); i++) {
    if (hierarchy[i][3] == -1 && cv::contourArea(contours[i]) > area) {
      preContourIndex = ContourIndex;
      ContourIndex = i;
      area = cv::contourArea(contours[i]);
    }
  }
  // 若没有至少找到两个，或者最大那个没有内层纹理，分割失败
  if (cv::contourArea(contours[ContourIndex]) / BinaryMask.rows *
              BinaryMask.cols <
          0.1 ||
      preContourIndex == -1 || ContourIndex == -1 ||
      hierarchy[ContourIndex][2] == -1) {
    cout << "seg failed" << endl;
    return CFMEE_PARAMANALYZE_ERROR;
  }
  vector<cv::Point> outerContour = contours[ContourIndex]; //所选最外层轮廓
  vector<cv::Point> innerContour =
      contours[hierarchy[ContourIndex][2]]; //圆环内层轮廓
  vector<cv::Point> centerContour = contours[preContourIndex]; // 内层圆轮廓
  // CFitCircle* fiter = new CFitCircle();
  float fCenterX_Outer = 0, fCenterY_Outer = 0, fRadius_Outer = 0;
  float fCenterX_Inner = 0, fCenterY_Inner = 0, fRadius_Inner = 0;
  float fCenterX_Center = 0, fCenterY_Center = 0, fRadius_Center = 0;
  CdetCircle *centerFiter = new CdetCircle();
  // 检验中心对称性
  if (JudgeCentralSymmetry(outerContour, 0.90, BinaryMask.size()) &&
      JudgeCentralSymmetry(innerContour, 0.90, BinaryMask.size()) &&
      JudgeCentralSymmetry(centerContour, 0.90, BinaryMask.size())) {
    vector<ContoursPoint> circlePoints;
    cv::Point3d circleParam;
    for (auto point : outerContour)
      circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
    // 外层圆拟合
    if (centerFiter->iterFitting(circlePoints, circleParam, false)) {
      fCenterX_Outer = circleParam.x;
      fCenterY_Outer = circleParam.y;
      fRadius_Outer = circleParam.z;
    } else {
      cout << "外层圆拟合失败" << endl;
      return CFMEE_PARAMANALYZE_ERROR;
    }

    circlePoints.clear();
    for (auto point : innerContour)
      circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));
    if (centerFiter->iterFitting(circlePoints, circleParam, false)) {
      fCenterX_Inner = circleParam.x;
      fCenterY_Inner = circleParam.y;
      fRadius_Inner = circleParam.z;
    } else {
      cout << "内层圆拟合失败" << endl;
      return CFMEE_PARAMANALYZE_ERROR;
    }
    // 进行中心圆的拟合
    circlePoints.clear();
    for (auto point : centerContour)
      circlePoints.push_back(ContoursPoint(point.x, point.y, 0, 0, 0, 255));

    if (centerFiter->iterFitting(circlePoints, circleParam, false)) {

      fCenterX_Center = circleParam.x;
      fCenterY_Center = circleParam.y;
      fRadius_Center = circleParam.z;
    } else {
      cout << "中心圆拟合失败" << endl;
      return CFMEE_PARAMANALYZE_ERROR;
    }
  } else {
    writeLog(strTxtPath, "中心对称性不足");
    return CFMEE_PARAMANALYZE_ERROR;
  }
  MarkParam.Diameter[0] = fRadius_Outer * 2;
  MarkParam.Diameter[1] = fRadius_Inner * 2;
  MarkParam.Diameter[2] = fRadius_Center * 2;
  MarkParam.IsWhite = JudgeIsWhite(
      ImageRoi, vector<vector<cv::Point>>{outerContour}, innerContour);
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, cv::Point(fCenterX_Outer, fCenterY_Outer),
             MarkParam.Diameter[0] / 2, 0, 2, 8, 0);
  cv::circle(result, cv::Point(fCenterX_Inner, fCenterY_Inner),
             MarkParam.Diameter[1] / 2, 0, 2, 8, 0);
  cv::circle(result, cv::Point(fCenterX_Center, fCenterY_Center),
             MarkParam.Diameter[2] / 2, 0, 2, 8, 0);
  if (MarkParam.IsWhite) {
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, "circleRing",
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  } else {
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    cv::putText(result, "circleRing",
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
  }
  CvWaitShowImage(result, "circleRing参数解析结果");

#endif
  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "外层直径" << MarkParam.Diameter[0] << endl;
  cout << "内层直径" << MarkParam.Diameter[1] << endl;
  cout << "内圆直径" << MarkParam.Diameter[2] << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "circleRing成功");
  return CFMEE_CODE_OK;
}

/// <summary>
/// 解析Cross靶标mask
/// </summary>
/// <param name="BinaryMask">分割输出二值mask</param>
/// <param name="MarkParam">解析靶标参数结构体引用</param>
/// <returns></returns>
bool GetCrossParam(cv::Mat BinaryMask, cv::Mat ImageRoi,
                   DetectParameterV3 &MarkParam) {
  std::string strTxtPath = "cross_GetParam.txt";
  isReadWriteFile(strTxtPath);
  writeLog(strTxtPath, "开始十字参数解析");
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(BinaryMask, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  // 首先进行最大轮廓点选取和中心对称性验证
  double area = 0;
  vector<cv::Point> contour{};
  for (auto con : contours) {
    if (area < cv::contourArea(con)) {
      if (contour.data()) {
        cv::drawContours(BinaryMask, vector<vector<cv::Point>>{contour}, -1, 0,
                         -1);
      }
      area = cv::contourArea(con);
      contour = con;
    }
  }
  if (!contour.data() ||
      !JudgeCentralSymmetry(contour, 0.80, BinaryMask.size())) {
    writeLog(strTxtPath, "分割失败，分割质量不佳");

    return CFMEE_PARAMANALYZE_ERROR;
  }
  auto con = contour;
  cv::Point2f center;
  float radiusOut;
  minEnclosingCircle(con, center, radiusOut);
  double radius = radiusOut;
  for (auto point : contour) {
    double distance = sqrt((center.x - point.x) * (center.x - point.x) +
                           (center.y - point.y) * (center.y - point.y));
    if (distance < radius) {
      radius = distance;
    }
  }
  MarkParam.Width[0] = 2 * sqrt(radiusOut * radiusOut - 0.4998 * radius);
  MarkParam.Height[0] = MarkParam.Width[0];
  MarkParam.Diameter[0] = radius * 1.414;
  MarkParam.IsWhite = JudgeIsWhite(ImageRoi, vector<vector<cv::Point>>{contour},
                                   vector<cv::Point>{});
  MarkParam.Threshold = 15;
#ifdef _DEBUG
  cv::Mat result;
  ImageRoi.copyTo(result);
  cv::circle(result, center, radius, 255, 2, 8, 0);
  if (MarkParam.IsWhite) {
    cv::putText(result, "white",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
    cv::putText(result, "cross",
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 0, 1);
  } else {
    cv::putText(result, "black",
                cv::Point(ImageRoi.rows / 2, ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
    cv::putText(result, "cross",
                cv::Point(ImageRoi.rows / 2, 40 + ImageRoi.cols / 2),
                cv::FONT_HERSHEY_COMPLEX, 1.0, 255, 1);
  }
  CvWaitShowImage(result, "十字参数解析结果");
#endif // WRITE_RESULT

  cout << "图像尺寸参考" << BinaryMask.size() << endl;
  cout << "宽高" << MarkParam.Width[0] << '\t' << MarkParam.Height[0] << endl;
  cout << "线宽" << MarkParam.Diameter[0] << endl;
  cout << "白靶标？" << MarkParam.IsWhite << endl;
  writeLog(strTxtPath, "十字解析成功");

  return CFMEE_CODE_OK;
}

// /// <summary>
// /// 写入从分割mask解析到的参数
// /// 跟据类别编号读取相应的靶标配置文件作为模板、正常解析模板xml
// /// markparam子节点并更新相应字段、调用tinyxml接口进行xml文件写入
// /// </summary>
// /// <param name="class_id">靶标类别</param>
// /// <param name="MarkParam">从分割mask中解析到的靶标参数</param>
// bool MarkParamWrite(short class_id, const DetectParameterV3 MarkParam,
//                     char *detectConfig) {
//   vector<string> classlist = {"circle",
//                               "complexCrossOne",
//                               "complexMatrixHole",
//                               "cross",
//                               "rectangle",
//                               "matrixHole",
//                               "plum",
//                               "ring",
//                               "complexPlum",
//                               "circleRing"};
//   string xmlTemplatePath = "./MarkParam/" + classlist[class_id] + ".xml";
//   tinyxml2::XMLDocument doc;
//   if (doc.LoadFile(xmlTemplatePath.c_str())) {
//     return false;
//   }
//   // 首先修改marktype
//   tinyxml2::XMLElement *root = doc.FirstChildElement("MarkConfig");
//   root->SetAttribute("MarkType", class_id);
//   // string newXmlPath = "./MarkParam/" + classlist[class_id] + "-new.xml";
//   tinyxml2::XMLElement *MarkParamLayer =
//       root->FirstChildElement(); // XML中MarkParam这一级
//   // 进行遍历与元素写入
//   for (tinyxml2::XMLElement *cur = MarkParamLayer->FirstChildElement(); cur;
//        cur = cur->NextSiblingElement()) {
//     if (!strcmp(cur->Attribute("name"), "Dimater1"))
//       cur->SetAttribute("param", MarkParam.Diameter[0]);
//     else if (!strcmp(cur->Attribute("name"), "Dimater2"))
//       cur->SetAttribute("param", MarkParam.Diameter[1]);
//     else if (!strcmp(cur->Attribute("name"), "Dimater3"))
//       cur->SetAttribute("param", MarkParam.Diameter[2]);
//     else if (!strcmp(cur->Attribute("name"), "width1"))
//       cur->SetAttribute("param", MarkParam.Width[0]);
//     else if (!strcmp(cur->Attribute("name"), "height1"))
//       cur->SetAttribute("param", MarkParam.Height[0]);
//     else if (!strcmp(cur->Attribute("name"), "littleNum"))
//       cur->SetAttribute("param", MarkParam.nLittleCount);
//     else if (!strcmp(cur->Attribute("name"), "cenDistance"))
//       cur->SetAttribute("param", MarkParam.nCenterDistance);
//     else if (!strcmp(cur->Attribute("name"), "CenterWeight"))
//       cur->SetAttribute("param", MarkParam.dCenterWeight);
//     else if (!strcmp(cur->Attribute("name"), "Threshold"))
//       cur->SetAttribute("param", MarkParam.Threshold);
//     else if (!strcmp(cur->Attribute("name"), "Color"))
//       cur->SetAttribute("param", MarkParam.IsWhite);
//     else if (!strcmp(cur->Attribute("name"), "cols"))
//       cur->SetAttribute("param", MarkParam.Cols);
//     else if (!strcmp(cur->Attribute("name"), "rows"))
//       cur->SetAttribute("param", MarkParam.Rows);
//     else if (!strcmp(cur->Attribute("name"), "colspan"))
//       cur->SetAttribute("param", MarkParam.Colspan);
//     else if (!strcmp(cur->Attribute("name"), "rowspan"))
//       cur->SetAttribute("param", MarkParam.Rowspan);
//   }
//   doc.SaveFile(detectConfig);
//   return false;
// }

/// <summary>
/// 计算轮廓中心对称性得分(中心旋转交并比)，仅针对单一轮廓
/// </summary>
/// <param name="contour">输入轮廓</param>
/// <param name="threshold">中心对称性阈值</param>
/// <param name="size">所处图片尺寸</param>
/// <returns></returns>
bool JudgeCentralSymmetry(std::vector<cv::Point> contour, double threshold,
                          cv::Size2i size) {
  cv::Mat blackBoard = cv::Mat::zeros(size, CV_8U);
  std::vector<cv::Point> tempContour;
  cv::Moments M = cv::moments(contour); // 计算轮廓矩
  cv::Point2f gp =
      cv::Point2f(static_cast<float>(M.m10 / M.m00),
                  static_cast<float>(M.m01 / M.m00)); //获取轮廓重心
  // 点集中每个点绕重心旋转
  for (int i = 0; i < contour.size(); i++) {
    tempContour.push_back(cv::Point(-1 * (contour[i].x - gp.x) + gp.x,
                                    -1 * (contour[i].y - gp.y) + gp.y));
  }
  cv::drawContours(blackBoard, vector<vector<cv::Point>>{tempContour}, -1, 255,
                   -1);
  // cv::imshow("img", blackBoard);
  // cv::waitKey(0);
  cv::drawContours(blackBoard, vector<vector<cv::Point>>{contour}, -1, 255, -1);
  // cv::imshow("img", blackBoard);
  // cv::waitKey(0);
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(blackBoard, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  double unionArea = cv::contourArea(contours[0]);
  double Area = cv::contourArea(tempContour);
  return ((Area * 2 - unionArea) / unionArea) >= threshold;
}

/// <summary>
/// 根据输入的评判数据对轮廓进行分类，数值接近的分为一类，将一组数据分为n个段
/// </summary>
/// <param name="dataVector">输入的数据</param>
/// <param name="clusterData">分类后的数据</param>
/// <returns></returns>
bool contourCluster(
    vector<pair<double, vector<cv::Point>>> inputContour,
    vector<pair<vector<vector<cv::Point>>, cv::Point2f>> &clusteredContour,
    double AverageThreshold, double VarianceThreshold) {
  cout << "inputnum" << inputContour.size();
  if (!inputContour.size()) {
    cout << "输入空轮廓" << endl;
    return false;
  }
  // 首先对输入的<评判数据，轮廓>对做一次排序
  sort(inputContour.begin(), inputContour.end(),
       [](auto x, auto y) { return x.first < y.first; });
  // vector<pair(轮廓vector，组内均值、方差)>
  vector<double> datalist;
  for (auto contour : inputContour) {
    datalist.push_back(contour.first);
    // 空的时候新建项
    if (!clusteredContour.size()) {
      //新建表项
      clusteredContour.push_back(
          make_pair(vector<vector<cv::Point>>{contour.second},
                    cv::Point2f(contour.first, 0)));
      continue;
    }

    // 计算假设变更后的均值和方差
    double average = clusteredContour.back().second.x +
                     (contour.first - clusteredContour.back().second.x) /
                         (clusteredContour.back().first.size() + 1);
    double variance = calNormalizationVar(datalist, average);
    cout << "var" << variance << endl;

    // 正常情况下,首先判断加入新的contour与均值间的离散度、是否会使样本方差产生较大波动
    // abs(clusteredContour.back().second.x - contour.first) <
    // AverageThreshold&&
    if (abs(clusteredContour.back().second.y - variance) < VarianceThreshold) {
      clusteredContour.back().first.push_back(
          contour.second);                         //修改分类contour列表
      clusteredContour.back().second.x = average;  //修改均值
      clusteredContour.back().second.y = variance; //修改相应方差
    } else {
      // 加不进去的时候新建项
      // 新建表项
      datalist.clear();
      datalist.push_back(contour.first);
      clusteredContour.push_back(
          make_pair(vector<vector<cv::Point>>{contour.second},
                    cv::Point2f(contour.first, 0)));
    }
  }
  return true;
}

/// <summary>
/// 计算添加某值后的归一化方差
/// </summary>
/// <param name="datalist">数据列表</param>
/// <param name="average">列表均值</param>
double calNormalizationVar(vector<double> datalist, double average) {
  double variance = 0;
  for (int i = 0; i < datalist.size(); i++)
    variance += (datalist[i] / average - 1) * (datalist[i] / average - 1);
  return variance / datalist.size();
}

/// <summary>
/// 判断靶标颜色
/// </summary>
/// <param name="mSrcImage">输入图像</param>
/// <param name="contour">靶标轮廓</param>
/// <returns></returns>
bool JudgeIsWhite(cv::Mat mSrcImage, vector<vector<cv::Point>> contour,
                  vector<cv::Point> innerContour) {
  // 使用输入contour计算出一个轮廓部位mask1(前景白，背景黑)
  cv::Mat contourMask = cv::Mat::zeros(mSrcImage.size(), CV_8U);
  cv::drawContours(contourMask, contour, -1, 255, -1);
  if (innerContour.data())
    cv::drawContours(contourMask, vector<vector<cv::Point>>{innerContour}, -1,
                     0, -1);
  // 对mask应用膨胀操作得到masknew，使用masknew-mask 得到外圈部分mask2
  cv::Mat contourRoundMask;
  cv::Mat dilateStruct = cv::getStructuringElement(0, cv::Size(9, 9));
  cv::dilate(contourMask, contourRoundMask, dilateStruct);
  cv::Mat RoundMask = contourRoundMask - contourMask;
  // 分别使用mask1，mask2计算相应位置像素均值
  cv::Scalar ContourMean;
  cv::Scalar RoundMean;
  cv::Mat stddevMat; //占位，接收统计标准差
  cv::meanStdDev(mSrcImage, ContourMean, stddevMat, contourMask);
  cv::meanStdDev(mSrcImage, RoundMean, stddevMat, RoundMask);
  return ContourMean[0] > RoundMean[0];
}