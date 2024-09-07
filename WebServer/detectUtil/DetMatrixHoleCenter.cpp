#include "DetMatrixHoleCenter.h"

#include "Common.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "Threshold.h"
#include "LogHelp.h"
#include "EnhanceImage.h"
#include "Edge.h"
#include "LeastSquare.h"
#include "Hist.h"

using namespace std;
using namespace cv;

/// <summary>
/// 构造函数
/// </summary>
/// <param name="strLog">日志文件路径</param>
/// <param name="param">靶标参数结构体</param>
/// <param name="algorithmPara">算法参数结构体</param>
CdetHoleMatrix::CdetHoleMatrix(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara)
{
    m_strLog = strLog;                                           //日志文档
    m_nRows = param.Rows;                                        //列
    m_nCols = param.Cols;                                        //行
    m_RowSpan = param.Rowspan;                                   //列间距
    m_ColSpan = param.Colspan;                                   //行间距
    m_nDiameter = param.Diameter[0];                             //直径参数
    m_nThreshold = param.Threshold;                              //误差参数
    m_bIsWhite = param.IsWhite;                                  //靶标为白色
    m_nAlgorithm = algorithmPara.nMarkAlgorithm;                 //识别算法
    m_bWhiteEdge = algorithmPara.nWhiteEdge;                     //白色靶标边缘
    m_bIsPolarChange = algorithmPara.IsPolarChange;              //极性检测
    m_fHoleMatrixSocre = algorithmPara.dSocreThreshold;          //检测阈值参数
    m_bIsEnhance = algorithmPara.IsEnhanceMap;                   //图像增强
    m_nSpanRange = abs(m_nDiameter - min(m_RowSpan, m_ColSpan)); //直径与间距差
}

CdetHoleMatrix::~CdetHoleMatrix()
{
}

/// <summary>
/// 获取每一个小圆的圆心坐标
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="dAngle">旋转角度</param>
/// <param name="vecEveryHoleMatrixPoint">检测到每一个小圆中心</param>
/// <param name="pHoleMatrixCenter">靶标中心点</param>
/// <returns></returns>
bool CdetHoleMatrix::getEveryLittleCircle(const cv::Mat mSrcImage, double dAngle,
                                          std::vector<cv::Point2d> &vecEveryAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mGrayImage;
    mSrcImage.copyTo(mGrayImage);
    m_dAngle = dAngle;
    //获取每一个小圆位置
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecEveryRoughPoint = getRoughPointLocation(mGrayImage);
#ifdef _SHOW_PROCESS_
    cv::Mat mShowRoughLocation = drawRoughLocation(mSrcImage, vecEveryRoughPoint, m_nDiameter);
    CvWaitShowImage(mShowRoughLocation, "靶标位置粗定位");
#endif
    cv::Mat mEvenImage;
    mGrayImage.copyTo(mEvenImage);
    cv::Point2d pExtactCenter(0, 0);
    //精确检测每一个小圆坐标
    if (detEveryCircleLocation(mEvenImage, vecEveryRoughPoint, vecEveryAccurateLocation,
                               pExtactCenter, dScore))
    {
        pHoleMatrixCenter.x = pExtactCenter.x;
        pHoleMatrixCenter.y = pExtactCenter.y;
        return true;
    }
    return false;
}

/// <summary>
/// 直接定位小圆，调用计算圆心计算算法
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="dAngle">旋转角度</param>
/// <param name="vecEveryHoleMatrixPoint">检测到每一个小圆中心</param>
/// <param name="pHoleMatrixCenter">靶标中心点</param>
/// <returns></returns>
bool CdetHoleMatrix::getEveryLittleCircleDirect(const cv::Mat mSrcImage, const std::vector<std::vector<pair<cv::Mat, cv::Point>>> vecEveryRoughPoint, std::vector<cv::Point2d> &vecEveryAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
#ifdef _SHOW_PROCESS_
    cv::Mat mShowRoughLocation = drawRoughLocation(mSrcImage, vecEveryRoughPoint, m_nDiameter);
    CvWaitShowImage(mShowRoughLocation, "靶标位置粗定位");
#endif
    cv::Mat mEvenImage;
    mSrcImage.copyTo(mEvenImage);
    cv::Point2d pExtactCenter(0, 0);
    //cout << "分割个数" << vecEveryRoughPoint[0].size() << endl;
    //精确检测每一个小圆坐标
    if (detEveryCircleLocation(mEvenImage, vecEveryRoughPoint, vecEveryAccurateLocation,
                               pExtactCenter, dScore))
    {
        pHoleMatrixCenter.x = pExtactCenter.x;
        pHoleMatrixCenter.y = pExtactCenter.y;
        return true;
    }
    return false;
}

//若干个靶点计算均值
cv::Point2d CdetHoleMatrix::getMeanPoint(std::vector<std::vector<cv::Point2d>> one)
{
    cv::Point2d meanPoint;
    double dX = 0, dY = 0;
    int nNum = 0;
    for (int i = 0; i < one.size(); ++i)
    {
        for (int j = 0; j < one[i].size(); ++j)
        {
            dX += one[i][j].x;
            dY += one[i][j].y;
            nNum++;
        }
    }
    meanPoint.x = dX / nNum;
    meanPoint.y = dY / nNum;
    return meanPoint;
}

CdetHoleMatrix &CdetHoleMatrix::operator=(const CdetHoleMatrix &)
{
    // TODO: 在此处插入 return 语句
    return *this;
}

/// <summary>
/// 通过入参确定是否存在中心圆
/// </summary>
/// <returns></returns>
bool CdetHoleMatrix::isExistCenterMark()
{
    int nColRemainder = m_nCols % 2;
    int nRowRemainder = m_nRows % 2;
    if (nColRemainder == 1 && nRowRemainder == 1)
        return true;
    else
        return false;
}

/// <summary>
/// 推测每个靶标的大致中心点位置
/// </summary>
/// <param name="mSrcImage"></param>
/// <returns></returns>
std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> CdetHoleMatrix::getRoughPointLocation(cv::Mat mSrcImage)
{
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughPoint; //靶点坐标合集
    std::vector<std::pair<cv::Mat, cv::Point>> singleRoughPoint;           //单行靶点坐标
    for (int i = 0; i < m_nRows; ++i)
    {
        singleRoughPoint.clear();
        for (int j = 0; j < m_nCols; ++j)
        {
            //int nX = (mSrcImage.cols  - (m_nCols - 1) * m_ColSpan) / 2 + j * m_ColSpan;
            //int nY = (mSrcImage.rows  - (m_nRows - 1) * m_RowSpan) / 2 + i * m_RowSpan;
            int nX = (mSrcImage.cols - m_nDiameter - (m_nCols - 1) * m_ColSpan) / 2 + j * m_ColSpan;
            int nY = (mSrcImage.rows - m_nDiameter - (m_nRows - 1) * m_RowSpan) / 2 + i * m_RowSpan;
            int nValueX[10] = {nX, (m_nDiameter + m_nThreshold) / 2, mSrcImage.cols - nX};
            int nValueY[10] = {nY, (m_nDiameter + m_nThreshold) / 2, mSrcImage.rows - nY};
            int nDisX = getMinValue(nValueX, 3);
            int nDisY = getMinValue(nValueY, 3);
            int nStartX = nX - nDisX;
            int nStartY = nY - nDisY;
            int nCols = nDisX * 2;
            int nRows = nDisY * 2;
            cv::Mat mDesImage = mSrcImage(Rect(nX, nY, nCols, nRows));
            //singleRoughPoint.push_back(std::make_pair(mDesImage, cv::Point(nX + m_nDiameter / 2, nY + m_nDiameter / 2)));
            singleRoughPoint.push_back(std::make_pair(mDesImage, cv::Point(nX, nY)));
        }
        vecRoughPoint.push_back(singleRoughPoint);
    }
    return vecRoughPoint;
}

/// <summary>
/// 检测每一个小圆的中心点坐标
/// </summary>
/// <param name="mSrcImage">输入图像</param>
/// <param name="vecRoughLocation">粗定位坐标</param>
/// <param name="vecAccurateLocation">精确检测坐标</param>
/// <param name="pHoleMatrixCenter">孔矩阵中心点坐标</param>
/// <param name="dScore">检测得分</param>
/// <returns></returns>
bool CdetHoleMatrix::detEveryCircleLocation(cv::Mat mSrcImage, std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                            std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    std::vector<std::vector<cv::Point2d>> vecPoint2d;
    //根据不同直径进行不同的检测计算
    if (m_nDiameter > 50)
    {
        writeLog(m_strLog, "圆拟合抓取");
        if (detEveryBigCircleDLPrior(vecRoughLocation, vecPoint2d))
        {
            writeLog(m_strLog, "圆拟合抓取成功");
        }
        else
        {
            writeLog(m_strLog, "圆拟合抓取失败");
        }
    }
    else
    {
        writeLog(m_strLog, "分割检测抓取");
        if (detEveryLittleCircle(vecRoughLocation, vecPoint2d))
        {
            writeLog(m_strLog, "分割检测抓取成功");
        }
        else
        {
            writeLog(m_strLog, "分割检测抓取失败");
        }
    }
#ifdef _SHOW_PROCESS_
    cv::Mat mShowImage = generateColorImage(mSrcImage);
    for (int i = 0; i < vecPoint2d.size(); ++i)
    {
        for (int j = 0; j < vecPoint2d[i].size(); ++j)
        {
            circle(mShowImage, Point(vecPoint2d[i][j].x, vecPoint2d[i][j].y), m_nDiameter / 2, cv::Scalar(255, 0, 0), 1, 8, 0);
        }
    }
    CvWaitShowImage(mShowImage, "抓取成功的小圆靶标");
#endif
    //对检测结果进行筛选
    if (analysisDetSuccessCircle(vecPoint2d, vecAccurateLocation, pHoleMatrixCenter, dScore))
    {
        writeLog(m_strLog, "检测结果筛选成功");

        return true;
    }
    else
    {
        writeLog(m_strLog, "ERROR:检测结果不符合筛选条件，检测失败");
        return false;
    }
    return false;
}

/// <summary>
/// 大直径单圆检测
/// </summary>
/// <param name="vecRoughLocation">每一个单圆存储容器</param>
/// <param name="vecCircleCenter">检测结果</param>
/// <returns></returns>
bool CdetHoleMatrix::detEveryBigCircleDLPrior(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                              std::vector<std::vector<cv::Point2d>> &vecCircleCenter)
{
    std::vector<cv::Point2d> vecOne;
    std::vector<std::vector<cv::Point2d>> vecPoint2d;
    cv::Point2d pExactOne;
    for (auto locationSet : vecRoughLocation)
    {
        std::vector<cv::Point2d> vecHorResult;
        for (auto location : locationSet)
        {
            if (getBigSingleCenter(location.first, pExactOne) && pExactOne.x != 0)
            {
                pExactOne.x = pExactOne.x + location.second.x;
                pExactOne.y = pExactOne.y + location.second.y;
                vecOne.push_back(pExactOne);
            }
            else
            {
                //cout << "圆拟合失败" << endl;
#define DEEPLEARNING
#ifdef DEEPLEARNING
                // 使用分割先验
                pExactOne.x = location.first.cols / 2 + location.second.x;
                pExactOne.y = location.first.rows / 2 + location.second.y;
                vecOne.push_back(pExactOne);
#endif // DEEPLEARNING
            }
        }
        vecCircleCenter.push_back(vecOne);
    }
    if (vecCircleCenter.size())
        return true;
    else
        return false;
    return false;
}

/// <summary>
/// 小直径单圆检测
/// </summary>
/// <param name="vecRoughLocation">每一个单圆存储容器</param>
/// <param name="vecCircleCenter">检测结果</param>
/// <returns></returns>
bool CdetHoleMatrix::detEveryLittleCircle(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                          std::vector<std::vector<cv::Point2d>> &vecCircleCenter)
{
    CFilter filter;
    cv::Mat mDesImage;
    int nThresholdParam;
    cv::Point2d pExactResult(0, 0);
    for (int i = 0; i < vecRoughLocation.size(); ++i)
    {
        std::vector<cv::Point2d> vecHorResult;
        for (int j = 0; j < vecRoughLocation[i].size(); ++j)
        {
            filter.getFilter(vecRoughLocation[i][j].first, 3, "5", mDesImage);
            int nThresholdParam = OtsuThres(mDesImage);
            switch (m_nAlgorithm)
            {
            case 0:
                writeLog(m_strLog, "自适应二值化分割");
                if (getLittleCircleAutoSeg(mDesImage, nThresholdParam, pExactResult))
                {
                    pExactResult.x += vecRoughLocation[i][j].second.x;
                    pExactResult.y += vecRoughLocation[i][j].second.y;
                    vecHorResult.push_back(pExactResult);
                }
                else
                {
#define DEEPLEARNING
#ifdef DEEPLEARNING
                    // 使用分割先验
                    pExactResult.x = vecRoughLocation[i][j].first.cols / 2 + vecRoughLocation[i][j].second.x;
                    pExactResult.y = vecRoughLocation[i][j].first.rows / 2 + vecRoughLocation[i][j].second.y;
                    vecHorResult.push_back(pExactResult);
#endif // DEEPLEARNING
                }
                break;
            case 1:
                writeLog(m_strLog, "统计法二值化分割");
                break;
            case 2:
                writeLog(m_strLog, "轮廓检测拟合抓取");
                break;
            default:
                break;
            }
        }
        vecCircleCenter.push_back(vecHorResult);
    }
    return true;
}

/// <summary>
/// 自适应阈值分割检测小圆算法
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="nThreshold">分割阈值</param>
/// <param name="roughPoint">检测到的中心点坐标</param>
/// <returns></returns>
bool CdetHoleMatrix::getLittleCircleAutoSeg(cv::Mat mSrcImage, int nThreshold, cv::Point2d &roughPoint)
{
    roughPoint = {0, 0};
    cv::Mat mThreshold;
    cv::adaptiveThreshold(mSrcImage, mThreshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
    Mat means, stddev, covar;
    meanStdDev(mThreshold, means, stddev); //计算src图片的均值和标准差
    double dd = means.at<double>(0, 0);
    if (dd > 200)
        mThreshold = ~mThreshold;
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(mThreshold, mThreshold, element);
    vector<vector<Point>> opencvContours;
    vector<Vec4i> opencvHierarchy;
    findContours(mThreshold, opencvContours, opencvHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (opencvContours.size() == 0)
        return false;
    sort(opencvContours.begin(), opencvContours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) { return a.size() > b.size(); });
    std::vector<cv::RotatedRect> vecRoateRect;
    int n = 0;
    for (int i = 0; i < opencvContours.size(); ++i)
    {
        RotatedRect boxCircle = minAreaRect(Mat(opencvContours[i]));
        if ((boxCircle.size.width / boxCircle.size.height < 3 && boxCircle.size.width / boxCircle.size.height > 0.3) &&
            boxCircle.size.width * boxCircle.size.height <= pow((float)m_nDiameter, 2) * 3.5 &&
            boxCircle.size.width * boxCircle.size.height >= pow((float)m_nDiameter, 2) * 0.3)
            vecRoateRect.push_back(boxCircle);
    }
    if (vecRoateRect.size() > 0)
    {
        roughPoint.x = vecRoateRect[0].center.x;
        roughPoint.y = vecRoateRect[0].center.y;
    }
    else
    {
        vector<Point> vecContours;
        mergeVector(opencvContours, vecContours, 10);
        if (vecContours.size() != 0)
        {
            RotatedRect box = minAreaRect(Mat(vecContours));
            if ((box.size.width / box.size.height < 2 && box.size.width / box.size.height > 0.5) &&
                box.size.width * box.size.height <= pow((float)m_nDiameter, 2) * 3 &&
                box.size.width * box.size.height >= pow((float)m_nDiameter, 2) * 0.3)
            {
                roughPoint.x = box.center.x;
                roughPoint.y = box.center.y;
            }
        }
    }
    if (roughPoint.x != 0 && roughPoint.y != 0)
        return true;
    else
        return false;
}

/// <summary>
/// 统计阈值分割检测小圆算法
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="roughPoint">检测到的中心点坐标</param>
/// <returns></returns>
bool CdetHoleMatrix::getLittleCircleStatisSeg(cv::Mat mSrcImage, cv::Point2d &roughPoint)
{
    roughPoint = {0, 0};
    cv::Mat mThresholdImage;
    MatND dstHist = getImageHist(mSrcImage);
    //cout << dstHist << endl;
    int nArea = m_nDiameter * m_nDiameter;
    double dBinValue = 0;
    int nIndex = 0;
    if (!m_bIsWhite) //黑色靶标
    {
        for (int i = 0; i < 256; ++i)
        {
            dBinValue += dstHist.at<float>(i);
            if (dBinValue >= nArea)
            {
                nIndex = i;
                break;
            }
        }
        cv::threshold(mSrcImage, mThresholdImage, nIndex, 255, THRESH_BINARY_INV);
    }
    else //白色靶标
    {
        for (int i = 256; i > 0; --i)
        {
            dBinValue += dstHist.at<float>(i);
            if (dBinValue >= nArea)
            {
                nIndex = i;
                break;
            }
        }
        cv::threshold(mSrcImage, mThresholdImage, nIndex, 255, THRESH_BINARY);
    }

#if _DEBUG
    CvWaitShowImage(mThresholdImage, "小靶标二值化");
#endif
    vector<vector<Point>> opencvContours;
    vector<Vec4i> opencvHierarchy;
    findContours(mThresholdImage, opencvContours, opencvHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (opencvContours.size() == 0)
        return false;
    sort(opencvContours.begin(), opencvContours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) { return a.size() > b.size(); });
    std::vector<cv::RotatedRect> vecRoateRect;
    int n = 0;
    for (int i = 0; i < opencvContours.size(); ++i)
    {
        RotatedRect boxCircle = minAreaRect(Mat(opencvContours[i]));
        if ((boxCircle.size.width / boxCircle.size.height < 2 && boxCircle.size.width / boxCircle.size.height > 0.5) &&
            boxCircle.size.width * boxCircle.size.height <= pow((float)m_nDiameter / 2, 2) * CV_PI * 2 &&
            boxCircle.size.width * boxCircle.size.height >= pow((float)m_nDiameter / 2, 2) * CV_PI * 0.5)
            vecRoateRect.push_back(boxCircle);
    }
    if (vecRoateRect.size() > 0)
    {
        roughPoint.x = vecRoateRect[0].center.x;
        roughPoint.y = vecRoateRect[0].center.y;
    }
    else
    {
        vector<Point> vecContours;
        mergeVector(opencvContours, vecContours, 10);
        if (vecContours.size() != 0)
        {
            RotatedRect box = minAreaRect(Mat(vecContours));
            if ((box.size.width / box.size.height < 2 && box.size.width / box.size.height > 0.5) &&
                box.size.width * box.size.height <= pow((float)m_nDiameter / 2, 2) * CV_PI * 2 &&
                box.size.width * box.size.height >= pow((float)m_nDiameter / 2, 2) * CV_PI * 0.5)
            {
                roughPoint.x = box.center.x;
                roughPoint.y = box.center.y;
            }
        }
    }
    if (roughPoint.x != 0 && roughPoint.y != 0)
        return true;
    else
        return false;
}

/// <summary>
/// 轮廓检测拟合检测小圆算法
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="roughPoint">检测到的中心点坐标</param>
/// <returns></returns>
bool CdetHoleMatrix::getLittleCircleContourFit(cv::Mat mSrcImage, cv::Point2d &roughPoint)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    float fRedius = 0;
    Point2f pContour(0, 0);            //最小外接圆圆心
    cv::Point3d circleCenter(0, 0, 0); //拟合后的圆中心坐标
    //滤波初始化
    CFilter filter;
    //梯度检测初始化
    CGradient getGradient;
    //增强初始化
    CEnhance enhance(m_bIsWhite);
    cv::Mat mFilterImage, mEnhanceImage, mGradientImage, mGradientImage2;

    filter.getFilter(mDesImage, 3, "5", mFilterImage);
    enhance.getEnhanceImage(mFilterImage, 1, mEnhanceImage);
    //enhance.getEnhanceImage(mFilterImage, 4, mEnhanceImage, 0, 0, 400, 10);
    //enhance.getLocalEnhance(mFilterImage, 0.8, 1, 0.1, 0.8, mEnhanceImage);
    filter.getFilter(mEnhanceImage, 4, "3", mEnhanceImage);
    getGradient.getGradientImage(mEnhanceImage, mGradientImage, 0);
    //轮廓连接
    ConnectEdge(mGradientImage, mGradientImage2);
    /*	MattoPoint(mGradientImage, true, mGradientImage.cols, mGradientImage.cols / 2, vecContours);
		MattoPoint(mGradientImage2, true, mGradientImage.cols, mGradientImage.cols / 2, vecOut);*/
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mGradientImage2, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mSrcImage);
    for (int x = 0; x < contours.size(); ++x)
    {
        Scalar color(rand() & 255, rand() & 255, rand() & 255);
        drawContours(mShowImage, contours, x, color, 2, 8, hierarchy);
    }
    CvWaitShowImage(mShowImage, "轮廓图");
#endif
    if (contours.size() <= 0)
    {
        writeLog(m_strLog, "轮廓搜索失败，没有满足条件的轮廓");
        return false;
    }
    sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) { return a.size() > b.size(); });
    for (int x = 0; x < contours.size(); ++x)
    {
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

        if (abs(mSrcImage.cols / 2 - pt1.x) > 10 ||
            abs(mSrcImage.rows / 2 - pt1.y) > 10)
            continue;
        //最小外接圆
        minEnclosingCircle(contours[x], pContour, fRedius);
        //float fLength = arcLength(contours[i], true);
        float fArea = contourArea(contours[x]);
        if (fRedius >= m_nDiameter / 2 - m_nThreshold && fRedius <= m_nDiameter / 2 + m_nThreshold &&
            fArea > m_nDiameter / 4 * m_nDiameter / 4 * CV_PI && fArea < m_nDiameter * m_nDiameter * CV_PI)
        {
            writeLog(m_strLog, "目标直径，面积校验通过");
#if _DEBUG
            cv::Mat mShowImage2 = generateColorImage(mSrcImage);
            drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
            CvWaitShowImage(mShowImage2, "轮廓图2");
#endif
            //精确计算
            writeLog(m_strLog, "启用精确计算");
            //满足条件的轮廓按照不同象限进行分类处理
            scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
            roughPoint.x = circleCenter.x;
            roughPoint.y = circleCenter.y;

            return true;
        }
    }
    return false;
}

/// <summary>
/// 象限轮廓筛选算法
/// </summary>
/// <param name="mSrcImage">输入图片</param>
/// <param name="vecInput">带筛选轮廓</param>
/// <param name="pRoughCenter">小圆粗定位中心点坐标</param>
/// <param name="pCenter">小圆精确检测到的参数结果</param>
/// <returns></returns>
int CdetHoleMatrix::scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput, cv::Point2d pRoughCenter, cv::Point3d &pCenter)
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
        nQuadrant = isQuadrant(cv::Point(vecInput[i].x, vecInput[i].y), pRoughCenter);
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
    {
        if (vecRough[i].size() != 0)
        {
            for (int j = 0; j < vecRough[i].size() - 1; ++j)
            {
                cv::Point pContours(0, 0);
                if (vecRough[i][j].x - pRoughCenter.x != 0)
                {
                    dK = double(vecRough[i][j].y - pRoughCenter.y) / double(vecRough[i][j].x - pRoughCenter.x);
                    dB = double(pRoughCenter.y * vecRough[i][j].x - vecRough[i][j].y * pRoughCenter.x) / double(vecRough[i][j].x - pRoughCenter.x);
                }
                else
                {
                    dK = 10e9;
                    dB = 0;
                }
                dDisRediusOne = sqrt(pow(double(vecRough[i][j].x - pRoughCenter.x), 2) + pow(double(vecRough[i][j].y - pRoughCenter.y), 2));
                dCheckDistance = dDisRediusOne;
                pContours = vecRough[i][j];
                for (int x = j + 1; x < vecRough[i].size(); ++x)
                {
                    if (vecRough[i][x].x - pRoughCenter.x != 0)
                    {
                        dK2 = double(vecRough[i][x].y - pRoughCenter.y) / double(vecRough[i][x].x - pRoughCenter.x);
                        dB2 = double(pRoughCenter.y * vecRough[i][x].x - vecRough[i][x].y * pRoughCenter.x) / double(vecRough[i][x].x - pRoughCenter.x);
                    }
                    else
                    {
                        dK2 = 10e9;
                        dB2 = 0;
                    }
                    //每2弧度选取一个轮廓点
                    if (abs(atan(dK) - atan(dK2)) <= double(CV_PI / 90))
                    {
                        dDisRediusTwo = sqrt(pow(double(vecRough[i][x].x - pRoughCenter.x), 2) + pow(double(vecRough[i][x].y - pRoughCenter.y), 2));
                        if (dCheckDistance > dDisRediusTwo)
                        {
                            pContours = vecRough[i][x];
                            dCheckDistance = dDisRediusTwo;
                        }
                        vecRough[i].erase(vecRough[i].begin() + x);
                        x--;
                    }
                }
                vecContours.push_back(ContoursPoint(pContours.x, pContours.y));
            }
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

// 拟合大圆中心
bool CdetHoleMatrix::getBigSingleCenter(cv::Mat mSrcImage, cv::Point2d &roughPoint)
{
    roughPoint.x = 0;
    roughPoint.y = 0;
    cv::Mat mGradientImage, mFilterImage, mEnhanceImage, mDesImage;
    mSrcImage.copyTo(mDesImage);
    CFilter filter;
    CGradient gradient;
    //增强初始化
    CEnhance enhance(m_bIsWhite);
    std::vector<cv::Point2d> vecContours, vecOut;
    double dDisRediusOne, dStepXOne, dStepYOne, dMaxDiffOne;
    enhance.getEnhanceImage(mDesImage, 1, mEnhanceImage);
    filter.getFilter(mEnhanceImage, 3, "7", mFilterImage);
    //filter.getFilter(mFilterImage, 1, "3", mFilterImage);
    std::vector<cv::Point2d> vecPoint;
    mGradientImage = cv::Mat::zeros(mFilterImage.rows, mFilterImage.cols, CV_8UC1);
    gradient.getSubPixelGradient(mFilterImage, mGradientImage, vecPoint);
    MattoPoint(mGradientImage, true, m_nDiameter, m_nSpanRange / 2, vecContours);

    cv::Point pp(tRound(mSrcImage.cols / 2), tRound(mSrcImage.rows / 2));
    for (int x = 0; x < vecContours.size(); ++x)
    {
        if (vecContours[x].x > 2 && vecContours[x].x < mSrcImage.cols - 2 &&
            vecContours[x].y > 2 && vecContours[x].y < mSrcImage.rows - 2)
        {
            dDisRediusOne = sqrt(pow(double(vecContours[x].x - pp.x), 2) + pow(double(vecContours[x].y - pp.y), 2));
            dStepXOne = tRound(double((vecContours[x].x - pp.x) / dDisRediusOne));
            dStepYOne = tRound(double((vecContours[x].y - pp.y) / dDisRediusOne));

            dMaxDiffOne = mSrcImage.at<uchar>(vecContours[x].y + 1 * dStepYOne, vecContours[x].x + 1 * dStepXOne) + mSrcImage.at<uchar>(vecContours[x].y + 2 * dStepYOne, vecContours[x].x + 2 * dStepXOne) -
                          mSrcImage.at<uchar>(vecContours[x].y - 1 * dStepYOne, vecContours[x].x - 1 * dStepXOne) - mSrcImage.at<uchar>(vecContours[x].y - 2 * dStepYOne, vecContours[x].x - 2 * dStepXOne);

            /*if ((m_isWhite && dMaxDiffOne < MIDDLE_DIFF_PIXEL) || (!m_isWhite && dMaxDiffOne > SUPER_DIFF_PIXEL))
					vecOut.push_back(vecContours[x]);*/
            if (abs(dMaxDiffOne) > 5)
                vecOut.push_back(vecContours[x]);
        }
    }
#if _DEBUG
    vector<Cfmee_Point> temp;
    for (auto p : vecOut)
        temp.push_back(Cfmee_Point(p.x, p.y));
    cv::Mat mShowImage = drawColorImage(mDesImage, temp);
    CvWaitShowImage(mShowImage, "筛选后的轮廓点");
#endif
    if (vecOut.size() < 3)
        return false;
    vector<Cfmee_Point> cfmee_vecOut;
    for (auto p : vecPoint)
        cfmee_vecOut.push_back(Cfmee_Point(p.x, p.y));
    double dSigma = getVariance(cfmee_vecOut).second;
    double dMeanDis = getVariance(cfmee_vecOut).first;
    double dDis = 0;
    cv::Point3d tempCircle(0, 0, 0);
    tempCircle = leastSquareFittingCircle(cfmee_vecOut);
    //int nn = 0;
    while (dSigma > 1)
    {
        tempCircle = leastSquareFittingCircle(cfmee_vecOut);
        for (int j = 0; j < cfmee_vecOut.size(); ++j)
        {
            dDis = abs(sqrt(pow((double)cfmee_vecOut[j].dX - tempCircle.x, 2) + pow((double)cfmee_vecOut[j].dY - tempCircle.y, 2)) - dMeanDis);
            if (dDis > dSigma)
            {
                //nn++;
                cfmee_vecOut.erase(cfmee_vecOut.begin() + j);
                j--;
            }
        }
        dMeanDis = getVariance(cfmee_vecOut).first;
        dSigma = getVariance(cfmee_vecOut).second;
        if (cfmee_vecOut.size() < 3)
            break;
    }
    //校验半径是否是为错误数字
    if (__finite(tempCircle.z) == 0)
        return false;
    float fRoundness = 0;
    vecOut.clear();
    for (auto p : cfmee_vecOut)
        vecOut.push_back(cv::Point2d(p.dX, p.dY));
    int nError = checkDefectRoundness(vecOut, tempCircle, fRoundness);
    if (tempCircle.x != 0 && tempCircle.y != 0 && tempCircle.z >= m_nDiameter / 2 - m_nSpanRange / 2 &&
        tempCircle.z <= m_nDiameter / 2 + m_nSpanRange / 2 && fRoundness > 0.4f)
    {
#if _DEBUG

        cv::Mat mShowImageResult = generateColorImage(mDesImage);
        circle(mShowImageResult, Point(tempCircle.x, tempCircle.y), m_nDiameter / 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        CvWaitShowImage(mShowImageResult, "抓取结果");
#endif
        roughPoint.x = tempCircle.x;
        roughPoint.y = tempCircle.y;
        return true;
    }
    else
        return false;
}

/// <summary>
/// 对检测成功的小圆进行筛选
/// </summary>
/// <param name="vecCircleCenter">检测成功小圆坐标容器</param>
/// <param name="vecAccurateLocation">筛选后坐标容器</param>
/// <param name="pHoleMatrixCenter">筛选后整体中心点坐标</param>
/// <param name="dScore">检测得分</param>
/// <returns></returns>
bool CdetHoleMatrix::analysisDetSuccessCircle(std::vector<std::vector<cv::Point2d>> vecCircleCenter,
                                              std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    // 把结果存入返回vec
    for (int i = 0; i < vecCircleCenter.size(); i++)
        for (int j = 0; j < vecCircleCenter[0].size(); j++)
            vecAccurateLocation.push_back(vecCircleCenter[i][j]);

    if (vecAccurateLocation.size() == m_nRows * m_nCols) //所以靶标均抓取成功
    {
        writeLog(m_strLog, "所有小靶标均抓取成功");
        dScore += 40;
    }
    else
    {
        dScore += 20 + vecAccurateLocation.size() * 20 / m_nRows / m_nCols;
    }
    pHoleMatrixCenter = getMeanPoint(vecCircleCenter);
    return true;
}