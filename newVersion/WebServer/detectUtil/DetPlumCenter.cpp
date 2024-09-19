#include "DetPlumCenter.h"
#include "paraType.h"
#include "Common.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "EnhanceImage.h"
#include "LogHelp.h"
#include "LeastSquare.h"
#include "Edge.h"
#include "DetCircleCenter.h"
#include "LogHelp.h"

using namespace std;
using namespace cv;

/// <summary>
/// 构造函数
/// </summary>
/// <param name="strLog">日志文件路径</param>
/// <param name="param">靶标参数结构体</param>
/// <param name="algorithmPara">算法参数结构体</param>
CdetPlum::CdetPlum(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara)
{
    m_strLog = strLog;
    m_nBigDiameter = param.Diameter[0];
    m_nLittleDiameter = param.Diameter[1];
    m_nThreshold = param.Threshold;
    m_bIsWhite = param.IsWhite;
    m_nAlgorithm = algorithmPara.nMarkAlgorithm;
    m_nLittleCount = param.nLittleCount;
    m_bIsPolarChange = algorithmPara.IsPolarChange;
    m_bIsEnhance = algorithmPara.IsEnhanceMap;
    m_dCircleRate = algorithmPara.dLittleCircleRate * param.nLittleCount;
}

CdetPlum::~CdetPlum()
{
}

/// <summary>
/// 使用深度学习先验的增强圆拟合
/// 区分大小圆半径
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="refinedRoi"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    bool bIsSuccess = false;
    vecLittleCenter.clear();
    writeLog(m_strLog, "圆拟合检测");
    if (m_nLittleDiameter > 50)
    {
        writeLog(m_strLog, "大半径外环圆拟合");
        if (getBigCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter, dScore))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" + to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" + to_string(vecLittleCenter.size()));
    }
    else
    {
        writeLog(m_strLog, "小半径圆拟合");
        if (getLittleCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" + to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" + to_string(vecLittleCenter.size()));
    }
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "小圆靶标位置图");
#endif
    return bIsSuccess;
}

bool CdetPlum::getLittleObjectCenterDirect(cv::Mat mSrcImage, const std::vector<pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    //std::vector<cv::Point2d> vecLittleCenter;
    bool bIsSuccess = false;
    vecLittleCenter.clear();
    writeLog(m_strLog, "圆拟合检测");
    if (getLittleCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter))
    {
        writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" +
                               to_string(vecLittleCenter.size()));
        bIsSuccess = true;
        dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
    }
    else
        writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" +
                               to_string(vecLittleCenter.size()));
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "小圆靶标位置图");
#endif
    return bIsSuccess;
}

CdetPlum &CdetPlum::operator=(const CdetPlum &)
{
    // TODO: 在此处插入 return 语句
    return *this;
}

//**********************环形梅花孔检测算法********************
/// <summary>
/// 检测每一个小圆坐标
/// </summary>
/// <param name="mSrcImage">检测图片</param>
/// <param name="vecLittleCenter">检测成功的小圆坐标</param>
/// <param name="dScore">得分</param>
/// <returns></returns>
bool CdetPlum::getLittleObjectCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    //std::vector<cv::Point2d> vecLittleCenter;
    bool bIsSuccess = false;
    vecLittleCenter.clear();
    switch (m_nAlgorithm)
    {
    case 1:
        if (getDistancePredictCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 2:
        writeLog(m_strLog, "局部阈值分割");
        if (getSegmentImageCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 3:
        writeLog(m_strLog, "圆拟合检测");
        if (getLittleCircleCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 4:
        writeLog(m_strLog, "定制算法");
        if (customLittleCircleCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "检测获取成功，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "检测获取失败，获取到的小圆靶标个数为：" +
                                   to_string(vecLittleCenter.size()));
        break;
    default:
        writeLog(m_strLog, "没有此算法");
        break;
    }
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "小圆靶标位置图");
#endif
    return bIsSuccess;
}

/// <summary>
/// 算法一：距离推测+自动阈值拟合
/// 此算发成功率很高，但是误检率也很高，应急使用，不推荐
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getDistancePredictCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    cv::Point pLittleCircleLocation;
    cv::Mat mFilterImage;
    std::vector<std::pair<cv::Mat, cv::Point>> vecImage;
    std::vector<std::pair<cv::Mat, cv::Point>> vecImage2;
    vector<vector<Point>> opencvContours;
    vector<Vec4i> opencvHierarchy;
    Point2f pContour(0, 0);
    float fRedius = 0;
    CFilter filter;
    CGradient getGradient;
    CEnhance enhance;
    double dRows = 0, dCols = 0;
    //获取每一个小靶标所在感兴趣区域
    getLittleCircleLocation(mDesImage, cv::Point2d(0, 0), 1, vecImage);
    //区域校验
    if (vecImage.size() != m_nLittleCount)
        return false;
    for (int i = 0; i < vecImage.size(); ++i)
    {
        filter.getFilter(vecImage[i].first, 5, "11", mFilterImage);
        //int nThreshold = OTSU(mFilterImage, 0, 80);
        /*cv::Mat mThresholdImage;*/
        //cv::threshold(mFilterImage, mFilterImage, nThreshold, 255, CV_THRESH_BINARY);
        cv::adaptiveThreshold(mFilterImage, mFilterImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
        mFilterImage = ~mFilterImage;

#if _DEBUG
        CvWaitShowImage(mFilterImage, "二值图");
#endif
        opencvContours.clear();
        opencvHierarchy.clear();
        findContours(mFilterImage, opencvContours, opencvHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (opencvContours.size() == 0)
        {
            vecImage2.push_back(make_pair(vecImage[i].first, vecImage[i].second));
            continue;
        }

        sort(opencvContours.begin(), opencvContours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });
        /*	std::vector<cv::Point>vecContours;
			mergeVector(opencvContours, vecContours, 3);*/
        /*Point2f pContour(0, 0);
			float fRedius = 0;*/
        if (opencvContours.size() == 1)
        {
            minEnclosingCircle(opencvContours[0], pContour, fRedius);

            if (fRedius >= m_nLittleDiameter / 2 * 0.5 && fRedius <= (m_nLittleDiameter / 2 + 5) * 2)
            {
                vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage[i].second.x, pContour.y + vecImage[i].second.y, fRedius));
            }
            else
                vecImage2.push_back(make_pair(vecImage[i].first, vecImage[i].second));
        }
        else
        {
            std::vector<cv::Point> vecContours;
            mergeContours(opencvContours, cv::Point(mFilterImage.cols / 2, mFilterImage.rows / 2), m_nLittleDiameter / 2, vecContours);
            if (vecContours.size() == 0)
            {
                vecImage2.push_back(make_pair(vecImage[i].first, vecImage[i].second));
                continue;
            }
            minEnclosingCircle(vecContours, pContour, fRedius);

            if (fRedius >= m_nLittleDiameter / 2 * 0.5 && fRedius <= (m_nLittleDiameter / 2 + 3) * 1.5)
            {
                vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage[i].second.x, pContour.y + vecImage[i].second.y, fRedius));
            }
            else
                vecImage2.push_back(make_pair(vecImage[i].first, vecImage[i].second));
        }
    }
    if (vecImage2.size() != 0)
    {
        for (int i = 0; i < vecImage2.size(); ++i)
        {
            filter.getFilter(vecImage2[i].first, 5, "11", mFilterImage);
            //图像增强
            enhance.getEnhanceImage(mFilterImage, 0, mFilterImage);
            //int nThreshold = OTSU(mFilterImage, 0, 80);
            /*cv::Mat mThresholdImage;*/
            //cv::threshold(mFilterImage, mFilterImage, nThreshold, 255, CV_THRESH_BINARY);
            cv::adaptiveThreshold(mFilterImage, mFilterImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
            mFilterImage = ~mFilterImage;

#if _DEBUG
            CvWaitShowImage(mFilterImage, "二值图");
#endif
            opencvContours.clear();
            opencvHierarchy.clear();
            findContours(mFilterImage, opencvContours, opencvHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (opencvContours.size() == 0)
            {
                continue;
            }

            sort(opencvContours.begin(), opencvContours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });
            /*	std::vector<cv::Point>vecContours;
				mergeVector(opencvContours, vecContours, 3);*/
            /*Point2f pContour(0, 0);
				float fRedius = 0;*/
            if (opencvContours.size() == 1)
            {
                minEnclosingCircle(opencvContours[0], pContour, fRedius);

                if (fRedius >= m_nLittleDiameter / 2 * 0.5 && fRedius <= (m_nLittleDiameter / 2 + 5) * 2)
                {
                    vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage2[i].second.x, pContour.y + vecImage2[i].second.y, fRedius));
                }
            }
            else
            {
                std::vector<cv::Point> vecContours;
                mergeContours(opencvContours, cv::Point(mFilterImage.cols / 2, mFilterImage.rows / 2), m_nLittleDiameter / 2, vecContours);
                if (vecContours.size() == 0)
                {
                    continue;
                }
                minEnclosingCircle(vecContours, pContour, fRedius);

                if (fRedius >= m_nLittleDiameter / 2 * 0.5 && fRedius <= (m_nLittleDiameter / 2 + 3) * 1.5)
                {
                    vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage2[i].second.x, pContour.y + vecImage2[i].second.y, fRedius));
                }
            }
        }
    }
    if (vecLittleCenter.size() > m_dCircleRate && vecLittleCenter.size() >= 3)
    {
        writeLog(m_strLog, "成功抓取个数为:" + std::to_string(vecLittleCenter.size()));
        return true;
    }
    else
    {
        writeLog(m_strLog, "ERROR:成功抓取个数为:" + std::to_string(vecLittleCenter.size()) + ",低于预设的成功比");
        return false;
    }
}

/// <summary>
/// 算法一：距离推测+自动阈值拟合
/// 此算发成功率很高，但是误检率也很高，应急使用，不推荐
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getSegmentImageCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    return false;
}

/// <summary>
/// 算法三：增强圆拟合
/// 默认使用此算法
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    cv::Point pLittleCircleLocation;
    cv::Mat mFilterImage, mEnhanceImage, mGradientImage, mGradientImage2;
    std::vector<std::pair<cv::Mat, cv::Point>> vecImage;
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
    //获取每一个小靶标所在感兴趣区域
    getLittleCircleLocation(mDesImage, cv::Point2d(0, 0), 1, vecImage);
    double dDisRediusOne, dStepXOne, dStepYOne, dMaxDiffOne;
    //区域校验
    if (vecImage.size() != m_nLittleCount)
        return false;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    for (int i = 0; i < vecImage.size(); ++i)
    {
        filter.getFilter(vecImage[i].first, 3, "2", mFilterImage);
        filter.getFilter(mFilterImage, 4, "3", mFilterImage);
        //		enhance.getEnhanceImage(mFilterImage, 3, mEnhanceImage);
        //		//double dMeanThreshold = getEnhanceThreshold(mFilterImage);
        //		//enhance.getLocalEnhance(mFilterImage, 1, 1, 0, 1, mEnhanceImage, true);
        //#if _DEBUG
        //		CvWaitShowImage(mEnhanceImage, "增强图片");
        //#endif
        //enhance.getEnhanceImage(mFilterImage, 4, mFilterImage, 0, 0, 400, 400);
        //			enhance.getEnhanceImage(mFilterImage, 1, mFilterImage);
        //#if _DEBUG
        //			CvWaitShowImage(mFilterImage, "增强图片");
        //#endif
        //filter.getFilter(mEnhanceImage, 4, "3", mEnhanceImage);
        std::vector<cv::Point2d> vecGradient;
        mGradientImage = cv::Mat::zeros(mFilterImage.rows, mFilterImage.cols, CV_8UC1);
        //getGradient.getSubPixelGradient(mFilterImage, mGradientImage, vecGradient);
        getGradient.getGradientImage(mFilterImage, mGradientImage, 0);
        //轮廓连接
        ConnectEdge(mGradientImage, mGradientImage2);
        //		std::vector<cv::Point>vecContoursPoint;
        //		scanGradientPoint(mGradientImage2, false, vecContoursPoint);
        //#if _DEBUG
        //		cv::Mat mShowImage3 = drawColorImage(mFilterImage, vecContoursPoint, cv::Scalar(255, 0, 0));
        //		CvWaitShowImage(mShowImage3, "筛选轮廓点");
        //#endif
        /*	MattoPoint(mGradientImage, true, mGradientImage.cols, mGradientImage.cols / 2, vecContours);
					MattoPoint(mGradientImage2, true, mGradientImage.cols, mGradientImage.cols / 2, vecOut);*/
        findContours(mGradientImage2, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
#if _DEBUG
        cv::Mat mShowImage = generateColorImage(vecImage[i].first);
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
            sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });

            //根据外接圆面积筛选轮廓
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

                /*if (abs(vecImage[i].first.cols/2- pt1.x)>10 ||
						abs(vecImage[i].first.rows / 2 - pt1.y)>10)
						continue;*/
                //最小外接圆
                minEnclosingCircle(contours[x], pContour, fRedius);
                //float fLength = arcLength(contours[i], true);
                float fArea = contourArea(contours[x]);
                if (fRedius >= m_nLittleDiameter / 2 - m_nThreshold && fRedius <= m_nLittleDiameter / 2 + m_nThreshold &&
                    fArea > m_nLittleDiameter / 4 * m_nLittleDiameter / 4 * CV_PI && fArea < m_nLittleDiameter * m_nLittleDiameter * CV_PI)
                {
#if _DEBUG
                    cv::Mat mShowImage2 = generateColorImage(vecImage[i].first);
                    drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
                    CvWaitShowImage(mShowImage2, "轮廓图2");
#endif
                    //精确计算
                    //if (m_nDeepCalculation)
                    {
                        writeLog(m_strLog, "启用精确计算");
                        //满足条件的轮廓按照不同象限进行分类处理
                        scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
                        //存储小圆中心点坐标
                        vecLittleCenter.push_back(cv::Point3d(circleCenter.x + vecImage[i].second.x, circleCenter.y + vecImage[i].second.y, fRedius));
                    }
                    /*else
							vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage[i].second.x, pContour.y + vecImage[i].second.y, fRedius));*/
                    break;
                }
            }
        }
    }
    if (vecLittleCenter.size() >= 3)
        return true;
    else
        return false;
}

/// <summary>
/// 算法四：使用深度学习先验的增强圆拟合
/// 自动检测下默认使用此算法
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getLittleCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    cv::Point pLittleCircleLocation;
    cv::Mat mFilterImage, mEnhanceImage, mGradientImage, mGradientImage2;
    //std::vector<std::pair<cv::Mat, cv::Point>>vecImage;
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
    //区域校验
    if (refinedRoi.size() != m_nLittleCount)
        return false;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    for (int i = 0; i < refinedRoi.size(); ++i)
    {
        filter.getFilter(refinedRoi[i].first, 3, "2", mFilterImage);
        filter.getFilter(mFilterImage, 4, "3", mFilterImage);

        std::vector<cv::Point2d> vecGradient;
        mGradientImage = cv::Mat::zeros(mFilterImage.rows, mFilterImage.cols, CV_8UC1);
        getGradient.getGradientImage(mFilterImage, mGradientImage, 0);
        //轮廓连接
        ConnectEdge(mGradientImage, mGradientImage2);
        findContours(mGradientImage2, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
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
            sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });

            //根据外接圆面积筛选轮廓
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

                /*if (abs(vecImage[i].first.cols/2- pt1.x)>10 ||
						abs(vecImage[i].first.rows / 2 - pt1.y)>10)
						continue;*/
                //最小外接圆
                minEnclosingCircle(contours[x], pContour, fRedius);
                //float fLength = arcLength(contours[i], true);
                float fArea = contourArea(contours[x]);
                if (fRedius >= m_nLittleDiameter / 2 - m_nThreshold && fRedius <= m_nLittleDiameter / 2 + m_nThreshold &&
                    fArea > m_nLittleDiameter / 4 * m_nLittleDiameter / 4 * CV_PI && fArea < m_nLittleDiameter * m_nLittleDiameter * CV_PI)
                {
#if _DEBUG
                    cv::Mat mShowImage2 = generateColorImage(refinedRoi[i].first);
                    drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
                    CvWaitShowImage(mShowImage2, "轮廓图2");
#endif
                    //精确计算
                    //if (m_nDeepCalculation)
                    {
                        writeLog(m_strLog, "启用精确计算");
                        //满足条件的轮廓按照不同象限进行分类处理
                        scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
                        //存储小圆中心点坐标
                        vecLittleCenter.push_back(cv::Point3d(circleCenter.x + refinedRoi[i].second.x, circleCenter.y + refinedRoi[i].second.y, fRedius));
                    }
                    /*else
							vecLittleCenter.push_back(cv::Point3d(pContour.x + vecImage[i].second.x, pContour.y + vecImage[i].second.y, fRedius));*/
                    break;
                }
            }
        }
    }
    if (vecLittleCenter.size() >= 3)
        return true;
    else
        return false;
}

/// <summary>
/// 使用深度学习先验的增强大圆圆拟合
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="refinedRoi"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getBigCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore)
{
    // 这里直接调圆拟合
    DetectParameterV3 param;
    ConfigAlgorithmParam algorithmPara;
    param.Diameter[0] = m_nLittleDiameter;
    param.Threshold = 15;
    param.IsWhite = m_bIsWhite;
    CdetCircle getCenter(m_strLog, param, algorithmPara);
    cv::Point3d CenterCirclePoint(0, 0, 0); //检测圆中心点
    int nSuccessNum = 0;
    for (auto roi : refinedRoi)
    {
        if (getCenter.getCircleCenter(roi.first, false, CenterCirclePoint, dScore))
            vecLittleCenter.push_back(cv::Point3d(CenterCirclePoint.x + roi.second.x, CenterCirclePoint.y + roi.second.y, CenterCirclePoint.z));
    }
    if (vecLittleCenter.size() > 3)
        return true;
    return false;
}

/// <summary>
/// 定制算法尝试
/// 适用于检测中心反光的白色小圆，对于小圆直径要求较高
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::customLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    return false;
}

/// <summary>
/// 粗定位小圆图片
/// </summary>
/// <param name="mSrcImage">待检测图片</param>
/// <param name="pCircleCenter">参考中心点坐标</param>
/// <param name="nType">获取小圆坐标方式</param>
/// <param name="vecMap">每个小圆区域参数</param>
/// <returns></returns>
int CdetPlum::getLittleCircleLocation(cv::Mat mSrcImage, cv::Point2d pCircleCenter, int nType, std::vector<std::pair<cv::Mat, cv::Point>> &vecMap)
{
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    cv::Point pLittleCircleLocation(0, 0);
    double dRows = 0, dCols = 0;
    //获取相邻两个小圆之间的间距距离
    int nDis = getOptimumCenterInterval();
    for (int nc = 0; nc < m_nLittleCount; nc++)
    {
        double yd = 0, xd = 0;
        if (pCircleCenter.x != 0 && pCircleCenter.y != 0)
        {
            //if (m_bHalfAngleRoate)
            //{
            //	yd = pCircleCenter.y + m_nDistanceCircleCenter * sin((angleRotate * nc) + angleRotate / 2);
            //	xd = pCircleCenter.x + m_nDistanceCircleCenter * cos((angleRotate * nc) + angleRotate / 2);
            //}
            //else
            //{
            //	yd = pCircleCenter.y + m_nDistanceCircleCenter * sin(angleRotate * nc);
            //	xd = pCircleCenter.x + m_nDistanceCircleCenter * cos(angleRotate * nc);
            //}
            yd = pCircleCenter.y + (m_nBigDiameter / 2) * sin(angleRotate * nc);
            xd = pCircleCenter.x + (m_nBigDiameter / 2) * cos(angleRotate * nc);
        }
        else
        {
            //if (m_bHalfAngleRoate)
            //{
            //	yd = mSrcImage.rows / 2 + m_nDistanceCircleCenter * sin((angleRotate * nc) + angleRotate / 2);
            //	xd = mSrcImage.cols / 2 + m_nDistanceCircleCenter * cos((angleRotate * nc) + angleRotate / 2);
            //}
            //else
            //{
            //	yd = mSrcImage.rows / 2 + m_nDistanceCircleCenter * sin(angleRotate * nc);
            //	xd = mSrcImage.cols / 2 + m_nDistanceCircleCenter * cos(angleRotate * nc);
            //}
            yd = mSrcImage.rows / 2 + (m_nBigDiameter / 2) * sin(angleRotate * nc);
            xd = mSrcImage.cols / 2 + (m_nBigDiameter / 2) * cos(angleRotate * nc);
        }
        int nValueX[10] = {xd, (m_nLittleDiameter + m_nThreshold * 2) / 2, mSrcImage.cols - xd};
        int nMinDisX = 0;
        int nValueY[10] = {yd, (m_nLittleDiameter + m_nThreshold * 2) / 2, mSrcImage.rows - yd};
        int nMinDisY = 0;
        switch (nType)
        {
        case 1:
            //小间距获取每一个小靶标位置
            nMinDisX = getMinValue(nValueX, 3);
            nMinDisY = getMinValue(nValueY, 3);
            pLittleCircleLocation.x = xd - nMinDisX;
            pLittleCircleLocation.y = yd - nMinDisY;
            dRows = nMinDisY * 2;
            dCols = nMinDisX * 2;
            break;
        case 2:
            //大间距获取每一个小靶标位置
            break;
        default:
            break;
        }
        if (pLittleCircleLocation.x != 0 || pLittleCircleLocation.y != 0)
        {
            cv::Mat mLittleImage = mDesImage(Rect(pLittleCircleLocation.x, pLittleCircleLocation.y, dRows, dCols));
            vecMap.push_back(make_pair(mLittleImage, pLittleCircleLocation));
        }
    }
    return 0;
}

/// <summary>
/// 轮廓点分象限删选
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecInput"></param>
/// <param name="pRoughCenter"></param>
/// <param name="pCenter"></param>
/// <returns></returns>
int CdetPlum::scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput, cv::Point2d pRoughCenter, cv::Point3d &pCenter)
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
        if (vecRough[i].size() > 1)
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

//获取小圆区域之间的最佳间距
int CdetPlum::getOptimumCenterInterval()
{
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    double dX1 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * sin(angleRotate);
    double dY1 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * cos(angleRotate);
    double dX2 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * sin(angleRotate * 0);
    double dY2 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * cos(angleRotate * 0);

    return sqrt(pow(dX1 - dX2, 2) + pow(dY1 - dY2, 2)) - m_nLittleDiameter;
}
