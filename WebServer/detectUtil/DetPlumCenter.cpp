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
/// ���캯��
/// </summary>
/// <param name="strLog">��־�ļ�·��</param>
/// <param name="param">�б�����ṹ��</param>
/// <param name="algorithmPara">�㷨�����ṹ��</param>
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
/// ʹ�����ѧϰ�������ǿԲ���
/// ���ִ�СԲ�뾶
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
    writeLog(m_strLog, "Բ��ϼ��");
    if (m_nLittleDiameter > 50)
    {
        writeLog(m_strLog, "��뾶�⻷Բ���");
        if (getBigCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter, dScore))
        {
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" + to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" + to_string(vecLittleCenter.size()));
    }
    else
    {
        writeLog(m_strLog, "С�뾶Բ���");
        if (getLittleCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter))
        {
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" + to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" + to_string(vecLittleCenter.size()));
    }
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "СԲ�б�λ��ͼ");
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
    writeLog(m_strLog, "Բ��ϼ��");
    if (getLittleCircleCenterDLPrior(mDesImage, refinedRoi, vecLittleCenter))
    {
        writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" +
                               to_string(vecLittleCenter.size()));
        bIsSuccess = true;
        dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
    }
    else
        writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" +
                               to_string(vecLittleCenter.size()));
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "СԲ�б�λ��ͼ");
#endif
    return bIsSuccess;
}

CdetPlum &CdetPlum::operator=(const CdetPlum &)
{
    // TODO: �ڴ˴����� return ���
    return *this;
}

//**********************����÷���׼���㷨********************
/// <summary>
/// ���ÿһ��СԲ����
/// </summary>
/// <param name="mSrcImage">���ͼƬ</param>
/// <param name="vecLittleCenter">���ɹ���СԲ����</param>
/// <param name="dScore">�÷�</param>
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
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 2:
        writeLog(m_strLog, "�ֲ���ֵ�ָ�");
        if (getSegmentImageCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 3:
        writeLog(m_strLog, "Բ��ϼ��");
        if (getLittleCircleCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
        break;
    case 4:
        writeLog(m_strLog, "�����㷨");
        if (customLittleCircleCenter(mDesImage, vecLittleCenter))
        {
            writeLog(m_strLog, "����ȡ�ɹ�����ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
            bIsSuccess = true;
            dScore += (float(vecLittleCenter.size()) / float(m_nLittleCount)) * 40;
        }
        else
            writeLog(m_strLog, "����ȡʧ�ܣ���ȡ����СԲ�б����Ϊ��" +
                                   to_string(vecLittleCenter.size()));
        break;
    default:
        writeLog(m_strLog, "û�д��㷨");
        break;
    }
#if _DEBUG
    cv::Mat mShowImage = generateColorImage(mDesImage);
    for (int i = 0; i < vecLittleCenter.size(); ++i)
        circle(mShowImage, Point(vecLittleCenter[i].x, vecLittleCenter[i].y), vecLittleCenter[i].z, cv::Scalar(255, 0, 0), 1, 8, 0);
    CvWaitShowImage(mShowImage, "СԲ�б�λ��ͼ");
#endif
    return bIsSuccess;
}

/// <summary>
/// �㷨һ�������Ʋ�+�Զ���ֵ���
/// ���㷢�ɹ��ʺܸߣ����������Ҳ�ܸߣ�Ӧ��ʹ�ã����Ƽ�
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
    //��ȡÿһ��С�б����ڸ���Ȥ����
    getLittleCircleLocation(mDesImage, cv::Point2d(0, 0), 1, vecImage);
    //����У��
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
        CvWaitShowImage(mFilterImage, "��ֵͼ");
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
            //ͼ����ǿ
            enhance.getEnhanceImage(mFilterImage, 0, mFilterImage);
            //int nThreshold = OTSU(mFilterImage, 0, 80);
            /*cv::Mat mThresholdImage;*/
            //cv::threshold(mFilterImage, mFilterImage, nThreshold, 255, CV_THRESH_BINARY);
            cv::adaptiveThreshold(mFilterImage, mFilterImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
            mFilterImage = ~mFilterImage;

#if _DEBUG
            CvWaitShowImage(mFilterImage, "��ֵͼ");
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
        writeLog(m_strLog, "�ɹ�ץȡ����Ϊ:" + std::to_string(vecLittleCenter.size()));
        return true;
    }
    else
    {
        writeLog(m_strLog, "ERROR:�ɹ�ץȡ����Ϊ:" + std::to_string(vecLittleCenter.size()) + ",����Ԥ��ĳɹ���");
        return false;
    }
}

/// <summary>
/// �㷨һ�������Ʋ�+�Զ���ֵ���
/// ���㷢�ɹ��ʺܸߣ����������Ҳ�ܸߣ�Ӧ��ʹ�ã����Ƽ�
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getSegmentImageCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    return false;
}

/// <summary>
/// �㷨������ǿԲ���
/// Ĭ��ʹ�ô��㷨
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
    Point2f pContour(0, 0);            //��С���ԲԲ��
    cv::Point3d circleCenter(0, 0, 0); //��Ϻ��Բ��������
    float fRedius = 0;
    //�˲���ʼ��
    CFilter filter;
    //�ݶȼ���ʼ��
    CGradient getGradient;
    //��ǿ��ʼ��
    CEnhance enhance(m_bIsWhite);
    double dRows = 0, dCols = 0;
    std::vector<cv::Point2d> vecContours, vecOut;
    //��ȡÿһ��С�б����ڸ���Ȥ����
    getLittleCircleLocation(mDesImage, cv::Point2d(0, 0), 1, vecImage);
    double dDisRediusOne, dStepXOne, dStepYOne, dMaxDiffOne;
    //����У��
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
        //		CvWaitShowImage(mEnhanceImage, "��ǿͼƬ");
        //#endif
        //enhance.getEnhanceImage(mFilterImage, 4, mFilterImage, 0, 0, 400, 400);
        //			enhance.getEnhanceImage(mFilterImage, 1, mFilterImage);
        //#if _DEBUG
        //			CvWaitShowImage(mFilterImage, "��ǿͼƬ");
        //#endif
        //filter.getFilter(mEnhanceImage, 4, "3", mEnhanceImage);
        std::vector<cv::Point2d> vecGradient;
        mGradientImage = cv::Mat::zeros(mFilterImage.rows, mFilterImage.cols, CV_8UC1);
        //getGradient.getSubPixelGradient(mFilterImage, mGradientImage, vecGradient);
        getGradient.getGradientImage(mFilterImage, mGradientImage, 0);
        //��������
        ConnectEdge(mGradientImage, mGradientImage2);
        //		std::vector<cv::Point>vecContoursPoint;
        //		scanGradientPoint(mGradientImage2, false, vecContoursPoint);
        //#if _DEBUG
        //		cv::Mat mShowImage3 = drawColorImage(mFilterImage, vecContoursPoint, cv::Scalar(255, 0, 0));
        //		CvWaitShowImage(mShowImage3, "ɸѡ������");
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
        CvWaitShowImage(mShowImage, "����ͼ");
#endif
        if (contours.size() <= 0)
        {
            writeLog(m_strLog, "δ��⵽��������������");
            continue;
        }
        else
        {
            sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });

            //�������Բ���ɸѡ����
            for (int x = 0; x < contours.size(); ++x)
            {
                //������������
                Moments moment;            //��
                cv::Mat temp(contours[x]); //��һ������
                moment = moments(temp, false);
                cv::Point2d pt1;
                if (moment.m00 != 0) //��������Ϊ0
                {
                    pt1.x = cvRound(moment.m10 / moment.m00); //�������ĺ�����
                    pt1.y = cvRound(moment.m01 / moment.m00); //��������������
                }

                /*if (abs(vecImage[i].first.cols/2- pt1.x)>10 ||
						abs(vecImage[i].first.rows / 2 - pt1.y)>10)
						continue;*/
                //��С���Բ
                minEnclosingCircle(contours[x], pContour, fRedius);
                //float fLength = arcLength(contours[i], true);
                float fArea = contourArea(contours[x]);
                if (fRedius >= m_nLittleDiameter / 2 - m_nThreshold && fRedius <= m_nLittleDiameter / 2 + m_nThreshold &&
                    fArea > m_nLittleDiameter / 4 * m_nLittleDiameter / 4 * CV_PI && fArea < m_nLittleDiameter * m_nLittleDiameter * CV_PI)
                {
#if _DEBUG
                    cv::Mat mShowImage2 = generateColorImage(vecImage[i].first);
                    drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
                    CvWaitShowImage(mShowImage2, "����ͼ2");
#endif
                    //��ȷ����
                    //if (m_nDeepCalculation)
                    {
                        writeLog(m_strLog, "���þ�ȷ����");
                        //�����������������ղ�ͬ���޽��з��ദ��
                        scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
                        //�洢СԲ���ĵ�����
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
/// �㷨�ģ�ʹ�����ѧϰ�������ǿԲ���
/// �Զ������Ĭ��ʹ�ô��㷨
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
    Point2f pContour(0, 0);            //��С���ԲԲ��
    cv::Point3d circleCenter(0, 0, 0); //��Ϻ��Բ��������
    float fRedius = 0;
    //�˲���ʼ��
    CFilter filter;
    //�ݶȼ���ʼ��
    CGradient getGradient;
    //��ǿ��ʼ��
    CEnhance enhance(m_bIsWhite);
    double dRows = 0, dCols = 0;
    std::vector<cv::Point2d> vecContours, vecOut;

    double dDisRediusOne, dStepXOne, dStepYOne, dMaxDiffOne;
    //����У��
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
        //��������
        ConnectEdge(mGradientImage, mGradientImage2);
        findContours(mGradientImage2, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
#if _DEBUG
        cv::Mat mShowImage = generateColorImage(refinedRoi[i].first);
        for (int x = 0; x < contours.size(); ++x)
        {
            Scalar color(rand() & 255, rand() & 255, rand() & 255);
            drawContours(mShowImage, contours, x, color, 2, 8, hierarchy);
        }
        CvWaitShowImage(mShowImage, "����ͼ");
#endif
        if (contours.size() <= 0)
        {
            writeLog(m_strLog, "δ��⵽��������������");
            continue;
        }
        else
        {
            sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); });

            //�������Բ���ɸѡ����
            for (int x = 0; x < contours.size(); ++x)
            {
                //������������
                Moments moment;            //��
                cv::Mat temp(contours[x]); //��һ������
                moment = moments(temp, false);
                cv::Point2d pt1;
                if (moment.m00 != 0) //��������Ϊ0
                {
                    pt1.x = cvRound(moment.m10 / moment.m00); //�������ĺ�����
                    pt1.y = cvRound(moment.m01 / moment.m00); //��������������
                }

                /*if (abs(vecImage[i].first.cols/2- pt1.x)>10 ||
						abs(vecImage[i].first.rows / 2 - pt1.y)>10)
						continue;*/
                //��С���Բ
                minEnclosingCircle(contours[x], pContour, fRedius);
                //float fLength = arcLength(contours[i], true);
                float fArea = contourArea(contours[x]);
                if (fRedius >= m_nLittleDiameter / 2 - m_nThreshold && fRedius <= m_nLittleDiameter / 2 + m_nThreshold &&
                    fArea > m_nLittleDiameter / 4 * m_nLittleDiameter / 4 * CV_PI && fArea < m_nLittleDiameter * m_nLittleDiameter * CV_PI)
                {
#if _DEBUG
                    cv::Mat mShowImage2 = generateColorImage(refinedRoi[i].first);
                    drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
                    CvWaitShowImage(mShowImage2, "����ͼ2");
#endif
                    //��ȷ����
                    //if (m_nDeepCalculation)
                    {
                        writeLog(m_strLog, "���þ�ȷ����");
                        //�����������������ղ�ͬ���޽��з��ദ��
                        scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
                        //�洢СԲ���ĵ�����
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
/// ʹ�����ѧϰ�������ǿ��ԲԲ���
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="refinedRoi"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::getBigCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore)
{
    // ����ֱ�ӵ�Բ���
    DetectParameterV3 param;
    ConfigAlgorithmParam algorithmPara;
    param.Diameter[0] = m_nLittleDiameter;
    param.Threshold = 15;
    param.IsWhite = m_bIsWhite;
    CdetCircle getCenter(m_strLog, param, algorithmPara);
    cv::Point3d CenterCirclePoint(0, 0, 0); //���Բ���ĵ�
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
/// �����㷨����
/// �����ڼ�����ķ���İ�ɫСԲ������СԲֱ��Ҫ��ϸ�
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="vecLittleCenter"></param>
/// <returns></returns>
bool CdetPlum::customLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter)
{
    return false;
}

/// <summary>
/// �ֶ�λСԲͼƬ
/// </summary>
/// <param name="mSrcImage">�����ͼƬ</param>
/// <param name="pCircleCenter">�ο����ĵ�����</param>
/// <param name="nType">��ȡСԲ���귽ʽ</param>
/// <param name="vecMap">ÿ��СԲ�������</param>
/// <returns></returns>
int CdetPlum::getLittleCircleLocation(cv::Mat mSrcImage, cv::Point2d pCircleCenter, int nType, std::vector<std::pair<cv::Mat, cv::Point>> &vecMap)
{
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    cv::Point pLittleCircleLocation(0, 0);
    double dRows = 0, dCols = 0;
    //��ȡ��������СԲ֮��ļ�����
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
            //С����ȡÿһ��С�б�λ��
            nMinDisX = getMinValue(nValueX, 3);
            nMinDisY = getMinValue(nValueY, 3);
            pLittleCircleLocation.x = xd - nMinDisX;
            pLittleCircleLocation.y = yd - nMinDisY;
            dRows = nMinDisY * 2;
            dCols = nMinDisX * 2;
            break;
        case 2:
            //�����ȡÿһ��С�б�λ��
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
/// �����������ɾѡ
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
    double dB2 = 0; //Բ���ϱ�Ե�㵽Բ��ֱ�߲���
    double dDisRediusOne = 0;
    double dDisRediusTwo = 0; //�����㵽Բ�ľ���
    double dCheckDistance = 0;
    //�������㰴�������޽��з���
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
    CvWaitShowImage(mQuadrantImage, "����������");
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
                    //ÿ2����ѡȡһ��������
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
    CvWaitShowImage(mShowImage, "ɾѡ���������");
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

//��ȡСԲ����֮�����Ѽ��
int CdetPlum::getOptimumCenterInterval()
{
    double angleRotate = 2 * CV_PI / m_nLittleCount;
    double dX1 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * sin(angleRotate);
    double dY1 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * cos(angleRotate);
    double dX2 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * sin(angleRotate * 0);
    double dY2 = (m_nBigDiameter / 2 - m_nLittleDiameter / 2) * cos(angleRotate * 0);

    return sqrt(pow(dX1 - dX2, 2) + pow(dY1 - dY2, 2)) - m_nLittleDiameter;
}
