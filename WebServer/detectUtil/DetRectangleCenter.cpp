#include "DetRectangleCenter.h"
#include "Common.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "LogHelp.h"
#include "ListSort.h"
#include "LogHelp.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "FitLine.h"
#include "FilterImage.h"
#include "EnhanceImage.h"
#include <algorithm>
#include "Threshold.h"

inline bool sortContour(const std::vector<cv::Point> a, const std::vector<cv::Point> b)
{
    return a.size() > b.size();
}
/// <summary>
/// ���캯��
/// </summary>
/// <param name="strLog">��־�ļ�·��</param>
/// <param name="param">�б�����ṹ��</param>
/// <param name="algorithmPara">�㷨�����ṹ��</param>
CdetRectangle::CdetRectangle(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara)
{
    m_strLog = strLog;
    m_nWidth = param.Width[0];
    m_nHeight = param.Height[0];
    m_bIsWhite = param.IsWhite;
    m_nAlgorithm = algorithmPara.nMarkAlgorithm;
}

CdetRectangle::~CdetRectangle()
{
}

/// <summary>
/// ���μ���㷨
/// </summary>
/// <param name="mSrcImage">����ͼ��</param>
/// <param name="bIsCalibration">�궨</param>
/// <param name="nRoateAngle">��ת�Ƕ�</param>
/// <param name="pCenterPoint">��⵽�ľ��β���</param>
/// <param name="dScore">���÷�</param>
/// <returns></returns>
bool CdetRectangle::getRectangleCenter(const cv::Mat mSrcImage, bool bIsCalibration, int nRoateAngle,
                                       Cfmee_Rectangle &pCenterPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    Cfmee_Rectangle pRoughCenterPoint;
    if (false)
    {
        //��ת�Ƕȹ�����������μ��
        //writeLog(m_strLog, "��ת�ϴ�,�������μ��");
    }
    else
    {
        //writeLog(m_strLog, "��ת��С�����μ��");
        if (bIsCalibration)
        {
            //writeLog(m_strLog, "DMD�궨����");
        }
        else
        {
            //writeLog(m_strLog, "����׼����");
            switch (m_nAlgorithm)
            {
            case 0:
                //writeLog(m_strLog, "ʹ�ñ��߼��");
                if (getFitEdgeLine2(mDesImage, 80, pRoughCenterPoint, dScore))
                {
                    sortRectanglePoint(pRoughCenterPoint);
                    pCenterPoint = pRoughCenterPoint;
                    return true;
                }
                break;
            case 1:
                //writeLog(m_strLog, "ʹ���������");

                if (getContoursCenter(mDesImage, pRoughCenterPoint, dScore))
                {
                    sortRectanglePoint(pRoughCenterPoint);
                    pCenterPoint = pRoughCenterPoint;
                    return true;
                }
                break;
            case 2:
                //writeLog(m_strLog, "ʹ��1,2�㷨���");
                if (getFitEdgeLine2(mDesImage, nRoateAngle, pRoughCenterPoint, dScore))
                {
                    sortRectanglePoint(pRoughCenterPoint);
                    pCenterPoint = pRoughCenterPoint;
                    return true;
                }
                else if (getContoursCenter(mDesImage, pRoughCenterPoint, dScore))
                {
                    sortRectanglePoint(pRoughCenterPoint);
                    pCenterPoint = pRoughCenterPoint;
                    return true;
                }
            case 3:
                //writeLog(m_strLog, "�ֶ�������̣�ʹ�������ޱ������");
                if (fourQuadrantLineFit(mDesImage, nRoateAngle, pRoughCenterPoint, dScore))
                {
                    sortRectanglePoint(pRoughCenterPoint);
                    pCenterPoint = pRoughCenterPoint;
                    return true;
                }
                break;
            default:
                break;
            }
        }
    }
    return false;
}

/// <summary>
/// ����������������ϵ�
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="nRoateAngle"></param>
/// <param name="pCenterPoint"></param>
/// <param name="dScore"></param>
/// <returns></returns>
bool CdetRectangle::fourQuadrantLineFit(const cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore)
{

    if (!mSrcImage.data)
        return false;
    CFilter filter;
    CGradient gradient;
    CEnhance enhance;
    cv::Mat mFilterImage, mGradientImage, mEnhanceImage, mDesImage;
    mSrcImage.copyTo(mDesImage);
    //�˲�
    filter.getFilter(mDesImage, 3, "11", mFilterImage);
    filter.getFilter(mFilterImage, 4, "3", mFilterImage);
    //���������
    std::vector<std::pair<cv::Mat, cv::Point>> vecLineMap;
    // ���Ķ���Ϊ����
    int height = mFilterImage.rows, width = mFilterImage.cols;
    vecLineMap.push_back(std::make_pair(mFilterImage(cv::Rect(cv::Point(0, 0), cv::Point(width / 2, height * 3 / 4))), cv::Point(0, 0)));
    vecLineMap.push_back(std::make_pair(mFilterImage(cv::Rect(cv::Point(width / 4, 0), cv::Point(width - 1, height / 2))), cv::Point(width / 4, 0)));
    vecLineMap.push_back(std::make_pair(mFilterImage(cv::Rect(cv::Point(width / 2, height / 4), cv::Point(width - 1, height - 1))), cv::Point(width / 2, height / 4)));
    vecLineMap.push_back(std::make_pair(mFilterImage(cv::Rect(cv::Point(0, height / 2), cv::Point(width * 3 / 4, height - 1))), cv::Point(0, height / 2)));

#ifdef _DEBUG
    cv::Mat drawImage;
    mFilterImage.copyTo(drawImage);
    for (auto line : vecLineMap)
        CvWaitShowImage(line.first, "ֱ������");
#endif // _DEBUG \
    // ����ֱ�����
    if (vecLineMap.size() != 4)
    {
        //writeLog(m_strLog, "ERROR:�������ָ�ʧ��");
        return false;
    }
    std::vector<std::vector<cv::Point2d>> vecline;
    double dA = 0, dB = 0, dC = 0;
    for (int i = 0; i < 4; ++i)
    {
        cv::Vec4f line_para;
        std::vector<cv::Point2d> twoPoint;
        if (!getLineCenter(vecLineMap[i].first, line_para, dA, dB, dC))
            return false;
        if (dA != 0 || dB != 0)
            twoPoint = transLinePara(vecLineMap[i].first, dA, dB, dC);
        else if (dA == 0 && dB == 0 && dC == 0)
        {
            //writeLog(m_strLog, "ERROR:ֱ����Ͻ��������ֹ���");
            return false;
        }
        else
            twoPoint = transCVLinePara(vecLineMap[i].first, line_para);
#if _DEBUG
        cv::Mat mLineImage = generateColorImage(vecLineMap[i].first);
        line(mLineImage, cv::Point(twoPoint[0].x, twoPoint[0].y), cv::Point(twoPoint[1].x, twoPoint[1].y), cv::Scalar(0, 0, 255), 1, 8);
        CvWaitShowImage(mLineImage, "���ֱ��ͼƬ");
#endif
        for (int x = 0; x < 2; ++x)
        {
            twoPoint[x].x = twoPoint[x].x + vecLineMap[i].second.x;
            twoPoint[x].y = twoPoint[x].y + vecLineMap[i].second.y;
        }
        vecline.push_back(twoPoint);
    }

    std::vector<cv::Point2d> vecCrossPoint;
    //��ȡֱ�߽���
    //writeLog(m_strLog, "����ֱ�߽���");
    for (int i = 0; i < 4; ++i)
    {
        cv::Point2d crossPoint = CrossPoint(vecline[i % 4][0], vecline[i % 4][1], vecline[(i + 1) % 4][0], vecline[(i + 1) % 4][1]);
        vecCrossPoint.push_back(crossPoint);
        // //writeLog(m_strLog, "��������" + to_string(i % 4) + "_" + to_string((i + 1) % 4) + "Ϊ����" +
        //                        to_string(crossPoint.x) + "," + to_string(crossPoint.y) + ")");
    }
#if _DEBUG
    cv::Mat mColorImage = generateColorImage(mSrcImage);
#else
#endif
    for (int i = 0; i < vecCrossPoint.size(); ++i)
    {
        pCenterPoint.dCenterX += vecCrossPoint[i].x;
        pCenterPoint.dCenterY += vecCrossPoint[i].y;
        //fourPoint.push_back(cv::Point(vecCrossPoint[i].x, vecCrossPoint[i].y));
#if _DEBUG
        circle(mColorImage, cv::Point(vecCrossPoint[i].x, vecCrossPoint[i].y), 3, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
    CvWaitShowImage(mColorImage, "������ʾ");
#else
    }
#endif
    pCenterPoint.dCenterX /= vecCrossPoint.size();
    pCenterPoint.dCenterY /= vecCrossPoint.size();
    if (pCenterPoint.dCenterX != 0 && pCenterPoint.dCenterY != 0)
        return true;
    else
        return false;
    return false;
}

bool CdetRectangle::getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c)
{
    cv::Mat mDesImage;
    CEnhance enhance(m_bIsWhite);
    enhance.getEnhanceImage(mSrcImage, 0, mDesImage);
    mSrcImage.copyTo(mDesImage);
    //enhance.getEnhanceImage(mDesImage, 4, mDesImage, 0, 0, 400, 10);
#if _DEBUG
    CvWaitShowImage(mDesImage, "ֱ������");
#endif
    std::vector<cv::Point> vecPoint;
    if (mDesImage.cols > mDesImage.rows) //����
    {
        for (int x = 5; x < mDesImage.cols - 5; ++x)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            for (int y = 3; y < mDesImage.rows - 3; ++y)
            {
                double dDiff = abs(mDesImage.at<uchar>(y - 1, x) + mDesImage.at<uchar>(y - 2, x) - mDesImage.at<uchar>(y + 1, x) - mDesImage.at<uchar>(y + 2, x));
                if (dDiff > nMax)
                {
                    if (mDesImage.at<uchar>(y, x) < mDesImage.at<uchar>(y + 1, x))
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                    }
                    else
                    {
                        pIndex.x = x;
                        pIndex.y = y + 1;
                    }
                    nMax = dDiff;
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vecPoint.push_back(pIndex);
        }
    }
    else //����
    {
        for (int y = 5; y < mDesImage.rows - 5; ++y)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            for (int x = 3; x < mDesImage.cols - 3; ++x)
            {
                double dDiff = abs(mDesImage.at<uchar>(y, x - 1) + mDesImage.at<uchar>(y, x - 2) - mDesImage.at<uchar>(y, x + 1) - mDesImage.at<uchar>(y, x + 2));
                if (dDiff > nMax)
                {
                    if (mDesImage.at<uchar>(y, x) < mDesImage.at<uchar>(y, x + 1))
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                    }
                    else
                    {
                        pIndex.x = x + 1;
                        pIndex.y = y;
                    }
                    nMax = dDiff;
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vecPoint.push_back(pIndex);
        }
    }
    double dSigma = getSigmaDis(vecPoint);
    //ֱ�����
    double dA = 0, dB = 0, dC = 0;
    std::vector<cv::Point> vecLast;
    if (vecPoint.size() >= (std::min(m_nWidth, m_nHeight) / 2) * 0.8 && lineFit(vecPoint, dA, dB, dC))
    {
        vecLast = scanPoint(vecPoint, dA, dB, dC);
        if (vecLast.size() > vecPoint.size() * 0.5 && (dA != 0 || dB != 0))
        {
            a = dA;
            b = dB;
            c = dC;
        }
        else
        {
            //writeLog(m_strLog, "ERROR:ֱ����ϼ��ʧ��");
            return false;
        }
    }
    else
    {
        //writeLog(m_strLog, "ERROR:����ϵ�ֱ�ߵ��������٣�ͼ�����ģ�����ֶ�λʧ�ܻ��������");
        return false;
    }
    //opencv��װ��ֱ�����----��Ͻ�����ɣ����ֳ���б�ʳ��ַ����������鲻ʹ��
    //cv::fitLine(vecPoint, line_para, 2, 0, 1e-2, 1e-2);
#if _DEBUG
    cv::Mat mColorImage = generateColorImage(mSrcImage);
    vector<cv::Point> lines;
    for (int i = 0; i < vecLast.size(); ++i)
    {
        mColorImage.at<cv::Vec3b>(vecLast[i].y, vecLast[i].x)[0] = 0;
        mColorImage.at<cv::Vec3b>(vecLast[i].y, vecLast[i].x)[1] = 0;
        mColorImage.at<cv::Vec3b>(vecLast[i].y, vecLast[i].x)[2] = 255;
    }
    CvWaitShowImage(mColorImage, "��ѡ��Ե����");
#endif
    return true;
}

bool CdetRectangle::getContoursCenter(cv::Mat mSrcImage, Cfmee_Rectangle &pCenterPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    CFilter filter;
    CGradient gradient;
    cv::Mat mFilterImage, mGradientImage, mDesImage, mEngancyImage;
    mSrcImage.copyTo(mDesImage);
    //�˲�
    filter.getFilter(mDesImage, 5, "11", mFilterImage);
    filter.getFilter(mFilterImage, 4, "3", mFilterImage);
    //��ֵ��
    int nThreshold = OtsuThres(mFilterImage);
    cv::threshold(mFilterImage, mEngancyImage, nThreshold, 255, cv::THRESH_BINARY);
    if (!m_bIsWhite)
        mEngancyImage = ~mEngancyImage;

#if _DEBUG
    CvWaitShowImage(mEngancyImage, "��ֵͼ");
#endif
    cv::RotatedRect boxCircle;
    //��������
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hireachy;
    cv::findContours(mEngancyImage, contours, hireachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    if (contours.size() < 1)
    {
        //writeLog(m_strLog, "ERROR:��������ʧ��");
        return false;
    }
    //��ȡ��С��Ӿ���
    else if (contours.size() == 1)
        boxCircle = cv::minAreaRect(cv::Mat(contours[0]));
    else
    {
        //��������Ϊ1ʱ��ȡ�������
        sort(contours.begin(), contours.end(), sortContour);
        boxCircle = cv::minAreaRect(cv::Mat(contours[0]));
    }
    //��С��Ӿ���У��
    if (boxCircle.size.width > m_nWidth * 0.8 && boxCircle.size.width < m_nWidth * 1.2 &&
        boxCircle.size.height > m_nHeight * 0.8 && boxCircle.size.height < m_nHeight * 1.2)
    {
        cv::Point2f fPoint[4];
        boxCircle.points(fPoint);
        pCenterPoint.dCenterX = boxCircle.center.x;
        pCenterPoint.dCenterY = boxCircle.center.y;
        for (int i = 0; i < 4; ++i)
        {
            pCenterPoint._Point[i].dX = fPoint[i].x;
            pCenterPoint._Point[i].dY = fPoint[i].y;
        }
        //writeLog(m_strLog, "��������ɹ�");
        return true;
    }
    else
    {
        //writeLog(m_strLog, "ERROR:��������ʧ�ܣ�����������������ƥ��");
        return false;
    }

    return false;
}

//��Ե����ϼ��---С�Ƕ���ת
bool CdetRectangle::getFitEdgeLine2(cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;

    CFilter filter;
    CGradient gradient;
    cv::Mat mFilterImage, mGradientImage, mDesImage, mEngancyImage;
    std::vector<cv::Point2d> vecCrossPoint;
    mSrcImage.copyTo(mDesImage);
    //�˲�
    filter.getFilter(mDesImage, 3, "11", mFilterImage);
    filter.getFilter(mFilterImage, 4, "3", mFilterImage);

    //��ֵ��
    int nThreshold = OtsuThres(mFilterImage);
    cv::threshold(mFilterImage, mEngancyImage, nThreshold, 255, cv::THRESH_BINARY);
    if (!m_bIsWhite)
        mEngancyImage = ~mEngancyImage;
#if _DEBUG
    CvWaitShowImage(mEngancyImage, "��ֵͼ");
#endif

    //�ָ���
    std::vector<cv::Mat> vecFourSegmentImage;
    vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(0, mEngancyImage.rows / 6, mEngancyImage.cols / 2, mEngancyImage.rows * 4 / 6)));
    vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 6, 0, mEngancyImage.cols * 4 / 6, mEngancyImage.rows / 2)));
    vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 2, mEngancyImage.rows / 6, mEngancyImage.cols / 2, mEngancyImage.rows * 4 / 6)));
    vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 6, mEngancyImage.rows / 2, mEngancyImage.cols * 4 / 6, mEngancyImage.rows / 2)));
    /*vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(0, mEngancyImage.rows / 8, mEngancyImage.cols / 2, mEngancyImage.rows * 5 / 8)));
		vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 8, 0, mEngancyImage.cols * 5 / 8, mEngancyImage.rows / 2)));
		vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 2, mEngancyImage.rows / 8, mEngancyImage.cols / 2, mEngancyImage.rows * 5 / 8)));
		vecFourSegmentImage.push_back(mEngancyImage(cv::Rect(mEngancyImage.cols / 8, mEngancyImage.rows / 2, mEngancyImage.cols * 5 / 8, mEngancyImage.rows / 2)));*/

    //ֱ�߼��
    std::vector<std::vector<cv::Point>> vecPixel;

    /*if (getLinePara4(mFilterImage(cv::Rect(0, mFilterImage.rows / 8, mFilterImage.cols / 2, mFilterImage.rows * 5 / 8)), vecFourSegmentImage[0], cv::Point(0, mFilterImage.rows / 8), true, nRoateAngle, vecPixel) &&
			getLinePara4(mFilterImage(cv::Rect(mFilterImage.cols / 2, mFilterImage.rows / 8, mFilterImage.cols / 2, mFilterImage.rows * 5 / 8)), vecFourSegmentImage[2], cv::Point(mFilterImage.cols / 2, mFilterImage.rows / 8), false, nRoateAngle, vecPixel) &&
			getLinePara3(mFilterImage(cv::Rect(mFilterImage.cols / 8, 0, mFilterImage.cols * 5 / 8, mFilterImage.rows / 2)), vecFourSegmentImage[1], cv::Point(mFilterImage.cols / 8, 0), true, nRoateAngle, vecPixel) &&
			getLinePara3(mFilterImage(cv::Rect(mFilterImage.cols / 8, mFilterImage.rows / 2, mFilterImage.cols * 5 / 8, mFilterImage.rows / 2)), vecFourSegmentImage[3], cv::Point(mFilterImage.cols / 8, mFilterImage.rows / 2), false, nRoateAngle, vecPixel))*/

    if (getLinePara4(mFilterImage(cv::Rect(0, mFilterImage.rows / 6, mFilterImage.cols / 2, mFilterImage.rows * 4 / 6)), vecFourSegmentImage[0], cv::Point(0, mFilterImage.rows / 6), true, nRoateAngle, vecPixel) &&
        getLinePara4(mFilterImage(cv::Rect(mFilterImage.cols / 2, mFilterImage.rows / 6, mFilterImage.cols / 2, mFilterImage.rows * 4 / 6)), vecFourSegmentImage[2], cv::Point(mFilterImage.cols / 2, mFilterImage.rows / 6), false, nRoateAngle, vecPixel) &&
        getLinePara3(mFilterImage(cv::Rect(mFilterImage.cols / 6, 0, mFilterImage.cols * 4 / 6, mFilterImage.rows / 2)), vecFourSegmentImage[1], cv::Point(mFilterImage.cols / 6, 0), true, nRoateAngle, vecPixel) &&
        getLinePara3(mFilterImage(cv::Rect(mFilterImage.cols / 6, mFilterImage.rows / 2, mFilterImage.cols * 4 / 6, mFilterImage.rows / 2)), vecFourSegmentImage[3], cv::Point(mFilterImage.cols / 6, mFilterImage.rows / 2), false, nRoateAngle, vecPixel))

    /*if (getLinePara4Angel(mFilterImage(cv::Rect(0, mFilterImage.rows / 6, mFilterImage.cols / 2, mFilterImage.rows * 4 / 6)), vecFourSegmentImage[0], cv::Point(0, mFilterImage.rows / 6), true, nRoateAngle, vecPixel) &&
				getLinePara4Angel(mFilterImage(cv::Rect(mFilterImage.cols / 2, mFilterImage.rows / 6, mFilterImage.cols / 2, mFilterImage.rows * 4 / 6)), vecFourSegmentImage[2], cv::Point(mFilterImage.cols / 2, mFilterImage.rows / 6), false, nRoateAngle, vecPixel) &&
				getLinePara3Angel(mFilterImage(cv::Rect(mFilterImage.cols / 6, 0, mFilterImage.cols * 4 / 6, mFilterImage.rows / 2)), vecFourSegmentImage[1], cv::Point(mFilterImage.cols / 6, 0), true, nRoateAngle, vecPixel) &&
				getLinePara3Angel(mFilterImage(cv::Rect(mFilterImage.cols / 6, mFilterImage.rows / 2, mFilterImage.cols * 4 / 6, mFilterImage.rows / 2)), vecFourSegmentImage[3], cv::Point(mFilterImage.cols / 6, mFilterImage.rows / 2), false, nRoateAngle, vecPixel))*/
    {
#if _DEBUG
        cv::Mat mShowImage = fourQuadrantMap(mEngancyImage, vecPixel);
        CvWaitShowImage(mShowImage, "��Եͼ");
#endif
        std::vector<std::vector<cv::Point2d>> vecline;
        for (int i = 0; i < 4; ++i)
        {
            //LineParam a;
            double dA, dB, dC;
            dA = 0;
            dB = 0;
            dC = 0;
            if (lineFit(vecPixel[i], dA, dB, dC))
            {
                std::vector<cv::Point> vecLast = scanPoint(vecPixel[i], dA, dB, dC);
                //��ֱ�߲���ת������������
                vecline.push_back(transLinePara(mEngancyImage, dA, dB, dC));
            }
        }
#if _DEBUG
        cv::Mat mShowImage2 = generateColorImage(mEngancyImage);
        for (int i = 0; i < vecline.size(); ++i)
        {
            line(mShowImage2, cv::Point(vecline[i][0].x, vecline[i][0].y), cv::Point(vecline[i][1].x, vecline[i][1].y), cv::Scalar(255, 0, 0), 2, 8);
        }
        CvWaitShowImage(mShowImage2, "���ֱ����ʾ");
#endif
        if (vecline.size() != 4)
        {
            //writeLog(m_strLog, "ERROR:��Ե�߼��ʧ��");
            return false;
        }
        //��ȡֱ�߽���
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                cv::Point2d crossPoint = CrossPoint(vecline[i][0], vecline[i][1], vecline[2 + j][0], vecline[2 + j][1]);
                if (crossPoint.x > 0 && crossPoint.x < mDesImage.cols &&
                    crossPoint.y > 0 && crossPoint.y < mDesImage.rows)
                    vecCrossPoint.push_back(crossPoint);
                //else
                    //writeLog(m_strLog, "ERROR:�������ʧ��");
            }
        }
#if _DEBUG
        cv::Mat mColorImage = generateColorImage(mSrcImage);
#endif
        if (vecCrossPoint.size() != 4)
            return false;
        for (int i = 0; i < vecCrossPoint.size(); ++i)
        {
            //writeLog(m_strLog, "��������Ϊ����" + to_string(vecCrossPoint[i].x) + "," + to_string(vecCrossPoint[i].y) + ")");
            pCenterPoint._Point[i].dX = vecCrossPoint[i].x;
            pCenterPoint._Point[i].dY = vecCrossPoint[i].y;
            pCenterPoint.dCenterX += vecCrossPoint[i].x;
            pCenterPoint.dCenterY += vecCrossPoint[i].y;
#if _DEBUG
            circle(mColorImage, cv::Point(vecCrossPoint[i].x, vecCrossPoint[i].y), 3, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
        CvWaitShowImage(mColorImage, "������ʾ");
#else
        }
#endif
    }
    else
    {
        ////writeLog(m_strLog, "ERROR:��������ʧ��");
        return false;
    }

    pCenterPoint.dCenterX /= vecCrossPoint.size();
    pCenterPoint.dCenterY /= vecCrossPoint.size();
    if (pCenterPoint.dCenterX != 0 && pCenterPoint.dCenterY != 0)
        return true;
    else
    {
        //writeLog(m_strLog, "ERROR:���������");
        return false;
    }
}

//�Ķ�������
void CdetRectangle::sortRectanglePoint(Cfmee_Rectangle &pCenterPoint)
{
    //writeLog(m_strLog, "��������");
    std::vector<cv::Point2d> vecPoint;
    for (int i = 0; i < 4; ++i)
    {
        vecPoint.push_back(cv::Point2d(pCenterPoint._Point[i].dX, pCenterPoint._Point[i].dY));
    }
    //X��������
    sort(vecPoint.begin(), vecPoint.end(), sortPoint2dX);
    std::vector<cv::Point2d> vecYPoint;
    vecYPoint.push_back(vecPoint[2]);
    vecYPoint.push_back(vecPoint[3]);
    if (vecPoint[0].y > vecPoint[1].y)
    {
        pCenterPoint._Point[0].dX = vecPoint[0].x;
        pCenterPoint._Point[0].dY = vecPoint[0].y;
        pCenterPoint._Point[2].dX = vecPoint[1].x;
        pCenterPoint._Point[2].dY = vecPoint[1].y;
    }
    else
    {
        pCenterPoint._Point[0].dX = vecPoint[1].x;
        pCenterPoint._Point[0].dY = vecPoint[1].y;
        pCenterPoint._Point[2].dX = vecPoint[0].x;
        pCenterPoint._Point[2].dY = vecPoint[0].y;
    }
    //Y��������
    sort(vecYPoint.begin(), vecYPoint.end(), sortPoint2dY);
    pCenterPoint._Point[1].dX = vecYPoint[1].x;
    pCenterPoint._Point[1].dY = vecYPoint[1].y;
    pCenterPoint._Point[3].dX = vecYPoint[0].x;
    pCenterPoint._Point[3].dY = vecYPoint[0].y;
}

// ���α�Եֱ�߼��
bool CdetRectangle::getLinePara3(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop, int nRoateAngle, std::vector<std::vector<cv::Point>> &vec)
{
    if (!mSrcImage.data || !mThresholdImage.data)
        return false;
    cv::Mat mGradientImage;
    CGradient gradient;
    gradient.getGradientImage(mThresholdImage, mGradientImage, 0);
    double dA, dB, dC;
    std::vector<cv::Point> vecPixelPoint;
    for (int i = 3; i < mGradientImage.rows - 3; ++i)
    {
        for (int j = 3; j < mGradientImage.cols - 3; ++j)
        {
            if (m_bIsWhite && bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /**cos((double)nRoateAngle)*/, j - 2 /**sin((double)nRoateAngle)*/)             /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                         - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                     < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (m_bIsWhite && !bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                               - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                           > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                               - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                           > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && !bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                                - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                            < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
        }
    }
    if (vecPixelPoint.size() > 3)
    {
        for (int i = 1; i < vecPixelPoint.size(); ++i)
        {
            if (vecPixelPoint[i - 1].y != vecPixelPoint[i].y && vecPixelPoint[i - 1].x == vecPixelPoint[i].x ||
                (bisTop && vecPixelPoint[i].y > mSrcImage.rows / 2) || (!bisTop && vecPixelPoint[i].y < mSrcImage.rows / 2 * 3))
            {
                vecPixelPoint.erase(vecPixelPoint.begin() + i);
                i--;
            }
        }
        vec.push_back(vecPixelPoint);
        return true;
    }
    return false;
}

// ���α�Եֱ�߼��
bool CdetRectangle::getLinePara4(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift, int nRoateAngle, std::vector<std::vector<cv::Point>> &vec)
{
    if (!mSrcImage.data || !mThresholdImage.data)
        return false;
    cv::Mat mGradientImage;
    CGradient gradient;
    gradient.getGradientImage(mThresholdImage, mGradientImage, 0);
    double dA, dB, dC;
    std::vector<cv::Point> vecPixelPoint;
    for (int i = 3; i < mGradientImage.rows - 3; ++i)
    {
        for (int j = 3; j < mGradientImage.cols - 3; ++j)
        {
            if (m_bIsWhite && !isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /**sin((double)nRoateAngle)*/, j + 2 /**cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (m_bIsWhite && isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && !isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
        }
    }
    if (vecPixelPoint.size() > 3)
    {
        for (int i = 1; i < vecPixelPoint.size(); ++i)
        {
            if (vecPixelPoint[i - 1].x != vecPixelPoint[i].x && vecPixelPoint[i - 1].y == vecPixelPoint[i].y ||
                (isLift && vecPixelPoint[i].x > mSrcImage.cols / 2) || (!isLift && vecPixelPoint[i].x < mSrcImage.cols / 2 * 3))
            {
                vecPixelPoint.erase(vecPixelPoint.begin() + i);
                i--;
            }
        }
        vec.push_back(vecPixelPoint);
        return true;
    }
    return false;
}

bool CdetRectangle::getLinePara3Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop, int nRoateAngle, std::vector<std::vector<cv::Point>> &vec)
{
    if (!mSrcImage.data || !mThresholdImage.data)
        return false;
    cv::Mat mGradientImage;
    CGradient gradient;
    gradient.getGradientImage(mThresholdImage, mGradientImage, 0);
    double dA, dB, dC;
    std::vector<cv::Point> vecPixelPoint;
    for (int i = 3; i < mGradientImage.rows - 3; ++i)
    {
        for (int j = 3; j < mGradientImage.cols - 3; ++j)
        {
            if (m_bIsWhite && bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /**cos((double)nRoateAngle)*/, j - 2 /**sin((double)nRoateAngle)*/)             /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                         - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                     < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (m_bIsWhite && !bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                               - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                           > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                               - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                           > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && !bisTop && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** cos((double)nRoateAngle)*/, j - 2 /** sin((double)nRoateAngle)*/)           /*+ mSrcImage.at<uchar>(i - 2, j - 2)*/
                                                                                                - mSrcImage.at<uchar>(i + 2 /** cos((double)nRoateAngle)*/, j + 2 /** sin((double)nRoateAngle)*/) /*- mSrcImage.at<uchar>(i + 2, j + 2)*/
                                                                                            < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
        }
    }
    if (vecPixelPoint.size() > 3)
    {
        for (int i = 1; i < vecPixelPoint.size(); ++i)
        {
            if (vecPixelPoint[i - 1].y != vecPixelPoint[i].y && vecPixelPoint[i - 1].x == vecPixelPoint[i].x ||
                (bisTop && vecPixelPoint[i].y > mSrcImage.rows / 2) || (!bisTop && vecPixelPoint[i].y < mSrcImage.rows / 2 * 3))
            {
                vecPixelPoint.erase(vecPixelPoint.begin() + i);
                i--;
            }
        }
        vec.push_back(vecPixelPoint);
        return true;
    }
    return false;
}

bool CdetRectangle::getLinePara4Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift, int nRoateAngle, std::vector<std::vector<cv::Point>> &vec)
{
    if (!mSrcImage.data || !mThresholdImage.data)
        return false;
    cv::Mat mGradientImage;
    CGradient gradient;
    gradient.getGradientImage(mThresholdImage, mGradientImage, 0);
    double dA, dB, dC;
    std::vector<cv::Point> vecPixelPoint;
    for (int i = 3; i < mGradientImage.rows - 3; ++i)
    {
        for (int j = 3; j < mGradientImage.cols - 3; ++j)
        {
            if (m_bIsWhite && !isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /**sin((double)nRoateAngle)*/, j + 2 /**cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (m_bIsWhite && isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && !isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) > 30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
            else if (!m_bIsWhite && isLift && mGradientImage.at<uchar>(i, j) >= 220 && mSrcImage.at<uchar>(i - 2 /** sin((double)nRoateAngle)*/, j + 2 /** cos((double)nRoateAngle)*/) - mSrcImage.at<uchar>(i + 2 /** sin((double)nRoateAngle)*/, j - 2 /** cos((double)nRoateAngle)*/) < -30)
                vecPixelPoint.push_back(cv::Point(j + pLoc.x, i + pLoc.y));
        }
    }
    if (vecPixelPoint.size() > 3)
    {
        for (int i = 1; i < vecPixelPoint.size(); ++i)
        {
            if (vecPixelPoint[i - 1].x != vecPixelPoint[i].x && vecPixelPoint[i - 1].y == vecPixelPoint[i].y ||
                (isLift && vecPixelPoint[i].x > mSrcImage.cols / 2) || (!isLift && vecPixelPoint[i].x < mSrcImage.cols / 2 * 3))
            {
                vecPixelPoint.erase(vecPixelPoint.begin() + i);
                i--;
            }
        }
        vec.push_back(vecPixelPoint);
        return true;
    }
    return false;
}

//�ĵ�ȷ��ֱ�߽���
cv::Point2d CdetRectangle::CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4)
{
    double x_member, x_denominator, y_member, y_denominator;
    cv::Point2d cross_point;
    x_denominator = line4.x * line2.y - line4.x * line1.y - line3.x * line2.y + line3.x * line1.y - line2.x * line4.y + line2.x * line3.y + line1.x * line4.y - line1.x * line3.y;

    x_member = line3.y * line4.x * line2.x - line4.y * line3.x * line2.x - line3.y * line4.x * line1.x + line4.y * line3.x * line1.x - line1.y * line2.x * line4.x + line2.y * line1.x * line4.x + line1.y * line2.x * line3.x - line2.y * line1.x * line3.x;

    if (x_denominator == 0)
        cross_point.x = 0;
    else
        cross_point.x = x_member / x_denominator;

    y_denominator = line4.y * line2.x - line4.y * line1.x - line3.y * line2.x + line1.x * line3.y - line2.y * line4.x + line2.y * line3.x + line1.y * line4.x - line1.y * line3.x;

    y_member = -line3.y * line4.x * line2.y + line4.y * line3.x * line2.y + line3.y * line4.x * line1.y - line4.y * line3.x * line1.y + line1.y * line2.x * line4.y - line1.y * line2.x * line3.y - line2.y * line1.x * line4.y + line2.y * line1.x * line3.y;

    if (y_denominator == 0)
        cross_point.y = 0;
    else
        cross_point.y = y_member / y_denominator;

    return cross_point; //ƽ�з���(0,0)
}

CdetRectangle &CdetRectangle::operator=(const CdetRectangle &)
{
    // TODO: �ڴ˴����� return ���
    return *this;
}
