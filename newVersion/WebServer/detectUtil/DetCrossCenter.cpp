#include "DetCrossCenter.h"
#include "paraType.h"
#include "Common.h"
#include "FilterImage.h"
#include "Gradient.h"
#include "EnhanceImage.h"
#include "LogHelp.h"
#include "FitLine.h"

using namespace std;
using namespace cv;

/// <summary>
/// ���캯��
/// </summary>
/// <param name="strLog">��־�ļ�·��</param>
/// <param name="param">�б�����ṹ��</param>
/// <param name="algorithmPara">�㷨�����ṹ��</param>
CdetCross::CdetCross(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara)
{
    m_strLog = strLog;
    m_nWidth = param.Width[0];
    m_nHeight = param.Height[0];
    m_nDimeter = param.Diameter[0];
    m_nThreshold = param.Threshold;
    m_bIsWhite = param.IsWhite;
    m_nAlgorithm = algorithmPara.nMarkAlgorithm;
    m_dLineCount = param.ReserveData[0];
    m_bIsPolarChange = algorithmPara.IsPolarChange;
}

CdetCross::~CdetCross()
{
}

/// <summary>
/// ͨ��ʮ�ּ���㷨
/// </summary>
/// <param name="mSrcImage">������ͼƬ</param>
/// <param name="bIsCalibrate">�궨���</param>
/// <param name="pCenterPoint">���б����ĵ�</param>
/// <param name="dScore">�б���÷�</param>
/// <returns></returns>
bool CdetCross::getCrossCenter(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    vector<cv::Point2d> fourPoint;
    mSrcImage.copyTo(mDesImage);
    cv::Point2d pRoughCenterPoint(0, 0);
    if (bIsCalibrate)
    {
        writeLog(m_strLog, "�궨���");
    }
    else
    {
        writeLog(m_strLog, "markץȡ���");
        switch (m_nAlgorithm)
        {
        case 0:
            writeLog(m_strLog, "��֧����ϼ��");
            if (detSingleLine(mDesImage, pRoughCenterPoint, fourPoint))
            {
                pCenterPoint = pRoughCenterPoint;
                writeLog(m_strLog, "��ϳɹ�");
                return true;
            }
            else
            {
                writeLog(m_strLog, "��֧�����ʧ��");
                return false;
            }

        case 1:
            writeLog(m_strLog, "˫֧����ϼ��");
            if (detDoubleLine(mDesImage, pRoughCenterPoint))
                writeLog(m_strLog, "��ϳɹ�");
            break;
        case 2:
            break;
        default:
            break;
        }
    }
    return false;
}

/// <summary>
/// ͨ��ʮ�ּ���㷨,���������Ľǵ�
/// </summary>
/// <param name="mSrcImage">������ͼƬ</param>
/// <param name="bIsCalibrate">�궨���</param>
/// <param name="pCenterPoint">���б����ĵ�</param>
/// <param name="dScore">�б���÷�</param>
/// <returns></returns>
bool CdetCross::getCrossCenterFourPoint(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, vector<cv::Point2d> &fourPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    cv::Point2d pRoughCenterPoint(0, 0);
    if (bIsCalibrate)
    {
        writeLog(m_strLog, "�궨���");
    }
    else
    {
        writeLog(m_strLog, "markץȡ���");
        switch (m_nAlgorithm)
        {
        case 0:
            writeLog(m_strLog, "��֧����ϼ��");
            if (detSingleLine(mDesImage, pRoughCenterPoint, fourPoint))
            {
                pCenterPoint = pRoughCenterPoint;
                writeLog(m_strLog, "��ϳɹ�");
                return true;
            }
            else
            {
                writeLog(m_strLog, "��֧�����ʧ��");
                return false;
            }

        case 1:
            writeLog(m_strLog, "˫֧����ϼ��");
            if (detDoubleLine(mDesImage, pRoughCenterPoint))
                writeLog(m_strLog, "��ϳɹ�");
            break;
        case 2:
            break;
        default:
            break;
        }
    }
    return false;
}

/// <summary>
/// �뵼�帴��ʮ�ּ���㷨
/// </summary>
/// <param name="mSrcImage">������ͼƬ</param>
/// <param name="vecCenterPoint">���б����ĵ㼯��</param>
/// <param name="dScore">�б���÷�</param>
/// <returns></returns>
bool CdetCross::getSEMIComplexCrossCenter(const cv::Mat mSrcImage, std::vector<cv::Point2d> &vecCenterPoint, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    return false;
}

CdetCross &CdetCross::operator=(const CdetCross &)
{
    // TODO: �ڴ˴����� return ���
    return *this;
}

/// <summary>
/// ��ֱ�߼���㷨
/// </summary>
/// <param name="mSrcImage">����ͼ��</param>
/// <param name="pCenterPoint">ֱ�����ĵ�����</param>
/// <returns></returns>
bool CdetCross::detSingleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint, vector<cv::Point2d> &fourPoint)
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
    std::vector<std::pair<cv::Mat, cv::Point>> vecLineMap = getSegmentLine(mFilterImage, min(m_nWidth, m_nHeight), m_nDimeter);
    if (vecLineMap.size() != 4)
    {
        writeLog(m_strLog, "ERROR:�������ָ�ʧ��");
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
            writeLog(m_strLog, "ERROR:ֱ����Ͻ��������ֹ���");
            return false;
        }
        else
            twoPoint = transCVLinePara(vecLineMap[i].first, line_para);
#if _DEBUG
        cv::Mat mLineImage = generateColorImage(vecLineMap[i].first);
        line(mLineImage, Point(twoPoint[0].x, twoPoint[0].y), Point(twoPoint[1].x, twoPoint[1].y), Scalar(0, 0, 255), 1, 8);
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
    writeLog(m_strLog, "����ֱ�߽���");
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            cv::Point2d crossPoint = CrossPoint(vecline[i][0], vecline[i][1], vecline[2 + j][0], vecline[2 + j][1]);
            vecCrossPoint.push_back(crossPoint);
            writeLog(m_strLog, "��������" + to_string(i) + "_" + to_string(j) + "Ϊ����" +
                                   to_string(crossPoint.x) + "," + to_string(crossPoint.y) + ")");
        }
    }
#if _DEBUG
    cv::Mat mColorImage = generateColorImage(mSrcImage);
#else
#endif
    for (int i = 0; i < vecCrossPoint.size(); ++i)
    {

        pCenterPoint.x += vecCrossPoint[i].x;
        pCenterPoint.y += vecCrossPoint[i].y;
        fourPoint.push_back(cv::Point(vecCrossPoint[i].x, vecCrossPoint[i].y));
#if _DEBUG
        circle(mColorImage, Point(vecCrossPoint[i].x, vecCrossPoint[i].y), 3, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
    CvWaitShowImage(mColorImage, "������ʾ");
#else
    }
#endif
    pCenterPoint.x /= vecCrossPoint.size();
    pCenterPoint.y /= vecCrossPoint.size();
    if (pCenterPoint.x != 0 && pCenterPoint.y != 0)
        return true;
    else
        return false;
}

/// <summary>
/// ƽ��˫ֱ�߼���㷨
/// </summary>
/// <param name="mSrcImage">����ͼ��</param>
/// <param name="pCenterPoint">ֱ�����ĵ�����</param>
/// <returns></returns>
bool CdetCross::detDoubleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint)
{
    return false;
}

/// <summary>
/// ��ֱ�������ȡ
/// </summary>
/// <param name="mSrcImage">�����ͼƬ</param>
/// <param name="nLength">��</param>
/// <param name="nBorderWidth">��</param>
/// <returns></returns>
std::vector<std::pair<cv::Mat, cv::Point>> CdetCross::getSegmentLine(const cv::Mat mSrcImage, int nLength,
                                                                     int nBorderWidth)
{
    std::vector<std::pair<cv::Mat, cv::Point>> vecMap;
    cv::Point pp(mSrcImage.cols / 2, mSrcImage.rows / 2);
    int nRate = nBorderWidth / 10 * 10;

    int xB[2], yB[2], xE[2], yE[2];
    xB[0] = pp.x - (nBorderWidth + nRate) / 2 > 0 ? pp.x - (nBorderWidth + nRate) / 2 : 0;
    yB[0] = pp.y - (nLength) / 2 > 0 ? pp.y - (nLength) / 2 : 0;

    xE[0] = pp.x + (nBorderWidth + nRate) / 2 > mSrcImage.cols - 1 ? mSrcImage.cols - 1 : pp.x + (nBorderWidth + nRate) / 2;
    yE[0] = pp.y + (nLength) / 2 > mSrcImage.rows - 1 ? mSrcImage.rows - 1 : pp.y + (nLength) / 2;

    xB[1] = pp.x - (nLength) / 2 > 0 ? pp.x - (nLength) / 2 : 0;
    yB[1] = pp.y - (nBorderWidth + nRate) / 2 > 0 ? pp.y - (nBorderWidth + nRate) / 2 : 0;

    xE[1] = pp.x + (nLength) / 2 > mSrcImage.cols - 1 ? mSrcImage.cols - 1 : pp.x + (nLength) / 2;
    yE[1] = pp.y + (nBorderWidth + nRate) / 2 > mSrcImage.rows - 1 ? mSrcImage.rows - 1 : pp.y + (nBorderWidth + nRate) / 2;

    //������
    vecMap.push_back(std::make_pair(mSrcImage(Rect(xB[0], yB[0], nRate, min(mSrcImage.rows - yB[0], nLength))), cv::Point(xB[0], yB[0])));
    //������
    vecMap.push_back(std::make_pair(mSrcImage(Rect(xE[0] - nRate, yB[0], nRate, min(mSrcImage.rows - yB[0], nLength))), cv::Point(xE[0] - nRate, yB[0])));
    //�Ϻ���
    vecMap.push_back(std::make_pair(mSrcImage(Rect(xB[1], yB[1], min(mSrcImage.cols - xB[1], nLength), nRate)), cv::Point(xB[1], yB[1])));
    //�º���
    vecMap.push_back(std::make_pair(mSrcImage(Rect(xB[1], yE[1] - nRate, min(mSrcImage.cols - xB[1], nLength), nRate)), cv::Point(xB[1], yE[1] - nRate)));
#if _DEBUG
    cv::Mat mColorImage = generateColorImage(mSrcImage);
    rectangle(mColorImage, Point(xB[0], yB[0]), Point(xB[0] + nRate, yB[0] + nLength),
              Scalar(0, 0, 255), 2, 8, 0);
    rectangle(mColorImage, Point(xE[0] - nRate, yB[0]), Point(xE[0], yB[0] + nLength),
              Scalar(255, 0, 0), 2, 8, 0);
    rectangle(mColorImage, Point(xB[1], yB[1]), Point(xB[1] + nLength, yB[1] + nRate),
              Scalar(255, 255, 0), 2, 8, 0);
    rectangle(mColorImage, Point(xB[1], yE[1] - nRate), Point(xB[1] + nLength, yE[1]),
              Scalar(0, 255, 0), 2, 8, 0);
    CvWaitShowImage(mColorImage, "�����ֱ������");
#endif
    return vecMap;
}

/// <summary>
/// ���������ֱ��
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="vecPoint">��⵽��ֱ�������</param>
/// <returns></returns>
bool CdetCross::detLiftVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint)
{
    cv::Mat mDesImage;
    CEnhance enhance(m_bIsWhite);
    enhance.getEnhanceImage(mSrcImage, 0, mDesImage);
    mSrcImage.copyTo(mDesImage);
    //enhance.getEnhanceImage(mDesImage, 4, mDesImage, 0, 0, 400, 10);
#if _DEBUG
    CvWaitShowImage(mDesImage, "ֱ������");
#endif
    std::vector<cv::Point> vec;
    if (mDesImage.cols > 5 && mDesImage.rows > 5)
    {
        for (int y = 3; y < mDesImage.rows - 3; ++y)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            double dDiff;
            for (int x = mDesImage.cols - 3; x > 3; --x)
            {
                //���Լ��
                if (m_bIsPolarChange)
                {
                    writeLog(m_strLog, "���ü��Լ��");
                    dDiff = mDesImage.at<uchar>(y, x - 1) + mDesImage.at<uchar>(y, x - 2) - mDesImage.at<uchar>(y, x + 1) - mDesImage.at<uchar>(y, x + 2);
                    //��ɫʮ��
                    if (m_bIsWhite)
                    {
                        if (dDiff < -nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                    else //��ɫʮ��
                    {
                        if (dDiff > nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                }
                else //�Ǽ��Լ��
                {
                    writeLog(m_strLog, "�����ü��Լ��");
                    dDiff = abs(mDesImage.at<uchar>(y, x - 1) + mDesImage.at<uchar>(y, x - 2) - mDesImage.at<uchar>(y, x + 1) - mDesImage.at<uchar>(y, x + 2));
                    if (dDiff > nMax)
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                        nMax = dDiff;
                    }
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vec.push_back(pIndex);
        }
    }
    if (vec.size() != 0)
    {
        vecPoint.push_back(vec);
        return true;
    }
    else
        return false;
}

/// <summary>
/// ���������ֱ��
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="vecPoint">��⵽��ֱ�������</param>
/// <returns></returns>
bool CdetCross::detRightVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint)
{
    cv::Mat mDesImage;
    CEnhance enhance(m_bIsWhite);
    enhance.getEnhanceImage(mSrcImage, 0, mDesImage);
    mSrcImage.copyTo(mDesImage);
    //enhance.getEnhanceImage(mDesImage, 4, mDesImage, 0, 0, 400, 10);
#if _DEBUG
    CvWaitShowImage(mDesImage, "ֱ������");
#endif
    std::vector<cv::Point> vec;
    if (mDesImage.cols > 5 && mDesImage.rows > 5)
    {
        for (int y = 3; y < mDesImage.rows - 3; ++y)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            double dDiff;
            for (int x = 3; x < mDesImage.cols - 3; ++x)
            {
                //���Լ��
                if (m_bIsPolarChange)
                {
                    writeLog(m_strLog, "���ü��Լ��");
                    dDiff = mDesImage.at<uchar>(y, x - 1) + mDesImage.at<uchar>(y, x - 2) - mDesImage.at<uchar>(y, x + 1) - mDesImage.at<uchar>(y, x + 2);
                    //��ɫʮ��
                    if (m_bIsWhite)
                    {
                        if (dDiff > nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                    else //��ɫʮ��
                    {
                        if (dDiff < -nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                }
                else //�Ǽ��Լ��
                {
                    writeLog(m_strLog, "�����ü��Լ��");
                    dDiff = abs(mDesImage.at<uchar>(y, x - 1) + mDesImage.at<uchar>(y, x - 2) - mDesImage.at<uchar>(y, x + 1) - mDesImage.at<uchar>(y, x + 2));
                    if (dDiff > nMax)
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                        nMax = dDiff;
                    }
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vec.push_back(pIndex);
        }
    }
    if (vec.size() != 0)
    {
        vecPoint.push_back(vec);
        return true;
    }
    else
        return false;
}

/// <summary>
/// ����Ϻ���ֱ��
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="vecPoint">��⵽��ֱ�������</param>
/// <returns></returns>
bool CdetCross::detTopHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint)
{
    cv::Mat mDesImage;
    CEnhance enhance(m_bIsWhite);
    enhance.getEnhanceImage(mSrcImage, 0, mDesImage);
    mSrcImage.copyTo(mDesImage);
    //enhance.getEnhanceImage(mDesImage, 4, mDesImage, 0, 0, 400, 10);
#if _DEBUG
    CvWaitShowImage(mDesImage, "ֱ������");
#endif
    std::vector<cv::Point> vec;
    if (mDesImage.cols > 5 && mDesImage.rows > 5)
    {
        for (int x = 3; x < mDesImage.cols - 3; ++x)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            double dDiff;
            for (int y = mDesImage.rows - 3; y > 3; --y)
            {
                //���Լ��
                if (m_bIsPolarChange)
                {
                    writeLog(m_strLog, "���ü��Լ��");
                    dDiff = mDesImage.at<uchar>(y - 1, x) + mDesImage.at<uchar>(y - 2, x) - mDesImage.at<uchar>(y + 1, x) - mDesImage.at<uchar>(y + 2, x);
                    //��ɫʮ��
                    if (m_bIsWhite)
                    {
                        if (dDiff < -nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                    else //��ɫʮ��
                    {
                        if (dDiff > nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                }
                else //�Ǽ��Լ��
                {
                    writeLog(m_strLog, "�����ü��Լ��");
                    dDiff = abs(mDesImage.at<uchar>(y - 1, x) + mDesImage.at<uchar>(y - 2, x) - mDesImage.at<uchar>(y + 1, x) - mDesImage.at<uchar>(y + 2, x));
                    if (dDiff > nMax)
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                        nMax = dDiff;
                    }
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vec.push_back(pIndex);
        }
    }
    if (vec.size() != 0)
    {
        vecPoint.push_back(vec);
        return true;
    }
    else
        return false;
}

/// <summary>
/// ����º���ֱ��
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="vecPoint">��⵽��ֱ�������</param>
/// <returns></returns>
bool CdetCross::detDownHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint)
{
    cv::Mat mDesImage;
    CEnhance enhance(m_bIsWhite);
    enhance.getEnhanceImage(mSrcImage, 0, mDesImage);
    mSrcImage.copyTo(mDesImage);
    //enhance.getEnhanceImage(mDesImage, 4, mDesImage, 0, 0, 400, 10);
#if _DEBUG
    CvWaitShowImage(mDesImage, "ֱ������");
#endif
    std::vector<cv::Point> vec;
    if (mDesImage.cols > 5 && mDesImage.rows > 5)
    {
        for (int x = 3; x < mDesImage.cols - 3; ++x)
        {
            int nMax = DIFF_PIXEL;
            cv::Point pIndex(0, 0);
            double dDiff;
            for (int y = 3; y < mDesImage.rows - 3; ++y)
            {
                //���Լ��
                if (m_bIsPolarChange)
                {
                    writeLog(m_strLog, "���ü��Լ��");
                    dDiff = mDesImage.at<uchar>(y - 1, x) + mDesImage.at<uchar>(y - 2, x) - mDesImage.at<uchar>(y + 1, x) - mDesImage.at<uchar>(y + 2, x);
                    //��ɫʮ��
                    if (m_bIsWhite)
                    {
                        if (dDiff > nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                    else //��ɫʮ��
                    {
                        if (dDiff < -nMax)
                        {
                            pIndex.x = x;
                            pIndex.y = y;
                            nMax = dDiff;
                        }
                    }
                }
                else
                {
                    writeLog(m_strLog, "�����ü��Լ��");
                    dDiff = abs(mDesImage.at<uchar>(y - 1, x) + mDesImage.at<uchar>(y - 2, x) - mDesImage.at<uchar>(y + 1, x) - mDesImage.at<uchar>(y + 2, x));
                    if (dDiff > nMax)
                    {
                        pIndex.x = x;
                        pIndex.y = y;
                        nMax = dDiff;
                    }
                }
            }
            if (pIndex.x != 0 || pIndex.y != 0)
                vec.push_back(pIndex);
        }
    }
    if (vec.size() != 0)
    {
        vecPoint.push_back(vec);
        return true;
    }
    else
        return false;
}

//��ȡ���ֱ�߲���
//ͨ��ɫ���ȡֱ�߷ֽ��ߣ�Ч�����Ǻ����룬�޷�ȷ����ѷֽ���ֵ
bool  CdetCross::getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c)
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
    if (vecPoint.size() >= (min(m_nWidth, m_nHeight) - m_nDimeter) * 0.8 && lineFit(vecPoint, dA, dB, dC))
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
    vector<Point> lines;
    for (int i = 0; i < vecLast.size(); ++i)
    {
        mColorImage.at<Vec3b>(vecLast[i].y, vecLast[i].x)[0] = 0;
        mColorImage.at<Vec3b>(vecLast[i].y, vecLast[i].x)[1] = 0;
        mColorImage.at<Vec3b>(vecLast[i].y, vecLast[i].x)[2] = 255;
    }
    CvWaitShowImage(mColorImage, "��ѡ��Ե����");
#endif
    return true;
}
//�ĵ�ȷ��ֱ�߽���
cv::Point2d CdetCross::CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4)
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