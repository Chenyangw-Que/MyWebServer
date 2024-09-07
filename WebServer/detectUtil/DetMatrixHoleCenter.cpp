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
/// ���캯��
/// </summary>
/// <param name="strLog">��־�ļ�·��</param>
/// <param name="param">�б�����ṹ��</param>
/// <param name="algorithmPara">�㷨�����ṹ��</param>
CdetHoleMatrix::CdetHoleMatrix(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara)
{
    m_strLog = strLog;                                           //��־�ĵ�
    m_nRows = param.Rows;                                        //��
    m_nCols = param.Cols;                                        //��
    m_RowSpan = param.Rowspan;                                   //�м��
    m_ColSpan = param.Colspan;                                   //�м��
    m_nDiameter = param.Diameter[0];                             //ֱ������
    m_nThreshold = param.Threshold;                              //������
    m_bIsWhite = param.IsWhite;                                  //�б�Ϊ��ɫ
    m_nAlgorithm = algorithmPara.nMarkAlgorithm;                 //ʶ���㷨
    m_bWhiteEdge = algorithmPara.nWhiteEdge;                     //��ɫ�б��Ե
    m_bIsPolarChange = algorithmPara.IsPolarChange;              //���Լ��
    m_fHoleMatrixSocre = algorithmPara.dSocreThreshold;          //�����ֵ����
    m_bIsEnhance = algorithmPara.IsEnhanceMap;                   //ͼ����ǿ
    m_nSpanRange = abs(m_nDiameter - min(m_RowSpan, m_ColSpan)); //ֱ�������
}

CdetHoleMatrix::~CdetHoleMatrix()
{
}

/// <summary>
/// ��ȡÿһ��СԲ��Բ������
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="dAngle">��ת�Ƕ�</param>
/// <param name="vecEveryHoleMatrixPoint">��⵽ÿһ��СԲ����</param>
/// <param name="pHoleMatrixCenter">�б����ĵ�</param>
/// <returns></returns>
bool CdetHoleMatrix::getEveryLittleCircle(const cv::Mat mSrcImage, double dAngle,
                                          std::vector<cv::Point2d> &vecEveryAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mGrayImage;
    mSrcImage.copyTo(mGrayImage);
    m_dAngle = dAngle;
    //��ȡÿһ��СԲλ��
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecEveryRoughPoint = getRoughPointLocation(mGrayImage);
#ifdef _SHOW_PROCESS_
    cv::Mat mShowRoughLocation = drawRoughLocation(mSrcImage, vecEveryRoughPoint, m_nDiameter);
    CvWaitShowImage(mShowRoughLocation, "�б�λ�ôֶ�λ");
#endif
    cv::Mat mEvenImage;
    mGrayImage.copyTo(mEvenImage);
    cv::Point2d pExtactCenter(0, 0);
    //��ȷ���ÿһ��СԲ����
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
/// ֱ�Ӷ�λСԲ�����ü���Բ�ļ����㷨
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="dAngle">��ת�Ƕ�</param>
/// <param name="vecEveryHoleMatrixPoint">��⵽ÿһ��СԲ����</param>
/// <param name="pHoleMatrixCenter">�б����ĵ�</param>
/// <returns></returns>
bool CdetHoleMatrix::getEveryLittleCircleDirect(const cv::Mat mSrcImage, const std::vector<std::vector<pair<cv::Mat, cv::Point>>> vecEveryRoughPoint, std::vector<cv::Point2d> &vecEveryAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
#ifdef _SHOW_PROCESS_
    cv::Mat mShowRoughLocation = drawRoughLocation(mSrcImage, vecEveryRoughPoint, m_nDiameter);
    CvWaitShowImage(mShowRoughLocation, "�б�λ�ôֶ�λ");
#endif
    cv::Mat mEvenImage;
    mSrcImage.copyTo(mEvenImage);
    cv::Point2d pExtactCenter(0, 0);
    //cout << "�ָ����" << vecEveryRoughPoint[0].size() << endl;
    //��ȷ���ÿһ��СԲ����
    if (detEveryCircleLocation(mEvenImage, vecEveryRoughPoint, vecEveryAccurateLocation,
                               pExtactCenter, dScore))
    {
        pHoleMatrixCenter.x = pExtactCenter.x;
        pHoleMatrixCenter.y = pExtactCenter.y;
        return true;
    }
    return false;
}

//���ɸ��е�����ֵ
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
    // TODO: �ڴ˴����� return ���
    return *this;
}

/// <summary>
/// ͨ�����ȷ���Ƿ��������Բ
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
/// �Ʋ�ÿ���б�Ĵ������ĵ�λ��
/// </summary>
/// <param name="mSrcImage"></param>
/// <returns></returns>
std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> CdetHoleMatrix::getRoughPointLocation(cv::Mat mSrcImage)
{
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughPoint; //�е�����ϼ�
    std::vector<std::pair<cv::Mat, cv::Point>> singleRoughPoint;           //���ае�����
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
/// ���ÿһ��СԲ�����ĵ�����
/// </summary>
/// <param name="mSrcImage">����ͼ��</param>
/// <param name="vecRoughLocation">�ֶ�λ����</param>
/// <param name="vecAccurateLocation">��ȷ�������</param>
/// <param name="pHoleMatrixCenter">�׾������ĵ�����</param>
/// <param name="dScore">���÷�</param>
/// <returns></returns>
bool CdetHoleMatrix::detEveryCircleLocation(cv::Mat mSrcImage, std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                            std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    if (!mSrcImage.data)
        return false;
    std::vector<std::vector<cv::Point2d>> vecPoint2d;
    //���ݲ�ֱͬ�����в�ͬ�ļ�����
    if (m_nDiameter > 50)
    {
        writeLog(m_strLog, "Բ���ץȡ");
        if (detEveryBigCircleDLPrior(vecRoughLocation, vecPoint2d))
        {
            writeLog(m_strLog, "Բ���ץȡ�ɹ�");
        }
        else
        {
            writeLog(m_strLog, "Բ���ץȡʧ��");
        }
    }
    else
    {
        writeLog(m_strLog, "�ָ���ץȡ");
        if (detEveryLittleCircle(vecRoughLocation, vecPoint2d))
        {
            writeLog(m_strLog, "�ָ���ץȡ�ɹ�");
        }
        else
        {
            writeLog(m_strLog, "�ָ���ץȡʧ��");
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
    CvWaitShowImage(mShowImage, "ץȡ�ɹ���СԲ�б�");
#endif
    //�Լ��������ɸѡ
    if (analysisDetSuccessCircle(vecPoint2d, vecAccurateLocation, pHoleMatrixCenter, dScore))
    {
        writeLog(m_strLog, "�����ɸѡ�ɹ�");

        return true;
    }
    else
    {
        writeLog(m_strLog, "ERROR:�����������ɸѡ���������ʧ��");
        return false;
    }
    return false;
}

/// <summary>
/// ��ֱ����Բ���
/// </summary>
/// <param name="vecRoughLocation">ÿһ����Բ�洢����</param>
/// <param name="vecCircleCenter">�����</param>
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
                //cout << "Բ���ʧ��" << endl;
#define DEEPLEARNING
#ifdef DEEPLEARNING
                // ʹ�÷ָ�����
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
/// Сֱ����Բ���
/// </summary>
/// <param name="vecRoughLocation">ÿһ����Բ�洢����</param>
/// <param name="vecCircleCenter">�����</param>
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
                writeLog(m_strLog, "����Ӧ��ֵ���ָ�");
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
                    // ʹ�÷ָ�����
                    pExactResult.x = vecRoughLocation[i][j].first.cols / 2 + vecRoughLocation[i][j].second.x;
                    pExactResult.y = vecRoughLocation[i][j].first.rows / 2 + vecRoughLocation[i][j].second.y;
                    vecHorResult.push_back(pExactResult);
#endif // DEEPLEARNING
                }
                break;
            case 1:
                writeLog(m_strLog, "ͳ�Ʒ���ֵ���ָ�");
                break;
            case 2:
                writeLog(m_strLog, "����������ץȡ");
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
/// ����Ӧ��ֵ�ָ���СԲ�㷨
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="nThreshold">�ָ���ֵ</param>
/// <param name="roughPoint">��⵽�����ĵ�����</param>
/// <returns></returns>
bool CdetHoleMatrix::getLittleCircleAutoSeg(cv::Mat mSrcImage, int nThreshold, cv::Point2d &roughPoint)
{
    roughPoint = {0, 0};
    cv::Mat mThreshold;
    cv::adaptiveThreshold(mSrcImage, mThreshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
    Mat means, stddev, covar;
    meanStdDev(mThreshold, means, stddev); //����srcͼƬ�ľ�ֵ�ͱ�׼��
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
/// ͳ����ֵ�ָ���СԲ�㷨
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="roughPoint">��⵽�����ĵ�����</param>
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
    if (!m_bIsWhite) //��ɫ�б�
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
    else //��ɫ�б�
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
    CvWaitShowImage(mThresholdImage, "С�б��ֵ��");
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
/// ���������ϼ��СԲ�㷨
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="roughPoint">��⵽�����ĵ�����</param>
/// <returns></returns>
bool CdetHoleMatrix::getLittleCircleContourFit(cv::Mat mSrcImage, cv::Point2d &roughPoint)
{
    if (!mSrcImage.data)
        return false;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    float fRedius = 0;
    Point2f pContour(0, 0);            //��С���ԲԲ��
    cv::Point3d circleCenter(0, 0, 0); //��Ϻ��Բ��������
    //�˲���ʼ��
    CFilter filter;
    //�ݶȼ���ʼ��
    CGradient getGradient;
    //��ǿ��ʼ��
    CEnhance enhance(m_bIsWhite);
    cv::Mat mFilterImage, mEnhanceImage, mGradientImage, mGradientImage2;

    filter.getFilter(mDesImage, 3, "5", mFilterImage);
    enhance.getEnhanceImage(mFilterImage, 1, mEnhanceImage);
    //enhance.getEnhanceImage(mFilterImage, 4, mEnhanceImage, 0, 0, 400, 10);
    //enhance.getLocalEnhance(mFilterImage, 0.8, 1, 0.1, 0.8, mEnhanceImage);
    filter.getFilter(mEnhanceImage, 4, "3", mEnhanceImage);
    getGradient.getGradientImage(mEnhanceImage, mGradientImage, 0);
    //��������
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
    CvWaitShowImage(mShowImage, "����ͼ");
#endif
    if (contours.size() <= 0)
    {
        writeLog(m_strLog, "��������ʧ�ܣ�û����������������");
        return false;
    }
    sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) { return a.size() > b.size(); });
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

        if (abs(mSrcImage.cols / 2 - pt1.x) > 10 ||
            abs(mSrcImage.rows / 2 - pt1.y) > 10)
            continue;
        //��С���Բ
        minEnclosingCircle(contours[x], pContour, fRedius);
        //float fLength = arcLength(contours[i], true);
        float fArea = contourArea(contours[x]);
        if (fRedius >= m_nDiameter / 2 - m_nThreshold && fRedius <= m_nDiameter / 2 + m_nThreshold &&
            fArea > m_nDiameter / 4 * m_nDiameter / 4 * CV_PI && fArea < m_nDiameter * m_nDiameter * CV_PI)
        {
            writeLog(m_strLog, "Ŀ��ֱ�������У��ͨ��");
#if _DEBUG
            cv::Mat mShowImage2 = generateColorImage(mSrcImage);
            drawContours(mShowImage2, contours, x, Scalar(0, 0, 255), 1, 8, hierarchy);
            CvWaitShowImage(mShowImage2, "����ͼ2");
#endif
            //��ȷ����
            writeLog(m_strLog, "���þ�ȷ����");
            //�����������������ղ�ͬ���޽��з��ദ��
            scanQuadrantContour(mFilterImage, contours[x], pt1, circleCenter);
            roughPoint.x = circleCenter.x;
            roughPoint.y = circleCenter.y;

            return true;
        }
    }
    return false;
}

/// <summary>
/// ��������ɸѡ�㷨
/// </summary>
/// <param name="mSrcImage">����ͼƬ</param>
/// <param name="vecInput">��ɸѡ����</param>
/// <param name="pRoughCenter">СԲ�ֶ�λ���ĵ�����</param>
/// <param name="pCenter">СԲ��ȷ��⵽�Ĳ������</param>
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
                vecContours.push_back(ContoursPoint(pContours.x, pContours.y));
            }
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

// ��ϴ�Բ����
bool CdetHoleMatrix::getBigSingleCenter(cv::Mat mSrcImage, cv::Point2d &roughPoint)
{
    roughPoint.x = 0;
    roughPoint.y = 0;
    cv::Mat mGradientImage, mFilterImage, mEnhanceImage, mDesImage;
    mSrcImage.copyTo(mDesImage);
    CFilter filter;
    CGradient gradient;
    //��ǿ��ʼ��
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
    CvWaitShowImage(mShowImage, "ɸѡ���������");
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
    //У��뾶�Ƿ���Ϊ��������
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
        CvWaitShowImage(mShowImageResult, "ץȡ���");
#endif
        roughPoint.x = tempCircle.x;
        roughPoint.y = tempCircle.y;
        return true;
    }
    else
        return false;
}

/// <summary>
/// �Լ��ɹ���СԲ����ɸѡ
/// </summary>
/// <param name="vecCircleCenter">���ɹ�СԲ��������</param>
/// <param name="vecAccurateLocation">ɸѡ����������</param>
/// <param name="pHoleMatrixCenter">ɸѡ���������ĵ�����</param>
/// <param name="dScore">���÷�</param>
/// <returns></returns>
bool CdetHoleMatrix::analysisDetSuccessCircle(std::vector<std::vector<cv::Point2d>> vecCircleCenter,
                                              std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore)
{
    // �ѽ�����뷵��vec
    for (int i = 0; i < vecCircleCenter.size(); i++)
        for (int j = 0; j < vecCircleCenter[0].size(); j++)
            vecAccurateLocation.push_back(vecCircleCenter[i][j]);

    if (vecAccurateLocation.size() == m_nRows * m_nCols) //���԰б��ץȡ�ɹ�
    {
        writeLog(m_strLog, "����С�б��ץȡ�ɹ�");
        dScore += 40;
    }
    else
    {
        dScore += 20 + vecAccurateLocation.size() * 20 / m_nRows / m_nCols;
    }
    pHoleMatrixCenter = getMeanPoint(vecCircleCenter);
    return true;
}