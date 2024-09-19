#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"
class CdetHoleMatrix
{
public:
    /// <summary>
    /// ���캯��
    /// </summary>
    /// <param name="strLog">��־�ļ�·��</param>
    /// <param name="param">�б�����ṹ��</param>
    /// <param name="algorithmPara">�㷨�����ṹ��</param>
    CdetHoleMatrix(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetHoleMatrix();

    /// <summary>
    /// ��ȡÿһ��СԲ��Բ������
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="dAngle">��ת�Ƕ�</param>
    /// <param name="vecEveryHoleMatrixPoint">��⵽ÿһ��СԲ����</param>
    /// <param name="pHoleMatrixCenter">�б����ĵ�</param>
    /// <returns></returns>
    bool getEveryLittleCircle(const cv::Mat mSrcImage, double dAngle, std::vector<cv::Point2d> &vecEveryAccurateLocation,
                              cv::Point2d &pHoleMatrixCenter, double &dScore);

    /// <summary>
    /// ֱ�Ӷ�λСԲ�����ü���Բ�ļ����㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="dAngle">��ת�Ƕ�</param>
    /// <param name="vecEveryHoleMatrixPoint">��⵽ÿһ��СԲ����</param>
    /// <param name="pHoleMatrixCenter">�б����ĵ�</param>
    /// <returns></returns>
    bool getEveryLittleCircleDirect(const cv::Mat mSrcImage, const std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> refinedRoi, std::vector<cv::Point2d> &vecEveryAccurateLocation,
                                    cv::Point2d &pHoleMatrixCenter, double &dScore);

    //���ɸ��е�����ֵ
    cv::Point2d getMeanPoint(std::vector<std::vector<cv::Point2d>> one);

private:
    CdetHoleMatrix &operator=(const CdetHoleMatrix &);

    /// <summary>
    /// ͨ�����ȷ���Ƿ��������Բ
    /// </summary>
    /// <returns></returns>
    bool isExistCenterMark();

    /// <summary>
    /// �Ʋ�ÿ���б�Ĵ������ĵ�λ��
    /// </summary>
    /// <param name="mSrcImage">����ͼ��</param>
    /// <returns></returns>
    std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> getRoughPointLocation(cv::Mat mSrcImage);

    /// <summary>
    /// ���ÿһ��СԲ�����ĵ�����
    /// </summary>
    /// <param name="mSrcImage">����ͼ��</param>
    /// <param name="vecRoughLocation">�ֶ�λ����</param>
    /// <param name="vecAccurateLocation">��ȷ�������</param>
    /// <param name="pHoleMatrixCenter">�׾������ĵ�����</param>
    /// <param name="dScore">���÷�</param>
    /// <returns></returns>
    bool detEveryCircleLocation(cv::Mat mSrcImage, std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore);

    /// <summary>
    /// ��ֱ����Բ���
    /// </summary>
    /// <param name="vecRoughLocation">ÿһ����Բ�洢����</param>
    /// <param name="vecCircleCenter">�����</param>
    /// <returns></returns>
    bool detEveryBigCircleDLPrior(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                                  std::vector<std::vector<cv::Point2d>> &vecCircleCenter);

    /// <summary>
    /// �Լ��ɹ���СԲ����ɸѡ
    /// </summary>
    /// <param name="vecCircleCenter">���ɹ�СԲ��������</param>
    /// <param name="vecAccurateLocation">ɸѡ����������</param>
    /// <param name="pHoleMatrixCenter">ɸѡ���������ĵ�����</param>
    /// <param name="dScore">���÷�</param>
    /// <returns></returns>
    bool analysisDetSuccessCircle(std::vector<std::vector<cv::Point2d>> vecCircleCenter,
                                  std::vector<cv::Point2d> &vecAccurateLocation, cv::Point2d &pHoleMatrixCenter, double &dScore);

private:
    //*********************************Сֱ����Բ���*********************************
    /// <summary>
    /// Сֱ����Բ���
    /// </summary>
    /// <param name="vecRoughLocation">ÿһ����Բ�洢����</param>
    /// <param name="vecCircleCenter">�����</param>
    /// <returns></returns>
    bool detEveryLittleCircle(std::vector<std::vector<std::pair<cv::Mat, cv::Point>>> vecRoughLocation,
                              std::vector<std::vector<cv::Point2d>> &vecCircleCenter);

    /// <summary>
    /// ����Ӧ��ֵ�ָ���СԲ�㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="nThreshold">�ָ���ֵ</param>
    /// <param name="roughPoint">��⵽�����ĵ�����</param>
    /// <returns></returns>
    bool getLittleCircleAutoSeg(cv::Mat mSrcImage, int nThreshold, cv::Point2d &roughPoint);

    /// <summary>
    /// ͳ����ֵ�ָ���СԲ�㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="roughPoint">��⵽�����ĵ�����</param>
    /// <returns></returns>
    bool getLittleCircleStatisSeg(cv::Mat mSrcImage, cv::Point2d &roughPoint);

    /// <summary>
    /// ���������ϼ��СԲ�㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="roughPoint">��⵽�����ĵ�����</param>
    /// <returns></returns>
    bool getLittleCircleContourFit(cv::Mat mSrcImage, cv::Point2d &roughPoint);

    /// <summary>
    /// ��������ɸѡ�㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="vecInput">��ɸѡ����</param>
    /// <param name="pRoughCenter">СԲ�ֶ�λ���ĵ�����</param>
    /// <param name="pCenter">СԲ��ȷ��⵽�Ĳ������</param>
    /// <returns></returns>
    int scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput,
                            cv::Point2d pRoughCenter, cv::Point3d &pCenter);

    //��ȡ��ֱ���б�����
    bool getBigSingleCenter(cv::Mat mSrcImage, cv::Point2d &roughPoint);

private:
    std::string m_strLog;     //��־�ĵ�
    int m_nRows;              //��
    int m_nCols;              //��
    int m_RowSpan;            //�м��
    int m_ColSpan;            //�м��
    int m_nDiameter;          //ֱ������
    int m_nThreshold;         //������
    int m_nSpanRange;         //ֱ�������
    bool m_bIsWhite;          //�б�Ϊ��ɫ
    int m_nAlgorithm;         //ʶ���㷨
    bool m_bWhiteEdge;        //��ɫ�б��Ե
    bool m_bIsPolarChange;    //���Լ��
    float m_fHoleMatrixSocre; //�����ֵ����
    bool m_bIsEnhance;        //ͼ����ǿ
    double m_dAngle;          //��ת�Ƕ�
};
