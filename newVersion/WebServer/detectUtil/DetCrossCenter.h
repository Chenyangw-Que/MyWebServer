#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetCross
{
public:
    /// <summary>
    /// ���캯��
    /// </summary>
    /// <param name="strLog">��־�ļ�·��</param>
    /// <param name="param">�б�����ṹ��</param>
    /// <param name="algorithmPara">�㷨�����ṹ��</param>
    CdetCross(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetCross();

    /// <summary>
    /// ͨ��ʮ�ּ���㷨
    /// </summary>
    /// <param name="mSrcImage">������ͼƬ</param>
    /// <param name="bIsCalibrate">�궨���</param>
    /// <param name="pCenterPoint">���б����ĵ�</param>
    /// <param name="dScore">�б���÷�</param>
    /// <returns></returns>
    bool getCrossCenter(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, double &dScore);

    /// <summary>
    /// ͨ��ʮ�ּ���㷨,���������Ľǵ�
    /// </summary>
    /// <param name="mSrcImage">������ͼƬ</param>
    /// <param name="bIsCalibrate">�궨���</param>
    /// <param name="pCenterPoint">���б����ĵ�</param>
    /// <param name="dScore">�б���÷�</param>
    /// <returns></returns>
    bool getCrossCenterFourPoint(const cv::Mat mSrcImage, bool bIsCalibrate, cv::Point2d &pCenterPoint, std::vector<cv::Point2d> &fourPoint, double &dScore);

    /// <summary>
    /// �뵼�帴��ʮ�ּ���㷨
    /// </summary>
    /// <param name="mSrcImage">������ͼƬ</param>
    /// <param name="vecCenterPoint">���б����ĵ㼯��</param>
    /// <param name="dScore">�б���÷�</param>
    /// <returns></returns>
    bool getSEMIComplexCrossCenter(const cv::Mat mSrcImage, std::vector<cv::Point2d> &vecCenterPoint, double &dScore);

private:
    CdetCross &operator=(const CdetCross &);

    /// <summary>
    /// ��ֱ�߼���㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼ��</param>
    /// <param name="pCenterPoint">ֱ�����ĵ�����</param>
    /// <returns></returns>
    bool detSingleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint, std::vector<cv::Point2d> &fourPoint);

    /// <summary>
    /// ƽ��˫ֱ�߼���㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼ��</param>
    /// <param name="pCenterPoint">ֱ�����ĵ�����</param>
    /// <returns></returns>
    bool detDoubleLine(const cv::Mat mSrcImage, cv::Point2d &pCenterPoint);

    /// <summary>
    /// ��ֱ�������ȡ
    /// </summary>
    /// <param name="mSrcImage">�����ͼƬ</param>
    /// <param name="nLength">��</param>
    /// <param name="nBorderWidth">��</param>
    /// <returns></returns>
    std::vector<std::pair<cv::Mat, cv::Point>> getSegmentLine(const cv::Mat mSrcImage,
                                                              int nLength, int nBorderWidth);

private:
    /// <summary>
    /// ���������ֱ��
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="vecPoint">��⵽��ֱ�������</param>
    /// <returns></returns>
    bool detLiftVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// ���������ֱ��
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="vecPoint">��⵽��ֱ�������</param>
    /// <returns></returns>
    bool detRightVolLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// ����Ϻ���ֱ��
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="vecPoint">��⵽��ֱ�������</param>
    /// <returns></returns>
    bool detTopHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    /// <summary>
    /// ����º���ֱ��
    /// </summary>
    /// <param name="mSrcImage">����ͼƬ</param>
    /// <param name="vecPoint">��⵽��ֱ�������</param>
    /// <returns></returns>
    bool detDownHorLine(const cv::Mat mSrcImage, std::vector<std::vector<cv::Point>> &vecPoint);

    //��ȡ���ֱ�߲���
    //ͨ��ɫ���ȡֱ�߷ֽ��ߣ�Ч�����Ǻ����룬�޷�ȷ����ѷֽ���ֵ
    bool getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c);

    //�ĵ�ȷ��ֱ�߽���
    cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4);

private:
    std::string m_strLog;    //��־�ĵ�
    int m_nWidth;            //ʮ�ֿ��
    int m_nHeight;           //ʮ�ָ߶�
    int m_nDimeter;          //ֱ������
    int m_nThreshold;        //������
    bool m_bIsWhite;         //�б�Ϊ��ɫ
    int m_nAlgorithm;        //ʶ���㷨
    double m_dLineCount;     //����ʮ���߶���
    double m_dAngle;         //��ת�Ƕ�
    bool m_bIsPolarChange;   //���Լ��
    bool m_bIsInnerToExtern; //���ڵ���ɸѡ
};
