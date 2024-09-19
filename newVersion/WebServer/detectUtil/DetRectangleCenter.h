#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetRectangle
{
public:
    /// <summary>
    /// ���캯��
    /// </summary>
    /// <param name="strLog">��־�ļ�·��</param>
    /// <param name="param">�б�����ṹ��</param>
    /// <param name="algorithmPara">�㷨�����ṹ��</param>
    CdetRectangle(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetRectangle();

    /// <summary>
    /// ���μ���㷨
    /// </summary>
    /// <param name="mSrcImage">����ͼ��</param>
    /// <param name="bIsCalibration">�궨</param>
    /// <param name="nRoateAngle">��ת�Ƕ�</param>
    /// <param name="pCenterPoint">��⵽�ľ��β���</param>
    /// <param name="dScore">���÷�</param>
    /// <returns></returns>
    bool getRectangleCenter(const cv::Mat mSrcImage, bool bIsCalibration, int nRoateAngle,
                            Cfmee_Rectangle &pCenterPoint, double &dScore);

private:
    /// <summary>
    /// ����������������ϵ�
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="nRoateAngle"></param>
    /// <param name="pCenterPoint"></param>
    /// <param name="dScore"></param>
    /// <returns></returns>
    bool fourQuadrantLineFit(const cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //��ȡ���ֱ�߲���
    //ͨ��ɫ���ȡֱ�߷ֽ��ߣ�Ч�����Ǻ����룬�޷�ȷ����ѷֽ���ֵ
    bool getLineCenter(const cv::Mat mSrcImage, cv::Vec4f &line_para, double &a, double &b, double &c);

    //��������㷨
    bool getContoursCenter(cv::Mat mSrcImage, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //��Ե����ϼ��---��Ƕ���ת
    //bool getFitEdgeLine(cv::Mat mSrcImage, DetectParameterV3 Param, std::string strPath, Cfmee_Rectangle& pCenterPoint, double& dScore);

    //��Ե����ϼ��---С�Ƕ���ת
    bool getFitEdgeLine2(cv::Mat mSrcImage, int nRoateAngle, Cfmee_Rectangle &pCenterPoint, double &dScore);

    //�Ķ�������
    void sortRectanglePoint(Cfmee_Rectangle &pCenterPoint);

    // ���α�Եֱ�߼��
    bool getLinePara3(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop,
                      int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara4(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift,
                      int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara3Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool bisTop,
                           int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    bool getLinePara4Angel(cv::Mat mSrcImage, cv::Mat mThresholdImage, cv::Point pLoc, bool isLift,
                           int nRoateAngle, std::vector<std::vector<cv::Point>> &vec);

    //�ĵ�ȷ��ֱ�߽���
    cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4);

private:
    CdetRectangle &operator=(const CdetRectangle &);

private:
    std::string m_strLog; //��־�ĵ�
    int m_nWidth;         //ʮ�ֿ��
    int m_nHeight;        //ʮ�ָ߶�
    int m_nThreshold;     //������
    bool m_bIsWhite;      //�б�Ϊ��ɫ
    int m_nAlgorithm;     //ʶ���㷨
    double m_dAngle;      //��ת�Ƕ�
};
