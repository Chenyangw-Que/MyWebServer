#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

class CdetPlum
{
public:
    /// <summary>
    /// ���캯��
    /// </summary>
    /// <param name="strLog">��־�ļ�·��</param>
    /// <param name="param">�б�����ṹ��</param>
    /// <param name="algorithmPara">�㷨�����ṹ��</param>
    CdetPlum(std::string strLog, DetectParameterV3 param, ConfigAlgorithmParam algorithmPara);
    ~CdetPlum();

    //**********************����÷���׼���㷨********************
    /// <summary>
    /// ���ÿһ��СԲ����
    /// </summary>
    /// <param name="mSrcImage">���ͼƬ</param>
    /// <param name="vecLittleCenter">���ɹ���СԲ����</param>
    /// <param name="dScore">�÷�</param>
    /// <returns></returns>
    bool getLittleObjectCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// ʹ�÷ָ���������ֱ�Ӷ�λ��ÿ��СԲ
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <param name="dScore"></param>
    /// <returns></returns>
    bool getLittleObjectCenterDirect(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// ʹ�����ѧϰ�������ǿԲ���
    /// ���ִ�СԲ�뾶
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="refinedRoi"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

private:
    CdetPlum &operator=(const CdetPlum &);
    //**********************����÷���׼���㷨********************

    /// <summary>
    /// �㷨һ�������Ʋ�+�Զ���ֵ���
    /// ���㷢�ɹ��ʺܸߣ����������Ҳ�ܸߣ�Ӧ��ʹ�ã����Ƽ�
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getDistancePredictCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// �㷨�������㷨�����ƻ��ɷ֣�����������ܲ�ͨ��
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getSegmentImageCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// �㷨������ǿԲ���
    /// Ĭ��ʹ�ô��㷨
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// �㷨�ģ�ʹ�����ѧϰ�������ǿԲ���
    /// �Զ������Ĭ��ʹ�ô��㷨
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getLittleCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter);

    /// <summary>
    /// ʹ�����ѧϰ�������ǿ��ԲԲ���
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="refinedRoi"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool getBigCircleCenterDLPrior(cv::Mat mSrcImage, const std::vector<std::pair<cv::Mat, cv::Point>> refinedRoi, std::vector<cv::Point3d> &vecLittleCenter, double &dScore);

    /// <summary>
    /// �����㷨����
    /// �����ڼ�����ķ���İ�ɫСԲ������СԲֱ��Ҫ��ϸ�
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecLittleCenter"></param>
    /// <returns></returns>
    bool customLittleCircleCenter(cv::Mat mSrcImage, std::vector<cv::Point3d> &vecLittleCenter);

private:
    /// <summary>
    /// �ֶ�λСԲͼƬ
    /// </summary>
    /// <param name="mSrcImage">�����ͼƬ</param>
    /// <param name="pCircleCenter">�ο����ĵ�����</param>
    /// <param name="nType">��ȡСԲ���귽ʽ</param>
    /// <param name="vecMap">ÿ��СԲ�������</param>
    /// <returns></returns>
    int getLittleCircleLocation(cv::Mat mSrcImage, cv::Point2d pCircleCenter, int nType, std::vector<std::pair<cv::Mat, cv::Point>> &vecMap);

    /// <summary>
    /// �����������ɾѡ
    /// </summary>
    /// <param name="mSrcImage"></param>
    /// <param name="vecInput"></param>
    /// <param name="pRoughCenter"></param>
    /// <param name="pCenter"></param>
    /// <returns></returns>
    int scanQuadrantContour(cv::Mat mSrcImage, std::vector<cv::Point> vecInput, cv::Point2d pRoughCenter, cv::Point3d &pCenter);

    //��ȡСԲ����֮�����Ѽ��
    int getOptimumCenterInterval();

private:
    std::string m_strLog;  //��־�ĵ�
    int m_nAlgorithm;      //�㷨
    int m_nLittleCount;    //СԲ����
    int m_nBigDiameter;    //��Χֱ��
    int m_nLittleDiameter; //��Բֱ��
    int m_nThreshold;      //������
    bool m_bIsWhite;       //�б�Ϊ��ɫ
    double m_dAngle;       //÷������ת�Ƕ�
    bool m_bWhiteEdge;     //��ɫ�б��Ե
    bool m_bIsPolarChange; //���Լ��
    bool m_bIsEnhance;     //ͼ����ǿ
    double m_dCircleRate;  //СԲ����
};
