#include "Threshold.h"

int OtsuThres(int pHist[256])
{
    double g = 0, G = 0, g0 = 0, n = 0;
    for (int i = 0; i < 256; i++)
    {
        n += pHist[i];
        G += i * pHist[i];
    }

    double n0 = 0, n1 = n, g1 = G;
    int nThreshold = 100;
    for (int i = 0; i < 256; i++)
    {
        n0 += pHist[i];
        n1 -= pHist[i];

        g0 += i * pHist[i];
        g1 -= i * pHist[i];

        if (n0 == 0 || n1 == 0 || n == 0)
        {
            continue;
        }

        double miu0 = g0 / n0;
        double miu1 = g1 / n1;
        double w0 = n0 / n;
        double w1 = n1 / n;

        double miu = w0 * miu0 + w1 * miu1;
        double cg = w0 * (miu0 - miu) * (miu0 - miu) + w1 * (miu1 - miu) * (miu1 - miu);
        if (cg > g)
        {
            g = cg;
            nThreshold = i;
        }
    }
    return nThreshold;
}

int OtsuThres(cv::Mat image)
{
    int row = image.rows, col = image.cols;
    //��ʼ��ͳ�Ʋ���
    int numPix[256], threshold = 0;
    double nProDis[256];
    for (int i = 0; i < 256; i++)
        numPix[i] = 0;
    //ͳ�ƻҶȼ��и�������������ͼ���еĸ���
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            numPix[(int)image.at<uchar>(i, j)]++;
    //ͳ��ÿ�����ȼ�ռͼ���еĸ��ʷֲ�
    for (int i = 0; i < 256; i++)
        nProDis[i] = (double)numPix[i] / (row * col);
    //�����Ҷȼ�������������䷽�����ֵ
    double w0, w1, u0_temp, u1_temp, u0, u1, delta_temp, delta_max = 0;
    for (int i = 0; i < 256; i++)
    {
        w0 = w1 = u0_temp = u1_temp = delta_temp = 0;
        for (int j = 0; j < 256; j++)
        {
            //��������
            if (j <= i)
            {
                //��ǰiΪ�ָ���ֵ����һ���ܵĸ���
                w0 += nProDis[j];
                u0_temp += j * nProDis[j];
            }
            //ǰ������
            else
            {
                //��ǰiΪ�ָ���ֵ����һ���ܵĸ���
                w1 += nProDis[j];
                u1_temp += j * nProDis[j];
            }
        }
        //�ֱ��������ƽ���Ҷ�ֵ
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (double)(w0 * w1 * pow((u0 - u1), 2));
        //�����ҵ������䷽���µ���ֵ
        if (delta_temp > delta_max)
        {
            threshold = i;
            delta_max = delta_temp;
        }
    }
    return threshold;
}

int OtsuThres(cv::Mat image, int nMin, int nMax)
{
    if (nMin < 0 || nMax > 256)
        return 0;
    int row = image.rows, col = image.cols;
    //��ʼ��ͳ�Ʋ���
    int numPix[256], threshold = 0;
    double nProDis[256];
    for (int i = nMin; i < nMax; i++)
        numPix[i] = 0;
    //ͳ�ƻҶȼ��и�������������ͼ���еĸ���
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            numPix[(int)image.at<uchar>(i, j)]++;
    //ͳ��ÿ�����ȼ�ռͼ���еĸ��ʷֲ�
    for (int i = nMin; i < nMax; i++)
        nProDis[i] = (double)numPix[i] / (row * col);
    //�����Ҷȼ�������������䷽�����ֵ
    double w0, w1, u0_temp, u1_temp, u0, u1, delta_temp, delta_max = 0;
    for (int i = nMin; i < nMax; i++)
    {
        w0 = w1 = u0_temp = u1_temp = delta_temp = 0;
        for (int j = nMin; j < nMax; j++)
        {
            //��������
            if (j <= i)
            {
                //��ǰiΪ�ָ���ֵ����һ���ܵĸ���
                w0 += nProDis[j];
                u0_temp += j * nProDis[j];
            }
            //ǰ������
            else
            {
                //��ǰiΪ�ָ���ֵ����һ���ܵĸ���
                w1 += nProDis[j];
                u1_temp += j * nProDis[j];
            }
        }
        //�ֱ��������ƽ���Ҷ�ֵ
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (double)(w0 * w1 * pow((u0 - u1), 2));
        //�����ҵ������䷽���µ���ֵ
        if (delta_temp > delta_max)
        {
            threshold = i;
            delta_max = delta_temp;
        }
    }
    return threshold;
}
