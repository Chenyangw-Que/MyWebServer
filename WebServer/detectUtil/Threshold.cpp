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
    //初始化统计参数
    int numPix[256], threshold = 0;
    double nProDis[256];
    for (int i = 0; i < 256; i++)
        numPix[i] = 0;
    //统计灰度级中各个像素在整幅图像中的个数
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            numPix[(int)image.at<uchar>(i, j)]++;
    //统计每个弧度级占图像中的概率分布
    for (int i = 0; i < 256; i++)
        nProDis[i] = (double)numPix[i] / (row * col);
    //遍历灰度级，计算出最大类间方差的阈值
    double w0, w1, u0_temp, u1_temp, u0, u1, delta_temp, delta_max = 0;
    for (int i = 0; i < 256; i++)
    {
        w0 = w1 = u0_temp = u1_temp = delta_temp = 0;
        for (int j = 0; j < 256; j++)
        {
            //背景部分
            if (j <= i)
            {
                //当前i为分割阈值，第一类总的概率
                w0 += nProDis[j];
                u0_temp += j * nProDis[j];
            }
            //前景部分
            else
            {
                //当前i为分割阈值，第一类总的概率
                w1 += nProDis[j];
                u1_temp += j * nProDis[j];
            }
        }
        //分别计算各类的平均灰度值
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (double)(w0 * w1 * pow((u0 - u1), 2));
        //依次找到最大类间方差下的阈值
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
    //初始化统计参数
    int numPix[256], threshold = 0;
    double nProDis[256];
    for (int i = nMin; i < nMax; i++)
        numPix[i] = 0;
    //统计灰度级中各个像素在整幅图像中的个数
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            numPix[(int)image.at<uchar>(i, j)]++;
    //统计每个弧度级占图像中的概率分布
    for (int i = nMin; i < nMax; i++)
        nProDis[i] = (double)numPix[i] / (row * col);
    //遍历灰度级，计算出最大类间方差的阈值
    double w0, w1, u0_temp, u1_temp, u0, u1, delta_temp, delta_max = 0;
    for (int i = nMin; i < nMax; i++)
    {
        w0 = w1 = u0_temp = u1_temp = delta_temp = 0;
        for (int j = nMin; j < nMax; j++)
        {
            //背景部分
            if (j <= i)
            {
                //当前i为分割阈值，第一类总的概率
                w0 += nProDis[j];
                u0_temp += j * nProDis[j];
            }
            //前景部分
            else
            {
                //当前i为分割阈值，第一类总的概率
                w1 += nProDis[j];
                u1_temp += j * nProDis[j];
            }
        }
        //分别计算各类的平均灰度值
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (double)(w0 * w1 * pow((u0 - u1), 2));
        //依次找到最大类间方差下的阈值
        if (delta_temp > delta_max)
        {
            threshold = i;
            delta_max = delta_temp;
        }
    }
    return threshold;
}
