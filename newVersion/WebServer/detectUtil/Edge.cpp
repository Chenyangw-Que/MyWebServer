#pragma once
#include "Edge.h"
void ConnectEdge(const cv::Mat SrcImage, cv::Mat &DesImage)
{
    SrcImage.copyTo(DesImage);

    if (!SrcImage.data)
        return;
    for (int i = 3; i < SrcImage.rows - 3; ++i)
    {
        for (int j = 3; j < SrcImage.cols - 3; ++j)
        {
            if (SrcImage.at<uchar>(i, j) >= 220)
            {
                int nNum = 0;
                for (int x = -1; x < 2; ++x)
                {
                    for (int y = -1; y < 2; ++y)
                    {
                        if (x != 0 && y != 0 && SrcImage.at<uchar>(i + x, j + y) >= 220)
                            nNum++;
                    }
                }
                if (nNum == 1)
                {
                    for (int k = -2; k < 3; k++)
                    {
                        for (int l = -2; l < 3; l++)
                        {
                            //如果该点的十六邻域中有255的点，则该点与中心点之间的点置为255
                            if (!(k < 2 && k > -2 && l < 2 && l > -2) && SrcImage.at<uchar>(i + k, j + l) >= 220)
                                DesImage.at<uchar>(i + k / 2, j + l / 2) = 255;
                        }
                    }
                }
            }
        }
    }
}