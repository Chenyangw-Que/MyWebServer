#include "FilterImage.h"
#include "Common.h"
#include <string>
using namespace std;
using namespace cv;

CFilter::CFilter()
{
}

CFilter::~CFilter()
{
}

/// <summary>
/// ÂË²¨
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="nType"></param>
/// <param name="strParament"></param>
/// <param name="mDesImage"></param>
void CFilter::getFilter(cv::Mat mSrcImage, int nType, std::string strParament, cv::Mat &mDesImage)
{
    string strDesImagePath = "ÂË²¨";
    if (!mSrcImage.data)
    {
        std::cout << "ÂË²¨ falied to read" << std::endl;
        return;
    }
    if (mSrcImage.channels() > 1)
        return;
    switch (nType)
    {
    case 0:
        break;
    case 1:
        if (!gaussianBlurFilter(mSrcImage, strParament, mDesImage))
        {
        }
        break;
    case 2:
        break;
    case 3:
        if (!bilateralFilter(mSrcImage, strParament, mDesImage))
        {
        }
        break;
    case 4:
        if (!medianBlurFilter(mSrcImage, strParament, mDesImage))
        {
        }
        break;
    case 5:
        break;
    default:
        break;
    }

#ifdef _SHOW_PROCESS_FILTER_
    CvWaitShowImage(mDesImage, "filter_Image");
#endif
}

bool CFilter::gaussianBlurFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage)
{
    if (!mSrcImage.data)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    if (mSrcImage.channels() > 1)
        return false;
    cv::Mat mOutImage(mSrcImage.rows, mSrcImage.cols, mSrcImage.type());
    cv::GaussianBlur(mSrcImage, mOutImage, cv::Size(atoi(strParament.c_str()) * 2 + 1, atoi(strParament.c_str()) * 2 + 1), 0, 0);
    mOutImage.copyTo(mDesImage);
    return true;
}
/// <summary>
/// ÖÐÖµÂË²¨
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="strParament"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CFilter::medianBlurFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage)
{
    if (!mSrcImage.data)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    if (mSrcImage.channels() > 1)
        return false;
    cv::Mat mOutImage(mSrcImage.rows, mSrcImage.cols, mSrcImage.type());
    cv::medianBlur(mSrcImage, mOutImage, atoi(strParament.c_str()) * 2 + 1);
    mOutImage.copyTo(mDesImage);
    return true;
}
/// <summary>
/// Ë«±ßÂË²¨-±£³Ö±ßÔµ¡¢½µÔëÆ½»¬
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="strParament"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CFilter::bilateralFilter(cv::Mat mSrcImage, std::string strParament, cv::Mat &mDesImage)
{
    if (!mSrcImage.data)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    if (mSrcImage.channels() > 1)
        return false;

    cv::Mat mOutImage(mSrcImage.rows, mSrcImage.cols, mSrcImage.type());
    cv::bilateralFilter(mSrcImage, mOutImage, atoi(strParament.c_str()), atoi(strParament.c_str()) * 2, atoi(strParament.c_str()) / 2);
    mOutImage.copyTo(mDesImage);
    return true;
}