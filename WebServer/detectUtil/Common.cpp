#include "Common.h"
#include <fstream>
#include "time.h"
#include "FilterImage.h"
#include "Hist.h"
#include"Threshold.h"
#include <queue>
using namespace std;
using namespace cv;
//获取当前时间
std::string getNowTime()
{
    return "zao";
}

/// <summary>
/// GrayBitmap转换Mat
/// </summary>
/// <param name="gBitmap"></param>
cv::Mat GrayBitmapToMat(GrayBitmap *gBitmap)
{
    Mat image, imageGray;
    if (gBitmap == NULL)
    {
        return imageGray;
    }
    int width = gBitmap->Width;
    int height = gBitmap->Height;
    int widthStep = gBitmap->Stride;
    int nChannels = gBitmap->Channels;
    if (nChannels == 1)
    {
        imageGray.create(height, widthStep, CV_8UC1);
        memcpy(imageGray.data, gBitmap->BitmapData, widthStep * height);
    }
    else
    {
        image.create(height, widthStep, CV_8UC3);
        memcpy(image.data, gBitmap->BitmapData, widthStep * height);
        cvtColor(image, imageGray, COLOR_BGR2GRAY);
        image.release();
    }
    return imageGray;
}




/// <summary>
/// 图像上画点
/// </summary>
/// <param name="mSrcImage">待算法图像</param>
/// <param name="color">颜色选择</param>
/// <param name="x">横坐标</param>
/// <param name="y">纵坐标</param>
/// <returns></returns>
void CvDrawPointOnImage(cv::Mat &mSrcImage, cv::Scalar color, int x, int y)
{
    if (!mSrcImage.data || x < 0 || y < 0 || x > mSrcImage.cols - 1 || y > mSrcImage.rows - 1)
        return;
    if (mSrcImage.channels() == 1)
    {
        mSrcImage.at<uchar>(y, x) == 0.11 * color.val[0] + 0.59 * color.val[1] + 0.3 * color.val[2];
    }
    else
    {
        mSrcImage.at<Vec3b>(y, x)[0] = color.val[0];
        mSrcImage.at<Vec3b>(y, x)[1] = color.val[1];
        mSrcImage.at<Vec3b>(y, x)[2] = color.val[2];
    }
}

/// <summary>
/// 保存图片名称
/// </summary>
/// <param name="nDiameter"></param>
/// <param name="nThreshold"></param>
/// <param name="strImagePath"></param>
void saveImage(std::vector<double> vecParam, std::string &strImagePath)
{
    time_t lt;
    lt = time(NULL);
    strImagePath = strImagePath + "//" + getNowTime() + "_";
    string strParam = "";
    for (int i = 0; i < vecParam.size(); ++i)
        strParam += to_string(vecParam[i]) + "_";
    strImagePath = strImagePath + strParam + ".jpg";
}

/// <summary>
/// 检测结果绘图保存
/// </summary>
/// <param name="strResultImagePath"></param>
/// <param name="mSrcImage"></param>
/// <param name="result"></param>
void saveResultImage(std::string strResultImagePath, cv::Mat mSrcImage, DetectResultV3 *result)
{
    for (int i = 0; i < result->CircleCount; ++i)
    {
        line(mSrcImage, Point(result->DetEveryResult[i].CenterX - 30, result->DetEveryResult[i].CenterY),
             Point(result->DetEveryResult[i].CenterX + 30, result->DetEveryResult[i].CenterY), Scalar(255, 0, 0), 1, 8);
        line(mSrcImage, Point(result->DetEveryResult[i].CenterX, result->DetEveryResult[i].CenterY - 30),
             Point(result->DetEveryResult[i].CenterX, result->DetEveryResult[i].CenterY + 30), Scalar(255, 0, 0), 1, 8);
        for (int j = 0; j < result->DetEveryResult[i].PartCount; ++j)
        {
            circle(mSrcImage, Point(result->DetEveryResult[i].PartResult[j].CenterX, result->DetEveryResult[i].PartResult[j].CenterY),
                   result->DetEveryResult[i].PartResult[j].Diameter / 2, cv::Scalar(0, 0, 255), 1, 8, 0);
        }
        if (result->MarkType != MT_RECTANGLE && result->MarkType != MT_CROSS)
            circle(mSrcImage, Point(result->DetEveryResult[i].CenterX, result->DetEveryResult[i].CenterY),
                   result->DetEveryResult[i].Diameter / 2, cv::Scalar(0, 0, 255), 1, 8, 0);
    }
    imwrite(strResultImagePath, mSrcImage);
}

/// <summary>
/// 判断模板背景颜色
/// </summary>
/// <param name="mSrcImage"></param>
/// <returns></returns>
bool isWhiteBackground(cv::Mat mSrcImage)
{
    int nOnePixel = mSrcImage.at<uchar>(1, 1);
    int nTwoPixel = mSrcImage.at<uchar>(mSrcImage.rows - 1, 1);
    int nThreePixel = mSrcImage.at<uchar>(1, mSrcImage.cols - 1);
    int nFourPixel = mSrcImage.at<uchar>(mSrcImage.rows - 1, mSrcImage.cols - 1);
    if (nOnePixel > 200 && nTwoPixel > 200 && nThreePixel > 200 && nFourPixel > 200)
        return true;
    else
        return false;
}

/// <summary>
/// 模板图像按中心点旋转
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="fAngle"></param>
/// <returns></returns>
cv::Mat rotateImageCenter(cv::Mat mSrcImage, float fAngle)
{
    cv::Mat mDesImage = mSrcImage.clone();
    fAngle = -fAngle;
    double angle = fAngle * CV_PI / 180; // 弧度
    double a = sin(angle), b = cos(angle);
    int width = mSrcImage.cols;
    int height = mSrcImage.rows;
    int width_rotate = int(height * fabs(a) + width * fabs(b));
    int height_rotate = int(width * fabs(a) + height * fabs(b));

    Mat map_matrix = Mat(2, 3, CV_64FC1);
    // 旋转中心
    cv::Point2f center = Point2f(width / 2, height / 2);

    map_matrix = getRotationMatrix2D(center, fAngle, 1.0);
    //扩充
    map_matrix.at<double>(0, 2) += (width_rotate - width) / 2;
    map_matrix.at<double>(1, 2) += (height_rotate - height) / 2;
    Mat img_rotate;
    bool bIsRoate = false;

    if (isWhiteBackground(mSrcImage))
    {
        bIsRoate = true;
        mDesImage = ~mSrcImage;
    }
    warpAffine(mDesImage, img_rotate, map_matrix, Size(width_rotate, height_rotate), 1, 0, 0);

    if (bIsRoate)
        img_rotate = ~img_rotate;

    return img_rotate;
}

/// <summary>
/// 多模板匹配时最终返回结果
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="result"></param>
void getLastCenter(const cv::Mat mSrcImage, DetectResultV3 *result)
{
    double dDis = 100000;
    for (int x = 0; x < result->CircleCount; ++x)
    {
        double dCenterDis = sqrt(pow(result->DetEveryResult[x].CenterX - mSrcImage.cols / 2, 2) + pow(result->DetEveryResult[x].CenterY - mSrcImage.rows / 2, 2));
        if (dCenterDis < dDis)
        {
            result->CenterX = result->DetEveryResult[x].CenterX;
            result->CenterY = result->DetEveryResult[x].CenterY;
            result->Width = result->DetEveryResult[x].Width;
            result->Height = result->DetEveryResult[x].Height;
            result->Diameter = result->DetEveryResult[x].Diameter;
            result->DetScore = result->DetEveryResult[x].DetScore;
            dDis = dCenterDis;
        }
    }
}

/// <summary>
/// 创建彩色图
/// </summary>
/// <param name="mGrayImage">输入图片</param>
/// <returns></returns>
cv::Mat generateColorImage(cv::Mat mGrayImage)
{
    cv::Mat mImage;
    if (mGrayImage.channels() > 1)
        return mImage;
    vector<cv::Mat> vecchannel;
    for (int i = 0; i < 3; ++i)
    {
        vecchannel.push_back(mGrayImage);
    }
    merge(vecchannel, mImage);
    return mImage;
}

	/// <summary>
	/// 检测图片是否是全白/黑图
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <returns></returns>
	int isPureColorImage(const cv::Mat mSrcImage)
	{
		cv::Mat mDesImage, mFilterImage, mThresholdImage;
		mSrcImage.copyTo(mDesImage);
		CFilter filter;
		filter.getFilter(mDesImage, 3, "3", mFilterImage);
		filter.getFilter(mFilterImage, 4, "3", mFilterImage);
		int nTheshold = OtsuThres(mFilterImage);
		cv::threshold(mFilterImage, mThresholdImage, nTheshold, 255, THRESH_BINARY);
#if _DEBUG	
		CvWaitShowImage(mThresholdImage, "二值图片");
#endif			
		MatND DesHist = getImageHist(mThresholdImage);
		float nSumPixel = mSrcImage.cols * mSrcImage.rows;
		float nBlackPixelNumber = DesHist.at<float>(0) + DesHist.at<float>(1);
		float nWhitePixelNumber = DesHist.at<float>(255) + DesHist.at<float>(254);
		if (nBlackPixelNumber / nSumPixel > 0.99)
			return 1;//全黑图片
		else if (nWhitePixelNumber / nSumPixel > 0.99)
			return 2;//全白图片
		else
			return 0;//正常图片
	}


/// <summary>
/// 自定义直线参数获取两点坐标
/// </summary>
/// <param name="mSrcImage">输入图像</param>
/// <param name="a"></param>
/// <param name="b"></param>
/// <param name="c"></param>
/// <returns></returns>
std::vector<cv::Point2d> transLinePara(cv::Mat mSrcImage, double a, double b, double c)
{
    std::vector<cv::Point2d> vecLine;
    //竖直线段，斜率无限大
    if (b == 0 && a == 0)
    {
        cv::Point2d pt1(c, 0);
        cv::Point2d pt2(c, mSrcImage.rows);
        vecLine.push_back(pt1);
        vecLine.push_back(pt2);
    }
    else
    {
        double cos_theta = b;
        double sin_theta = a;

        double phi = atan2(sin_theta, cos_theta) + CV_PI / 2.0;
        if (abs(sin_theta) >= abs(cos_theta)) // ~vertical line
        {
            cv::Point2d pt1(-c / a, 0);
            cv::Point2d pt2((-c - mSrcImage.rows * b) / a, mSrcImage.rows);
            vecLine.push_back(pt1);
            vecLine.push_back(pt2);
        }
        else
        {
            cv::Point2d pt1(0, -c / b);
            cv::Point2d pt2(mSrcImage.cols, (-c - mSrcImage.cols * a) / b);
            vecLine.push_back(pt1);
            vecLine.push_back(pt2);
        }
    }

    return vecLine;
}
//根据直线参数获取两点坐标
std::vector<cv::Point2d> transCVLinePara(cv::Mat mSrcImage, cv::Vec4f line_para)
{
    std::vector<cv::Point2d> vecLine;
    double cos_theta = line_para[0];
    double sin_theta = line_para[1];
    double x0 = line_para[2], y0 = line_para[3];

    double theta = atan2(sin_theta, cos_theta) + CV_PI / 2.0;
    //theta = CV_PI - theta;
    double rho = y0 * cos_theta - x0 * sin_theta;
    if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) // ~vertical line
    {
        cv::Point2d pt1(rho / cos(theta), 0);
        cv::Point2d pt2((rho - mSrcImage.rows * sin(theta)) / cos(theta), mSrcImage.rows);
        vecLine.push_back(pt1);
        vecLine.push_back(pt2);
    }
    else
    {
        cv::Point2d pt1(0, rho / sin(theta));
        cv::Point2d pt2(mSrcImage.cols, (rho - mSrcImage.cols * cos(theta)) / sin(theta));
        vecLine.push_back(pt1);
        vecLine.push_back(pt2);
    }
    return vecLine;
}

//四点确定直线交点
cv::Point2d CrossPoint(const cv::Point2d line1, const cv::Point2d line2, const cv::Point2d line3, const cv::Point2d line4)
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

    return cross_point; //平行返回(0,0)
}
void DistanceOfPointToCircle(float fPtX, float fPtY,
                             float fCenterX, float fCenterY, float fRadius,
                             float &fDist)
{
  //float dist=0;

  float fEx = fPtX - fCenterX;
  float fEy = fPtY - fCenterY;

  fDist = abs(sqrt(fEx * fEx + fEy * fEy) - fRadius);
  //fDist = abs( FastSqrtFloat( fEx*fEx + fEy*fEy ) -fRadius );

  return;
}