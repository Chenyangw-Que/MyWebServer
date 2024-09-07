#include "EnhanceImage.h"
#include "Common.h"
using namespace cv;
using namespace std;

CEnhance::CEnhance()
{
}

CEnhance::CEnhance(bool bIsWhite)
{
}

CEnhance::~CEnhance()
{
}

/// <summary>
/// 图像增强
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="nAlgorithm"></param>
/// <param name="mDesImageint"></param>
/// <param name="nDiameter"></param>
/// <param name="nDiameter2"></param>
/// <param name="n"></param>
/// <param name="factor"></param>
/// <returns></returns>
bool CEnhance::getEnhanceImage(const cv::Mat mSrcImage, int nAlgorithm, cv::Mat &mDesImage,
                               int nDiameter, int nDiameter2, int n, double factor)
{
    if (!mSrcImage.data || mSrcImage.channels() > 1)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    switch (nAlgorithm)
    {
    case 0:
        //对数变换增强
        getLogImage(mSrcImage, mDesImage);
        break;
    case 1:
        //伽马变换增强
        getGammaImage(mSrcImage, mDesImage);
        break;
    case 2:
        //高反差保留
        getHighPass(mSrcImage, mDesImage);
        break;
    case 3:
        //直方图均衡
        equliaze_hist(mSrcImage, mDesImage, nDiameter, nDiameter2);
        break;
    case 4:
        //高频对比度增强
        getEmphasize(mSrcImage, mDesImage, n, factor);
        break;
    case 5:
        //图像整体增强
        EmphasizeScale(mSrcImage, mDesImage, n, factor);
        break;
    default:
        break;
    }
    return true;
}

/// <summary>
/// 傅里叶变换
/// </summary>
/// <param name="mSrcImage-输入图片"></param>
/// <param name="nAlgorithm-处理算法"></param>
/// <param name="mDesImage-输入图片"></param>
/// <param name="nDiameter-中心区域直径"></param>
/// <param name="width-线宽"></param>
/// <returns></returns>
bool CEnhance::FourierTransform(const cv::Mat mSrcImage, int nAlgorithm, cv::Mat &mDesImage, bool bHor, int nDiameter, int width)
{
    if (!mSrcImage.data)
        return false;
    //2.将单通道图像转换成双通道图像
    cv::Mat mImg2;
    mSrcImage.convertTo(mImg2, CV_32FC1);
    //创建通道，存储dft后的实部与虚部（CV_32F，必须为单通道数）
    cv::Mat plane[] = {mImg2.clone(), cv::Mat::zeros(mImg2.size(), CV_32FC1)};
    cv::Mat complexIm;
    cv::merge(plane, 2, complexIm);   // 合并通道 （把两个矩阵合并为一个2通道的Mat类容器）
    cv::dft(complexIm, complexIm, 0); // 进行傅立叶变换，结果保存在自身

    // 分离通道（数组分离）
    cv::split(complexIm, plane);
    // 以下的操作是频域迁移
    dftshift(plane[0], plane[1]);
    // 计算幅值
    cv::Mat mag, ph, mag_log, mag_nor;
    phase(plane[0], plane[1], ph);
    cv::magnitude(plane[0], plane[1], mag);
    //幅值对数化：log（1+m），便于观察频谱信息
    mag += Scalar::all(1);
    cv::log(mag, mag_log);
    cv::normalize(mag_log, mag_nor, 1, 0, NORM_MINMAX);
#if _DEBUG
    CvWaitShowImage(mag_nor, "傅里叶频谱图");
#endif

    switch (nAlgorithm)
    {
    case 1: //低通滤波器
        break;
    case 2: //高通滤波器
        break;
    case 3: //带通滤波器
        break;
    case 4: //特定纹理滤波器
        rectRoi(mag, bHor, nDiameter, width);
        break;
    default:
        break;
    }
    polarToCart(mag, ph, plane[0], plane[1]);
    cv::Mat BLUR;
    // 再次搬移回来进行逆变换
    dftshift(plane[0], plane[1]);
    cv::merge(plane, 2, BLUR); // 实部与虚部合并
    cv::idft(BLUR, BLUR);      // idft结果也为复数
    BLUR = BLUR / BLUR.rows / BLUR.cols;
    cv::split(BLUR, plane); //分离通道，主要获取通道

    BLUR = plane[0] / 255;
    cv::normalize(BLUR, BLUR, 0, 255, NORM_MINMAX);
    BLUR.convertTo(mDesImage, CV_8UC1);
#if _DEBUG
    CvWaitShowImage(mDesImage, "傅里叶还原图");
#endif
    if (mDesImage.data /*&& mImg_dft.data*/)
        return true;
    else
        return false;
}

CEnhance &CEnhance::operator=(const CEnhance &)
{
    // TODO: 在此处插入 return 语句
    return *this;
}

/// <summary>
/// 对数变换增强
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CEnhance::getLogImage(const cv::Mat mSrcImage, cv::Mat &mDesImage)
{
    if (!mSrcImage.data || mSrcImage.channels() > 1)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    Mat imageLog(mSrcImage.size(), CV_64FC1);
    for (int i = 0; i < mSrcImage.rows; i++)
    {
        for (int j = 0; j < mSrcImage.cols; j++)
        {
            imageLog.at<double>(i, j) = log((double)(1 + mSrcImage.at<uchar>(i, j)));
        }
    }
    //归一化到0~255

    normalize(imageLog, imageLog, 0, 255, 32);
    //转换成8bit图像显示
    convertScaleAbs(imageLog, imageLog);
    imageLog.copyTo(mDesImage);
    return true;
}

/// <summary>
/// 伽马变换增强
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CEnhance::getGammaImage(const cv::Mat mSrcImage, cv::Mat &mDesImage)
{
    if (!mSrcImage.data || mSrcImage.channels() > 1)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    Mat imageLog(mSrcImage.size(), CV_64FC1);
    for (int i = 0; i < mSrcImage.rows; i++)
    {
        for (int j = 0; j < mSrcImage.cols; j++)
        {
            imageLog.at<double>(i, j) = mSrcImage.at<uchar>(i, j) * mSrcImage.at<uchar>(i, j) * mSrcImage.at<uchar>(i, j);
        }
    }
    //归一化到0~255
    normalize(imageLog, imageLog, 0, 255, 32);
    //转换成8bit图像显示
    convertScaleAbs(imageLog, imageLog);
    imageLog.copyTo(mDesImage);
    return true;
}

/// <summary>
/// 直方图均衡化
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CEnhance::getEqualizeHist(const cv::Mat mSrcImage, cv::Mat &mDesImage)
{
    if (!mSrcImage.data || mSrcImage.channels() > 1)
    {
        std::cout << "falied to read" << std::endl;
        return false;
    }
    equalizeHist(mSrcImage, mDesImage);
    return true;
}

/// <summary>
/// 高反差保留
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CEnhance::getHighPass(const cv::Mat mSrcImage, cv::Mat &mDesImage)
{
    cv::Mat temp;
    GaussianBlur(mSrcImage, temp, Size(7, 7), 1.6, 1.6);
    int r = 3;
    mDesImage = mSrcImage + r * (mSrcImage - temp); //高反差保留算法
    return true;
}

/// <summary>
/// 增强图像高频对比度
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <param name="n"></param>
/// <param name="factor"></param>
/// <returns></returns>
bool CEnhance::getEmphasize(const cv::Mat mSrcImage, cv::Mat &mDesImage, int n, double factor)
{
    cv::Mat mEngancyImage, mInputImage;
    blur(mSrcImage, mEngancyImage, Size(n, n));
    mDesImage.create(mSrcImage.size(), mSrcImage.type());
    if (mSrcImage.type() == CV_8UC1)
    {
        for (int i = 0; i < mSrcImage.rows; i++)
        {
            const uchar *rptr = mSrcImage.ptr<uchar>(i);
            uchar *mptr = mEngancyImage.ptr<uchar>(i);
            uchar *optr = mDesImage.ptr<uchar>(i);
            for (int j = 0; j < mSrcImage.cols; j++)
            {
                optr[j] = cv::saturate_cast<uchar>(round((rptr[j] - mptr[j]) * factor) + rptr[j] * 1.0f);
            }
        }
    }
    ////换算成局部标准差
    //mEngancyImage.convertTo(mEngancyImage, CV_32F);
    //Mat highFreq = (mInputImage - mEngancyImage) * factor + mInputImage;//

    //highFreq.convertTo(mDesImage, CV_8UC1);
    return true;
}

/// <summary>
/// 图像整体增强
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <param name="n"></param>
/// <param name="factor"></param>
/// <returns></returns>
bool CEnhance::EmphasizeScale(const cv::Mat mSrcImage, cv::Mat &mDesImage, int n, double factor)
{
    if (mSrcImage.data)
        cv::convertScaleAbs(mSrcImage, mDesImage, n, factor);
    else
        return false;
    return true;
}

/// <summary>
/// 直方图均衡化自实现
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <param name="nMaxDiameter">上限直径</param>
/// <param name="nMinDiameter">下限直径</param>
/// <returns></returns>
bool CEnhance::equliaze_hist(cv::Mat mSrcImage, cv::Mat &mDesImage, int nMaxDiameter, int nMinDiameter)
{
    int gray[256] = {0};                 //记录每个灰度级别下的像素个数
    double gray_prob[256] = {0};         //记录灰度分布密度
    double gray_distribution[256] = {0}; //记录累计密度
    int gray_equal[256] = {0};           //均衡化后的灰度值

    int gray_sum = 0; //像素总数
    mDesImage = mSrcImage.clone();
    gray_sum = 0; //统计像素个数

    if (nMaxDiameter == 0) //不设置外围上限
    {
        //统计每个灰度下的像素个数
        for (int i = 0; i < mSrcImage.rows; i++)
        {
            //uchar* p = mSrcImage.ptr<uchar>(i);
            for (int j = 0; j < mSrcImage.cols; j++)
            {
                if (sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) >= nMinDiameter / 2)
                {
                    gray[mSrcImage.at<uchar>(i, j)]++;
                    gray_sum++;
                }
            }
        }
    }
    else
    {
        //统计每个灰度下的像素个数
        for (int i = 0; i < mSrcImage.rows; i++)
        {
            //uchar* p = mSrcImage.ptr<uchar>(i);
            for (int j = 0; j < mSrcImage.cols; j++)
            {
                if (sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) >= nMinDiameter / 2 &&
                    sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) <= nMaxDiameter / 2)
                {
                    gray[mSrcImage.at<uchar>(i, j)]++;
                    gray_sum++;
                }
            }
        }
    }

    //统计灰度频率
    for (int i = 0; i < 256; i++)
    {
        gray_prob[i] = ((double)gray[i] / gray_sum);
    }

    //计算累计密度
    gray_distribution[0] = gray_prob[0];
    for (int i = 1; i < 256; i++)
    {
        gray_distribution[i] = gray_distribution[i - 1] + gray_prob[i];
    }

    //重新计算均衡化后的灰度值，四舍五入。参考公式：(N-1)*T+0.5
    for (int i = 0; i < 256; i++)
    {
        //int nn = round(255 * gray_distribution[i]);
        gray_equal[i] = (uchar)(255 * gray_distribution[i] > 0.0 ? floor(255 * gray_distribution[i] + 0.5) : ceil(255 * gray_distribution[i] - 0.5));
    }

    //直方图均衡化,更新原图每个点的像素值
    if (nMaxDiameter == 0) //不设置外围上限
    {
        for (int i = 0; i < mDesImage.rows; i++)
        {
            //uchar* p = mDesImage.ptr<uchar>(i);
            for (int j = 0; j < mDesImage.cols; j++)
            {
                if (sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) >= nMinDiameter / 2)
                    mDesImage.at<uchar>(i, j) = gray_equal[mDesImage.at<uchar>(i, j)];
            }
        }
    }
    else
    {
        for (int i = 0; i < mDesImage.rows; i++)
        {
            //uchar* p = mDesImage.ptr<uchar>(i);
            for (int j = 0; j < mDesImage.cols; j++)
            {
                if (sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) >= nMinDiameter / 2 &&
                    sqrt(pow(double(i - mSrcImage.rows / 2), 2) + pow(double(j - mSrcImage.cols / 2), 2)) <= nMaxDiameter / 2)
                    mDesImage.at<uchar>(i, j) = gray_equal[mDesImage.at<uchar>(i, j)];
            }
        }
    }

    return true;
}

/// <summary>
/// 傅里叶频谱图频率迁移
/// </summary>
/// <param name="plane0"></param>
/// <param name="plane1"></param>
void CEnhance::dftshift(cv::Mat &plane0, cv::Mat &plane1)
{ // 以下的操作是移动图像  (零频移到中心)
    int cx = plane0.cols / 2;
    int cy = plane0.rows / 2;
    cv::Mat part1_r(plane0, cv::Rect(0, 0, cx, cy)); // 元素坐标表示为(cx, cy)
    cv::Mat part2_r(plane0, cv::Rect(cx, 0, cx, cy));
    cv::Mat part3_r(plane0, cv::Rect(0, cy, cx, cy));
    cv::Mat part4_r(plane0, cv::Rect(cx, cy, cx, cy));

    cv::Mat temp;
    part1_r.copyTo(temp); //左上与右下交换位置(实部)
    part4_r.copyTo(part1_r);
    temp.copyTo(part4_r);

    part2_r.copyTo(temp); //右上与左下交换位置(实部)
    part3_r.copyTo(part2_r);
    temp.copyTo(part3_r);

    cv::Mat part1_i(plane1, cv::Rect(0, 0, cx, cy)); //元素坐标(cx,cy)
    cv::Mat part2_i(plane1, cv::Rect(cx, 0, cx, cy));
    cv::Mat part3_i(plane1, cv::Rect(0, cy, cx, cy));
    cv::Mat part4_i(plane1, cv::Rect(cx, cy, cx, cy));

    part1_i.copyTo(temp); //左上与右下交换位置(虚部)
    part4_i.copyTo(part1_i);
    temp.copyTo(part4_i);

    part2_i.copyTo(temp); //右上与左下交换位置(虚部)
    part3_i.copyTo(part2_i);
    temp.copyTo(part3_i);
}

/// <summary>
/// 矩形区域纹理去除
/// </summary>
/// <param name="SrcImage"></param>
/// <param name="bHor"></param>
/// <param name="nDiameter"></param>
/// <param name="width"></param>
void CEnhance::rectRoi(cv::Mat &SrcImage, bool bHor, int nDiameter, int width)
{
    if (nDiameter != 0 && width != 0)
    {
        //if (bHor)//横向区域纹理去除
        //{
        //	rectangle(SrcImage, Point(0, SrcImage.rows / 2 - width), Point(SrcImage.cols / 2 - nDiameter, SrcImage.rows / 2 + width),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 + nDiameter, SrcImage.rows / 2 - width), Point(SrcImage.cols, SrcImage.rows / 2 + width),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, 0), Point(SrcImage.cols / 2 + width, SrcImage.rows / 2 - nDiameter),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, SrcImage.rows / 2 + nDiameter), Point(SrcImage.cols / 2 + width, SrcImage.rows),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //}
        //else//竖向区域纹理去除
        //{
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, 0), Point(SrcImage.cols / 2 + width, SrcImage.rows / 2 - nDiameter),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, SrcImage.rows / 2 + nDiameter), Point(SrcImage.cols / 2 + width, SrcImage.rows),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //}
        //横向条纹
        rectangle(SrcImage, Point(0, SrcImage.rows / 2 - width), Point(SrcImage.cols / 2 - nDiameter, SrcImage.rows / 2 + width),
                  Scalar(0, 0, 0), -1, 8, 0);
        rectangle(SrcImage, Point(SrcImage.cols / 2 + nDiameter, SrcImage.rows / 2 - width), Point(SrcImage.cols, SrcImage.rows / 2 + width),
                  Scalar(0, 0, 0), -1, 8, 0);
        //竖向条纹
        /*rectangle(SrcImage, Point(SrcImage.cols / 2 - width, 0), Point(SrcImage.cols / 2 + width, SrcImage.rows / 2 - nDiameter),
				Scalar(0, 0, 0), -1, 8, 0);
			rectangle(SrcImage, Point(SrcImage.cols / 2 - width, SrcImage.rows / 2 + nDiameter), Point(SrcImage.cols / 2 + width, SrcImage.rows),
				Scalar(0, 0, 0), -1, 8, 0);*/
    }
}

bool CEnhance::getLocalEnhance(const cv::Mat mSrcImage, double dMeanMinScalar, double dMeanMaxScalar, double dStdMinScalar, double dStdMaxScalar, cv::Mat &mDesImage, bool bIsEnchance)
{
    mSrcImage.copyTo(mDesImage);
    cv::Mat mat_mean, mat_stddev;
    //全局均值和方差
    meanStdDev(mSrcImage, mat_mean, mat_stddev);
    double m, s;
    m = mat_mean.at<double>(0, 0);
    s = mat_stddev.at<double>(0, 0);
    double dE = 1.5;
    for (int i = 2; i < mSrcImage.rows - 2; ++i)
    {
        for (int j = 2; j < mSrcImage.cols - 2; ++j)
        {
            cv::Mat mDetImage = mSrcImage(Rect(j - 1, i - 1, 3, 3));
            cv::Mat mMean, mStd;
            //局部均值和方差
            meanStdDev(mDetImage, mMean, mStd);
            double dM1 = mMean.at<double>(0, 0);
            double dS1 = mStd.at<double>(0, 0);
            //增强白色部分，局部均值>全局均值,使用MaxMean，MinStd，MaxStd参数
            if (m_bIsWhite && dM1 >= dMeanMaxScalar * m && dStdMinScalar * s <= dS1 && dS1 <= dStdMaxScalar * s)
            {
                //灰度拉升
                if (bIsEnchance)
                    mDesImage.at<uchar>(i, j) = mDesImage.at<uchar>(i, j) * dE > 255 ? 255 : mDesImage.at<uchar>(i, j) * dE;
                else
                    mDesImage.at<uchar>(i, j) = 255;
            }
            //增强黑色部分，局部均值<全局均值，使用MinMean，MinStd，MaxStd参数
            else if (!m_bIsWhite && dM1 <= dMeanMinScalar * m && dStdMinScalar * s <= dS1 && dS1 <= dStdMaxScalar * s)
            {
                //灰度拉升
                if (bIsEnchance)
                    mDesImage.at<uchar>(i, j) = mDesImage.at<uchar>(i, j) / dE < 0 ? 0 : mDesImage.at<uchar>(i, j) / dE;
                else
                    mDesImage.at<uchar>(i, j) = 0;
            }
        }
    }
#if _DEBUG
    //cv::Mat mShowImage = generateColorImage(mDesImage);
    CvWaitShowImage(mDesImage, "增强拉升图");
#endif
    return true;
}