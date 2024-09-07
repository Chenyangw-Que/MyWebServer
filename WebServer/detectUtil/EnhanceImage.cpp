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
/// ͼ����ǿ
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
        //�����任��ǿ
        getLogImage(mSrcImage, mDesImage);
        break;
    case 1:
        //٤��任��ǿ
        getGammaImage(mSrcImage, mDesImage);
        break;
    case 2:
        //�߷����
        getHighPass(mSrcImage, mDesImage);
        break;
    case 3:
        //ֱ��ͼ����
        equliaze_hist(mSrcImage, mDesImage, nDiameter, nDiameter2);
        break;
    case 4:
        //��Ƶ�Աȶ���ǿ
        getEmphasize(mSrcImage, mDesImage, n, factor);
        break;
    case 5:
        //ͼ��������ǿ
        EmphasizeScale(mSrcImage, mDesImage, n, factor);
        break;
    default:
        break;
    }
    return true;
}

/// <summary>
/// ����Ҷ�任
/// </summary>
/// <param name="mSrcImage-����ͼƬ"></param>
/// <param name="nAlgorithm-�����㷨"></param>
/// <param name="mDesImage-����ͼƬ"></param>
/// <param name="nDiameter-��������ֱ��"></param>
/// <param name="width-�߿�"></param>
/// <returns></returns>
bool CEnhance::FourierTransform(const cv::Mat mSrcImage, int nAlgorithm, cv::Mat &mDesImage, bool bHor, int nDiameter, int width)
{
    if (!mSrcImage.data)
        return false;
    //2.����ͨ��ͼ��ת����˫ͨ��ͼ��
    cv::Mat mImg2;
    mSrcImage.convertTo(mImg2, CV_32FC1);
    //����ͨ�����洢dft���ʵ�����鲿��CV_32F������Ϊ��ͨ������
    cv::Mat plane[] = {mImg2.clone(), cv::Mat::zeros(mImg2.size(), CV_32FC1)};
    cv::Mat complexIm;
    cv::merge(plane, 2, complexIm);   // �ϲ�ͨ�� ������������ϲ�Ϊһ��2ͨ����Mat��������
    cv::dft(complexIm, complexIm, 0); // ���и���Ҷ�任���������������

    // ����ͨ����������룩
    cv::split(complexIm, plane);
    // ���µĲ�����Ƶ��Ǩ��
    dftshift(plane[0], plane[1]);
    // �����ֵ
    cv::Mat mag, ph, mag_log, mag_nor;
    phase(plane[0], plane[1], ph);
    cv::magnitude(plane[0], plane[1], mag);
    //��ֵ��������log��1+m�������ڹ۲�Ƶ����Ϣ
    mag += Scalar::all(1);
    cv::log(mag, mag_log);
    cv::normalize(mag_log, mag_nor, 1, 0, NORM_MINMAX);
#if _DEBUG
    CvWaitShowImage(mag_nor, "����ҶƵ��ͼ");
#endif

    switch (nAlgorithm)
    {
    case 1: //��ͨ�˲���
        break;
    case 2: //��ͨ�˲���
        break;
    case 3: //��ͨ�˲���
        break;
    case 4: //�ض������˲���
        rectRoi(mag, bHor, nDiameter, width);
        break;
    default:
        break;
    }
    polarToCart(mag, ph, plane[0], plane[1]);
    cv::Mat BLUR;
    // �ٴΰ��ƻ���������任
    dftshift(plane[0], plane[1]);
    cv::merge(plane, 2, BLUR); // ʵ�����鲿�ϲ�
    cv::idft(BLUR, BLUR);      // idft���ҲΪ����
    BLUR = BLUR / BLUR.rows / BLUR.cols;
    cv::split(BLUR, plane); //����ͨ������Ҫ��ȡͨ��

    BLUR = plane[0] / 255;
    cv::normalize(BLUR, BLUR, 0, 255, NORM_MINMAX);
    BLUR.convertTo(mDesImage, CV_8UC1);
#if _DEBUG
    CvWaitShowImage(mDesImage, "����Ҷ��ԭͼ");
#endif
    if (mDesImage.data /*&& mImg_dft.data*/)
        return true;
    else
        return false;
}

CEnhance &CEnhance::operator=(const CEnhance &)
{
    // TODO: �ڴ˴����� return ���
    return *this;
}

/// <summary>
/// �����任��ǿ
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
    //��һ����0~255

    normalize(imageLog, imageLog, 0, 255, 32);
    //ת����8bitͼ����ʾ
    convertScaleAbs(imageLog, imageLog);
    imageLog.copyTo(mDesImage);
    return true;
}

/// <summary>
/// ٤��任��ǿ
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
    //��һ����0~255
    normalize(imageLog, imageLog, 0, 255, 32);
    //ת����8bitͼ����ʾ
    convertScaleAbs(imageLog, imageLog);
    imageLog.copyTo(mDesImage);
    return true;
}

/// <summary>
/// ֱ��ͼ���⻯
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
/// �߷����
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CEnhance::getHighPass(const cv::Mat mSrcImage, cv::Mat &mDesImage)
{
    cv::Mat temp;
    GaussianBlur(mSrcImage, temp, Size(7, 7), 1.6, 1.6);
    int r = 3;
    mDesImage = mSrcImage + r * (mSrcImage - temp); //�߷�����㷨
    return true;
}

/// <summary>
/// ��ǿͼ���Ƶ�Աȶ�
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
    ////����ɾֲ���׼��
    //mEngancyImage.convertTo(mEngancyImage, CV_32F);
    //Mat highFreq = (mInputImage - mEngancyImage) * factor + mInputImage;//

    //highFreq.convertTo(mDesImage, CV_8UC1);
    return true;
}

/// <summary>
/// ͼ��������ǿ
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
/// ֱ��ͼ���⻯��ʵ��
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <param name="nMaxDiameter">����ֱ��</param>
/// <param name="nMinDiameter">����ֱ��</param>
/// <returns></returns>
bool CEnhance::equliaze_hist(cv::Mat mSrcImage, cv::Mat &mDesImage, int nMaxDiameter, int nMinDiameter)
{
    int gray[256] = {0};                 //��¼ÿ���Ҷȼ����µ����ظ���
    double gray_prob[256] = {0};         //��¼�Ҷȷֲ��ܶ�
    double gray_distribution[256] = {0}; //��¼�ۼ��ܶ�
    int gray_equal[256] = {0};           //���⻯��ĻҶ�ֵ

    int gray_sum = 0; //��������
    mDesImage = mSrcImage.clone();
    gray_sum = 0; //ͳ�����ظ���

    if (nMaxDiameter == 0) //��������Χ����
    {
        //ͳ��ÿ���Ҷ��µ����ظ���
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
        //ͳ��ÿ���Ҷ��µ����ظ���
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

    //ͳ�ƻҶ�Ƶ��
    for (int i = 0; i < 256; i++)
    {
        gray_prob[i] = ((double)gray[i] / gray_sum);
    }

    //�����ۼ��ܶ�
    gray_distribution[0] = gray_prob[0];
    for (int i = 1; i < 256; i++)
    {
        gray_distribution[i] = gray_distribution[i - 1] + gray_prob[i];
    }

    //���¼�����⻯��ĻҶ�ֵ���������롣�ο���ʽ��(N-1)*T+0.5
    for (int i = 0; i < 256; i++)
    {
        //int nn = round(255 * gray_distribution[i]);
        gray_equal[i] = (uchar)(255 * gray_distribution[i] > 0.0 ? floor(255 * gray_distribution[i] + 0.5) : ceil(255 * gray_distribution[i] - 0.5));
    }

    //ֱ��ͼ���⻯,����ԭͼÿ���������ֵ
    if (nMaxDiameter == 0) //��������Χ����
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
/// ����ҶƵ��ͼƵ��Ǩ��
/// </summary>
/// <param name="plane0"></param>
/// <param name="plane1"></param>
void CEnhance::dftshift(cv::Mat &plane0, cv::Mat &plane1)
{ // ���µĲ������ƶ�ͼ��  (��Ƶ�Ƶ�����)
    int cx = plane0.cols / 2;
    int cy = plane0.rows / 2;
    cv::Mat part1_r(plane0, cv::Rect(0, 0, cx, cy)); // Ԫ�������ʾΪ(cx, cy)
    cv::Mat part2_r(plane0, cv::Rect(cx, 0, cx, cy));
    cv::Mat part3_r(plane0, cv::Rect(0, cy, cx, cy));
    cv::Mat part4_r(plane0, cv::Rect(cx, cy, cx, cy));

    cv::Mat temp;
    part1_r.copyTo(temp); //���������½���λ��(ʵ��)
    part4_r.copyTo(part1_r);
    temp.copyTo(part4_r);

    part2_r.copyTo(temp); //���������½���λ��(ʵ��)
    part3_r.copyTo(part2_r);
    temp.copyTo(part3_r);

    cv::Mat part1_i(plane1, cv::Rect(0, 0, cx, cy)); //Ԫ������(cx,cy)
    cv::Mat part2_i(plane1, cv::Rect(cx, 0, cx, cy));
    cv::Mat part3_i(plane1, cv::Rect(0, cy, cx, cy));
    cv::Mat part4_i(plane1, cv::Rect(cx, cy, cx, cy));

    part1_i.copyTo(temp); //���������½���λ��(�鲿)
    part4_i.copyTo(part1_i);
    temp.copyTo(part4_i);

    part2_i.copyTo(temp); //���������½���λ��(�鲿)
    part3_i.copyTo(part2_i);
    temp.copyTo(part3_i);
}

/// <summary>
/// ������������ȥ��
/// </summary>
/// <param name="SrcImage"></param>
/// <param name="bHor"></param>
/// <param name="nDiameter"></param>
/// <param name="width"></param>
void CEnhance::rectRoi(cv::Mat &SrcImage, bool bHor, int nDiameter, int width)
{
    if (nDiameter != 0 && width != 0)
    {
        //if (bHor)//������������ȥ��
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
        //else//������������ȥ��
        //{
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, 0), Point(SrcImage.cols / 2 + width, SrcImage.rows / 2 - nDiameter),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //	rectangle(SrcImage, Point(SrcImage.cols / 2 - width, SrcImage.rows / 2 + nDiameter), Point(SrcImage.cols / 2 + width, SrcImage.rows),
        //		Scalar(0, 0, 0), -1, 8, 0);
        //}
        //��������
        rectangle(SrcImage, Point(0, SrcImage.rows / 2 - width), Point(SrcImage.cols / 2 - nDiameter, SrcImage.rows / 2 + width),
                  Scalar(0, 0, 0), -1, 8, 0);
        rectangle(SrcImage, Point(SrcImage.cols / 2 + nDiameter, SrcImage.rows / 2 - width), Point(SrcImage.cols, SrcImage.rows / 2 + width),
                  Scalar(0, 0, 0), -1, 8, 0);
        //��������
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
    //ȫ�־�ֵ�ͷ���
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
            //�ֲ���ֵ�ͷ���
            meanStdDev(mDetImage, mMean, mStd);
            double dM1 = mMean.at<double>(0, 0);
            double dS1 = mStd.at<double>(0, 0);
            //��ǿ��ɫ���֣��ֲ���ֵ>ȫ�־�ֵ,ʹ��MaxMean��MinStd��MaxStd����
            if (m_bIsWhite && dM1 >= dMeanMaxScalar * m && dStdMinScalar * s <= dS1 && dS1 <= dStdMaxScalar * s)
            {
                //�Ҷ�����
                if (bIsEnchance)
                    mDesImage.at<uchar>(i, j) = mDesImage.at<uchar>(i, j) * dE > 255 ? 255 : mDesImage.at<uchar>(i, j) * dE;
                else
                    mDesImage.at<uchar>(i, j) = 255;
            }
            //��ǿ��ɫ���֣��ֲ���ֵ<ȫ�־�ֵ��ʹ��MinMean��MinStd��MaxStd����
            else if (!m_bIsWhite && dM1 <= dMeanMinScalar * m && dStdMinScalar * s <= dS1 && dS1 <= dStdMaxScalar * s)
            {
                //�Ҷ�����
                if (bIsEnchance)
                    mDesImage.at<uchar>(i, j) = mDesImage.at<uchar>(i, j) / dE < 0 ? 0 : mDesImage.at<uchar>(i, j) / dE;
                else
                    mDesImage.at<uchar>(i, j) = 0;
            }
        }
    }
#if _DEBUG
    //cv::Mat mShowImage = generateColorImage(mDesImage);
    CvWaitShowImage(mDesImage, "��ǿ����ͼ");
#endif
    return true;
}