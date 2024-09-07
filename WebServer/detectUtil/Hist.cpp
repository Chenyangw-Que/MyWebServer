    #include "Hist.h"
    cv::MatND getImageHist(cv::Mat mSrcImage)
	{
		cv::MatND dstHist;
		int nDims = 1;
		float fHrangs[] = { 0,256 };
		const float* fRange[] = { fHrangs };
		int nSize = 256;
		int nChannels = 0;
		calcHist(&mSrcImage, 1, &nChannels, cv::Mat(), dstHist, nDims, &nSize, fRange);
		return dstHist;
	}