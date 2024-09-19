#pragma once
#include "opencv2/opencv.hpp"
int OtsuThres(int pHist[256]);

int OtsuThres(cv::Mat image);

int OtsuThres(cv::Mat image, int nMin, int nMax);