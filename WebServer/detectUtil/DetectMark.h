#pragma once
#include "paraType.h"
#include <opencv2/opencv.hpp>
extern "C" int AutoGetParam(const cv::Mat& imgData, DetectResultV3 *result);

extern "C" int AutoDetMark(const cv::Mat& imgData, DetectResultV3 *result, std::vector<cv::Mat>& resultBox);

extern "C" void drawRes(const cv::Mat& inputImg, const DetectResultV3& result);
