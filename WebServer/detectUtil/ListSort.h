#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "paraType.h"

//从小到大排序
inline bool sortList3d(const std::pair<int, cv::Point3d> a, const std::pair<int, cv::Point3d> b) { return a.first < b.first; }

//从小到大排序
inline bool sortList(const std::pair<int, cv::Point> a, const std::pair<int, cv::Point> b) { return a.first < b.first; }

//从大到小排序
inline bool sortMaxtoMin3d(const std::pair<double, cv::Point3d> a, const std::pair<double, cv::Point3d> b) { return a.first > b.first; }

//从大到小排序
inline bool sortMaxtoMinVec(const std::pair<double, std::vector<cv::Point>> a, const std::pair<double, std::vector<cv::Point>> b) { return a.first > b.first; }

//从大到小排序
inline bool sortMaxtoMin(const std::pair<double, cv::Point> a, const std::pair<double, cv::Point> b) { return a.first > b.first; }

//从大到小排序
inline bool sortPoint(const std::vector<cv::Point> a, const std::vector<cv::Point> b) { return a.size() > b.size(); }

//从大到小排序
inline bool sortPoint3f(const std::vector<cv::Point3f> a, const std::vector<cv::Point3f> b) { return a.size() > b.size(); }

//从小到大排序
inline bool sortPointX(const cv::Point a, const cv::Point b) { return a.x < b.x; }

//从小到大排序
inline bool sortPoint2dX(const cv::Point2d a, const cv::Point2d b) { return a.x < b.x; }

//从小到大排序
inline bool sortPointY(const cv::Point a, const cv::Point b) { return a.y < b.y; }

//从小到大排序
inline bool sortPoint2dY(const cv::Point2d a, const cv::Point2d b) { return a.y < b.y; }

//从大到小排序
inline bool sortNum(const std::pair<int, int> a, const std::pair<int, int> b) { return a.first > b.first; }
