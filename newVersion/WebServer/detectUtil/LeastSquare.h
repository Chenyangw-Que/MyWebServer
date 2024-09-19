#pragma once
#include<vector>
#include "opencv2/opencv.hpp"
//最小二乘拟合圆
template <typename TypePoint>
cv::Point3f leastSquareFittingCircle(TypePoint T)
{
	float fX1 = 0;
	float fX2 = 0;
	float fX3 = 0;
	float fY1 = 0;
	float fY2 = 0;
	float fY3 = 0;
	float fX1Y1 = 0;
	float fX1Y2 = 0;
	float fX2Y1 = 0;
	int nNum;
	cv::Point3f tempCircle;
	nNum = T.size();
	for (auto k = T.begin(); k != T.end(); ++k)
	{
		fX1 = fX1 + (*k).dX;
		fX2 = fX2 + (*k).dX * (*k).dX;
		fX3 = fX3 + (*k).dX * (*k).dX * (*k).dX;
		fY1 = fY1 + (*k).dY;
		fY2 = fY2 + (*k).dY * (*k).dY;
		fY3 = fY3 + (*k).dY * (*k).dY * (*k).dY;
		fX1Y1 = fX1Y1 + (*k).dX * (*k).dY;
		fX1Y2 = fX1Y2 + (*k).dX * (*k).dY * (*k).dY;
		fX2Y1 = fX2Y1 + (*k).dX * (*k).dX * (*k).dY;
	}
	cv::Mat mLeft_matrix = (cv::Mat_<float>(3, 3) << fX2, fX1Y1, fX1, fX1Y1, fY2, fY1, fX1, fY1, nNum);
	//cout << "Left_matrix=" << mLeft_matrix << endl;
	cv::Mat mRight_matrix = (cv::Mat_<float>(3, 1) << -(fX3 + fX1Y2), -(fX2Y1 + fY3), -(fX2 + fY2));
	//cout << "Right_matrix=" << mRight_matrix << endl;
	cv::Mat mSolution(3, 1, CV_32F);
	mSolution.at<float>(0, 0) = 0;
	mSolution.at<float>(1, 0) = 0;
	mSolution.at<float>(2, 0) = 0;
	//mSolution.zeros(3, 1, CV_32F);
	//cout << mSolution << endl;
	solve(mLeft_matrix, mRight_matrix, mSolution, cv::DECOMP_LU);
	//cout << "Solution=" << mSolution << endl;
	float a, b, c;
	a = mSolution.at<float>(0);
	b = mSolution.at<float>(1);
	c = mSolution.at<float>(2);
	tempCircle.x = -a / 2;
	tempCircle.y = -b / 2;
	tempCircle.z = sqrt(a * a + b * b - 4 * c) / 2;
	return tempCircle;
}