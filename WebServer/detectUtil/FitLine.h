#pragma once
#include "paraType.h"
#include "opencv2/opencv.hpp"

enum EdgeDirection
{
	Top = 1,
	Bottom = 2,
	Left = 3,
	Right = 4
};

//直线拟合
template <class TypePoint>
bool lineFit(TypePoint &points, double &a, double &b, double &c)
{
	int size = points.size();
	if (size < 2)
	{
		a = 0;
		b = 0;
		c = 0;
		return false;
	}
	double x_mean = 0;
	double y_mean = 0;
	for (int i = 0; i < size; i++)
	{
		x_mean += points[i].x;
		y_mean += points[i].y;
	}
	x_mean /= size;
	y_mean /= size; //至此，计算出了 x y 的均值

	double Dxx = 0, Dxy = 0, Dyy = 0;

	for (int i = 0; i < size; i++)
	{
		Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
		Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
		Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
	}
	double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
	double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
	if (den == 0)
	{
		a = 0;
		b = 0;
		c = 0;
	}
	else
	{
		a = Dxy / den;
		b = (lambda - Dxx) / den;
		c = -a * x_mean - b * y_mean;
	}
	return true;
}

std::vector<cv::Point> scanPoint(const std::vector<cv::Point> vecPoint, double &a, double &b, double &c);



