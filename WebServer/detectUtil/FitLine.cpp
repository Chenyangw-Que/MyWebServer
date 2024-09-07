#pragma once
#include "FitLine.h"
#include "Common.h"
std::vector<cv::Point> scanPoint(const std::vector<cv::Point> vecPoint, double &a, double &b, double &c)
{
	std::vector<cv::Point> vecScanPoint = vecPoint;
	if (vecPoint.size() == 0)
		return vecScanPoint;
	double dMeanDis = 0, dSigma = 0;
	for (int i = 0; i < vecScanPoint.size(); ++i)
	{
		dMeanDis += getPointToLineDistance(vecScanPoint[i], a, b, c);
	}
	dMeanDis = dMeanDis / vecScanPoint.size();
	for (int i = 0; i < vecScanPoint.size(); ++i)
	{
		dSigma += abs(getPointToLineDistance(vecScanPoint[i], a, b, c) - dMeanDis);
	}
	dSigma /= vecScanPoint.size();
	while (dSigma > 1)
	{
		for (int i = 0; i < vecScanPoint.size(); ++i)
		{
			if (vecScanPoint.size() < 2)
				break;
			double dDisPointLine = getPointToLineDistance(vecScanPoint[i], a, b, c);
			if (dDisPointLine > dMeanDis)
			{
				vecScanPoint.erase(vecScanPoint.begin() + i);
				i--;
			}
		}
		if (lineFit(vecScanPoint, a, b, c))
		{
			dMeanDis = 0, dSigma = 0;
			for (int i = 0; i < vecScanPoint.size(); ++i)
			{
				dMeanDis += getPointToLineDistance(vecScanPoint[i], a, b, c);
			}
			dMeanDis = dMeanDis / vecScanPoint.size();
			for (int i = 0; i < vecScanPoint.size(); ++i)
			{
				dSigma += abs(getPointToLineDistance(vecScanPoint[i], a, b, c) - dMeanDis);
			}
			dSigma /= vecScanPoint.size();
		}
		else
			break;
	}
	return vecScanPoint;
}