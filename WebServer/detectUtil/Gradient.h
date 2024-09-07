#pragma once
#include "opencv2/opencv.hpp"

class CGradient {
public:
  CGradient();
  ~CGradient();

  /// <summary>
  /// 边缘检测
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <param name="nType"></param>
  void getGradientImage(cv::Mat mSrcImage, cv::Mat &mDesImage, int nType);

  /// <summary>
  /// 亚像素边缘检测
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="DesImage"></param>
  /// <param name="vecContourPoints"></param>
  void getSubPixelGradient(cv::Mat mSrcImage, cv::Mat &DesImage,
                           std::vector<cv::Point2d> &vecContourPoints);

private:
  /// <summary>
  /// sobel算子
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getSobel(cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// canny算子
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getCanny(cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// laplacian算子
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getLaplacian(cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// scharr算子
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getScharr(cv::Mat mSrcImage, cv::Mat &mDesImage);

  /// <summary>
  /// prewitt算子
  /// </summary>
  /// <param name="mSrcImage"></param>
  /// <param name="mDesImage"></param>
  /// <returns></returns>
  bool getPrewitt(cv::Mat mSrcImage, cv::Mat &mDesImage);

private:
  /// <summary>
  /// 亚像素边缘检测
  /// </summary>
  /// <param name="x"></param>
  /// <param name="y"></param>
  /// <param name="N"></param>
  /// <param name="curve_limits"></param>
  /// <param name="M"></param>
  /// <param name="image"></param>
  /// <param name="gauss"></param>
  /// <param name="X"></param>
  /// <param name="Y"></param>
  /// <param name="sigma"></param>
  /// <param name="th_h"></param>
  /// <param name="th_l"></param>
  void devernay(double **x, double **y, int *N, int **curve_limits, int *M,
                uchar *image, uchar *gauss, int X, int Y, double sigma,
                double th_h, double th_l);
  void error(char *msg);
  void *xmalloc(size_t size);
  int Greater(double a, double b);
  double dist(double x1, double y1, double x2, double y2);
  void gaussian_kernel(double *pfGaussFilter, int n, double sigma, double mean);
  void gaussian_filter(uchar *image, uchar *out, int X, int Y, double sigma);
  double chain(int from, int to, double *Ex, double *Ey, double *Gx, double *Gy,
               int X, int Y);
  void compute_gradient(double *Gx, double *Gy, double *modG, uchar *image,
                        int X, int Y);
  void compute_edge_points(double *Ex, double *Ey, double *modG, double *Gx,
                           double *Gy, int X, int Y);
  void chain_edge_points(int *next, int *prev, double *Ex, double *Ey,
                         double *Gx, double *Gy, int X, int Y);
  void thresholds_with_hysteresis(int *next, int *prev, double *modG, int X,
                                  int Y, double th_h, double th_l);
  void list_chained_edge_points(double **x, double **y, int *N,
                                int **curve_limits, int *M, int *next,
                                int *prev, double *Ex, double *Ey, int X,
                                int Y);
};
