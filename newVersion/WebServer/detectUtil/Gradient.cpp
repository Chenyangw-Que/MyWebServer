#include "Gradient.h"
#include "paraType.h"
#include "Common.h"
#include "Threshold.h"
using namespace std;
using namespace cv;

	CGradient::CGradient()
	{
	}

	CGradient::~CGradient()
	{
	}

	/// <summary>
	/// 边缘检测
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <param name="nType"></param>
	void CGradient::getGradientImage(cv::Mat mSrcImage, cv::Mat& mDesImage, int nType)
	{		
		if (!mSrcImage.data)
		{
			std::cout << "轮廓 falied to read" << std::endl;
			return;
		}
		if (mSrcImage.channels() > 1)
			return;	
		switch (nType)
		{
		case 0:
			if (!getCanny(mSrcImage, mDesImage))
			{
			}
			break;
		case 1:
			if (!getLaplacian(mSrcImage, mDesImage))
			{
			}
			break;
		case 2:
			if (!getPrewitt(mSrcImage, mDesImage))
			{
			}
			break;
		case 3:
			if (!getScharr(mSrcImage, mDesImage))
			{
			}
			break;
		case 4:
			if (!getSobel(mSrcImage, mDesImage))
			{
			}
			break;
		default:
			break;
		}
#if _DEBUG
		CvWaitShowImage(mDesImage, "轮廓");
#endif	
	}

	/// <summary>
	/// 亚像素边缘检测
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="DesImage"></param>
	/// <param name="vecContourPoints"></param>
	void CGradient::getSubPixelGradient(cv::Mat mSrcImage, cv::Mat& DesImage, std::vector<cv::Point2d>& vecContourPoints)
	{
		if (!DesImage.data)
			return;
		double* x;          /* x[n] y[n] coordinates of result contour point n */
		double* y;
		int* curve_limits;  /* limits of the curves in the x[] and y[] */
		int N, M;         /* result: N contour points, forming M curves */
		double S = 2; /* default sigma=0 */
		double H = 4.2; /* default th_h=0  */
		double L = 0.81; /* default th_l=0  */
		double W = 1.0; /* default W=1.3   */
		cv::Mat dstImage = mSrcImage;
		const int iHeight = dstImage.rows;
		const int iWidth = dstImage.cols;
		uchar* pSrc = mSrcImage.data;//new uchar[iHeight*iWidth];
		uchar* pDst = dstImage.data;
		devernay(&x, &y, &N, &curve_limits, &M, pSrc, pDst, iWidth, iHeight, S, H, L);
		for (int k = 0; k < M; k++) /* write curves */
		{
			for (int i = curve_limits[k]; i < curve_limits[k + 1]; i++)
			{
				vecContourPoints.push_back(cv::Point2d(x[i], y[i]));
				DesImage.at<uchar>(y[i], x[i]) = 255;
			}
		}
#if _DEBUG
		CvWaitShowImage(DesImage, "亚像素轮廓");
#endif	
		free(x); x = NULL;
		free(y); y = NULL;
		free(curve_limits); curve_limits = NULL;
	}

	/// <summary>
	/// sobel算子
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <returns></returns>
	bool CGradient::getSobel(cv::Mat mSrcImage, cv::Mat& mDesImage)
	{
		//【0】创建 grad_x 和 grad_y 矩阵
		cv::Mat mGrad_x, mGrad_y;
		cv::Mat mAbs_grad_x, mAbs_grad_y;
	
		if (!mSrcImage.data)
		{
			std::cout << "falied to read" << std::endl;
			return false;
		}

		if (mSrcImage.channels() > 1)
			return false;
		//【3】求 X方向梯度
		Sobel(mSrcImage, mGrad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
		convertScaleAbs(mGrad_x, mAbs_grad_x);

		//【4】求Y方向梯度
		Sobel(mSrcImage, mGrad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
		convertScaleAbs(mGrad_y, mAbs_grad_y);

		//【5】合并梯度(近似)
		addWeighted(mAbs_grad_x, 0.5, mAbs_grad_y, 0.5, 0, mDesImage);

		return true;
	}

	/// <summary>
	/// canny算子
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <returns></returns>
	bool CGradient::getCanny(cv::Mat mSrcImage, cv::Mat& mDesImage)
	{		
		if (!mSrcImage.data)
		{
			std::cout << "falied to read" << std::endl;
			return false;
		}

		// 【2】将原图像转换为灰度图像
		if (mSrcImage.channels() > 1)
			return false;

		//二值化处理
		int thresh = OtsuThres(mSrcImage);
		double cannyHighThreshTemplate = max(10, thresh);
		// 【3】运行Canny算子
		cv::Canny(mSrcImage, mDesImage, cannyHighThreshTemplate / 3, cannyHighThreshTemplate, 3);
		//cv::Canny(mSrcImage, mDesImage, 20, 60, 3);b
		return true;
	}

	/// <summary>
	/// laplacian算子
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <returns></returns>
	bool CGradient::getLaplacian(cv::Mat mSrcImage, cv::Mat& mDesImage)
	{
		if (!mSrcImage.data)
		{
			std::cout << "falied to read" << std::endl;
			return false;
		}

		// 【2】将原图像转换为灰度图像
		if (mSrcImage.channels() > 1)
			return false;

		// 【3】运行Laplacian算子
		cv::Laplacian(mSrcImage, mDesImage, CV_16S);

		return true;
	}

	/// <summary>
	/// scharr算子
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <returns></returns>
	bool CGradient::getScharr(cv::Mat mSrcImage, cv::Mat& mDesImage)
	{
		//【0】创建 grad_x 和 grad_y 矩阵
		cv::Mat mGrad_x, mGrad_y;
		cv::Mat mAbs_grad_x, mAbs_grad_y;
		
		if (!mSrcImage.data)
		{
			std::cout << "falied to read" << std::endl;
			return false;
		}
		if (mSrcImage.channels() > 1)
			return false;
		//【3】求 X方向梯度
		Scharr(mSrcImage, mGrad_x, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
		convertScaleAbs(mGrad_x, mAbs_grad_x);

		//【4】求Y方向梯度
		Scharr(mSrcImage, mGrad_y, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
		convertScaleAbs(mGrad_y, mAbs_grad_y);

		//【5】合并梯度(近似)
		addWeighted(mAbs_grad_x, 0.5, mAbs_grad_y, 0.5, 0, mDesImage);

		return true;
	}

	/// <summary>
	/// prewitt算子
	/// </summary>
	/// <param name="mSrcImage"></param>
	/// <param name="mDesImage"></param>
	/// <returns></returns>
	bool CGradient::getPrewitt(cv::Mat mSrcImage, cv::Mat& mDesImage)
	{
		cv::Mat mGrayImage, mKernelx, mKernely;		 
		if (!mSrcImage.data)
		{
			std::cout << "falied to read" << std::endl;
			return false;
		}
		if (mSrcImage.channels() > 1)
			return false;

		mKernelx = (cv::Mat_<double>(3, 3) << 1, 1, 1, 0, 0, 0, -1, -1, -1);
		mKernely = (cv::Mat_<double>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);

		cv::Mat mGrad_x, mGrad_y;
		cv::Mat mAbs_grad_x, mAbs_grad_y, mGrad;

		filter2D(mSrcImage, mGrad_x, CV_16S, mKernelx, cv::Point(-1, -1));
		filter2D(mSrcImage, mGrad_y, CV_16S, mKernely, cv::Point(-1, -1));

		convertScaleAbs(mGrad_x, mAbs_grad_x);
		convertScaleAbs(mGrad_y, mAbs_grad_y);

		addWeighted(mAbs_grad_x, 0.5, mAbs_grad_y, 0.5, 0, mDesImage);

		return true;
	}

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
	void CGradient::devernay(double** x, double** y, int* N, int** curve_limits, int* M, uchar* image, uchar* gauss, int X, int Y, double sigma, double th_h, double th_l)
	{
		double* Gx = (double*)xmalloc(X * Y * sizeof(double));     /* grad_x */
		double* Gy = (double*)xmalloc(X * Y * sizeof(double));     /* grad_y */
		double* modG = (double*)xmalloc(X * Y * sizeof(double));   /* |grad| */
		double* Ex = (double*)xmalloc(X * Y * sizeof(double));     /* edge_x */
		double* Ey = (double*)xmalloc(X * Y * sizeof(double));     /* edge_y */
		int* next = (int*)xmalloc(X * Y * sizeof(int));			 /* next point in chain */
		int* prev = (int*)xmalloc(X * Y * sizeof(int));			 /* prev point in chain */

		if (sigma == 0.0) compute_gradient(Gx, Gy, modG, image, X, Y);
		else
		{
			//高斯滤波
			gaussian_filter(image, gauss, X, Y, sigma);
			//计算每个像素点的Gx，Gy梯度和梯度模值modG
			compute_gradient(Gx, Gy, modG, gauss, X, Y);
		}

		compute_edge_points(Ex, Ey, modG, Gx, Gy, X, Y);

		chain_edge_points(next, prev, Ex, Ey, Gx, Gy, X, Y);

		thresholds_with_hysteresis(next, prev, modG, X, Y, th_h, th_l);

		list_chained_edge_points(x, y, N, curve_limits, M, next, prev, Ex, Ey, X, Y);

		/* free memory */
		free((void*)Gx);
		free((void*)Gy);
		free((void*)modG);
		free((void*)Ex);
		free((void*)Ey);
		free((void*)next);
		free((void*)prev);
		Gx = NULL; Gy = NULL; modG = NULL; Ex = NULL; Ey = NULL; next = NULL; prev = NULL;
	}


	void CGradient::error(char* msg)
	{
		fprintf(stderr, "error: %s\n", msg);
		exit(EXIT_FAILURE);
	}

	void* CGradient::xmalloc(size_t size)
	{
		void* p;
		if (size == 0) error("xmalloc: zero size");
		p = malloc(size);
		if (p == NULL)
		{
			free(p);
			error("xmalloc: out of memory");
		}
		return p;
	}

	int CGradient::Greater(double a, double b)
	{
		if (a <= b) return 0;  /* trivial case, return as soon as possible */
		if ((a - b) < 1000 * DBL_EPSILON) return 0;
		return 1; /* greater */
	}

	double CGradient::dist(double x1, double y1, double x2, double y2)
	{
		return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	}

	void CGradient::gaussian_kernel(double* pfGaussFilter, int n, double sigma, double mean)
	{
		double sum = 0.0;
		double val;
		int i;
		if (pfGaussFilter == NULL) error("gaussian_kernel: kernel not allocated");
		if (sigma <= 0.0) error("gaussian_kernel: sigma must be positive");

		// compute Gaussian kernel
		for (i = 0; i < n; i++)
		{
			val = ((double)i - mean) / sigma;
			pfGaussFilter[i] = exp(-0.5 * val * val);
			sum += pfGaussFilter[i];
		}

		// normalization
		if (sum > 0.0) for (i = 0; i < n; i++)
			pfGaussFilter[i] /= sum;
	}

	void CGradient::gaussian_filter(uchar* image, uchar* out, int X, int Y, double sigma)
	{
		int x, y, offset, i, j, nx2, ny2, n;
		double* kernel;
		double* tmp;
		double val, prec;

		if (sigma <= 0.0) error("gaussian_filter: sigma must be positive");
		if (image == NULL || X < 1 || Y < 1) error("gaussian_filter: invalid image");


		tmp = (double*)xmalloc(X * Y * sizeof(double));
		prec = 3.0;
		offset = (int)ceil(sigma * sqrt(2.0 * prec * log(10.0)));
		n = 1 + 2 * offset; //kernel size
		kernel = (double*)xmalloc(n * sizeof(double));
		gaussian_kernel(kernel, n, sigma, (double)offset);

		// auxiliary variables for the double of the image size
		nx2 = 2 * X;
		ny2 = 2 * Y;

		// x axis convolution
		for (x = 0; x < X; x++)
			for (y = 0; y < Y; y++)
			{
				val = 0.0;
				for (i = 0; i < n; i++)
				{
					j = x - offset + i;

					// symmetry boundary condition
					while (j < 0)
						j += nx2;
					while (j >= nx2)
						j -= nx2;
					if (j >= X)
						j = nx2 - 1 - j;

					val += (double)image[j + y * X] * kernel[i];
				}
				tmp[x + y * X] = val;
			}

		// y axis convolution
		for (x = 0; x < X; x++)
			for (y = 0; y < Y; y++)
			{
				val = 0.0;
				for (i = 0; i < n; i++)
				{
					j = y - offset + i;

					// symmetry boundary condition 
					while (j < 0)
						j += ny2;
					while (j >= ny2)
						j -= ny2;
					if (j >= Y)
						j = ny2 - 1 - j;

					val += tmp[x + j * X] * kernel[i];
				}
				out[x + y * X] = (uchar)val;
			}

		free((void*)kernel);
		free((void*)tmp);
		kernel = NULL, tmp = NULL;
	}

	double CGradient::chain(int from, int to, double* Ex, double* Ey, double* Gx, double* Gy, int X, int Y)
	{
		double dx, dy;
		if (Ex == NULL || Ey == NULL || Gx == NULL || Gy == NULL)
			error("chain: invalid input");
		if (from < 0 || to < 0 || from >= X * Y || to >= X * Y)
			error("chain: one of the points is out the image");

		//check that the points are different and valid edge points,otherwise return invalid chaining
		if (from == to)
			return 0.0; // same pixel, not a valid chaining
		if (Ex[from] < 0.0 || Ey[from] < 0.0 || Ex[to] < 0.0 || Ey[to] < 0.0)
			return 0.0; // one of them is not an edge point, not a valid chaining

						/* in a good chaining, the gradient should be roughly orthogonal
						to the line joining the two points to be chained:
						when Gy * dx - Gx * dy > 0, it corresponds to a forward chaining,
						when Gy * dx - Gx * dy < 0, it corresponds to a backward chaining.

						first check that the gradient at both points to be chained agree
						in one direction, otherwise return invalid chaining. */
		dx = Ex[to] - Ex[from];
		dy = Ey[to] - Ey[from];
		if ((Gy[from] * dx - Gx[from] * dy) * (Gy[to] * dx - Gx[to] * dy) <= 0.0)
			return 0.0; /* incompatible gradient angles, not a valid chaining */

						/* return the chaining score: positive for forward chaining,negative for backwards.
						the score is the inverse of the distance to the chaining point, to give preference to closer points */
		if ((Gy[from] * dx - Gx[from] * dy) >= 0.0)
			return  1.0 / dist(Ex[from], Ey[from], Ex[to], Ey[to]); /* forward chaining  */
		else
			return -1.0 / dist(Ex[from], Ey[from], Ex[to], Ey[to]); /* backward chaining */
	}

	void CGradient::compute_gradient(double* Gx, double* Gy, double* modG, uchar* image, int X, int Y)
	{
		int x, y;

		if (Gx == NULL || Gy == NULL || modG == NULL || image == NULL)
			error("compute_gradient: invalid input");

		// approximate image gradient using centered differences
		for (x = 1; x < (X - 1); x++)
			for (y = 1; y < (Y - 1); y++)
			{
				Gx[x + y * X] = (double)image[(x + 1) + y * X] - (double)image[(x - 1) + y * X];
				Gy[x + y * X] = (double)image[x + (y + 1) * X] - (double)image[x + (y - 1) * X];
				modG[x + y * X] = sqrt(Gx[x + y * X] * Gx[x + y * X] + Gy[x + y * X] * Gy[x + y * X]);
			}
	}

	void CGradient::compute_edge_points(double* Ex, double* Ey, double* modG, double* Gx, double* Gy, int X, int Y)
	{
		int x, y, i;

		if (Ex == NULL || Ey == NULL || modG == NULL || Gx == NULL || Gy == NULL)
			error("compute_edge_points: invalid input");
		/* initialize Ex and Ey as non-edge points for all pixels */
		for (i = 0; i < X * Y; i++) Ex[i] = Ey[i] = -1.0;
		/* explore pixels inside a 2 pixel margin (so modG[x,y +/- 1,1] is defined) */
		for (x = 2; x < (X - 2); x++)
			for (y = 2; y < (Y - 2); y++)
			{
				int Dx = 0;                     /* interpolation is along Dx,Dy		*/
				int Dy = 0;                     /* which will be selected below		*/
				double mod = modG[x + y * X];     /* modG at pixel					*/
				double L = modG[x - 1 + y * X];   /* modG at pixel on the left			*/
				double R = modG[x + 1 + y * X];   /* modG at pixel on the right		*/
				double U = modG[x + (y + 1) * X]; /* modG at pixel up					*/
				double D = modG[x + (y - 1) * X]; /* modG at pixel below				*/
				double gx = fabs(Gx[x + y * X]);  /* absolute value of Gx				*/
				double gy = fabs(Gy[x + y * X]);  /* absolute value of Gy				*/
												/* when local horizontal maxima of the gradient modulus and the gradient direction
												is more horizontal (|Gx| >= |Gy|),=> a "horizontal" (H) edge found else,
												if local vertical maxima of the gradient modulus and the gradient direction is more
												vertical (|Gx| <= |Gy|),=> a "vertical" (V) edge found */

												/* it can happen that two neighbor pixels have equal value and are both	maxima, for example
												when the edge is exactly between both pixels. in such cases, as an arbitrary convention,
												the edge is marked on the left one when an horizontal max or below when a vertical max.
												for	this the conditions are L < mod >= R and D < mod >= U,respectively. the comparisons are
												done using the function Greater() instead of the operators > or >= so numbers differing only
												due to rounding errors are considered equal */
				if (Greater(mod, L) && !Greater(R, mod) && gx >= gy) Dx = 1; /* H */
				else if (Greater(mod, D) && !Greater(U, mod) && gx <= gy) Dy = 1; /* V */

																				  /* Devernay sub-pixel correction

																				  the edge point position is selected as the one of the maximum of a quadratic interpolation of the magnitude of
																				  the gradient along a unidimensional direction. the pixel must be a local maximum. so we	have the values:

																				  the x position of the maximum of the parabola passing through(-1,a), (0,b), and (1,c) is
																				  offset = (a - c) / 2(a - 2b + c),and because b >= a and b >= c, -0.5 <= offset <= 0.5	*/
				if (Dx > 0 || Dy > 0)
				{
					/* offset value is in [-0.5, 0.5] */
					double a = modG[x - Dx + (y - Dy) * X];
					double b = modG[x + y * X];
					double c = modG[x + Dx + (y + Dy) * X];
					double offset = 0.5 * (a - c) / (a - b - b + c);

					/* store edge point */
					Ex[x + y * X] = x + offset * Dx;
					Ey[x + y * X] = y + offset * Dy;
				}
			}
	}

	void CGradient::chain_edge_points(int* next, int* prev, double* Ex, double* Ey, double* Gx, double* Gy, int X, int Y)
	{
		int x, y, i, j, alt;

		if (next == NULL || prev == NULL || Ex == NULL || Ey == NULL || Gx == NULL || Gy == NULL)
			error("chain_edge_points: invalid input");

		/* initialize next and prev as non linked */
		for (i = 0; i < X * Y; i++) next[i] = prev[i] = -1;

		/* try each point to make local chains */
		for (x = 2; x < (X - 2); x++)   /* 2 pixel margin to include the tested neighbors */
			for (y = 2; y < (Y - 2); y++)
				if (Ex[x + y * X] >= 0.0 && Ey[x + y * X] >= 0.0) /* must be an edge point */
				{
					int from = x + y * X;  /* edge point to be chained			*/
					double fwd_s = 0.0;  /* score of best forward chaining		*/
					double bck_s = 0.0;  /* score of best backward chaining		*/
					int fwd = -1;        /* edge point of best forward chaining */
					int bck = -1;        /* edge point of best backward chaining*/

										 /* try all neighbors two pixels apart or less.
										 looking for candidates for chaining two pixels apart, in most such cases,
										 is enough to obtain good chains of edge points that	accurately describes the edge.	*/
					for (i = -2; i <= 2; i++)
						for (j = -2; j <= 2; j++)
						{
							int to = x + i + (y + j) * X; /* candidate edge point to be chained */
							double s = chain(from, to, Ex, Ey, Gx, Gy, X, Y);  /* score from-to */

							if (s > fwd_s)	/* a better forward chaining found    */
							{
								fwd_s = s;  /* set the new best forward chaining  */
								fwd = to;
							}
							if (s < bck_s)	/* a better backward chaining found	  */
							{
								bck_s = s;  /* set the new best backward chaining */
								bck = to;
							}
						}

					/* before making the new chain, check whether the target was
					already chained and in that case, whether the alternative
					chaining is better than the proposed one.

					x alt                        x alt
					\                          /
					\                        /
					from x---------x fwd              bck x---------x from

					we know that the best forward chain starting at from is from-fwd.
					but it is possible that there is an alternative chaining arriving
					at fwd that is better, such that alt-fwd is to be preferred to
					from-fwd. an analogous situation is possible in backward chaining,
					where an alternative link bck-alt may be better than bck-from.

					before making the new link, check if fwd/bck are already chained,
					and in such case compare the scores of the proposed chaining to
					the existing one, and keep only the best of the two.

					there is an undesirable aspect of this procedure: the result may
					depend on the order of exploration. consider the following
					configuration:

					a x-------x b
					/
					/
					c x---x d    with score(a-b) < score(c-b) < score(c-d)
					or equivalently ||a-b|| > ||b-c|| > ||c-d||

					let us consider two possible orders of exploration.

					order: a,b,c
					we will first chain a-b when exploring a. when analyzing the
					backward links of b, we will prefer c-b, and a-b will be unlinked.
					finally, when exploring c, c-d will be preferred and c-b will be
					unlinked. the result is just the chaining c-d.

					order: c,b,a
					we will first chain c-d when exploring c. then, when exploring
					the backward connections of b, c-b will be the preferred link;
					but because c-d exists already and has a better score, c-b
					cannot be linked. finally, when exploring a, the link a-b will
					be created because there is no better backward linking of b.
					the result is two chainings: c-d and a-b.

					we did not found yet a simple algorithm to solve this problem. by
					simple, we mean an algorithm without two passes or the need to
					re-evaluate the chaining of points where one link is cut.

					for most edge points, there is only one possible chaining and this
					problem does not arise. but it does happen and a better solution
					is desirable.
					*/
					if (fwd >= 0 && next[from] != fwd &&
						((alt = prev[fwd]) < 0 || chain(alt, fwd, Ex, Ey, Gx, Gy, X, Y) < fwd_s))
					{
						if (next[from] >= 0)     /* remove previous from-x link if one */
							prev[next[from]] = -1;  /* only prev requires explicit reset  */
						next[from] = fwd;         /* set next of from-fwd link          */
						if (alt >= 0)            /* remove alt-fwd link if one         */
							next[alt] = -1;         /* only next requires explicit reset  */
						prev[fwd] = from;         /* set prev of from-fwd link          */
					}
					if (bck >= 0 && prev[from] != bck &&
						((alt = next[bck]) < 0 || chain(alt, bck, Ex, Ey, Gx, Gy, X, Y) > bck_s))
					{
						if (alt >= 0)            /* remove bck-alt link if one         */
							prev[alt] = -1;         /* only prev requires explicit reset  */
						next[bck] = from;         /* set next of bck-from link          */
						if (prev[from] >= 0)     /* remove previous x-from link if one */
							next[prev[from]] = -1;  /* only next requires explicit reset  */
						prev[from] = bck;         /* set prev of bck-from link          */
					}
				}
	}

	void CGradient::thresholds_with_hysteresis(int* next, int* prev, double* modG, int X, int Y, double th_h, double th_l)
	{
		int* valid;
		int i, j, k;

		/* check input */
		if (next == NULL || prev == NULL || modG == NULL)
			error("thresholds_with_hysteresis: invalid input");

		/* get memory */
		valid = (int*)xmalloc(X * Y * sizeof(int));
		for (i = 0; i < X * Y; i++) valid[i] = 0;

		/* validate all edge points over th_h or connected to them and over th_l */
		for (i = 0; i < X * Y; i++)   /* prev[i]>=0 or next[i]>=0 implies an edge point */
			if ((prev[i] >= 0 || next[i] >= 0) && !valid[i] && modG[i] >= th_h)
			{
				valid[i] = 1; /* mark as valid the new point */

								 /* follow the chain of edge points forwards */
				for (j = i; j >= 0 && (k = next[j]) >= 0 && !valid[k]; j = next[j])
					if (modG[k] < th_l)
					{
						next[j] = -1;  /* cut the chain when the point is below th_l */
						prev[k] = -1;  /* j must be assigned to next[j] and not k,
									   so the loop is chained in this case */
					}
					else
						valid[k] = 1; /* otherwise mark the new point as valid */

										 /* follow the chain of edge points backwards */
				for (j = i; j >= 0 && (k = prev[j]) >= 0 && !valid[k]; j = prev[j])
					if (modG[k] < th_l)
					{
						prev[j] = -1;  /* cut the chain when the point is below th_l */
						next[k] = -1;  /* j must be assigned to prev[j] and not k,
									   so the loop is chained in this case */
					}
					else
						valid[k] = 1; /* otherwise mark the new point as valid */
			}

		/* remove any remaining non-valid chained point */
		for (i = 0; i < X * Y; i++)   /* prev[i]>=0 or next[i]>=0 implies edge point */
			if ((prev[i] >= 0 || next[i] >= 0) && !valid[i])
				prev[i] = next[i] = -1;

		/* free memory */
		free((void*)valid);
	}

	void CGradient::list_chained_edge_points(double** x, double** y, int* N, int** curve_limits, int* M, int* next, int* prev, double* Ex, double* Ey, int X, int Y)
	{
		int i, k, n;

		/* initialize output: x, y, curve_limits, N, and M

		there cannot be more than X*Y edge points to be put in the output list:
		edge points must be local maxima of gradient modulus, so at most half of
		the pixels could be so. when a closed curve is found, one edge point will
		be put twice to the output. even if all possible edge points (half of the
		pixels in the image) would form one pixel closed curves (which is not
		possible) that would lead to output X*Y edge points.

		for the same reason, there cannot be more than X*Y curves: the worst case
		is when all possible edge points (half of the pixels in the image) would
		form one pixel chains. in that case (which is not possible) one would need
		a size for curve_limits of X*Y/2+1. so X*Y is enough.

		(curve_limits requires one more item than the number of curves.
		a simplest example is when only one chain of length 3 is present:
		curve_limits[0] = 0, curve_limits[1] = 3.)
		*/
		* x = (double*)xmalloc(X * Y * sizeof(double));
		*y = (double*)xmalloc(X * Y * sizeof(double));
		*curve_limits = (int*)xmalloc(X * Y * sizeof(int));
		*N = 0;
		*M = 0;

		/* copy chained edge points to output */
		for (i = 0; i < X * Y; i++)   /* prev[i]>=0 or next[i]>=0 implies an edge point */
			if (prev[i] >= 0 || next[i] >= 0)
			{
				/* a new chain found, set chain starting index to the current point
				and then increase the curve counter */
				(*curve_limits)[*M] = *N;
				++(*M);

				/* set k to the beginning of the chain, or to i if closed curve */
				for (k = i; (n = prev[k]) >= 0 && n != i; k = n);

				/* follow the chain of edge points starting on k */
				do
				{
					/* store the current point coordinates in the output lists */
					(*x)[*N] = Ex[k];
					(*y)[*N] = Ey[k];
					++(*N);

					n = next[k];   /* save the id of the next point in the chain */

					next[k] = -1;  /* unlink chains from k so it is not used again */
					prev[k] = -1;


					k = n;  /* set the current point to the next in the chain */
				} while (k >= 0); /* continue while there is a next point in the chain */
			}
		(*curve_limits)[*M] = *N; /* store end of the last chain */
	}