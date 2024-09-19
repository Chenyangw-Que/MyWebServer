
#pragma once
#include "paraType.h"
#include <algorithm>
#include <vector>
#include "Common.h"
// template <class vecPoint>
// bool FitCircleRansacBetter(vecPoint &pPoints, cv::Point3d &pCent,
//                            float &fRoundness, float fDistThreshold,
//                            const float fNominalRadius, int sampleTimes) {
//   const int MIN_SAMPLE_NUM = 3;        // 最少样本数
//   const int maxTimes = 2000;           // 最大采样次数
//   const int MAX_SEARCH_TIMES = 100000; // 最大搜索次数
//   srand((unsigned)time(0));            //随机采样

//   if (pPoints.size() < MIN_SAMPLE_NUM)
//     return false;

//   double disThreshold = 2.0;
//   int ptNum = pPoints.size();
//   int realSamTimes = 0;
//   int maxCnt = 0;
//   float fMinRadius = fNominalRadius - fDistThreshold;
//   float fMaxRadius = fNominalRadius + fDistThreshold;

//   fRoundness = 0;
//   sampleTimes = (sampleTimes > maxTimes) ? maxTimes : sampleTimes;
//   for (int t = 0; t < sampleTimes;) {
//     realSamTimes++;
//     if (realSamTimes >= MAX_SEARCH_TIMES) {
//       break;
//     }

//     int aSampleIndex[MIN_SAMPLE_NUM];
//     vecPoint samPoint;
//     vecPoint samPointBuffer;
//     do {
//       aSampleIndex[0] = rand() % ptNum;
//       aSampleIndex[1] = rand() % ptNum;
//       aSampleIndex[2] = rand() % ptNum;
//     } while ((aSampleIndex[0] == aSampleIndex[1]) ||
//              (aSampleIndex[1] == aSampleIndex[2]) ||
//              (aSampleIndex[0] == aSampleIndex[2])); // 要求采样3个不同的点
//     for (int x = 0; x < 3; ++x)
//       samPoint.push_back(pPoints[aSampleIndex[x]]);
//     // 考查3点分布，是否满足要求，若不满足要求，抛弃
//     /*if (!Is3PointsValid2(samPoint, fNominalRadius))
//             continue;	*/
//     cv::Point3f pCen3f = leastSquareFittingCircle(samPoint);
//     // 如果当前拟合的圆的半径，超出容许范围，则跳过（2010-10-21添加）
//     if ((pCen3f.z < fMinRadius) || (pCen3f.z > fMaxRadius))
//       continue;

//     //计算支持度
//     int supportCnt = 0;
//     for (int u = 0; u < ptNum; u++) {
//       //求点到圆的距离
//       float distToCircle = abs(sqrt(pow(pPoints[u].x - pCen3f.x, 2) +
//                                     pow(pPoints[u].y - pCen3f.y, 2)) -
//                                pCen3f.z);

//       if (distToCircle < disThreshold) {
//         supportCnt++;
//       }
//     }
//     if (supportCnt > maxCnt) {
//       maxCnt = supportCnt;
//       pCent.x = pCen3f.x;
//       pCent.y = pCen3f.y;
//       pCent.z = pCen3f.z;
//     }
//     t++;
//   }

//   vecPoint pPointN, pPointNBuffer;
//   for (int u = 0; u < ptNum; u++) {
//     //求点到圆的距离
//     float distToCircle = abs(
//         sqrt(pow(pPoints[u].x - pCent.x, 2) + pow(pPoints[u].y - pCent.y, 2)) -
//         pCent.z);

//     if (distToCircle < disThreshold)
//       pPointN.push_back(pPoints[u]);
//   }
//   pCent = leastSquareFittingCircle(pPointN);
//   pPoints = pPointN;
// }

class CFitCircle
{
public:
  /************************************************************************/
  /* 像素级的圆拟合                                                       */
  /************************************************************************/
  template <class TypePoint>
  int FitCircleRansacBetter(TypePoint *pPoints, int nPoints,
                            float &fCenterX, float &fCenterY, float &fRadius,
                            float &fRoundness,
                            float fDistThreshold,
                            const float fNominalRadius,
                            int sampleTimes)
  {
    const int MIN_SAMPLE_NUM = 3;        // 最少样本数
    const int maxTimes = 2000;           // 最大采样次数
    const int MAX_SEARCH_TIMES = 100000; // 最大搜索次数
    srand((unsigned)time(0));            //随机采样

    if (pPoints == NULL || nPoints < MIN_SAMPLE_NUM)
      return -4;

    double disThreshold = 2.0;
    int ptNum = nPoints;
    int realSamTimes = 0;
    int maxCnt = 0;
    float fMinRadius = fNominalRadius - fDistThreshold;
    float fMaxRadius = fNominalRadius + fDistThreshold;

    fRoundness = 0;
    sampleTimes = (sampleTimes > maxTimes) ? maxTimes : sampleTimes;

    for (int t = 0; t < sampleTimes;)
    {
      realSamTimes++;
      if (realSamTimes >= MAX_SEARCH_TIMES)
      {
        break;
      }

      int aSampleIndex[MIN_SAMPLE_NUM];

      TypePoint samPoint[MIN_SAMPLE_NUM];

      TypePoint samPointBuffer[MIN_SAMPLE_NUM];

      do
      {
        aSampleIndex[0] = rand() % ptNum;
        aSampleIndex[1] = rand() % ptNum;
        aSampleIndex[2] = rand() % ptNum;
      } while ((aSampleIndex[0] == aSampleIndex[1]) ||
               (aSampleIndex[1] == aSampleIndex[2]) ||
               (aSampleIndex[0] == aSampleIndex[2])); // 要求采样3个不同的点

      samPoint[0] = pPoints[aSampleIndex[0]];
      samPoint[1] = pPoints[aSampleIndex[1]];
      samPoint[2] = pPoints[aSampleIndex[2]];

      // 考查3点分布，是否满足要求，若不满足要求，抛弃
      if (!Is3PointsValid(samPoint, fNominalRadius))
      {
        continue;
      }

      float cx = 0.0f, cy = 0.0f, R = 0.0f;
      LeastSquaresFitting2(MIN_SAMPLE_NUM, samPoint, samPointBuffer, cx, cy, R);

      // 如果当前拟合的圆的半径，超出容许范围，则跳过（2010-10-21添加）
      if ((R < fMinRadius) || (R > fMaxRadius))
        continue;

      //计算支持度
      int supportCnt = 0;
      for (int u = 0; u < ptNum; u++)
      {
        //求点到圆的距离
        float distToCircle;
        DistanceOfPointToCircle((float)pPoints[u].x, (float)pPoints[u].y, cx, cy, R, distToCircle);
        if (distToCircle < disThreshold)
        {
          supportCnt++;
        }
      }

      if (supportCnt > maxCnt)
      {
        maxCnt = supportCnt;
        fCenterX = cx;
        fCenterY = cy;
        fRadius = R;
      }
      t++;
    }

    TypePoint *pPointN = new TypePoint[ptNum];
    TypePoint *pPointNBuffer = new TypePoint[ptNum];
    int nCount = 0;
    for (int u = 0; u < ptNum; u++)
    {
      //求点到圆的距离
      float distToCircle;
      DistanceOfPointToCircle((float)pPoints[u].x, (float)pPoints[u].y, fCenterX, fCenterY, fRadius, distToCircle);
      if (distToCircle < disThreshold)
      {
        pPointN[nCount] = pPoints[u];
        nCount++;
      }
    }

    LeastSquaresFitting2(nCount, pPointN, pPointNBuffer, fCenterX, fCenterY, fRadius);

    // 计算圆形度
    float fAngleIntervalT = 2 * asin(0.5f / fRadius) * 3;
    fAngleIntervalT *= (180.0 / PI_32F); // 转成度
    fAngleIntervalT = std::max((float)2, (float)fAngleIntervalT);
    fRoundness = 0;

    CalOutOfRoundness(pPointN, nCount, fDistThreshold, fAngleIntervalT,
                      fCenterX, fCenterY, fRadius, fRoundness);
    //
    // 			GrayBitmap *bmpShow = WritePointsOnImage(pPointN,nCount,1626,1236);
    // 			CvWaitShowImage(bmpShow,"Fit_Pixel_拟合_pix");
    // 			ReleaseBitmap8U(bmpShow);

    delete[] pPointN;
    pPointN = NULL;
    delete[] pPointNBuffer;
    pPointNBuffer = NULL;

    if (fRoundness < 0.5f)
    {
      return -2;
    }
    return 0;
  }

  /************************************************************************/
  /* 亚像素级别的圆拟合												    */
  /************************************************************************/
  template <class TypePoint>
  int FitCircleRansacBetterSubPix(TypePoint *pPoints, int nPoints,
                                  float &fCenterX, float &fCenterY, float &fRadius,
                                  float &fRoundness,
                                  float fDistThreshold,
                                  const float fNominalRadius,
                                  int sampleTimes)
  {
    if (nPoints < 4)
    {
      fRoundness = 0;
      return 0;
    }

    int ptNum = nPoints;
    TypePoint *pPointN = new TypePoint[ptNum];
    TypePoint *pPointNBuffer = new TypePoint[ptNum];
    int nCount = 0;
    float disThreshold = 2.0;
    for (int u = 0; u < ptNum; u++)
    {
      //求点到圆的距离
      float distToCircle;
      DistanceOfPointToCircle((float)pPoints[u].x, (float)pPoints[u].y, fCenterX, fCenterY, fRadius, distToCircle);
      if (distToCircle < disThreshold)
      {
        pPointN[nCount] = pPoints[u];
        nCount++;
      }
    }
    //
    // 			GrayBitmap *bmpShow = WritePointsOnImage(pPointN,nCount,1626,1236);
    // 			CvWaitShowImage(bmpShow,"Fit_Pixel_拟合-subpix");
    // 			ReleaseBitmap8U(bmpShow);

    LeastSquaresFitting2(nCount, pPointN, pPointNBuffer, fCenterX, fCenterY, fRadius);

    // 计算圆形度
    float fAngleIntervalT = 2 * asin(0.5f / fRadius) * 3;
    fAngleIntervalT *= (180.0 / PI_32F); // 转成度
    fAngleIntervalT = std::max(float(2.0), fAngleIntervalT);
    fRoundness = 0;
    CalOutOfRoundness(pPointN, nCount, fDistThreshold, fAngleIntervalT,
                      fCenterX, fCenterY, fRadius, fRoundness);

    delete[] pPointN;
    pPointN = NULL;
    delete[] pPointNBuffer;
    pPointNBuffer = NULL;

    if (fRoundness < 0.5f)
    {
      return -2;
    }
    return 0;
  }

  /************************************************************************/
  /*   // 功能：计算圆度
		// fAngleInterval, 角度间隔, 如果大于此间隔，则认为有空缺               */
  /************************************************************************/
  template <class TypePoint>
  bool CalOutOfRoundness(const TypePoint *pSrcPoints,
                         const int ptN,
                         const float fDistThresold,
                         const float fAngleIntervalDegT,
                         float fCenterX, float fCenterY, float fRadius,
                         float &roundness)
  {
    roundness = 0;

    //int ptN = vecSrcPoints.size();
    if (ptN <= 0)
      return false;
    if (fRadius <= 1e-6f)
      return false;

    std::vector<float> vecAngle;

    for (int i = 0; i < ptN; i++)
    {
      float fDist = 0;

      DistanceOfPointToCircle((float)pSrcPoints[i].x, (float)pSrcPoints[i].y,
                              fCenterX, fCenterY, fRadius, fDist);

      // 如果在圆带内
      if (fDist < fDistThresold)
      {
        // 角度
        float fAngleArc = 0;
        AngleofPoint360(pSrcPoints[i].x - fCenterX, pSrcPoints[i].y - fCenterY, fAngleArc);

        vecAngle.push_back(fAngleArc);
      }
    }

    // 排序
    std::sort(vecAngle.begin(), vecAngle.end());
    int ptAngleNum = vecAngle.size();

    float fAngleIntervalArcT = fAngleIntervalDegT * PI_32F / 180; // 转成弧度

    //
    float fAccInterval = 0;
    for (int i = 1; i < ptAngleNum; i++)
    {
      float angleInterval = abs(vecAngle[i] - vecAngle[i - 1]);

      if (angleInterval > fAngleIntervalArcT)
      {
        fAccInterval += angleInterval;
      }
    }
    //
    fAccInterval += abs(abs(vecAngle[ptAngleNum - 1] - vecAngle[0]) - PI_32F * 2);

    roundness = 1 - fAccInterval / (2 * PI_32F);

    return true;
  }

  /************************************************************************/
  /* 判断3点是否有效                                                     */
  /************************************************************************/
  template <class TypePoint>
  bool Is3PointsValid(TypePoint samPoint[3], float R)
  {
    float aDist[3];
    // 2点之间的距离
    DistanceOfTwoPoints(samPoint[0].x, samPoint[0].y,
                        samPoint[1].x, samPoint[1].y,
                        aDist[0]);
    DistanceOfTwoPoints(samPoint[1].x, samPoint[1].y,
                        samPoint[2].x, samPoint[2].y,
                        aDist[1]);
    DistanceOfTwoPoints(samPoint[0].x, samPoint[0].y,
                        samPoint[2].x, samPoint[2].y,
                        aDist[2]);

    if ((aDist[0] < R) || (aDist[1] < R) || (aDist[2] < R))
    {
      return false;
    }
    else
    {
      return true;
    }
    return true;
  }

  /************************************************************************/
  /* 最小二乘法拟合圆                                                     */
  /************************************************************************/
  template <class TypePoint>
  static int LeastSquaresFitting2(int m_nNum, TypePoint *m_points0,
                                  TypePoint *pBuffer,
                                  float &fCenterX, float &fCenterY, float &fRadius)
  {
    if (m_nNum < 3)
      return 0;

    memcpy(pBuffer, m_points0, m_nNum * sizeof(TypePoint));

    float fMinX = (float)m_points0[0].x;
    float fMinY = (float)m_points0[0].y;
    for (int i = 0; i < m_nNum; i++)
    {
      if (m_points0[i].x < fMinX)
        fMinX = (float)m_points0[i].x;
      if (m_points0[i].y < fMinY)
        fMinY = (float)m_points0[i].y;
    }
    for (int i = 0; i < m_nNum; i++)
    {
#pragma warning(disable : 4244)
      pBuffer[i].x -= fMinX; // 请不要在此给我加(int),加了就错了！
      pBuffer[i].y -= fMinY; // 请不要在此给我加(int),加了就错了！
#pragma warning(default : 4244)
    }

    int i = 0;

    double X1 = 0;
    double Y1 = 0;
    double X2 = 0;
    double Y2 = 0;
    double X3 = 0;
    double Y3 = 0;
    double X1Y1 = 0;
    double X1Y2 = 0;
    double X2Y1 = 0;

    for (i = 0; i < m_nNum; i++)
    {
      X1 = X1 + pBuffer[i].x;
      Y1 = Y1 + pBuffer[i].y;

      X2 = X2 + pBuffer[i].x * pBuffer[i].x;
      Y2 = Y2 + pBuffer[i].y * pBuffer[i].y;
      X3 = X3 + pBuffer[i].x * pBuffer[i].x * pBuffer[i].x;
      Y3 = Y3 + pBuffer[i].y * pBuffer[i].y * pBuffer[i].y;

      X1Y1 = X1Y1 + pBuffer[i].x * pBuffer[i].y;
      X1Y2 = X1Y2 + pBuffer[i].x * pBuffer[i].y * pBuffer[i].y;
      X2Y1 = X2Y1 + pBuffer[i].x * pBuffer[i].x * pBuffer[i].y;
    }

    double C = 0, D = 0, E = 0, G = 0, H = 0, N = 0;
    double a = 0, b = 0, c = 0;

    N = m_nNum;
    C = N * X2 - X1 * X1;
    D = N * X1Y1 - X1 * Y1;
    E = N * X3 + N * X1Y2 - (X2 + Y2) * X1;
    G = N * Y2 - Y1 * Y1;
    H = N * X2Y1 + N * Y3 - (X2 + Y2) * Y1;

    double fenmu = C * G - D * D;
    if (abs(fenmu) < 1e-6)
      return 0;

    fenmu = 1.0 / fenmu;
    a = (H * D - E * G) * fenmu;
    b = -(H * C - E * D) * fenmu;
    c = -(a * X1 + b * Y1 + X2 + Y2) / N;

    double A = 0, B = 0, R = 0;

    A = a * -0.5;
    B = b * -0.5;
    R = sqrt(a * a + b * b - 4 * c) * 0.5;

    //fCenterX = (float)A;
    //fCenterY = (float)B;
    //fRadius = (float)R;
    fCenterX = (float)A + fMinX;
    fCenterY = (float)B + fMinY;
    fRadius = (float)R;

    return 1;
  }

  /************************************************************************/
  /* hough变换拟合圆                                                      */
  /************************************************************************/
  template <class TypePoint>
  static int HoughArc(TypePoint *pPoints, int pointCnt, int radiusIn, float &fCenterX, float &fCenterY, float &fRaiuds)
  { //int X[] , int Y[]
    std::vector<IntPoint> center;
    std::vector<int> VoteCnt;
    double theta;
    int a, b;
    int minA, maxA, minB, maxB;
    int VotedFlag = 0;
    double deltaTheta = PI_32F / 180; //间隔1度
    double startAngle = 150.0 * PI_32F / 180;
    double endAngle = PI_32F * 2 + PI_32F / 6;
    center.clear();
    VoteCnt.clear();
    minA = maxA = /*X[0]*/ pPoints[0].x - radiusIn;
    minB = maxB = /*X[0]*/ pPoints[0].x; //theta = 0
    //计算a，b的最小和最大值
    for (int i = 0; i < pointCnt; i++)
    {
      for (theta = startAngle; theta < endAngle; theta += deltaTheta)
      {
        a = (int)(/*X[i]*/ pPoints[i].x - radiusIn * cos(theta) + 0.5);
        b = (int)(/*Y[i]*/ pPoints[i].y - radiusIn * sin(theta) + 0.5);
        if (a > maxA)
        {
          maxA = a;
        }
        else if (a < minA)
        {
          minA = a;
        }

        if (b > maxB)
        {
          maxB = b;
        }
        else if (b < minB)
        {
          minB = b;
        }
      }
    }
    //确定a，b的范围之后，即确定了票箱的大小
    int aScale = maxA - minA + 1;
    int bScale = maxB - minB + 1;

    int *VoteBox = new int[aScale * bScale];
    //VoteBox初始化为0
    for (int i = 0; i < aScale * bScale; i++)
    {
      VoteBox[i] = 0;
    }
    //开始投票
    for (int i = 0; i < pointCnt; i++)
    {
      //printf("%d  ",i);
      for (theta = startAngle; theta < endAngle; theta += deltaTheta)
      {

        a = (int)(/*X[i]*/ pPoints[i].x - radiusIn * cos(theta) + 0.5);
        b = (int)(/*Y[i]*/ pPoints[i].y - radiusIn * sin(theta) + 0.5);
        VoteBox[(b - minB) * aScale + a - minA] = VoteBox[(b - minB) * aScale + a - minA] + 1;
      }
    }

    //筛选票箱
    int VoteMax = 0;
    int VoteMaxX, VoteMaxY;
    for (int i = 0; i < bScale; i++)
    {
      for (int j = 0; j < aScale; j++)
      {
        if (VoteBox[i * aScale + j] > VoteMax)
        {
          VoteMax = VoteBox[i * aScale + j];
          VoteMaxY = i;
          VoteMaxX = j;
        }
      }
    }

    int Count = 0;
    printf("VoteMax: %d", VoteMax);
    for (int i = 0; i < bScale; i++)
    {
      for (int j = 0; j < aScale; j++)
      {
        if (VoteBox[i * aScale + j] >= VoteMax)
        {
          Count++;
        }
      }
    }
    printf("   %d \n", Count);
    //释放内存
    delete[] VoteBox;
    if (VoteMax > 3)
    {
      //Arc->center.x = VoteMaxX + minA;
      //Arc->center.y = VoteMaxY + minB;
      //Arc->r = r;
      fCenterX = VoteMaxX + minA;
      fCenterY = VoteMaxY + minB;
      fRaiuds = radiusIn;
      return 1;
    }
    else
    {
      return 0;
    }
    return 1;
  }
};
