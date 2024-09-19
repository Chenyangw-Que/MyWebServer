#pragma once
#include <iostream>
#define PARAM_SIZE 5
#define MATCH_POINT_SCALAR 32
#define SAFE_DELETE(X) \
  {                    \
    if (X)             \
    {                  \
      delete X;        \
      X = NULL;        \
    }                  \
  }
#define DIFF_PIXEL2 -5
#define DIFF_PIXEL 3
const float PI_32F = 3.1415926f;
enum MarkTypes
{
  MT_CIRCLE = 0,
  MT_COMPLEX_MATRIX = 2,
  MT_CROSS = 3,
  MT_RECTANGLE = 4,
  MT_MATRIX = 5,
  MT_PLUM,
  MT_RING = 7,
  MT_COMPLEX_PLUM,
  MT_CIRCLE_RING,
  MT_CORNER,
  MT_COMPLEX,
  MT_POLYGON,
  MT_PATTERN
};
enum MyEnum
{
  CFMEE_CODE_OK = 0,                  //成功
  CFMEE_CODE_FAILED = 1,              //失败
  CFMEE_CODE_PARAM = 2,               //参数错误
  CFMEE_CODE_IMAGE_ERROR = 3,         //图片错误
  CFMEE_CODE_GETROI_ERROR = 4,        //粗定位失败
  CFMEE_EFFECTIVE_ERROR = 5,          //有效参数过少
  CFMEE_DETECTION_NUMBER_ERROR = 6,   //检测数量设置错误
  CFMEE_ALGORITHM_ERROR = 7,          //算法参数错误
  CFMEE_MARKNUMBER_ERROR = 8,         //靶标数量设置错误
  CFMEE_WB_IMAGE_ERROR = 9,           //全黑或者全白图片
  CFMEE_READXML_ERROR = 10,           // XML文件解析失败
  CFMEE_OBJECT_ERROR = 11,            //目标检测失败
  CFMEE_SEGMENT_ERROR = 12,           //分割失败
  CFMEE_LINEMOD_READMODEL_ERROR = 13, // linemod 模型读取失败
  CFMEE_FITTING_ERROR = 14,           //拟合失败（精计算失败）
  CFMEE_PARAMANALYZE_ERROR = 15,      // 参数自动解析错误
  CFMEE_AUTODET_ERROR = 16,           //自动精定位失败
};
//点结构体
struct Cfmee_Point
{
  double dX;
  double dY;

  Cfmee_Point()
  {
    dX = 0;
    dY = 0;
  }

  Cfmee_Point(double dX, double dY)
  {
    this->dX = dX;
    this->dY = dY;
  }
};

//矩形结构体
struct Cfmee_Rectangle
{
  double dX;
  double dY;
  double dCenterX;
  double dCenterY;
  double dAngle;
  double dWidth;
  double dHeight;
  Cfmee_Point _Point[4]; // 四顶点坐标
  Cfmee_Rectangle()
  {
    dX = 0;
    dY = 0;
    dCenterX = 0;
    dCenterY = 0;
    dAngle = 0;
    dWidth = 0;
    dHeight = 0;
  }
  Cfmee_Rectangle(double dCenterX, double dCenterY, double dAngle,
                  double dWidth, double dHeight)
  {
    this->dCenterX = dCenterX;
    this->dCenterY = dCenterY;
    this->dAngle = dAngle;
    this->dWidth = dWidth;
    this->dHeight = dHeight;
  }
  Cfmee_Rectangle(Cfmee_Point *_Point)
  {
    for (int i = 0; i < 4; ++i)
      this->_Point[i] = _Point[i];
  }
};

struct GrayBitmap
{
  GrayBitmap()
  {
    Width = 0;
    Height = 0;
    Stride = 0;
    Channels = 1;
    BitmapData = nullptr;
  }

  int Width;
  int Height;
  int Stride;
  int Channels;
  unsigned char *BitmapData;
};
struct DetectParameterV3
{
  int MarkType;                //靶标类型
  int IsWhite;                 //靶标颜色(1:白色；0：黑色)
  double Width[PARAM_SIZE];    //靶标宽度
  double Height[PARAM_SIZE];   //靶标高度
  double Diameter[PARAM_SIZE]; //靶标直径
  int WHCount;                 //宽高数量
  int DiameterCount;           //直径数量
  int Rows;                    //列数
  int Cols;                    //行数
  double Rowspan;              //列间距
  double Colspan;              //行间距
  double Threshold;            //直径1误差范围
  double dCenterWeight;        //权重值(范围0-1)
  int nLittleCount;            //小圆个数
  int nCenterDistance;         //中心点距离
  char *cTemplateData;         //模板参数
  double ReserveData[10];      //补充参数

  DetectParameterV3()
  {
    MarkType = MT_CIRCLE;
    IsWhite = 0;
    for (int i = 0; i < PARAM_SIZE; ++i)
    {
      Width[i] = 0;
      Height[i] = 0;
      Diameter[i] = 0;
    }
    WHCount = 0;
    DiameterCount = 0;
    Rows = 1;
    Cols = 1;
    Rowspan = 0;
    Colspan = 0;
    Threshold = 0;
    dCenterWeight = 0.5;
    nLittleCount = 12;
    nCenterDistance = 280;
    cTemplateData = nullptr;
  }
};

//直线结构体
struct Cfmee_Line
{
  Cfmee_Point _Point[2]; // 四顶点坐标
};

//十字结构体
struct Cfmee_Cross
{
  double dWidth;
  double dHeight;
  double dBorderWidth;
  double dAngle;
  Cfmee_Point _Point[8]; // 四顶点坐标
};

//圆结构体
struct Cfmee_Circle
{
  double dCenterX;
  double dCenterY;
  double dRedius;
  double dSocre;

  Cfmee_Circle()
  {
    dCenterX = 0;
    dCenterY = 0;
    dRedius = 0;
    dSocre = 0;
  }

  Cfmee_Circle(double dX, double dY, double dRedius, double dSocre)
  {
    this->dCenterX = dX;
    this->dCenterY = dY;
    this->dRedius = dRedius;
    this->dSocre = dSocre;
  }
};
struct DetectROI
{
  Cfmee_Circle circleRoi[MATCH_POINT_SCALAR]; //圆模板参数
  int nCircleCount;
  Cfmee_Rectangle rectRoi[MATCH_POINT_SCALAR]; //矩形模板参数
  int nRectangleCount;
  Cfmee_Cross crossRoi[MATCH_POINT_SCALAR]; //十字模板参数
  int nCrossCount;
  Cfmee_Line lineRoi[MATCH_POINT_SCALAR]; //直线模板参数
  int nLineCount;
  std::pair<Cfmee_Line, Cfmee_Line> pairLineRoi[MATCH_POINT_SCALAR]; //直线对模板参数
  int nPairLineCount;
  DetectROI()
  {
    nCircleCount = 0;
    nRectangleCount = 0;
    nCrossCount = 0;
    nLineCount = 0;
    nPairLineCount = 0;
  }
};

//目标检测结构体
struct TargetDetect
{
  int class_id;
  float confidence;
  Cfmee_Rectangle box;
  TargetDetect()
  {
    class_id = -1;
    confidence = 0;
  }
};

//*****************************检测结果返回结构体（详细）*****************************
//复合靶标组成模块结构体
struct DetectPart
{
  int MarkType;                            //复合靶标组成模块类型
  double Width;                            //宽
  double Height;                           //高
  double CenterX;                          //中心点横坐标
  double CenterY;                          //中心点纵坐标
  double Diameter;                         //直径
  double DetScore;                         //靶标得分
  int ReserveCount;                        //备用扩展模块个数
  double ReserveModel[MATCH_POINT_SCALAR]; //备用扩展区域
  DetectPart()
  {
    MarkType = 0;
    Width = 0;
    Height = 0;
    CenterX = 0;
    CenterY = 0;
    Diameter = 0;
    DetScore = 0;
    ReserveCount = 0;
  }
};

//单目标检测结构体
struct SingleResult
{
  int MarkType;                              //距离视场中心最近靶标靶标类型
  double Width;                              //宽
  double Height;                             //高
  double CenterX;                            //靶标中心横坐标
  double CenterY;                            //靶标中心纵坐标
  double Diameter;                           //靶标直径
  double DetScore;                           //靶标得分
  int PartCount;                             //靶标组成模块个数
  int ReserveCount;                          //备用扩展模块个数
  DetectPart PartResult[MATCH_POINT_SCALAR]; //靶标组成模块检测结果
  double ReserveModel[MATCH_POINT_SCALAR];   //备用扩展区域
  SingleResult()
  {
    Width = 0;
    Height = 0;
    CenterX = 0;
    CenterY = 0;
    Diameter = 0;
    DetScore = 0;
    PartCount = 0;
    ReserveCount = 0;
  }
};
//检测结果结构体
struct DetectResultV3
{
  int MarkType;                    //距离视场中心最近靶标靶标类型
  double Width;                    //距离视场中心最近靶标靶标宽
  double Height;                   //距离视场中心最近靶标靶标高
  double CenterX;                  //距离视场中心最近靶标中心点横坐标
  double CenterY;                  //距离视场中心最近靶标中心点纵坐标
  double Diameter;                 //距离视场中心最近靶标直径
  double DetScore;                 //距离视场中心最近靶标得分
  int CircleCount;                 //检测成功靶标数量
  SingleResult DetEveryResult[32]; //单个检测成功靶标存储结构体
  double excuteTime;               //检测时间

  DetectResultV3()
  {
    MarkType = 1;
    Width = 0;
    Height = 0;
    CenterX = 0;
    CenterY = 0;
    Diameter = 0;
    DetScore = 0;
    CircleCount = 0;
  }
};
//边缘像素点参数
struct ContoursPoint
{
  double dX;         //边缘点横坐标
  double dY;         //边缘点纵坐标
  double dMagnitude; //边缘点梯度幅值
  double dAngle;     //边缘点梯度方向
  double dWeight;    //边缘点权重值
  double nPixel;     //像素值
  ContoursPoint()
  {
    dX = 0;
    dY = 0;
    dMagnitude = 0;
    dAngle = 0;
    dWeight = 1;
    nPixel = 0;
  }
  ContoursPoint(double x = 0, double y = 0, double dMagnitude = 0,
                double angle = 0, double weight = 0, double pixel = 0)
  {
    this->dX = x;
    this->dY = y;
    this->dMagnitude = dMagnitude;
    this->dAngle = angle;
    this->dWeight = weight;
    this->nPixel = pixel;
  }
};

//算法配置文件
struct ConfigAlgorithmParam
{
  int nMarkAlgorithm;       //检测算法
  int nMatchType;           //粗定位算法；0：自绘模板匹配，1：截图模板匹配，2：目标检测
  char *MarkData;           //模板路径
  int nTemMatchType;        //模板匹配方式；0:颜色匹配，1：形状匹配
  int nDetMarkNumber;       //检测靶标数量
  int nSaveImage;           //是否自动保存图片；0：不保存，1：保存
  double dLittleCircleRate; //小圆抓取成功比率
  int nWhiteEdge;           //靶标边缘颜色	；0：黑色，1：白色
  int nInnerRing;           //检测内环（圆环）；0：内外环均检测，1：内环检测，2：外环检测
  int IsPolarChange;        //极性变化
  int IsMatchPrior;         //通过匹配得分确定匹配返回值
  int nDeepCalculation;     //精确计算；0：不启用，1：启用
  double dSocreThreshold;   //圆检测得分阈值
  int nEllipseDetection;    //椭圆检测
  int IsEnhanceMap;         //使用图像增强算法
  int nHalfAngleRoate;      //梅花孔半角旋转
  int IsClosestDistance;    //最近距离检测
  int IsLinePureCheck;      //直线纯度检测
  int IsInnerToExtern;      //由内到外筛选

  ConfigAlgorithmParam()
  {
    nMarkAlgorithm = 0;
    nMatchType = 0;
    MarkData = nullptr;
    nTemMatchType = 1;
    nDetMarkNumber = 1;
    nSaveImage = 0;
    dLittleCircleRate = 0.3;
    nWhiteEdge = 0;
    nInnerRing = 1;
    IsPolarChange = 0;
    IsMatchPrior = 0;
    nDeepCalculation = 1;
    dSocreThreshold = 0.15;
    nEllipseDetection = 0;
    IsEnhanceMap = 0;
    nHalfAngleRoate = 0;
    IsClosestDistance = 0;
    IsLinePureCheck = 0;
    IsInnerToExtern = 0;
  }
};

struct IntPoint
{
  int x;
  int y;

  IntPoint()
  {
    x = 0;
    y = 0;
  }

  IntPoint(int x, int y)
  {
    this->x = x;
    this->y = y;
  }
};