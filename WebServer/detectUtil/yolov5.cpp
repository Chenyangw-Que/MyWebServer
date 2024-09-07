#include "opencv2/opencv.hpp"
#include "yolov5.h"
#include "Common.h"
#include "LogHelp.h"
#include "openvino/openvino.hpp"
using namespace std;
using namespace cv;
using namespace ov;

CYoloV5::CYoloV5(std::string strModelPath){
    m_strModel = strModelPath;
    m_strLog = "./detectLog.txt";
    compiled_model_ = core_.compile_model(m_strModel, "CPU");
    //推理初始化需求
    infer_request_ = compiled_model_.create_infer_request();
    writeLog(m_strLog, "模型加载成功");
}

CYoloV5::CYoloV5(std::string strModelPath, std::string strLog, const DetectParameterV3 temParament)
{
    m_strModel = strModelPath;
    m_strLog = strLog;
    m_nMarkType = temParament.MarkType;
    memcpy(m_dDiameter, temParament.Diameter, sizeof(temParament.Diameter));
    memcpy(m_dWidth, temParament.Width, sizeof(temParament.Width));
    memcpy(m_dHeight, temParament.Height, sizeof(temParament.Height));
    m_nWHCount = temParament.WHCount;
    m_nDiameterCount = temParament.DiameterCount;
    m_nRows = temParament.Rows;
    m_nCols = temParament.Cols;
    m_dRowspan = temParament.Rowspan;
    m_dColspan = temParament.Colspan;
    m_nCenterDistance = temParament.nCenterDistance;
    getMarkRange();
}

CYoloV5::CYoloV5(std::string strModelPath, std::string strLog)
{
    m_strModel = strModelPath;
    m_strLog = strLog;
    m_strLog = "./detectLog.txt";
    compiled_model_ = core_.compile_model(m_strModel, "CPU");
    //推理初始化需求
    infer_request_ = compiled_model_.create_infer_request();
    writeLog(m_strLog, "模型加载成功");
}

CYoloV5::~CYoloV5()
{
}

cv::Mat CYoloV5::getResultImage(){return curResultPic_;}

/// <summary>
/// 靶标区域粗定位并返回类别信息
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CYoloV5::getTargetClassRoi(const cv::Mat mSrcImage, std::vector<std::pair<cv::Mat, cv::Point3d>> &mROiImage)
{
    std::string strTxtPath = m_strLog;
    isReadWriteFile(strTxtPath);
    std::vector<TargetDetect> vecRoiBoxes;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    this->image_col = mSrcImage.cols;
    this->image_row = mSrcImage.rows;
    writeLog(strTxtPath, "调用yolov5");
    //目标检测
    targetDetect(mDesImage, vecRoiBoxes);
    //定位结果校验
    writeLog(strTxtPath, "开始过滤检测框");
    checkBoxResult(vecRoiBoxes);
    if (vecRoiBoxes.size() != 0)
    {
        for (int i = 0; i < vecRoiBoxes.size(); ++i)
        {
            cv::Mat mMatchImage = mDesImage(cv::Rect(vecRoiBoxes[i].box.dX, vecRoiBoxes[i].box.dY, vecRoiBoxes[i].box.dWidth, vecRoiBoxes[i].box.dHeight));
            cv::Point3i pLocation{(int)vecRoiBoxes[i].box.dX, (int)vecRoiBoxes[i].box.dY, vecRoiBoxes[i].class_id};
            mROiImage.push_back(std::make_pair(mMatchImage, pLocation));
        }
    }
    if (mROiImage.size() != 0)
        return true;
    else
        return false;
}
/// <summary>
/// 靶标区域粗定位
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="mDesImage"></param>
/// <returns></returns>
bool CYoloV5::getTargetRoi(const cv::Mat mSrcImage, std::vector<std::pair<cv::Mat, cv::Point>> &mROiImage)
{
    std::vector<TargetDetect> vecRoiBoxes;
    cv::Mat mDesImage;
    mSrcImage.copyTo(mDesImage);
    //目标检测
    targetDetect(mDesImage, vecRoiBoxes);
    //定位结果校验
    scanBoxResult(vecRoiBoxes);
    if (vecRoiBoxes.size() != 0)
    {
        for (int i = 0; i < vecRoiBoxes.size(); ++i)
        {
            cv::Mat mMatchImage = mDesImage(cv::Rect(vecRoiBoxes[i].box.dX, vecRoiBoxes[i].box.dY, vecRoiBoxes[i].box.dWidth, vecRoiBoxes[i].box.dHeight));
            cv::Point pLocation{(int)vecRoiBoxes[i].box.dX, (int)vecRoiBoxes[i].box.dY};
            mROiImage.push_back(std::make_pair(mMatchImage, pLocation));
        }
    }
    if (mROiImage.size() != 0)
        return true;
    else
        return false;
}

/// <summary>
/// 目标检测
/// </summary>
/// <param name="mSrcImage"></param>
/// <param name="strModelPath"></param>
/// <param name="vecBoxes"></param>
/// <returns></returns>
void CYoloV5::targetDetect(cv::Mat mSrcImage, std::vector<TargetDetect> &vecBoxes)
{
    std::string strTxtPath = "auto_detection.txt";
    isReadWriteFile(strTxtPath);
    writeLog(strTxtPath, "开始加载模型");

    std::vector<float> paddings(3);
    //图像8UC1->8UC3
    cv::Mat mDesImage = generateColorImage(mSrcImage);

    cv::Mat resized_img = letterbox(mDesImage, paddings);
    // BGR->RGB, u8(0-255)->f32(0.0-1.0), HWC->NCHW
    cv::Mat blob = cv::dnn::blobFromImage(resized_img, 1 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true);
    auto input_port = compiled_model_.input();
    //创造张量
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
    infer_request_.set_input_tensor(input_tensor);
    //开始推理
    infer_request_.infer();
    //获取推理结果
    auto output = infer_request_.get_output_tensor(0);
    auto output_shape = output.get_shape();
    // 25200 x 85 Matrix
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());
    float conf_threshold = 0.25;
    float nms_threshold = 0.5;
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> class_scores;
    std::vector<float> confidences;
    // cx,cy,w,h,confidence,c1,c2,...c80
    for (int i = 0; i < output_buffer.rows; i++)
    {
        float confidence = output_buffer.at<float>(i, 4);
        if (confidence < conf_threshold)
        {
            continue;
        }
        cv::Mat classes_scores = output_buffer.row(i).colRange(5, 15);
        cv::Point class_id;
        double score;
        cv::minMaxLoc(classes_scores, NULL, &score, NULL, &class_id);

        // class score: 0~1
        if (score > 0.25)
        {
            float cx = output_buffer.at<float>(i, 0);
            float cy = output_buffer.at<float>(i, 1);
            float w = output_buffer.at<float>(i, 2);
            float h = output_buffer.at<float>(i, 3);
            int left = static_cast<int>((cx - 0.5 * w - paddings[2]) / paddings[0]);
            int top = static_cast<int>((cy - 0.5 * h - paddings[1]) / paddings[0]);
            int width = static_cast<int>(w / paddings[0]);
            int height = static_cast<int>(h / paddings[0]);
            cv::Rect box;
            box.x = left;
            box.y = top;
            box.width = width;
            box.height = height;
            boxes.push_back(box);
            class_ids.push_back(class_id.x);
            class_scores.push_back(score);
            confidences.push_back(confidence);
        }
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);


    curResultPic_ = generateColorImage(mSrcImage);
    for (size_t i = 0; i < indices.size(); i++)
    {
        TargetDetect detTarget;
        int index = indices[i];
        int class_id = class_ids[index];
        int confidence = confidences[index];
        cv::rectangle(curResultPic_, boxes[index], m_colors[class_id % 10], 2, 8);
        //std::string label = m_class_names[class_id] + ":" + std::to_string(class_scores[index]);
        std::string label = m_class_names[class_id] + ":" + std::to_string(confidences[index]);
        cv::putText(curResultPic_, label, cv::Point(boxes[index].tl().x, boxes[index].tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, 1.5, m_colors[class_id % 10], 2);
        //CvWaitShowImage(mShowImage, "yolo");
        detTarget.class_id = class_id;
        detTarget.confidence = confidences[index];
        detTarget.box.dX = boxes[index].x;
        detTarget.box.dY = boxes[index].y;
        detTarget.box.dWidth = boxes[index].width;
        detTarget.box.dHeight = boxes[index].height;
        vecBoxes.push_back(detTarget);
    }
}

CYoloV5 &CYoloV5::operator=(const CYoloV5 &)
{
    // TODO: 在此处插入 return 语句
    return *this;
}

/// <summary>
/// 图像尺度变化
/// </summary>
/// <param name="img"></param>
/// <param name="paddings"></param>
/// <param name="new_shape"></param>
/// <returns></returns>
cv::Mat CYoloV5::letterbox(cv::Mat &img, std::vector<float> &paddings, std::vector<int> new_shape)
{
    int img_h = img.rows;
    int img_w = img.cols;

    // Compute scale ratio(new / old) and target resized shape
    float scale = std::min(new_shape[1] * 1.0 / img_h, new_shape[0] * 1.0 / img_w);
    int resize_h = int(round(img_h * scale));
    int resize_w = int(round(img_w * scale));
    paddings[0] = scale;

    // Compute padding
    int pad_h = new_shape[1] - resize_h;
    int pad_w = new_shape[0] - resize_w;

    // Resize and pad image while meeting stride-multiple constraints
    cv::Mat resized_img;
    cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

    // divide padding into 2 sides
    float half_h = pad_h * 1.0 / 2;
    float half_w = pad_w * 1.0 / 2;
    paddings[1] = half_h;
    paddings[2] = half_w;

    // Compute padding boarder
    int top = int(round(half_h - 0.1));
    int bottom = int(round(half_h + 0.1));
    int left = int(round(half_w - 0.1));
    int right = int(round(half_w + 0.1));

    cv::copyMakeBorder(resized_img, resized_img, top, bottom, left, right, 0, cv::Scalar(100, 100, 100));
    return resized_img;
}

/// <summary>
/// 根据入参获取靶标规格范围
/// </summary>
void CYoloV5::getMarkRange()
{
    if (m_nMarkType == MT_MATRIX || m_nMarkType == MT_COMPLEX_MATRIX) //孔矩阵
        m_nMinRange = min((m_nRows - 1) * m_dRowspan + m_dDiameter[0], (m_nCols - 1) * m_dColspan + m_dDiameter[0]);
    else if (m_nMarkType == MT_RECTANGLE || m_nMarkType == MT_CROSS) //矩形或十字
        m_nMinRange = min(m_dWidth[0], m_dHeight[0]);
    else if (m_nMarkType == MT_PLUM) //梅花孔
        m_nMinRange = m_dDiameter[0];
    else if (m_nMarkType == MT_RING || m_nMarkType == MT_CIRCLE_RING) //圆环或复合圆环
        m_nMinRange = getMaxParam(m_dDiameter, PARAM_SIZE);
    else if (m_nMarkType == MT_COMPLEX_PLUM) //复合梅花孔
        m_nMinRange = m_nCenterDistance * 2 + m_dDiameter[1];
    else
        m_nMinRange = m_dDiameter[0];
    m_nMaxRange = m_nMinRange * 1.5;
}

/// <summary>
/// 检测结果校验
/// </summary>
/// <param name="vecBoxes"></param>
void CYoloV5::scanBoxResult(std::vector<TargetDetect> &vecBoxes)
{
    if (vecBoxes.size() != 0)
    {
        std::vector<TargetDetect> vecInput = vecBoxes;
        vecBoxes.clear();
        for (int i = 0; i < vecInput.size(); ++i)
        {
            if (vecInput[i].class_id == m_nMarkType && vecInput[i].confidence > 0.6 &&
                m_nMinRange < min(vecInput[i].box.dWidth, vecInput[i].box.dHeight) &&
                m_nMaxRange > max(vecInput[i].box.dWidth, vecInput[i].box.dHeight))
                vecBoxes.push_back(vecInput[i]);
        }
    }
}
/// <summary>
/// 基于置信度得分和检测框长宽比阈值进行检测框过滤
/// </summary>
/// <param name = "vecBoxes"></param>
void CYoloV5::checkBoxResult(std::vector<TargetDetect> &vecBoxes)
{
    if (vecBoxes.size() != 0)
    {
        std::vector<TargetDetect> vecInput = vecBoxes;
        vecBoxes.clear();
        for (int i = 0; i < vecInput.size(); ++i)
        {
            float ratio = vecInput[i].box.dWidth > vecInput[i].box.dHeight ? vecInput[i].box.dWidth / vecInput[i].box.dHeight : vecInput[i].box.dHeight / vecInput[i].box.dWidth; //计算一下边长比(长的/短的)
            if (ratio < 1.2 && vecInput[i].confidence > 0.6)
            {
                vecInput[i].box.dX < 0 ? vecInput[i].box.dX = 0 : vecInput[i].box.dX = vecInput[i].box.dX;
                vecInput[i].box.dY < 0 ? vecInput[i].box.dY = 0 : vecInput[i].box.dY = vecInput[i].box.dY;
                vecInput[i].box.dX + vecInput[i].box.dWidth > this->image_col ? vecInput[i].box.dWidth = this->image_col - vecInput[i].box.dX : vecInput[i].box.dWidth = vecInput[i].box.dWidth;
                vecInput[i].box.dY + vecInput[i].box.dHeight > this->image_row ? vecInput[i].box.dHeight = this->image_row - vecInput[i].box.dY : vecInput[i].box.dHeight = vecInput[i].box.dHeight;
                vecBoxes.push_back(vecInput[i]);

                //将检测框外扩几个像素，防止过于贴合影响计算结果
                //int enlargePixel = 50;
                //vecInput[i].box.dX = vecInput[i].box.dX - enlargePixel < 0 ? 0 : vecInput[i].box.dX - enlargePixel;
                //vecInput[i].box.dY = vecInput[i].box.dY - enlargePixel < 0 ? 0 : vecInput[i].box.dY - enlargePixel;

                //vecInput[i].box.dWidth = vecInput[i].box.dX + vecInput[i].box.dWidth + 2 * enlargePixel >= this->image_col ? this->image_col - vecInput[i].box.dX - enlargePixel - 1: vecInput[i].box.dWidth + 2 * enlargePixel;
                //vecInput[i].box.dHeight = vecInput[i].box.dY + vecInput[i].box.dHeight + 2 * enlargePixel >= this->image_row ? this->image_row - vecInput[i].box.dY - enlargePixel - 1 : vecInput[i].box.dHeight + 2 * enlargePixel;
                //vecBoxes.push_back(vecInput[i]);
            }
        }
    }
}
