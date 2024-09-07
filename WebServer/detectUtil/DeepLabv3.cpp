#include "DeepLabv3.h"
#include "Common.h"
#include "LogHelp.h"
#include "openvino/openvino.hpp"
using namespace std;
using namespace ov;
using namespace cv;


int argmax_temp(vector<float> temp) {
  return max_element(temp.begin(), temp.end()) - temp.begin();
}

cv::Mat img_pad(cv::Mat image) {
  short int size = image.cols > image.rows ? image.cols : image.rows;
  // 生成灰色背景
  Mat grayboard = Mat(size, size, CV_8UC3, Scalar(128, 128, 128));
  //设置画布绘制区域并复制
  //(size - image.cols) / 2, (size - image.rows) / 2
  cv::Rect roi_rect = cv::Rect((size - image.cols) / 2, (size - image.rows) / 2,
                               image.cols, image.rows);
  image.copyTo(grayboard(roi_rect));
  return grayboard;
}

/*cv::Mat img_pad(cv::Mat image)
{
        int h = image.rows;
        int w = image.cols;
        int fixed_size = 512;
        if (h >= w) {
                float factor = h / float(fixed_size);
                int new_w = int(w / factor);
                if (new_w % 2 != 0) new_w -= 1;
                int pad_w = int((fixed_size - new_w) / 2);
                resize(image, image, { new_w, fixed_size });
                cv::copyMakeBorder(image, image, pad_w, fixed_size - new_w -
pad_w, BORDER_CONSTANT, 0, 0, Scalar(0, 0, 0));
        }
        else {
                float factor = w / float(fixed_size);
                int new_h = int(h / factor);
                if (new_h % 2 != 0) new_h -= 1;
                int pad_h = int((fixed_size - new_h) / 2);
                resize(image, image, { new_h, fixed_size });
                cv::copyMakeBorder(image, image, 0, 0, pad_h, fixed_size - new_h
- pad_h, BORDER_CONSTANT, Scalar(0, 0, 0));
        }
        return image;
}*/

CDeeplabV3::CDeeplabV3(std::string strModelPath, std::string strLog) {
  m_strModel = strModelPath;
  m_strLog = strLog;
  modelInit();
}

CDeeplabV3::~CDeeplabV3() {}

//图像分割
bool CDeeplabV3::getSegmentImage(const cv::Mat mSrcImage, cv::Mat &mDesImage) {
  preprocess(mSrcImage);
  predict();
  postprocess(mDesImage);
  if (!mDesImage.data)
    return false;
  else
    return true;
}

bool CDeeplabV3::getSegmentMask(const cv::Mat mSrcImage, cv::Mat &mDesImage,
                                short classId) {
  preprocessPadding(mSrcImage);
  predict();
  postprocessPadding(mDesImage, classId);
#ifdef _DEBUG
  cv::imshow("segmentation", mDesImage);
  cv::waitKey(0);
#endif // !_DEBUG

  if (!mDesImage.data)
    return false;
  else
    return true;
  return false;
}

//模型参数初始化
void CDeeplabV3::modelInit() {
  try {
    clock_t start, end;
    start = clock();
    ov::Core ie;
    ov::CompiledModel model = ie.compile_model(m_strModel, "AUTO"); // 加载模型
    this->infer_request = model.create_infer_request(); // 创建推理请求
    this->input_tensor =
        infer_request.get_input_tensor(); // 为输入张量开辟一个区域存放数据
    this->input_shape = this->input_tensor.get_shape(); // 获得输入张量的shape
    end = clock();
    std::cout << "model_init = " << double(end - start) / CLOCKS_PER_SEC << "s"
              << std::endl;
  } catch (const ov::Exception &e) {
    std::cout << "Inference error : " << e.what() << std::endl;
    exit(0);
  }
}

void CDeeplabV3::preprocess(cv::Mat mSrcImage) {
  Mat image = mSrcImage;
  this->image_h = image.rows;
  this->image_w = image.cols;
  // cout << c << endl;
  int num_channels = this->input_shape[1];
  int w = this->input_shape[2];
  int h = this->input_shape[3];
  // vector<cv::Mat> mvbegin;
  // cv::split(image, mvbegin);
  // vector<cv::Mat> output;
  // output.push_back(mvbegin[2]); // r
  // output.push_back(mvbegin[1]); // g
  // output.push_back(mvbegin[0]); // b
  // Mat out_result;
  // cv::merge(output, out_result);
  // Mat new_img = img_pad(out_result); // 将图片变为512x512的，并且填充成正方形
  /*imshow("new", new_img);
  waitKey(0);*/

  size_t image_size = w * h;
  cvtColor(image, image, COLOR_BGR2RGB); // 将BGR格式转为RGB
  resize(image, image, {h, w});          // 将图片resize成512x512

  image.convertTo(image, CV_32F, 1.0 / 255); // 归一化

  this->input_values = this->input_tensor.data<float>();
  // 注意遍历顺序
  for (size_t row = 0; row < h; row++) {
    for (size_t col = 0; col < w; col++) {
      for (size_t k = 0; k < num_channels; k++) {
        this->input_values[image_size * k + row * w + col] =
            image.at<Vec3f>(row, col)[k];
      }
    }
  }
}

void CDeeplabV3::predict() {
  clock_t start, end;
  start = clock();
  this->infer_request.infer(); // 执行推理
  end = clock();
  std::cout << "infer time = " << double(end - start) / CLOCKS_PER_SEC << "s"
            << std::endl;
  auto output =
      infer_request
          .get_output_tensor(); // 获取推理的输出结果，get_output_tensor()用于只有一个输出的情况，多个输出使用get_tensor()
  this->output_values = output.data<float>();
}

void CDeeplabV3::postprocess(cv::Mat &mDesImage) {
  int output_size = 512 * 512 * 11;
  vector<float> results(512 * 512);
  int n = 0, m = 0;
  vector<float> temp(11);
  for (unsigned i = 0; i < output_size; i++) {
    // 得到的结果是按通道排的，每11个数为一组
    if (i % 11 == 0) {
      n = 0;
      int idx = argmax_temp(temp);
      // cout << "id: " << idx << endl;
      results[m++] =
          (idx > 0)
              ? 255
              : 0; // 得分最大的下标如果不是0，说明不是背景，则将像素值设为255可视化
                   // cout << endl;
      /*if (argmax_temp(temp) > 0)
              cout << argmax_temp(temp) << endl;*/
    }
    temp[n++] = this->output_values[i];
    // cout << this->output_values[i] << " ";
  }
  Mat output_tensor_mat(results);
  output_tensor_mat = output_tensor_mat.reshape(1, {512, 512});
  resize(output_tensor_mat, output_tensor_mat, {this->image_w, this->image_h});
  mDesImage = output_tensor_mat;
#ifdef _SHOW_PROCESS_TARGET_DET
  CvWaitShowImage(mDesImage, "deeplab");
#endif
}
//先填充再resize的预处理
void CDeeplabV3::preprocessPadding(const cv::Mat mSrcImage) {
  clock_t start, end;
  start = clock();
  Mat image = mSrcImage;
  this->image_h = image.rows;
  this->image_w = image.cols;

  int num_channels = this->input_shape[1];
  int w = this->input_shape[2];
  int h = this->input_shape[3];

  size_t image_size = w * h;
  cvtColor(image, image, COLOR_GRAY2RGB); // 将BGR格式转为RGB
  image = img_pad(image);

  cvtColor(image, image, COLOR_BGR2RGB);     // 将BGR格式转为RGB
  resize(image, image, {h, w});              // 将图片resize成512x512
  image.convertTo(image, CV_32F, 1.0 / 255); // 归一化

  this->input_values = this->input_tensor.data<float>();
  // 注意遍历顺序
  for (size_t row = 0; row < h; row++) {
    for (size_t col = 0; col < w; col++) {
      for (size_t k = 0; k < num_channels; k++) {
        this->input_values[image_size * k + row * w + col] =
            image.at<Vec3f>(row, col)[k];
      }
    }
  }
  end = clock();
  std::cout << "preprocess time = " << double(end - start) / CLOCKS_PER_SEC
            << "s" << std::endl;
}
//填充预处理配套的后处理，并添加类别限定
void CDeeplabV3::postprocessPadding(cv::Mat &mDesImage,
                                    unsigned short int classnum) {
  clock_t start, end;
  start = clock();
  int output_size = this->model_size * this->model_size * 11;
  vector<float> results(this->model_size * this->model_size);
  int n = 0, m = 0;
  vector<float> temp(11);
  // for (unsigned i = 0; i < output_size; i++)
  for (unsigned i = 1; i <= output_size; i++) {
    // 得到的结果是按通道排的，每11个数为一组
    if (i % 11 == 0) {
      n = 0;
      int idx = argmax_temp(temp);
      // results[m++] = (idx == classnum+1) ? 255 : 0;
      // //得分最高通道编号等于检测输出类别编号+1则设为255
      results[m++] = (idx != 0) ? 255 : 0;
    }
    temp[n++] = this->output_values[i];
  }
  // 获取到输出mat，先resize， 再去掉填充
  Mat output_tensor_mat(results);
  short int size =
      this->image_h > this->image_w ? this->image_h : this->image_w;
  output_tensor_mat =
      output_tensor_mat.reshape(1, {this->model_size, this->model_size});
  resize(output_tensor_mat, output_tensor_mat, {size, size}, INTER_NEAREST);
  cv::Rect roi_rect =
      cv::Rect((size - this->image_w) / 2, (size - this->image_h) / 2,
               this->image_w, this->image_h);
  cv::Mat res = output_tensor_mat(roi_rect);
  cv::threshold(res, mDesImage, 0, 255, 0);
  end = clock();
  std::cout << "postprocess time = " << double(end - start) / CLOCKS_PER_SEC
            << "s" << std::endl;
}
