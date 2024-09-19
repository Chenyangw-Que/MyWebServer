#include "httpDataUtil.h"
#include "base64.h"

std::string Mat2Base64(const cv::Mat &image, std::string imgType) {
     if(image.empty()){
        return "zao";
     }
     std::vector<uchar> buf;
     cv::imencode(imgType, image, buf);
     //uchar *enc_msg = reinterpret_cast<unsigned char*>(buf.data());
     std::string img_data = base64_encode(buf.data(), buf.size(), false);
     return img_data;
 }

 cv::Mat Base2Mat(std::string &base64_data) {
    std::string data = base64_decode(base64_data);
    std::vector<uint8_t> buffer(data.begin(), data.end());
    cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);  // 使用解码器解码字节流为图像
    return img;
 }