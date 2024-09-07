#include "cgiHandler.h"
#include "base/httpDataUtil.h"
#include <jsoncpp/json/json.h>

std::string processImage(std::string readBuffer)
{
    readBuffer = "{" + readBuffer;
    Json::Reader reader;
    Json::Value val;
    reader.parse(readBuffer, val);
    // 读取或生成图像
    std::string base64Image0 = val["content"].asString();
    cv::Mat img = Base2Mat(base64Image0);
    if (img.empty())
    {
        std::cerr << "Could not open or find the image!" << std::endl;
        return "";
    }
    DetectResultV3 result;
    std::vector<cv::Mat> resultBox;
    AutoDetMark(img, &result,resultBox);
    std::cout<<"resbox"<<resultBox.size()<<std::endl;
    std::string base64Image = Mat2Base64(resultBox[0], ".jpg");
    std::stringstream ss;
    ss << "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " << base64Image.size() << "\r\n\r\n"
       << base64Image;
    std::string format_str = ss.str();
    return format_str;
}