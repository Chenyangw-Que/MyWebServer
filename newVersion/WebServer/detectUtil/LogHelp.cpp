#include "LogHelp.h"
#include "Common.h"

using namespace std;

	/// <summary>
	/// 判断日志文件是否存在和大小
	/// </summary>
	/// <param name="strFilePath"></param>
	void isReadWriteFile(std::string strFilePath)
	{
		ifstream in(strFilePath);
		if (in)
		{
			in.seekg(0, ios::end); //设置文件指针到文件流的尾部
			streampos ps = in.tellg(); //读取文件指针的位置	
			double dFileSize = (double)ps / 1000;
			if (dFileSize > 50000)
			{
				in.close(); //关闭文件流
				ofstream file_writer(strFilePath, ios_base::out);
				file_writer.close();
			}
			else
			{
				in.close(); //关闭文件流
			}
		}
	}

	/// <summary>
	/// 写日志
	/// </summary>
	/// <param name="strLogPath"></param>
	/// <param name="strMessage"></param>
	void writeLog(std::string strLogPath, std::string strMessage)
	{
		ofstream out(strLogPath, ios::app);
		out << getNowTime() << strMessage << endl;
		out.close();
	}
