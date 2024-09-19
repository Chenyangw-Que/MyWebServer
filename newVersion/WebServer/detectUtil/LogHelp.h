#pragma once
#include <fstream>
#include <iostream>
/// <summary>
/// 判断日志文件是否存在和大小
/// </summary>
/// <param name="strFilePath"></param>
void isReadWriteFile(std::string strFilePath);

/// <summary>
/// 写日志
/// </summary>
/// <param name="strLogPath"></param>
/// <param name="strMessage"></param>
void writeLog(std::string strLogPath, std::string strMessage);

/// <summary>
/// spdlog日志创建
/// </summary>
/// <param name="strFilePath"></param>
int creatLog(std::string strFilePath);