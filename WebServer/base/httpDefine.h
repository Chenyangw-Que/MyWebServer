// 对http解析类型的定义，以及对解析状态机的定义
#pragma once
namespace httpDefine {
enum ProcessState {
  STATE_PARSE_LINE,
  STATE_PARSE_HEADERS,
  STATE_RECV_BODY,
  STATE_RESPONSE,
  STATE_FINISH
};

enum LineState { PARSE_LINE_SUCCESS, PARSE_LINE_AGAIN, PARSE_LINE_ERROR };

enum HeaderState {
  PARSE_HEADER_SUCCESS,
  PARSE_HEADER_AGAIN,
  PARSE_HEADER_ERROR
};

enum ResponseState { RESPONSE_SUCCESS, RESPONSE_ERROR };

//解析状态
enum ParseState {
  START,
  KEY,
  COLON,
  SPACES_AFTER_COLON,
  VALUE,
  CR,
  LF,
  END_CR,
  END_LF
};
//服务器和客户端的连接状态
enum ConnectionState { CONNECTED, DISCONNECTING, DISCONNECTED };

// Http请求方法
enum RequestMethod { METHOD_GET, METHOD_POST, METHOD_HEAD };

// Http版本
enum HttpVersion { HTTP_1_0, HTTP_1_1 };
} // namespace httpDefine
