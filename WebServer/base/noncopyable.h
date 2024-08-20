#pragma once
class noncopyable {
public:
  // 禁用拷贝构造和赋值
  noncopyable(const noncopyable &) = delete;
  noncopyable &operator=(const noncopyable &) = delete;

  noncopyable(noncopyable &&) = default; // 运行移动构造，比如要返回一个返回值
  noncopyable &operator=(noncopyable &&) = default; // 移动赋值比如move

  // 仅允许被继承
protected:
  noncopyable() = default;
  ~noncopyable() = default;
};