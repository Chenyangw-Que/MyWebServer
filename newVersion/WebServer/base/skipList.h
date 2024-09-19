#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
// #define DEBUG
const static std::string STORE_FILE = "../store/dumpFile";
const static std::string DELIMITER = ":";

static std::mutex mtx;

template <typename K, typename V> class Node {
public:
  Node() {}
  Node(K k, V v, int level);
  ~Node();

  K getKey() const;
  V getValue() const;
  void setValue(V v);

  // 不同层数的下一节点
  Node<K, V> **forward_;
  int level_;

private:
  K key_;
  V value_;
};

template <typename K, typename V>
Node<K, V>::Node(const K k, const V v, int level) {
  key_ = k;
  value_ = v;
  level_ = level;
  forward_ = new Node<K, V> *[level_ + 1];
}

template <typename K, typename V> Node<K, V>::~Node() { delete[] forward_; }

template <typename K, typename V> K Node<K, V>::getKey() const { return key_; }

template <typename K, typename V> V Node<K, V>::getValue() const {
  return value_;
}

template <typename K, typename V> void Node<K, V>::setValue(V v) { value_ = v; }


template <typename K, typename V> class SkipList {
public:
  SkipList();
  SkipList(int maxLevel);
  ~SkipList();

  Node<K, V> *createNode(K, V, int);
  void displayList();
  int insertElement(const K, const V);
  bool searchElement(K, V &);
  bool deleteElement(K);

  bool rangeDelete(K,K); // 进行区间删除

  int size();

  void dumpFile();
  void loadFile();

private:
  void parseKeyValue(const std::string &str, std::string *key,
                                 std::string *value);
  bool is_valid_string(const std::string &str);
  int getRandomLevel();

private:
  // 跳表的最大层数
  int maxLevel_;

  // 当前所在的层数
  int curLevel_;

  // 跳表头节点指针
  Node<K, V> *head_;

  // 当前元素个数
  int elementCount_;

  // 文件操作
  std::ofstream fileWriter_;
  std::ifstream fileReader_;

#ifdef RANDOM
  Random rnd;
#endif
};

// 创建节点
template <typename K, typename V>
Node<K, V> *SkipList<K, V>::createNode(const K k, const V v, int level) {
  Node<K, V> *node = new Node<K, V>(k, v, level);
  return node;
}

template <typename K, typename V> SkipList<K, V>::SkipList() {
  maxLevel_ = 32;
  curLevel_ = 0;
  elementCount_ = 0;

  // 创建头节点，并将K与V初始化为NULL
  K k;
  V v;
  head_ = new Node<K, V>(k, v, maxLevel_);
  loadFile();
}

// 建造跳表
template <typename K, typename V> SkipList<K, V>::SkipList(int maxLevel):maxLevel_(maxLevel),curLevel_(0),elementCount_(0) {
  // 创建头节点，并将K与V初始化为NULL
  K k;
  V v;
  head_ = new Node<K, V>(k, v, maxLevel_);
}

// 析构
template <typename K, typename V> SkipList<K, V>::~SkipList() {
  if (fileReader_.is_open()) {
    fileReader_.close();
  }
  if (fileWriter_.is_open()) {
    fileWriter_.close();
  }

  delete head_;
}

template <typename K, typename V>
int SkipList<K, V>::insertElement(const K key, const V value) {
  mtx.lock();
  Node<K, V> *cur = head_;

  // 创建数组update并初始化该数组
  Node<K, V> **update = new Node<K, V> *[maxLevel_ + 1]();

  // 从跳表的左上角节点开始查找
  for (int i = curLevel_; i >= 0; --i) {
    // cur在该层的下一个节点的 _key 小于要找的 key
    while (cur->forward_[i] != nullptr && cur->forward_[i]->getKey() < key) {
      cur = cur->forward_[i];
    }
    update[i] = cur;
  }

  // 到达最底层（第0层）,并且当前的 forward 指针指向第一个大于待插入节点的节点
  cur = cur->forward_[0];

  // 如果当前节点的 key 值和待插入节点 key
  // 相等，说明节点已经存在，修改节点值即可
  if (cur != nullptr && cur->getKey() == key) {
#ifdef DEBUG
    std::cout << "key: " << key << ", exists. Change it" << std::endl;
#endif
    // FIXME:do we need to change it?
    cur->setValue(value);
    mtx.unlock();
    return 1;
  }

  //如果current节点为null 这就意味着要将该元素插入最后一个节点。
  //如果current的key值和待插入的key不等，代表我们应该在update[0]和current之间插入该节点。
  if (cur == nullptr || cur->getKey() != key) {
    int randomLevel = getRandomLevel();

    if (randomLevel > curLevel_) {
      for (int i = curLevel_ + 1; i <= randomLevel; ++i) {
        update[i] = head_;
      }
      curLevel_ = randomLevel;
    }

    // 使用生成的random level创建新节点
    Node<K, V> *insertNode = createNode(key, value, randomLevel);

    // 插入节点
    for (int i = 0; i <= randomLevel; ++i) {
      insertNode->forward_[i] = update[i]->forward_[i];
      update[i]->forward_[i] = insertNode;
    }
#ifdef DEBUG
    std::cout << "Successfully inserted key:" << key << ", value:" << value
              << std::endl;
#endif
    ++elementCount_;
  }

  mtx.unlock();
  return 0;
}

template <typename K, typename V>
bool SkipList<K, V>::searchElement(K key, V &value) {
#ifdef DEBUG
  std::cout << "search_element-------------" << std::endl;
#endif
  Node<K, V> *cur = head_;

  // 从跳表的最左上角开始查找
  for (int i = curLevel_; i >= 0; --i) {
    while (cur->forward_[i] && cur->forward_[i]->getKey() < key) {
      cur = cur->forward_[i];
    }
  }

  // 最底层的下一个，会出现两种情况
  // 1、就是需要找的 key
  // 2、比找的key 要大
  cur = cur->forward_[0];

  if (cur != nullptr && cur->getKey() == key) {
    value = cur->getValue();
#ifdef DEBUG
    std::cout << "Found key: " << key << ", value: " << cur->getValue()
              << std::endl;
#endif
    return true;
  }
#ifdef DEBUG
  std::cout << "Not Found Key:" << key << std::endl;
#endif

  return false;
}

template <typename K, typename V> bool SkipList<K, V>::deleteElement(K key) {
  mtx.lock();

  Node<K, V> **update = new Node<K, V> *[maxLevel_ + 1]();

  Node<K, V> *cur = head_;

  for (int i = curLevel_; i >= 0; --i) {
    while (cur->forward_[i] != nullptr && cur->forward_[i]->getKey() < key) {
      cur = cur->forward_[i];
    }
    update[i] = cur;
  }
  cur = cur->forward_[0];

  if (cur != nullptr && cur->getKey() == key) {
    for (int i = 0; i <= curLevel_; ++i) {
      if (update[i]->forward_[i] != cur) {
        break;
      }
      update[i]->forward_[i] = cur->forward_[i];
    }
    delete cur;

    while (curLevel_ > 0 && head_->forward_[curLevel_] == nullptr) {
      --curLevel_;
    }
    --elementCount_;

    mtx.unlock();

    return true;
  } else {
    mtx.unlock();
    return false;
  }
}

// 打印整个跳表
template <typename K, typename V> void SkipList<K, V>::displayList() {
#ifdef DEBUG
  std::cout << "display skipList : " << std::endl;
#endif

  Node<K, V> *cur;

  for (int i = curLevel_; i >= 0; --i) {
    cur = head_->forward_[i];
#ifdef DEBUG
    std::cout << "Level : " << i << ':' << std::endl;
#endif
    while (cur != nullptr) {
#ifdef DEBUG
      std::cout << cur->getKey() << ':' << cur->getValue() << ' ';
#endif
      cur = cur->forward_[i];
    }
#ifdef DEBUG
    std::cout << std::endl;
#endif
  }
  return;
}

#ifndef RANDOM
template <typename K, typename V> int SkipList<K, V>::getRandomLevel() {
  int k = 0;
  while (rand() % 2) {
    ++k;
  }
  k = (k < maxLevel_) ? k : maxLevel_;
  return k;
}

#else
int SkipList<K, V>::getRandomLevel() {
  int level = static_cast<int>(rnd.Uniform(maxLevel_));
  while (rand() % 2) {
    ++k;
  }
  k = (k < maxLevel_) ? k : maxLevel_;
  return k;
}
#endif

template <typename K, typename V>
bool SkipList<K, V>::is_valid_string(const std::string &str) {
  if (str.empty() || str.find(DELIMITER) == std::string::npos) {
    return false;
  }
  return true;
}

template <typename K, typename V>
void SkipList<K, V>::parseKeyValue(const std::string &str,
                                               std::string *key,
                                               std::string *value) {
  if (!is_valid_string(str)) {
    return;
  }
  int pos = str.find(DELIMITER);
  *key = str.substr(0, pos);
  *value = str.substr(pos + 1, str.size());
}

// 将数据导出到文件
template <typename K, typename V> void SkipList<K, V>::dumpFile() {
#ifdef DEBUG
  std::cout << "dumpFile ---------------- " << std::endl;
#endif

  if (!fileWriter_.is_open()) {
    fileWriter_.open(STORE_FILE, std::ios::out | std::ios::trunc);
  }

  // fileWriter_.open(STORE_FILE);
  Node<K, V> *cur = head_->forward_[0];

  while (cur != nullptr) {
    fileWriter_ << cur->getKey() << ":" << cur->getValue() << "\n";

#ifdef DEBUG
    std::cout << cur->getKey() << ":" << cur->getValue() << ";\n";
#endif

    cur = cur->forward_[0];
  }

  fileWriter_.flush();
  fileWriter_.close();
  return;
}

// 从磁盘读取数据
template <typename K, typename V> void SkipList<K, V>::loadFile() {
#ifdef DEBUG
  std::cout << "loadFile ---------------- " << std::endl;
#endif

  fileReader_.open(STORE_FILE);
  std::string line;
  // FIXME: why use new?
  std::string *key = new std::string();
  std::string *value = new std::string();
  while (getline(fileReader_, line)) {
    parseKeyValue(line, key, value);
    if (key->empty() || value->empty()) {
      continue;
    }
    insertElement(stoi(*key), *value);
#ifdef DEBUG
    std::cout << "key:" << *key << "value:" << *value << std::endl;
#endif
  }
  fileReader_.close();
}

// 返回跳表的元素个数
template <typename K, typename V> int SkipList<K, V>::size() {
  return elementCount_;
}