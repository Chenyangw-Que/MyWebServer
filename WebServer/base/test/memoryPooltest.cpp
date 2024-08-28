// #include "../MemoryPool.h"
// #include <memory>
// using namespace std;
// class Person {
// public:
//   int id_;
//   std::string name_;

// public:
//   Person(int id, std::string name) : id_(id), name_(name) {
//     printf("构造函数调⽤\n");
//   }
//   ~Person() { printf("析构函数调⽤\n"); }
// };

// void test01() {
//   printf("creating a person\n");
//   shared_ptr<Person> p1(QueMemory::newElement<Person>(11, "Lawson"),
//                         QueMemory::deleteElement<Person>);
//   printf("sizeof(name_) = %d\n", sizeof(p1->name_));
//   printf("sizeof(Person) = %d\n", sizeof(Person));
// }
// int main() {
//   QueMemory::init();
//   test01();
//   return 0;
// }

#include <iostream>
#include"../MemoryPool.h"
#include<vector>
using namespace std;
struct TreeNode // 一个树结构的节点，等会申请空间的时候就用这个树节点来申请
{
	int _val;
	TreeNode* _left;
	TreeNode* _right;

	TreeNode()
		:_val(0)
		, _left(nullptr)
		, _right(nullptr)
	{}
};

void TestObjectPool() // malloc和当前定长内存池性能对比
{
	// 申请释放的轮次
	const size_t Rounds = 100;

	// 每轮申请释放多少次
	const size_t N = 30000;
	
	// 这里总共申请和释放的次数就是Rounds * N次，测试这么些次谁更快

	std::vector<TreeNode*> v1;
	v1.reserve(N);
	
	// 测试malloc的性能
	size_t begin1;
    size_t end1;
    begin1 = clock();
	for (size_t j = 0; j < Rounds; ++j)
	{
        
		for (int i = 0; i < N; ++i)
		{
			v1.push_back(new TreeNode); // 这里虽然用的是new，但是new底层用的也是malloc
		}
        
        
		for (int i = 0; i < N; ++i)
		{
			delete v1[i]; // 同样的，delete底层也是free
		}
		v1.clear(); // 这里clear作用就是将vector中的内容清空，size置零，
		// 但capacity保持不变，这样才能循环上去重新push_back
        
	}
	end1 = clock();
	

	std::vector<TreeNode*> v2;
	v2.reserve(N);
	
	// 定长内存池，其中申请和释放的T类型就是树节点
	size_t begin2;
    size_t end2;
    begin2 = clock();
	for (size_t j = 0; j < Rounds; ++j)
	{
        
		for (int i = 0; i < N; ++i)
		{
			v2.push_back(QueMemory::newElement<TreeNode>()); // 定长内存池中的申请空间
		}
        
        
		for (int i = 0; i < N; ++i)
		{
			QueMemory::deleteElement(v2[i]);
		}
        
		v2.clear();// 这里clear作用就是将vector中的内容清空，size置零，
		// 但capacity保持不变，这样才能循环上去重新push_back
        
	}
    end2 = clock();


	cout << "new cost time:" << end1 - begin1 << endl; // 这里可以认为时间单位就是ms
	cout << "object pool cost time:" << end2 - begin2 << endl;
}
int main(int argc, const char** argv) {
    QueMemory::init();
    TestObjectPool();
    return 0;
}