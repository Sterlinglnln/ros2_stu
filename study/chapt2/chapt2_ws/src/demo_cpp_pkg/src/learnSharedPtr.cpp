#include <iostream>
#include <memory>
using namespace std;

int main() {
    auto p1 = make_shared<string>("This is a str.");
    cout << "P1的引用计数为：" << p1.use_count() << ", 指向内存的地址为：" << p1.get() << endl;

    auto p2 = p1;
    cout << "P1的引用计数为：" << p1.use_count() << ", 指向内存的地址为：" << p1.get() << endl;
    cout << "P2的引用计数为：" << p2.use_count() << ", 指向内存的地址为：" << p2.get() << endl;

    p1.reset();
    cout << "P1的引用计数为：" << p1.use_count() << ", 指向内存的地址为：" << p1.get() << endl;
    cout << "P2的引用计数为：" << p2.use_count() << ", 指向内存的地址为：" << p2.get() << endl;
    cout << "P2指向资源的内容为：" << p2->c_str() << endl;

    return 0;
}