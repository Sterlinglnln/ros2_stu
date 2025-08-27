#include <iostream>
#include <functional>
using namespace std;

void saveWithFreeFun(const string& fileName) {
    cout << "调用了自由函数，保存：" << fileName << endl;
}

class fileSave {
public:
    void saveWithMemberFun(const string& fileName) {
        cout << "调用了成员方法，保存：" << fileName << endl;
    }
};

int main() {
    fileSave f1;

    auto saveWithLambdaFun = [] (const string& fileName) -> void {
        cout << "调用了lambda函数，保存：" << fileName << endl;
    };
    
    function<void(const string&) > save1 = saveWithFreeFun;
    function<void(const string&) > save2 = saveWithLambdaFun;
    function<void(const string&) > save3 = bind(&fileSave::saveWithMemberFun, &f1, placeholders::_1);

    save1("C Primer Plus.pdf");
    save2("C++ Primer Plus.pdf");
    save3("ROS2.pdf");

    return 0;
}
