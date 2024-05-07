#include <iostream>
#include <vector>
#include <memory>

using namespace std;

void cong(int& a)
{
    a++;
}

int main()
{
    // std::vector<std::vector<double>> m;
    // m.push_back(std::vector<double>({1,2,3}));
    // cout << m[0].size() << endl;
    // cout << m[0].data() << endl;
    std::shared_ptr<int> a = std::make_shared<int>(0);
    cout << *a  << endl;
    cong(*a);
    cout << *a  << endl;
}