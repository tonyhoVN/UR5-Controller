#include <iostream>
#include <vector>

using namespace std;
int main()
{
    std::vector<std::vector<double>> m;
    m.push_back(std::vector<double>({1,2,3}));
    cout << m[0].size() << endl;
}