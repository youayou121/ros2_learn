#include<iostream>
#include<algorithm>

int main()
{
    int a = 10;
    int b = 20;
    auto lambda_sum = [](int x, int y) -> int { return x + y;};
    auto print_sum = [](int sum) -> void { std::cout << "sum: " << sum << std::endl;};
    print_sum(lambda_sum(a, b));
    return 0;
}