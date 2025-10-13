#include<iostream>
#include<functional>
void save_with_free_function(const std::string &file_name)
{
    std::cout << "free function save file: " << file_name << std::endl;
}

class LearnFunctional
{
private:
public:
    LearnFunctional() = default;
    ~LearnFunctional() = default;
    void save_with_member_function(const std::string &file_name)
    {
        std::cout << "member function save file: " << file_name << std::endl;
    }
};

int main()
{
    LearnFunctional lf;
    auto lambda_fun = [](const std::string &file_name)
    {
        std::cout << "lambda function save file: " << file_name << std::endl;
    };
    save_with_free_function("file1.txt");
    lf.save_with_member_function("file2.txt");
    lambda_fun("file3.txt");
    std::function<void(const std::string &)> func1 = save_with_free_function;
    func1("file4.txt");
    std::function<void(const std::string &)> func2 = std::bind(&LearnFunctional::save_with_member_function, &lf, std::placeholders::_1);
    func2("file5.txt");
    std::function<void(const std::string &)> func3 = lambda_fun;
    func3("file6.txt");
    return 0;
}

