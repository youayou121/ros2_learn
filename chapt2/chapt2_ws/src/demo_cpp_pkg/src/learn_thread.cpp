#include<iostream>
#include<thread>
#include<chrono>
#include<functional>
#include<cpp-httplib/httplib.h>
class Download
{
private:
    
public:
    Download() = default;
    ~Download() = default;
    void download(const std::string &host, const std::string &path, const std::function<
        void(const std::string &, const std::string &)>callback_word_count)
    {
        std::cout<<"this thread id: "<< std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback_word_count(path, response->body);
        }
    };
    void start_download(const std::string &host, const std::string &path, const std::function<
        void(const std::string &, const std::string &)>callback_word_count)
    {
        auto download_func = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        std::thread thread_download(download_func, host, path, callback_word_count);
        thread_download.detach();
    };
};

int main()
{
    Download download = Download();
    auto word_count = [](const std::string &path, const std::string &result) -> void
    {
        std::cout << "path: "<< path << ", word count: " << result.length() << std::endl;};
    download.start_download("http://0.0.0.0:8000", "/novels1.txt", word_count);
    download.start_download("http://0.0.0.0:8000", "/novels2.txt", word_count);
    download.start_download("http://0.0.0.0:8000", "/novels3.txt", word_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));
    return 0;
}