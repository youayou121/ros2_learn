#include <rclcpp/rclcpp.hpp>
class PersonNode : public rclcpp::Node
{
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string &node_name, const std::string &name, const int &age) : Node(node_name)
    {
        this->name_ = name;
        this->age_ = age;
        RCLCPP_INFO(this->get_logger(), "Hello, I am %s, %d years old.", this->name_.c_str(), this->age_);
    }

    void eat(const std::string &food)
    {
        RCLCPP_INFO(this->get_logger(), "%s is eating %s.", this->name_.c_str(), food.c_str());
    }
    
    void set_name(const std::string &name)
    {
        this->name_ = name;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PersonNode> zhangsan_node = std::make_shared<PersonNode>("zhangsan_node", "Zhang San", 18);
    std::shared_ptr<PersonNode> lisi_node = std::make_shared<PersonNode>("lisi_node", "Li Si", 18);
    zhangsan_node->eat("apple");
    lisi_node->eat("banana");
    std::cout << "zhangsan_node use_count: " << zhangsan_node.use_count() << " address: " << zhangsan_node.get() << std::endl;
    auto zhangsan_node2 = zhangsan_node;
    zhangsan_node2->set_name("Zhang San 2");
    zhangsan_node2->eat("pear");
    zhangsan_node->eat("apple");
    std::cout << "zhangsan_node use_count: " << zhangsan_node.use_count() << " address: " << zhangsan_node.get() << std::endl;
    rclcpp::spin(lisi_node);
    rclcpp::spin(zhangsan_node);
    rclcpp::shutdown();
    return 0;
}