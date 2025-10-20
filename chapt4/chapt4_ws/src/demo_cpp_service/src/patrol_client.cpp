#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<turtlesim/msg/pose.hpp>
#include<chrono>
#include<chapt4_interfaces/srv/patrol.hpp>
#include<ctime>
using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;
class TurtlePatrolClient : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
public:
    explicit TurtlePatrolClient(const std::string &node_name) : Node(node_name)
    {
        patrol_client_ = this->create_client<Patrol>("/turtle1/patrol");
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtlePatrolClient::send_patrol, this));
        srand(time(NULL));
    }
    
    void send_patrol()
    {
        while(this->patrol_client_->wait_for_service(1s) == false)
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "rclcpp is shutdown, exiting");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "patrol service not available, waiting again...");

        }
        Patrol::Request::SharedPtr request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 12;
        request->target_y = rand() % 12;
        RCLCPP_INFO(this->get_logger(), "Sending patrol request: (%.2f, %.2f)", request->target_x, request->target_y);
        patrol_client_->async_send_request(request, std::bind(&TurtlePatrolClient::send_target_callback, this, std::placeholders::_1));
    }

    void send_target_callback(rclcpp::Client<Patrol>::SharedFuture future)
    {
        Patrol::Response::SharedPtr response = future.get();
        if(response->result == Patrol::Response::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Request for target success");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Request for target failed");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TurtlePatrolClient> turtle_patrol_client = std::make_shared<TurtlePatrolClient>("turtle_patrol_client");
    rclcpp::spin(turtle_patrol_client);
    rclcpp::shutdown();
    return 0;
}