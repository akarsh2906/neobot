#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


float MaxLinearVel = 0.314;
float MinLinearVel = -0.314;
float MaxAngVel = 0.4;
float MinAngVel = -0.4;


float L = 0.352;  //Wheel Separation
float r = 0.05;  //Wheel radius

float conv = 1/r;  //Conversion from m/s to rad/s


class SpeedPublisher : public rclcpp::Node
{
  public:
    SpeedPublisher()
    : Node("speed_publisher")
    {
      subCmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&SpeedPublisher::calcSpeeds, this, _1));

      leftVelPub = this->create_publisher<std_msgs::msg::Float64>("left_wheel_speed", 20);

      rightVelPub = this->create_publisher<std_msgs::msg::Float64>("right_wheel_speed", 20);

    }

    std_msgs::msg::Float64 leftVel;
    std_msgs::msg::Float64 rightVel;
    

  private:

    void calcSpeeds(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        float linVel = cmd->linear.x;
        float angVel = cmd->angular.z;

        if(linVel<MinLinearVel)
            linVel = MinLinearVel;

        if(linVel>MaxLinearVel)
            linVel = MaxLinearVel;

        if(angVel<MinAngVel)
            angVel = MinAngVel;

        if(angVel>MaxAngVel)
            angVel = MaxAngVel;



        leftVel.data = (linVel - ((angVel * L)/2)) * conv;
        rightVel.data = (linVel + ((angVel * L)/2)) * conv;
    
        leftVelPub->publish(leftVel);
        rightVelPub->publish(rightVel);
      
    }


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftVelPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightVelPub;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}