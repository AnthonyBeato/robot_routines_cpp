#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class RoutineNode : public rclcpp::Node
{
public:
  RoutineNode() : Node("routine_node")
  {
    // Inicializa el publisher para /diffbot_base_controller/cmd_vel_unstamped
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Timer para ejecutar la rutina periódicamente
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RoutineNode::execute_routine, this)
    );

    // Inicializar el tiempo de inicio
    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Routine node has been started.");
  }

private:
  void execute_routine()
  {
    auto message = geometry_msgs::msg::Twist();

    // Verificar el tiempo transcurrido
    rclcpp::Duration elapsed_time = this->now() - start_time_;
    if (elapsed_time.seconds() < target_duration_)
    {
      // Mover hacia adelante
      message.linear.x = 0.1;
      message.angular.z = 0.0;
    }
    else
    {
      // Detener el robot
      message.linear.x = 0.0;
      message.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  const double target_duration_ = 10.0; // Limitar a 10 segundos
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoutineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
