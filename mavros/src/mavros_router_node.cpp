#include <mavros/mavros_router.hpp>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto router_node = std::make_shared<mavros::router::Router>();
  exec.add_node(router_node);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
