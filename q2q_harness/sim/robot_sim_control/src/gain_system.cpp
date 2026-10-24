// Apache-2.0. Delegate to the pinned upstream backend, without modifying it.
#include <cmath>
#include <stdexcept>
#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_sim_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class GainSystem : public gz_ros2_control::GazeboSimSystemInterface
{
  // Loader must outlive its backend instance. Composition avoids the upstream
  // concrete class's incomplete-PImpl destructor restriction on subclassing.
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> backend_;
public:
  GainSystem() : loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"),
    backend_(loader_.createSharedInstance("gz_ros2_control/GazeboSimSystem")) {}

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    const auto result = hardware_interface::SystemInterface::on_init(info);
    if (result != CallbackReturn::SUCCESS) {return result;}
    return backend_->on_init(info);
  }
  bool initSim(rclcpp::Node::SharedPtr & node,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & info,
    sim::EntityComponentManager & ecm, int & rate) override
  {
    const double gain = std::stod(info.hardware_parameters.at("q2q_position_gain"));
    if (!std::isfinite(gain) || gain < 0.05 || gain > 1.0) {
      throw std::invalid_argument("q2q_position_gain must be within [0.05,1.0]");
    }
    if (node->has_parameter("position_proportional_gain")) {
      const auto result = node->set_parameter(rclcpp::Parameter("position_proportional_gain", gain));
      if (!result.successful) {throw std::runtime_error(result.reason);}
    } else {
      node->declare_parameter<double>("position_proportional_gain", gain);
    }
    RCLCPP_INFO(node->get_logger(), "Q2Q %s position gain %.3f", info.name.c_str(), gain);
    return backend_->initSim(node, joints, info, ecm, rate);
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {return backend_->export_state_interfaces();}
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {return backend_->export_command_interfaces();}
  CallbackReturn on_configure(const rclcpp_lifecycle::State & s) override
  {return backend_->on_configure(s);}
  CallbackReturn on_activate(const rclcpp_lifecycle::State & s) override
  {return backend_->on_activate(s);}
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & s) override
  {return backend_->on_deactivate(s);}
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {return backend_->prepare_command_mode_switch(start, stop);}
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {return backend_->perform_command_mode_switch(start, stop);}
  hardware_interface::return_type read(const rclcpp::Time & t, const rclcpp::Duration & d) override
  {return backend_->read(t, d);}
  hardware_interface::return_type write(const rclcpp::Time & t, const rclcpp::Duration & d) override
  {return backend_->write(t, d);}
};
}
PLUGINLIB_EXPORT_CLASS(robot_sim_control::GainSystem, gz_ros2_control::GazeboSimSystemInterface)
