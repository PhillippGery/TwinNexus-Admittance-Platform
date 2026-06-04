#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <array>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace admittance_controller
{

class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_parameters();

  bool validate_cartesian_parameter(
    const std::string & parameter_name, const std::vector<double> & values) const;

  std::vector<std::string> build_command_interface_names() const;

  std::vector<std::string> build_state_interface_names() const;

  inline static constexpr std::array<const char *, 6> kWrenchInterfaceSuffixes = {
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

  std::vector<std::string> joints_;
  std::string command_interface_type_{hardware_interface::HW_IF_POSITION};
  std::vector<std::string> state_interface_types_;
  std::string ft_sensor_name_{"force_torque_sensor"};
  double ft_filter_coefficient_{0.005};
  double wrench_deadband_{0.5};
  double max_position_offset_{0.05};
  std::vector<double> mass_;
  std::vector<double> damping_;
  std::vector<double> stiffness_;
  std::array<double, 6> filtered_wrench_{};
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
