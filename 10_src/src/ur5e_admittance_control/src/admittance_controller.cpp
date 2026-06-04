#include "admittance_controller/admittance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace
{

constexpr std::size_t kCartesianDof = 6;
constexpr double kMinimumStiffness = 1.0e-6;

}  // namespace

namespace admittance_controller
{

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::vector<std::string>>(
    "command_interfaces",
    std::vector<std::string>{hardware_interface::HW_IF_POSITION});
  auto_declare<std::vector<std::string>>(
    "state_interfaces",
    std::vector<std::string>{
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY});
  auto_declare<std::string>("ft_sensor.name", ft_sensor_name_);
  auto_declare<double>("ft_sensor.filter_coefficient", ft_filter_coefficient_);
  auto_declare<double>("wrench_deadband", wrench_deadband_);
  auto_declare<double>("max_position_offset", max_position_offset_);
  auto_declare<std::vector<double>>("mass", std::vector<double>(kCartesianDof, 0.0));
  auto_declare<std::vector<double>>("damping", std::vector<double>(kCartesianDof, 0.0));
  auto_declare<std::vector<double>>("stiffness", std::vector<double>(kCartesianDof, 1.0));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    build_command_interface_names()};
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    build_state_interface_names()};
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!read_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  filtered_wrench_.fill(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces but received %zu.",
      joints_.size(),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != ((joints_.size() * 2u) + kCartesianDof))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces but received %zu.",
      (joints_.size() * 2u) + kCartesianDof,
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  filtered_wrench_.fill(0.0);

  for (std::size_t index = 0; index < joints_.size(); ++index)
  {
    const double position = state_interfaces_[index].get_value();
    if (!std::isfinite(position))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint state interface '%s' reported a non-finite position.",
        state_interfaces_[index].get_name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!command_interfaces_[index].set_value(position))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unable to seed command interface '%s' from the current joint position.",
        command_interfaces_[index].get_name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t index = 0; index < std::min(command_interfaces_.size(), joints_.size()); ++index)
  {
    const double position = state_interfaces_[index].get_value();
    if (std::isfinite(position))
    {
      command_interfaces_[index].set_value(position);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::size_t joint_count = joints_.size();
  const std::size_t velocity_offset = joint_count;
  const std::size_t wrench_offset = joint_count * 2;
  const std::size_t expected_state_count = wrench_offset + kCartesianDof;

  if (command_interfaces_.size() != joint_count || state_interfaces_.size() != expected_state_count)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Admittance controller interfaces are misconfigured: %zu command, %zu state, expected %zu.",
      command_interfaces_.size(),
      state_interfaces_.size(),
      expected_state_count);
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t axis = 0; axis < kCartesianDof; ++axis)
  {
    const double raw_wrench = state_interfaces_[wrench_offset + axis].get_value();
    if (!std::isfinite(raw_wrench))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Wrench interface '%s' reported a non-finite value.",
        state_interfaces_[wrench_offset + axis].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }

    filtered_wrench_[axis] =
      ((1.0 - ft_filter_coefficient_) * filtered_wrench_[axis]) +
      (ft_filter_coefficient_ * raw_wrench);
  }

  for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index)
  {
    const double position = state_interfaces_[joint_index].get_value();
    const double velocity = state_interfaces_[velocity_offset + joint_index].get_value();

    if (!std::isfinite(position) || !std::isfinite(velocity))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Joint '%s' reported a non-finite position or velocity.",
        joints_[joint_index].c_str());
      return controller_interface::return_type::ERROR;
    }

    double offset = 0.0;
    const double wrench = filtered_wrench_[joint_index];
    if (
      std::abs(wrench) >= wrench_deadband_ &&
      std::abs(stiffness_[joint_index]) > kMinimumStiffness)
    {
      // Minimal joint-space compliance law until the full Cartesian solver is wired in.
      const double compliance_term = wrench - (damping_[joint_index] * velocity);
      offset = std::clamp(
        compliance_term / stiffness_[joint_index],
        -max_position_offset_,
        max_position_offset_);
    }

    if (!command_interfaces_[joint_index].set_value(position + offset))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Unable to write the command interface '%s'.",
        command_interfaces_[joint_index].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

bool AdmittanceController::read_parameters()
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter must not be empty.");
    return false;
  }

  const auto command_interfaces = get_node()->get_parameter("command_interfaces").as_string_array();
  if (
    command_interfaces.size() != 1u ||
    command_interfaces.front() != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'command_interfaces' must contain exactly one '%s' entry.",
      hardware_interface::HW_IF_POSITION);
    return false;
  }
  command_interface_type_ = command_interfaces.front();

  state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
  const bool has_position =
    std::find(
      state_interface_types_.cbegin(),
      state_interface_types_.cend(),
      hardware_interface::HW_IF_POSITION) != state_interface_types_.cend();
  const bool has_velocity =
    std::find(
      state_interface_types_.cbegin(),
      state_interface_types_.cend(),
      hardware_interface::HW_IF_VELOCITY) != state_interface_types_.cend();
  if (!has_position || !has_velocity)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'state_interfaces' must include both '%s' and '%s'.",
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY);
    return false;
  }

  ft_sensor_name_ = get_node()->get_parameter("ft_sensor.name").as_string();
  if (ft_sensor_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'ft_sensor.name' must not be empty.");
    return false;
  }

  ft_filter_coefficient_ = get_node()->get_parameter("ft_sensor.filter_coefficient").as_double();
  if (ft_filter_coefficient_ < 0.0 || ft_filter_coefficient_ > 1.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'ft_sensor.filter_coefficient' must be between 0.0 and 1.0.");
    return false;
  }

  wrench_deadband_ = get_node()->get_parameter("wrench_deadband").as_double();
  if (wrench_deadband_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'wrench_deadband' must be non-negative.");
    return false;
  }

  max_position_offset_ = get_node()->get_parameter("max_position_offset").as_double();
  if (max_position_offset_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'max_position_offset' must be greater than zero.");
    return false;
  }

  mass_ = get_node()->get_parameter("mass").as_double_array();
  damping_ = get_node()->get_parameter("damping").as_double_array();
  stiffness_ = get_node()->get_parameter("stiffness").as_double_array();

  return
    validate_cartesian_parameter("mass", mass_) &&
    validate_cartesian_parameter("damping", damping_) &&
    validate_cartesian_parameter("stiffness", stiffness_);
}

bool AdmittanceController::validate_cartesian_parameter(
  const std::string & parameter_name, const std::vector<double> & values) const
{
  if (values.size() != kCartesianDof)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'%s' must provide exactly %zu values.",
      parameter_name.c_str(),
      kCartesianDof);
    return false;
  }

  if (
    !std::all_of(
      values.cbegin(),
      values.cend(),
      [](double value) { return std::isfinite(value); }))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'%s' must only contain finite values.",
      parameter_name.c_str());
    return false;
  }

  return true;
}

std::vector<std::string> AdmittanceController::build_command_interface_names() const
{
  std::vector<std::string> interface_names;
  interface_names.reserve(joints_.size());

  for (const auto & joint : joints_)
  {
    interface_names.push_back(joint + "/" + command_interface_type_);
  }

  return interface_names;
}

std::vector<std::string> AdmittanceController::build_state_interface_names() const
{
  std::vector<std::string> interface_names;
  interface_names.reserve((joints_.size() * 2u) + kWrenchInterfaceSuffixes.size());

  for (const auto & joint : joints_)
  {
    interface_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  for (const auto & joint : joints_)
  {
    interface_names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (const auto * suffix : kWrenchInterfaceSuffixes)
  {
    interface_names.push_back(ft_sensor_name_ + "/" + suffix);
  }

  return interface_names;
}

}  // namespace admittance_controller

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController,
  controller_interface::ControllerInterface)