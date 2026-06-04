#include "admittance_controller/admittance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace admittance_controller {
  controller_interface::CallbackReturn AdmittanceController::on_init() { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State&) { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State&) { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::CallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State&) { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::return_type AdmittanceController::update(const rclcpp::Time&, const rclcpp::Duration&) { return controller_interface::return_type::OK; }
}

PLUGINLIB_EXPORT_CLASS(admittance_controller::AdmittanceController, controller_interface::ControllerInterface)