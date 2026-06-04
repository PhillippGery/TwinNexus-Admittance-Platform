#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

namespace admittance_controller {
  class AdmittanceController : public controller_interface::ControllerInterface {
  public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  };
}
#endif