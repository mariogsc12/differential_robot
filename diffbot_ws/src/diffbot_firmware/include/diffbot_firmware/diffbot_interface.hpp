#ifndef DIFFBOT_INTERFACE_HPP
#define DIFFBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>


namespace diffbot_firmware{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class DiffbotInterface : public hardware_interface::SystemInterface{
        private:
            LibSerial::SerialPort microcontroller_;
            std::string port_;
            std::vector<double> velocity_commands_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;

            rclcpp::Time last_run_;

        public:
            DiffbotInterface();
            virtual ~DiffbotInterface();

            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };

}
#endif