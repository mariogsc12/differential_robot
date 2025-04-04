#include <diffbot_firmware/diffbot_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace diffbot_firmware{

    DiffbotInterface::DiffbotInterface(){
        
    }

    DiffbotInterface::~DiffbotInterface(){
        if(microcontroller_.IsOpen()){
            try
            {
                microcontroller_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("DiffbotInterface"), "Something went wrong while closing the connection with port "<<port_);

            }
            
        }
    }

    CallbackReturn DiffbotInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
        RCLCPP_INFO(rclcpp::get_logger("DiffbotInterface"), "Starting robot hardware...");
        velocity_commands_ = {0.0, 0.0};
        position_states_   = {0.0, 0.0};
        velocity_states_   = {0.0, 0.0};

        try
        {
            microcontroller_.Open(port_);
            microcontroller_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("DiffbotInterface"), "Something went wrong while interacting with port "<<port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffbotInterface"),"Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffbotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
        RCLCPP_INFO(rclcpp::get_logger("DiffbotInterface"),"Stopping robot hardware...");
        if(microcontroller_.IsOpen()){
            try
            {
                microcontroller_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("DiffbotInterface"), "Something went wrong while closing the port "<<port_);
                return CallbackReturn::FAILURE;
            }
            
        }       
    }

    CallbackReturn DiffbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info){
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS){
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("DiffbotInterface"), "No serial port provided! Aborting ");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffbotInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i=0; i<info_.joints.size();i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                            hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffbotInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i=0; i<info_.joints.size();i++){
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                                hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }


    hardware_interface::return_type DiffbotInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        if(microcontroller_.IsDataAvailable()){
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            std::string res;
            int multiplier = 1;

            microcontroller_.ReadLine(message);
            std::stringstream ss(message);

            while(std::getline(ss, res, ',')){
                multiplier = res.at(1) == 'p' ? 1 : -1;
                if(res.at(0) == 'r'){
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                    position_states_.at(0) += velocity_states_.at(0)*dt; 
                }
                else if(res.at(1) == 'l'){
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                    position_states_.at(1) += velocity_states_.at(1)*dt;
                }
            }

            last_run_ = rclcpp::Clock().now();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffbotInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands_.at(0)) <= 10.0){
            compensate_zeros_right = "0";
        } 
        else {
            compensate_zeros_right = "";
        }

        if(std::abs(velocity_commands_.at(1)) <= 10.0){
            compensate_zeros_left = "0";
        } 
        else {
            compensate_zeros_left = "";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
                                                            ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            microcontroller_.Write(message_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffbotInterface"), "Something went wrong while sending the message "<<message_stream.str() << " on the port " << port_);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
                                      
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(diffbot_firmware::DiffbotInterface, hardware_interface::SystemInterface)