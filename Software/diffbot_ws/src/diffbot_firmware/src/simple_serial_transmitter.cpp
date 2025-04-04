#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        LibSerial::SerialPort microcontroller_;
        std::string port_;

        void msgCallback(const std_msgs::msg::String &msg){
            microcontroller_.Write(msg.data);
        }

    public:
        SimpleSerialTransmitter():Node("simple_serial_transmitter"){
            declare_parameter("port","/dev/ttyACM0");
            port_ = get_parameter("port").as_string();

            microcontroller_.Open(port_);
            microcontroller_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            sub_ = create_subscription<std_msgs::msg::String>(
                "serial_transmitter",10, std::bind(&SimpleSerialTransmitter::msgCallback,this,_1));
        }
        ~SimpleSerialTransmitter(){
            microcontroller_.Close();
        }


};


int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleSerialTransmitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}