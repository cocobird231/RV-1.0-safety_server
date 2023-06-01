#include "vehicle_interfaces/safety.h"


class Params : public rclcpp::Node
{
public:
    std::string service_Safety_nodeName = "safetyserver_node";
    std::string service_Safety_serviceName = "safety_0";
    std::string mainNodeName = "safetyserver_node";
    std::string mainNamespace = "V0_Z0";

private:
    void _getParams()
    {
        this->get_parameter("service_Safety_nodeName", this->service_Safety_nodeName);
        this->get_parameter("service_Safety_serviceName", this->service_Safety_serviceName);
        this->get_parameter("mainNodeName", this->mainNodeName);
        this->get_parameter("mainNamespace", this->mainNamespace);
    }

public:
    Params(std::string nodeName) : Node(nodeName)
    {
        this->declare_parameter<std::string>("service_Safety_nodeName", this->service_Safety_nodeName);
        this->declare_parameter<std::string>("service_Safety_serviceName", this->service_Safety_serviceName);
        this->declare_parameter<std::string>("mainNodeName", this->mainNodeName);
        this->declare_parameter<std::string>("mainNamespace", this->mainNamespace);
        this->_getParams();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("safetyserver_params_node");
    auto server = std::make_shared<SafetyServer>(params->mainNodeName, params->service_Safety_serviceName);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}