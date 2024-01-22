#include <random>

#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "safetytest_0_node"

using namespace std::chrono_literals;

class SafetyTestNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::TimerBase::SharedPtr timer0_;
    rclcpp::TimerBase::SharedPtr timer1_;
    std::string nodeName_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

private:
    void _timer0Callback()
    {
        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
        this->setEmergency(this->nodeName_, uniDistrib(this->gen_), vehicle_interfaces::EmergencyScoreDirection::FORWARD);
    }

    void _timer1Callback()
    {
        std::array<float, 8> emPs;
        this->getEmergency("nearest", emPs);
        printf("[SafetyTestNode::_timer1Callback] %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f\n", 
                emPs[0], emPs[1], emPs[2], emPs[3], emPs[4], emPs[5], emPs[6], emPs[7]);
    }

public:
    SafetyTestNode(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(gParams->nodeName)
    {
        this->nodeName_ = gParams->nodeName;
        this->timer0_ = this->create_wall_timer(50ms, std::bind(&SafetyTestNode::_timer0Callback, this));
        this->timer1_ = this->create_wall_timer(100ms, std::bind(&SafetyTestNode::_timer1Callback, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("safetytest_params_node");
    params->nodeName = NODE_NAME;
    params->devInfoService = "";
    params->qosService = "";
    params->timesyncService = "";
    auto safetyTestNode = std::make_shared<SafetyTestNode>(params);
    rclcpp::spin(safetyTestNode);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}