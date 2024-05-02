#pragma once
#include <iostream>
#include <iomanip>

#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <cmath>
#include <random>

#include <string>
#include <vector>
#include <deque>
#include <array>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg_json.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/surround_emergency.hpp"
#include "vehicle_interfaces/srv/safety_reg.hpp"
#include "vehicle_interfaces/srv/safety_req.hpp"

using namespace std::chrono_literals;

std::atomic<bool> __global_exit_flag = false;

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string serviceName = "safety_0";

private:
    void _getParams()
    {
        this->get_parameter("serviceName", this->serviceName);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("serviceName", this->serviceName);
        this->_getParams();
    }
};

struct DeviceSafetyScores
{
    std::string device_id;
    std::array<float, 8> scores;
    float lifetime_ms;
};


vehicle_interfaces::ReasonResult<bool> SendRegister(std::string serviceName, vehicle_interfaces::srv::SafetyReg::Request::SharedPtr req)
{
    auto node = rclcpp::Node::make_shared("tmp_safetyservertest_reg_node");
    auto client = node->create_client<vehicle_interfaces::srv::SafetyReg>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        return { res->response, "" };
    }
    return { false, "Request failed." };
}

vehicle_interfaces::ReasonResult<bool> SendRequest(std::string serviceName, vehicle_interfaces::srv::SafetyReq::Request::SharedPtr req, std::deque<DeviceSafetyScores>& devQue)
{
    auto node = rclcpp::Node::make_shared("tmp_safetyservertest_req_node");
    auto client = node->create_client<vehicle_interfaces::srv::SafetyReq>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        if (res->response && res->device_id_vec.size() == res->emergency_scores.size())
        {
            for (int i = 0; i < res->device_id_vec.size(); i++)
            {
                devQue.push_back({ res->device_id_vec[i], res->emergency_scores[i].emergency_percentages, res->emergency_scores[i].lifetime_ms });
            }
        }
        return { res->response, "" };
    }
    return { false, "Request failed." };
}


DeviceSafetyScores GenDeviceEmP()
{
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    DeviceSafetyScores ret;
    ret.device_id = "";

    std::uniform_int_distribution<> uniInt26Distrib{0, 25};
    std::uniform_int_distribution<> uniInt101Distrib{0, 100};

    ret.device_id += "/V0/";
    for (int i = 0; i < 8; i++)
    {
        ret.device_id += uniInt26Distrib(gen_) + 97;
        ret.scores[i] = (float)uniInt101Distrib(gen_) / 100.0;
    }
    ret.device_id += "_" + std::to_string(uniInt26Distrib(gen_));
    ret.lifetime_ms = (float)uniInt101Distrib(gen_) * 1000.0;
    return ret;
}

void PrintHelp()
{
    printf("/** \n\
 * Set emergency (random): \n\
 *   s \n\
 * \n\
 * Get emergency: \n\
 *   g <device_name> \n\
 *   g nearest \n\
 *   g all \n\
 * \n\
 * Quit: \n\
 *   !\n\
 * Help: \n\
 *   ?\n\
 */\n");
}

int main(int argc, char** argv)
{
    // ctrl-c handler
    signal(SIGINT, 
        [](int)
        {
            __global_exit_flag = true;
        });

    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("safetyservertest_params_node");
    std::cout << "Service name: " << params->serviceName << std::endl;

    PrintHelp();

    while (!__global_exit_flag)
    {
        std::this_thread::sleep_for(100ms);
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "!")
            __global_exit_flag = true;
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "?")
            PrintHelp();
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "s")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::SafetyReg::Request>();
            auto src = GenDeviceEmP();
            vehicle_interfaces::msg::SurroundEmergency msg;
            msg.emergency_percentages = src.scores;
            msg.lifetime_ms = src.lifetime_ms;
            req->device_id_vec.push_back(src.device_id);
            req->emergency_scores.push_back(msg);
            auto res = SendRegister(params->serviceName + "_Reg", req);
            std::cout << std::boolalpha << "Request: " << res.result << " Reason: " << res.reason << std::endl;
        }
        else if (inputStrVec.size() < 2)
            continue;

        // inputStrVec size is at least 2.
        if (inputStrVec[0] == "g")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
            req->device_id = inputStrVec[1];

            std::deque<DeviceSafetyScores> dst;
            auto res = SendRequest(params->serviceName + "_Req", req, dst);
            std::cout << std::boolalpha << "Request: " << res.result << " Reason: " << res.reason << std::endl;
            if (res.result)
            {
                std::cout << std::left << std::setw(5) << "Idx" << std::setw(20) << "Device ID" << std::setw(48) << "Scores" << "Lifetime (ms)" << std::endl;
                std::cout << "--------------------------------------------------------------------------------------" << std::endl;
                for (int i = 0; i < dst.size(); i++)
                {
                    std::cout << std::left << std::setw(5) << i << std::setw(20) << dst[i].device_id;

                    std::stringstream ss;
                    for (int j = 0; j < dst[i].scores.size(); j++)
                        ss << std::fixed << std::setprecision(2) << dst[i].scores[j] << " ";
                    std::cout << std::setw(48) << ss.str() << dst[i].lifetime_ms << std::endl;
                }
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}
