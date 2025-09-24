#ifndef BZ2_SUBSCRIBER_HPP
#define BZ2_SUBSCRIBER_HPP
#include <bzlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/char.hpp"


class Bz2Subscriber : public rclcpp::Node
{
public:
    Bz2Subscriber(const std::string& name);

private: 

    std::vector<unsigned char> hexToBytes(const std::string &hex);
    std::pair<std::vector<unsigned char>, int> calculation(std::string &input);
    std::string decompressBZ2(const std::vector<unsigned char>& compressedData);
    void topicCallback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr pub_;
    std::string input_;
};

#endif // BZ2_SUBSCRIBER_HPP