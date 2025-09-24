#include "rover_assignment/bz2_subscriber.hpp"

/*
* Bz2 Subscriber Node Constructor

* param type : string, param name : name

*/
Bz2Subscriber::Bz2Subscriber(const std::string& name)
: Node(name)
{
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/bz2_message", 10, std::bind(&Bz2Subscriber::topicCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Char>("/solution", 10);
}

/*
* Callback function for the subscribed topic

* param type : std_msgs::msg::String::SharedPtr, param name : msg

* return type : void
*/
void Bz2Subscriber::topicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty message");
        return;
    }

    if(msg->data != input_) {
        
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        auto compressedData = hexToBytes(msg->data);
        try {
            std::string decompressedMessage = decompressBZ2(compressedData);
            RCLCPP_INFO(this->get_logger(), "Decompressed message: '%s'", decompressedMessage.c_str());
            auto [characters_, max_number] = calculation(decompressedMessage);

            for(const auto &ch : characters_){
                RCLCPP_INFO(this->get_logger(), "Most frequent character : %c and its ASCII value: %d with repetition: %d",
                                                ch, static_cast<int>(ch), max_number);
                auto message = std_msgs::msg::Char();
                message.data = static_cast<char>(static_cast<int>(ch));
                pub_->publish(message);
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Decompression failed: %s", e.what());
            
        }
        input_ = msg->data;
    }
    else{
        return;
    }
    
}

/*
* Calculate the most frequent character(s) in the decompressed message

* param type : string, param name : input

* return type : pair<vector<unsigned char>, int>
*/
std::pair<std::vector<unsigned char>, int> Bz2Subscriber::calculation(std::string &input) {
    std::unordered_map<std::string, int> dict;
    std::vector<unsigned char> total_input_;
    int max_number = 0; 

    for (char &c : input) {
        if (dict.find(std::string(1, c)) == dict.end()) {
            dict[std::string(1, c)] = 1;
        }
        else if (dict.find(std::string(1, c)) != dict.end()) {
            dict[std::string(1, c)] += 1;
        }
    }

    for (const auto &pair : dict) {
        if (pair.second > max_number) {
            max_number = pair.second;
            total_input_.clear();
            total_input_.push_back(pair.first[0]);
        }
        else if (pair.second == max_number) {
            total_input_.push_back(pair.first[0]);
            std::sort(total_input_.begin(), total_input_.end());
        }
    }
    return std::make_pair(total_input_, max_number);
}

/*
* Convert hex string to byte vector

* param type : string, param name : hex

* return type : vector<unsigned char>
*/
std::vector<unsigned char> Bz2Subscriber::hexToBytes(const std::string &hex) {
    std::vector<unsigned char> bytes;
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        unsigned char byte = static_cast<unsigned char>(strtol(byteString.c_str(), nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

/*
* Decompress BZ2 compressed data

* param type : vector<unsigned char>, param name : compressedData

* return type : string
*/
std::string Bz2Subscriber::decompressBZ2(const std::vector<unsigned char>& compressedData) {
    // Çıkış buffer'ı tahmini (mesajın max boyutunu biliyorsan daha net verebilirsin)
    unsigned int uncompressedSize = compressedData.size() * 10; 
    std::vector<char> uncompressed(uncompressedSize);

    int ret = BZ2_bzBuffToBuffDecompress(
        uncompressed.data(),
        &uncompressedSize,
        (char*)compressedData.data(),
        compressedData.size(),
        0,
        0  
    );

    if (ret != BZ_OK) {
        throw std::runtime_error("BZ2 decompression failed with code " + std::to_string(ret));
    }

    return std::string(uncompressed.data(), uncompressedSize);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Bz2Subscriber>("bz2_subscriber_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}