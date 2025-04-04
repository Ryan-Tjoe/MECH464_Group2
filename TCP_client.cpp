#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
using json = nlohmann::json;

void receiveXYZ() {
    boost::asio::io_service io_service;
    tcp::socket socket(io_service);

    // Connect to the Python server
    socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 5000));

    while (true) {
        std::cout << "Waiting for data..." << std::endl;
        boost::asio::streambuf buf;
        boost::asio::read_until(socket, buf, "\n");  // Read data until newline

        std::istream is(&buf);
        std::string data;
        std::getline(is, data);

        // Parse JSON data
        json parsed_data = json::parse(data);
        double x = parsed_data["x"];
        double y = parsed_data["y"];
        double z = parsed_data["z"];

        std::cout << "Received XYZ: " << x << ", " << y << ", " << z << std::endl;

        // TODO: Convert XYZ into ROS message and publish to Franka Emika robot
    }
}

int main() {
    try {
        receiveXYZ();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
