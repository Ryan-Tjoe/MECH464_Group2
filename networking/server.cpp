#include <iostream>
#include <ixwebsocket/IXWebSocketServer.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class WebSocketServer {
public:
    WebSocketServer(const std::string& address, int port) 
        : address(address), port(port) {}

    void startServer() {
        ix::WebSocketServer server;

        server.setOnClientMessageCallback([this](std::shared_ptr<ix::WebSocket> socket, const ix::WebSocketMessagePtr& msg) {
            if (msg->type == ix::WebSocketMessageType::Message) {
                std::cout << "Received message: " << msg->str << std::endl;

                try {
                    // Parse the received JSON message
                    json j = json::parse(msg->str);
                    std::cout << "Parsed JSON: " << j.dump() << std::endl;

                    // Prepare a JSON response
                    json response = {
                        {"status", "success"},
                        {"received", j}
                    };

                    // Send the JSON response back to the client
                    socket->send(response.dump());
                }
                catch (json::parse_error& e) {
                    std::cerr << "Error parsing JSON: " << e.what() << std::endl;
                    socket->send("Error: Invalid JSON format");
                }
            }
        });

        server.listen(port);
        std::cout << "Server listening on ws://" << address << ":" << port << std::endl;

        server.start();
    }

private:
    std::string address;
    int port;
};

int main() {
    // Instantiate and start the WebSocket server
    WebSocketServer server("localhost", 8765);
    server.startServer();

    return 0;
}
