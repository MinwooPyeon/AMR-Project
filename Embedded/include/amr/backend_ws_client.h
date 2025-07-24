#pragma once
#include <websocketpp/config/asio_no_tls_client.h>
#include <websocketpp/client.h>
#include <string>
#include <mutex>
#include <thread>

namespace amr {

class BackendWsClient {
public:
    explicit BackendWsClient(const std::string& ws_url);
    ~BackendWsClient();

    void sendJson(const std::string& json);
    void sendLog(const std::string& msg);
    void sendPosition(float x, float y);
    void sendSensor(float distance);
    bool isConnected() const;

private:
    using client_t = websocketpp::client<websocketpp::config::asio_client>;
    client_t client_;
    websocketpp::connection_hdl hdl_;
    mutable std::mutex mtx_;
    std::thread ws_thread_;
    bool connected_ = false;
};

} // namespace amr
