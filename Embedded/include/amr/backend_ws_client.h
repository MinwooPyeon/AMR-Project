#pragma once

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <jsoncpp/json/json.h>
#include <string>
#include <queue>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

namespace amr {

using client_t = websocketpp::client<websocketpp::config::asio_client>;

class BackendWsClient {
public:
    explicit BackendWsClient(const std::string& ws_url);
    ~BackendWsClient();

    void sendState(float battery, const std::string& motor_state);
    void sendLidar(const std::vector<float>& ranges);
    void sendCamera(const std::string& jpeg_base64);
    void sendLog(const std::string& msg);

    void setRecvHandler(const std::function<void(const std::string&)>& handler);

private:
    void sendJson(const std::string& json);
    void saveMsgToBackup(const std::string& json);
    void tryResendBackup();
    void startReconnectThreadIfNeeded();

    client_t client_;
    websocketpp::connection_hdl hdl_;
    std::string ws_url_;
    std::string backup_folder_ = "ws_message_backup";

    std::function<void(const std::string&)> recv_handler_;
    std::queue<std::string> backup_queue_;

    std::mutex mtx_;
    std::thread ws_thread_;
    std::unique_ptr<std::thread> reconnect_thread_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> reconnect_thread_running_{false};
};

}