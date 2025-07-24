#include "amr/backend_ws_client.h"
#include <jsoncpp/json/json.h>
#include <iostream>

namespace amr {

BackendWsClient::BackendWsClient(const std::string& ws_url)
    : connected_(false)
{
    client_.init_asio();

    client_.set_open_handler([this](websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx_);
        hdl_ = hdl;
        connected_ = true;
        std::cout << "[BackendWsClient] WebSocket 연결 성공\n";
    });

    client_.set_close_handler([this](websocketpp::connection_hdl) {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_ = false;
        std::cerr << "[BackendWsClient] WebSocket 연결 종료됨\n";
    });

    websocketpp::lib::error_code ec;
    auto con = client_.get_connection(ws_url, ec);
    if (ec) {
        std::cerr << "[BackendWsClient] 연결 실패: " << ec.message() << std::endl;
        return;
    }

    hdl_ = con->get_handle();
    client_.connect(con);

    ws_thread_ = std::thread([this]() {
        try {
            client_.run();
        } catch (const std::exception& e) {
            std::cerr << "[BackendWsClient] WebSocket 런타임 오류: " << e.what() << std::endl;
        }
    });
}

BackendWsClient::~BackendWsClient() {
    client_.stop();
    if (ws_thread_.joinable()) {
        ws_thread_.join();
    }
}

void BackendWsClient::sendJson(const std::string& json) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!connected_) {
        std::cerr << "[BackendWsClient] 전송 실패: 연결되지 않음.\n";
        return;
    }

    websocketpp::lib::error_code ec;
    client_.send(hdl_, json, websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cerr << "[BackendWsClient] 전송 실패: " << ec.message() << std::endl;
    }
}

void BackendWsClient::sendLog(const std::string& msg) {
    Json::Value root;
    root["type"] = "log";
    root["msg"] = msg;
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::sendPosition(float x, float y) {
    Json::Value root;
    root["type"] = "position";
    root["x"] = x;
    root["y"] = y;
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::sendSensor(float distance) {
    if (!std::isfinite(distance) || distance <= 0.01f) {
        std::cerr << "[BackendWsClient] 잘못된 센서값: " << distance << std::endl;
        return;
    }

    Json::Value root;
    root["type"] = "sensor";
    root["distance"] = distance;
    sendJson(Json::FastWriter().write(root));
}

}
