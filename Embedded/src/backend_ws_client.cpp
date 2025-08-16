#include "amr/backend_ws_client.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <cmath>

namespace amr {

BackendWsClient::BackendWsClient(const std::string& ws_url)
    : ws_url_(ws_url)
{
    client_.init_asio();

    client_.set_open_handler([this](websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx_);
        hdl_ = hdl;
        connected_ = true;
        RCLCPP_INFO(rclcpp::get_logger("BackendWsClient"), "WebSocket 연결 성공");
        reconnect_thread_running_ = false;
        tryResendBackup();
    });

    client_.set_close_handler([this](websocketpp::connection_hdl) {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_ = false;
        RCLCPP_WARN(rclcpp::get_logger("BackendWsClient"), "WebSocket 연결 종료됨");
        startReconnectThreadIfNeeded();
    });

    client_.set_fail_handler([this](websocketpp::connection_hdl) {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "WebSocket 연결 실패");
        startReconnectThreadIfNeeded();
    });

    client_.set_message_handler([this](websocketpp::connection_hdl, client_t::message_ptr msg) {
        if (msg->get_opcode() == websocketpp::frame::opcode::text && recv_handler_) {
            recv_handler_(msg->get_payload());
        }
    });

    websocketpp::lib::error_code ec;
    auto con = client_.get_connection(ws_url_, ec);
    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "WebSocket 연결 실패: %s", ec.message().c_str());
        startReconnectThreadIfNeeded();
    } else {
        hdl_ = con->get_handle();
        client_.connect(con);
    }

    std::error_code ec_fs;
    std::filesystem::create_directories(backup_folder_, ec_fs);
    if (ec_fs) {
        RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "backup 폴더 생성 실패: %s", ec_fs.message().c_str());
    }

    ws_thread_ = std::thread([this]() {
        try {
            client_.run();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "WebSocket 런타임 오류: %s", e.what());
        }
    });
}

BackendWsClient::~BackendWsClient() {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_flag_ = true;
    }
    try {
        if (connected_) {
            websocketpp::lib::error_code ec;
            client_.close(hdl_, websocketpp::close::status::normal, "Client shutdown", ec);
            if (ec) {
                RCLCPP_WARN(rclcpp::get_logger("BackendWsClient"), "WebSocket close 실패: %s", ec.message().c_str());
            }
        } else {
            client_.stop();
        }
    } catch (...) {}

    if (ws_thread_.joinable()) ws_thread_.join();
    if (reconnect_thread_ && reconnect_thread_->joinable()) reconnect_thread_->join();
}

void BackendWsClient::sendState(float battery, const std::string& motor_state) {
    Json::Value root;
    root["type"] = "state";
    root["timestamp"] = static_cast<Json::Int64>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    root["battery"] = battery;
    root["motor"] = motor_state;
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::sendLidar(const std::vector<float>& ranges) {
    Json::Value root;
    root["type"] = "lidar";
    root["timestamp"] = static_cast<Json::Int64>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    for (float r : ranges) root["ranges"].append(r);
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::sendCamera(const std::string& jpeg_base64) {
    Json::Value root;
    root["type"] = "camera";
    root["timestamp"] = static_cast<Json::Int64>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    root["image"] = jpeg_base64;
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::sendLog(const std::string& msg) {
    Json::Value root;
    root["type"] = "log";
    root["timestamp"] = static_cast<Json::Int64>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    root["msg"] = msg;
    sendJson(Json::FastWriter().write(root));
}

void BackendWsClient::setRecvHandler(const std::function<void(const std::string&)>& handler) {
    recv_handler_ = handler;
}

void BackendWsClient::sendJson(const std::string& json) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!connected_) {
        RCLCPP_WARN(rclcpp::get_logger("BackendWsClient"), "전송 실패: WebSocket 연결 없음. 백업 큐 저장");
        saveMsgToBackup(json);
        return;
    }

    websocketpp::lib::error_code ec;
    client_.send(hdl_, json, websocketpp::frame::opcode::text, ec);
    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "메시지 전송 실패: %s", ec.message().c_str());
        saveMsgToBackup(json);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BackendWsClient"), "메시지 전송 성공");
    }
}

void BackendWsClient::saveMsgToBackup(const std::string& json) {
    backup_queue_.push(json);
    std::string filename = backup_folder_ + "/ws_backup.log";
    std::ofstream ofs(filename, std::ios::app);
    if (ofs.is_open()) {
        ofs << json << std::endl;
        ofs.close();
        RCLCPP_INFO(rclcpp::get_logger("BackendWsClient"), "Backup message saved successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "Backup file save failed");
    }
}

void BackendWsClient::tryResendBackup() {
    while (connected_ && !backup_queue_.empty()) {
        const std::string& json = backup_queue_.front();
        websocketpp::lib::error_code ec;
        client_.send(hdl_, json, websocketpp::frame::opcode::text, ec);
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("BackendWsClient"), "백업 재전송 실패: %s", ec.message().c_str());
            break;
        } else {
            backup_queue_.pop();
            RCLCPP_INFO(rclcpp::get_logger("BackendWsClient"), "백업 재전송 성공");
        }
    }
}

void BackendWsClient::startReconnectThreadIfNeeded() {
    if (reconnect_thread_running_) return;
    reconnect_thread_running_ = true;

    reconnect_thread_ = std::make_unique<std::thread>([this]() {
        const int retry_interval_sec = 5;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (stop_flag_ || connected_) {
                    reconnect_thread_running_ = false;
                    break;
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("BackendWsClient"), "WebSocket 재접속 시도");
            websocketpp::lib::error_code ec;
            auto con = client_.get_connection(ws_url_, ec);
            if (!ec) {
                std::lock_guard<std::mutex> lock(mtx_);
                client_.connect(con);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("BackendWsClient"), "재접속 실패: %s", ec.message().c_str());
            }
            std::this_thread::sleep_for(std::chrono::seconds(retry_interval_sec));
        }
    });
}

}
