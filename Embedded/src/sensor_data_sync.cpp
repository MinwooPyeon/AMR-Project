#include "amr/sensor_data_sync.h"
#include <iostream>
#include <cmath>
SensorDataSync::SensorDataSync(const std::string& robot_id) 
    : robot_id_(robot_id)
    , sync_running_(false)
    , sync_interval_(0.1)
    , total_sync_count_(0)
    , last_sync_time_(0.0) {
    
    std::cout << "Sensor Data Sync initialization completed - Robot ID: " << robot_id_ << std::endl;
}

SensorDataSync::~SensorDataSync() {
    stopSync();
}
    
    void updateSensorData(SensorType sensor_type, const std::map<std::string, double>& data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        sensor_data_[sensor_type] = data;
    }
    
    std::map<std::string, double> getSensorData(SensorType sensor_type) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return sensor_data_[sensor_type];
    }
    
    std::map<std::string, std::map<std::string, double>> getAllSensorData() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::map<std::string, std::map<std::string, double>> result;
        for (const auto& pair : sensor_data_) {
            std::string sensor_name;
            switch (pair.first) {
                case SensorType::LIDAR: sensor_name = "lidar"; break;
                case SensorType::CAMERA: sensor_name = "camera"; break;
                case SensorType::IMU: sensor_name = "imu"; break;
                case SensorType::MOTOR_STATUS: sensor_name = "motor_status"; break;
            }
            result[sensor_name] = pair.second;
        }
        return result;
    }
    
    void setDataCallback(std::function<void(const std::map<std::string, double>&)> callback) {
        data_callback_ = callback;
    }
    
    void startSync() {
        if (sync_running_) {
            std::cout << "Sensor data sync is already running" << std::endl;
            return;
        }
        
        sync_running_ = true;
        sync_thread_ = std::thread(&SensorDataSync::syncWorker, this);
        std::cout << "Sensor data sync started" << std::endl;
    }
    
    void stopSync() {
        sync_running_ = false;
        if (sync_thread_.joinable()) {
            sync_thread_.join();
        }
        std::cout << "Sensor data sync stopped" << std::endl;
    }
    
private:
    void syncWorker() {
        while (sync_running_) {
            try {
                std::map<std::string, double> sync_data;
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    sync_data["robot_id"] = std::stod(robot_id_.substr(3));
                    sync_data["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
                    
                    for (const auto& pair : sensor_data_) {
                        for (const auto& data_pair : pair.second) {
                            sync_data[data_pair.first] = data_pair.second;
                        }
                    }
                }
                
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    total_sync_count_++;
                    last_sync_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
                }
                
                if (data_callback_) {
                    data_callback_(sync_data);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sync_interval_ * 1000)));
                
            } catch (const std::exception& e) {
                std::cerr << "Sensor data sync error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sync_interval_ * 1000)));
            }
        }
    }
    
public:
    std::map<std::string, std::string> createDataPacket() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        auto motor_data = sensor_data_[SensorType::MOTOR_STATUS];
        double left_speed = motor_data.count("left_speed") ? motor_data["left_speed"] : 0.0;
        double right_speed = motor_data.count("right_speed") ? motor_data["right_speed"] : 0.0;
        double average_speed = (std::abs(left_speed) + std::abs(right_speed)) / 2.0;
        
        auto imu_data = sensor_data_[SensorType::IMU];
        double x = imu_data.count("x") ? imu_data["x"] : 0.0;
        double y = imu_data.count("y") ? imu_data["y"] : 0.0;
        
        std::map<std::string, std::string> data_packet;
        data_packet["serial"] = robot_id_;
        data_packet["state"] = "RUNNING";
        data_packet["x"] = std::to_string(x);
        data_packet["y"] = std::to_string(y);
        data_packet["speed"] = std::to_string(average_speed);
        
        return data_packet;
    }
    
    std::map<std::string, double> getSyncStats() {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        std::map<std::string, double> stats;
        stats["total_sync_count"] = total_sync_count_;
        stats["last_sync_time"] = last_sync_time_;
        stats["sync_running"] = sync_running_ ? 1.0 : 0.0;
        return stats;
    }
};

void testSensorDataSync() {
    auto sensor_sync = std::make_unique<SensorDataSync>("AMR001");
    
    auto data_callback = [](const std::map<std::string, double>& sync_data) {
        std::cout << "Sensor data sync: ";
        for (const auto& pair : sync_data) {
            std::cout << pair.first << "=" << pair.second << " ";
        }
        std::cout << std::endl;
    };
    
    sensor_sync->setDataCallback(data_callback);
    sensor_sync->startSync();
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::map<std::string, double> motor_data{{"left_speed", 25.0}, {"right_speed", 25.0}};
    sensor_sync->updateSensorData(SensorType::MOTOR_STATUS, motor_data);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::map<std::string, double> imu_data{{"x", 10.5}, {"y", 20.3}, {"heading", 45.0}};
    sensor_sync->updateSensorData(SensorType::IMU, imu_data);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::map<std::string, double> lidar_data{{"distance", 150.0}, {"angle", 0.0}};
    sensor_sync->updateSensorData(SensorType::LIDAR, lidar_data);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    auto data_packet = sensor_sync->createDataPacket();
    std::cout << "Data packet: ";
    for (const auto& pair : data_packet) {
        std::cout << pair.first << "=" << pair.second << " ";
    }
    std::cout << std::endl;
    
    auto stats = sensor_sync->getSyncStats();
    std::cout << "Sync stats: ";
    for (const auto& pair : stats) {
        std::cout << pair.first << "=" << pair.second << " ";
    }
    std::cout << std::endl;
    
    sensor_sync->stopSync();
}

int main() {
    testSensorDataSync();
    return 0;
}
