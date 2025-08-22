#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <functional>
#include <memory>

enum class SensorType {
    LIDAR,
    CAMERA,
    IMU,
    MOTOR_STATUS
};

class SensorDataSync {
private:
    std::string robot_id_;
    std::map<SensorType, std::map<std::string, double>> sensor_data_;
    std::mutex data_mutex_;
    
    bool sync_running_;
    std::thread sync_thread_;
    double sync_interval_;
    
    std::function<void(const std::map<std::string, double>&)> data_callback_;
    
    std::mutex stats_mutex_;
    int total_sync_count_;
    double last_sync_time_;
    
    void syncWorker();
    
public:
    explicit SensorDataSync(const std::string& robot_id = "AMR001");
    ~SensorDataSync();
    
    void updateSensorData(SensorType sensor_type, const std::map<std::string, double>& data);
    std::map<std::string, double> getSensorData(SensorType sensor_type);
    std::map<std::string, std::map<std::string, double>> getAllSensorData();
    
    void setDataCallback(std::function<void(const std::map<std::string, double>&)> callback);
    
    void startSync();
    void stopSync();
    
    std::map<std::string, std::string> createDataPacket();
    std::map<std::string, double> getSyncStats();
};
