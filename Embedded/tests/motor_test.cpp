#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "amr/i2c/MyI2CImplementation.h"
#include "amr/i2c/i2c_address_manager.h"
#include "amr/module/motor_driver.h"
#include "amr/module/motor_controller.h"

using namespace amr;

class MotorTester {
private:
    std::shared_ptr<amr::MyI2CImplementation> i2c_;
    std::shared_ptr<amr::I2CAddressManager> addressManager_;
    std::shared_ptr<amr::MotorDriver> leftMotor_;
    std::shared_ptr<amr::MotorDriver> rightMotor_;
    std::shared_ptr<amr::MotorController> motorController_;
    
public:
    MotorTester(const std::string& i2cDevice = "/dev/i2c-1") {
        try {
            i2c_ = std::make_shared<amr::MyI2CImplementation>(i2cDevice);
            addressManager_ = std::make_shared<amr::I2CAddressManager>(i2c_);
            std::cout << "I2C 인터페이스 초기화 완료: " << i2cDevice << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "I2C 인터페이스 초기화 실패: " << e.what() << std::endl;
            throw;
        }
    }
    
    bool initializeMotors() {
        std::cout << "\n=== 모터 초기화 시작 ===" << std::endl;
        
        // I2C 버스 상태 확인
        addressManager_->printBusStatus();
        
        // Waveshare Motor Driver HAT 주소 사용 (0x60)
        uint8_t motorAddr = MotorConstants::DEFAULT_I2C_ADDRESS;
        
        // 모터 드라이버 생성 (Waveshare HAT은 하나의 드라이버로 양쪽 모터 제어)
        leftMotor_ = std::make_shared<amr::MotorDriver>(i2c_, motorAddr, "Waveshare Motor Driver HAT");
        
        // 모터 컨트롤러 생성 (오른쪽 모터는 null로 설정, 실제로는 사용하지 않음)
        motorController_ = std::make_shared<amr::MotorController>(leftMotor_, leftMotor_, "AMR 모터 컨트롤러");
        
        if (motorController_->isConnected()) {
            std::cout << "모터 초기화 성공!" << std::endl;
            std::cout << "Waveshare Motor Driver HAT 주소: 0x" << std::hex << (int)motorAddr << std::dec << std::endl;
            return true;
        } else {
            std::cerr << "모터 초기화 실패: " << motorController_->getStatusString() << std::endl;
            return false;
        }
    }
    
    void runBasicTest() {
        std::cout << "\n=== 기본 모터 테스트 ===" << std::endl;
        
        if (!motorController_->isConnected()) {
            std::cerr << "모터가 연결되지 않았습니다." << std::endl;
            return;
        }
        
        // 전진 테스트
        std::cout << "전진 테스트 (속도: 50)" << std::endl;
        motorController_->moveForward(50);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 정지
        std::cout << "정지" << std::endl;
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 후진 테스트
        std::cout << "후진 테스트 (속도: 50)" << std::endl;
        motorController_->moveBackward(50);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 정지
        std::cout << "정지" << std::endl;
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 좌회전 테스트
        std::cout << "좌회전 테스트 (속도: 40)" << std::endl;
        motorController_->turnLeft(40);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 정지
        std::cout << "정지" << std::endl;
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 우회전 테스트
        std::cout << "우회전 테스트 (속도: 40)" << std::endl;
        motorController_->turnRight(40);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 정지
        std::cout << "정지" << std::endl;
        motorController_->stop();
        
        std::cout << "기본 테스트 완료!" << std::endl;
    }
    
    void runAdvancedTest() {
        std::cout << "\n=== 고급 모터 테스트 ===" << std::endl;
        
        if (!motorController_->isConnected()) {
            std::cerr << "모터가 연결되지 않았습니다." << std::endl;
            return;
        }
        
        // 차동 이동 테스트
        std::cout << "차동 이동 테스트 (왼쪽: 30, 오른쪽: 70)" << std::endl;
        motorController_->moveWithDifferential(30, 70);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 곡률 기반 이동 테스트
        std::cout << "곡률 기반 이동 테스트 (속도: 60, 곡률: 0.3)" << std::endl;
        motorController_->moveWithCurvature(60, 0.3);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 회전 테스트
        std::cout << "좌회전 테스트 (속도: 50)" << std::endl;
        motorController_->rotateLeft(50);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        motorController_->stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "우회전 테스트 (속도: 50)" << std::endl;
        motorController_->rotateRight(50);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        motorController_->stop();
        
        std::cout << "고급 테스트 완료!" << std::endl;
    }
    
    void runDiagnosticTest() {
        std::cout << "\n=== 진단 테스트 ===" << std::endl;
        
        // 모터 상태 확인
        std::cout << "모터 상태: " << motorController_->getStatusString() << std::endl;
        
        // 개별 모터 테스트
        std::cout << "개별 모터 테스트..." << std::endl;
        if (motorController_->testMotors()) {
            std::cout << "모터 테스트 성공!" << std::endl;
        } else {
            std::cerr << "모터 테스트 실패!" << std::endl;
        }
        
        // 모터 캘리브레이션
        std::cout << "모터 캘리브레이션..." << std::endl;
        if (motorController_->calibrateMotors()) {
            std::cout << "캘리브레이션 성공!" << std::endl;
        } else {
            std::cerr << "캘리브레이션 실패!" << std::endl;
        }
        
        // 현재 모터 속도 확인
        std::cout << "현재 왼쪽 모터 속도: " << motorController_->getLeftMotorSpeed() << std::endl;
        std::cout << "현재 오른쪽 모터 속도: " << motorController_->getRightMotorSpeed() << std::endl;
    }
    
    void runInteractiveTest() {
        std::cout << "\n=== 대화형 테스트 ===" << std::endl;
        std::cout << "명령어:" << std::endl;
        std::cout << "  f <속도> - 전진" << std::endl;
        std::cout << "  b <속도> - 후진" << std::endl;
        std::cout << "  l <속도> - 좌회전" << std::endl;
        std::cout << "  r <속도> - 우회전" << std::endl;
        std::cout << "  s - 정지" << std::endl;
        std::cout << "  e - 비상 정지" << std::endl;
        std::cout <<  "q - 종료" << std::endl;
        
        std::string command;
        while (true) {
            std::cout << "\n명령어 입력: ";
            std::getline(std::cin, command);
            
            if (command == "q") {
                break;
            } else if (command == "s") {
                motorController_->stop();
                std::cout << "정지" << std::endl;
            } else if (command == "e") {
                motorController_->emergencyStop();
                std::cout << "비상 정지" << std::endl;
            } else if (command.length() >= 2) {
                char cmd = command[0];
                int speed = std::stoi(command.substr(2));
                
                switch (cmd) {
                    case 'f':
                        motorController_->moveForward(speed);
                        std::cout << "전진 (속도: " << speed << ")" << std::endl;
                        break;
                    case 'b':
                        motorController_->moveBackward(speed);
                        std::cout << "후진 (속도: " << speed << ")" << std::endl;
                        break;
                    case 'l':
                        motorController_->turnLeft(speed);
                        std::cout << "좌회전 (속도: " << speed << ")" << std::endl;
                        break;
                    case 'r':
                        motorController_->turnRight(speed);
                        std::cout << "우회전 (속도: " << speed << ")" << std::endl;
                        break;
                    default:
                        std::cout << "알 수 없는 명령어" << std::endl;
                        break;
                }
            }
        }
    }
    
private:
    // 이 메서드는 더 이상 필요하지 않습니다. I2CAddressManager가 대신 처리합니다.
};

int main() {
    try {
        std::cout << "=== AMR 모터 테스트 프로그램 ===" << std::endl;
        
        MotorTester tester;
        
        if (!tester.initializeMotors()) {
            std::cerr << "모터 초기화 실패. 프로그램을 종료합니다." << std::endl;
            return 1;
        }
        
        // 기본 테스트
        tester.runBasicTest();
        
        // 고급 테스트
        tester.runAdvancedTest();
        
        // 진단 테스트
        tester.runDiagnosticTest();
        
        // 대화형 테스트
        tester.runInteractiveTest();
        
        std::cout << "모터 테스트 완료!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 