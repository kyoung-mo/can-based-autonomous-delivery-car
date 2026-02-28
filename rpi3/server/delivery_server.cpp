#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <ctime>

using json = nlohmann::json;

class DeliveryServer : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::map<std::string, std::chrono::steady_clock::time_point> vehicle_heartbeat;
    const int HEARTBEAT_TIMEOUT_MS = 5000;  // 5초 = 공식 요구사항

public:
    DeliveryServer() : mosqpp::mosquittopp("rpi3-server") {
        sqlite3_open("/home/pi/catnip/database/delivery_system.db", &db); 
        username_pw_set("hoji", "1234");
    }

    ~DeliveryServer() {
        if (db) sqlite3_close(db);
    }

    bool start() {
        mosqpp::lib_init();

        int rc = connect("localhost", 1883, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ MQTT connection failed: " << rc << std::endl;
            return false;
        }

        if (loop_start() != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ Loop start failed" << std::endl;
            return false;
        }

        for (int i = 0; i < 50; i++) {
            if (connected) {
                std::cout << "✓ Connected to MQTT broker" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << "✗ Connection timeout" << std::endl;
        return false;
    }

    // Heartbeat 모니터링 (공식 요구사항: 5초 이상 신호 없음 → 경고)
    void monitor_heartbeat() {
        auto now = std::chrono::steady_clock::now();
        
        for (auto& [vehicle_id, last_time] : vehicle_heartbeat) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_time).count();
            
            if (elapsed > HEARTBEAT_TIMEOUT_MS) {
                std::cerr << "⚠️  Vehicle " << vehicle_id << " - Heartbeat timeout (" 
                         << elapsed << "ms)" << std::endl;
                log_event(vehicle_id, "heartbeat_loss", "", "warning");
            }
        }
    }

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            std::cout << "✓ Connected to MQTT broker" << std::endl;
            
            // 공식 요구사항의 12개 토픽 구독
            subscribe(nullptr, "delivery/order/+", 1);
            subscribe(nullptr, "delivery/start/+/onboard", 1);
            subscribe(nullptr, "delivery/vehicle/+/status", 0);
            subscribe(nullptr, "delivery/vehicle/+/alert", 1);
            subscribe(nullptr, "delivery/arrived/+/onboard", 1);
            subscribe(nullptr, "delivery/complete/+/onboard", 1);
            subscribe(nullptr, "delivery/log/+", 1);
            
            std::cout << "✓ Topics subscribed (7 topics)" << std::endl;
        } else {
            std::cerr << "✗ Connection failed: " << rc << std::endl;
        }
    }

    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        
        std::cout << "\n[📨] " << topic << std::endl;
        
        try {
            auto data = json::parse(payload);
            
            // 공식 요구사항: 서버 로직 흐름에 따른 핸들러 호출
            if (topic.find("delivery/order/") != std::string::npos) {
                handle_order(data);
            }
            else if (topic.find("delivery/start/") != std::string::npos) {
                handle_delivery_start(data);
            }
            else if (topic.find("delivery/vehicle/") != std::string::npos && 
                     topic.find("status") != std::string::npos) {
                handle_vehicle_status(data);
            }
            else if (topic.find("delivery/vehicle/") != std::string::npos && 
                     topic.find("alert") != std::string::npos) {
                handle_vehicle_alert(data);
            }
            else if (topic.find("delivery/arrived/") != std::string::npos) {
                handle_delivery_arrived(data);
            }
            else if (topic.find("delivery/complete/") != std::string::npos) {
                handle_delivery_complete(data);
            }
            else if (topic.find("delivery/log/") != std::string::npos) {
                handle_auth_log(data);
            }
            
        } catch (const std::exception& e) {
            std::cerr << "✗ JSON parse error: " << e.what() << std::endl;
        }
    }

    void on_disconnect(int rc) override {
        connected = false;
        std::cout << "⚠️  Disconnected (rc: " << rc << ")" << std::endl;
    }

private:
    // MQTT 메시지 발행 (공식 요구사항: 올바른 함수 호출)
    void publish_message(const std::string& topic, const json& data, int qos) {
        std::string payload = data.dump();
        int ret = publish(nullptr, topic.c_str(), payload.length(), 
                         (const void*)payload.c_str(), qos, false);
        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ Publish failed to " << topic << std::endl;
        }
    }

    // SQL 쿼리 실행
    bool execute_query(const std::string& query) {
        char* err = nullptr;
        if (sqlite3_exec(db, query.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
            std::cerr << "  ✗ DB Error: " << err << std::endl;
            sqlite3_free(err);
            return false;
        }
        return true;
    }

    // 이벤트 로그 저장 (공식 DB 스키마)
    void log_event(const std::string& vehicle_id, const std::string& event_type,
                   const std::string& delivery_id, const std::string& severity) {
        std::string query = 
            "INSERT INTO event_log (vehicle_id, event_type, delivery_id, severity) "
            "VALUES ('" + vehicle_id + "', '" + event_type + "', '" + delivery_id + "', '" + severity + "');";
        execute_query(query);
    }

    // 공식 요구사항: 주문 완료 처리
    void handle_order(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string pin_offboard = data["pin_offboard"].get<std::string>();
            std::string pin_onboard = data["pin_onboard"].get<std::string>();
            std::string destination = data.value("destination", "");
            std::string receiver = data.value("receiver", "");
            
            std::cout << "  Delivery: " << delivery_id << ", Vehicle: " << vehicle_id << std::endl;
            
            // 1. delivery_table에 저장: status='ordered'
            std::string query = 
                "INSERT INTO delivery_table (delivery_id, vehicle_id, destination, receiver, status) "
                "VALUES ('" + delivery_id + "', '" + vehicle_id + "', '" + destination + 
                "', '" + receiver + "', 'ordered');";
            if (!execute_query(query)) return;
            
            // 2. password_table에 PIN 저장 (offboard)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_offboard + 
                   "', 'offboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            // 3. password_table에 PIN 저장 (onboard)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_onboard + 
                   "', 'onboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            // 4. event_log에 "order_received" 기록
            log_event(vehicle_id, "order_received", delivery_id, "info");
            std::cout << "✓ Order saved to DB" << std::endl;
            
            // 5. delivery/pin/001/onboard (QoS 2) → RPi1/RPi2
            json pin_msg = {
                {"delivery_id", delivery_id},
                {"pin_onboard", pin_onboard}
            };
            publish_message("delivery/pin/" + vehicle_id + "/onboard", pin_msg, 2);
            
            // 6. delivery/command/001 (QoS 1) → RPi1/RPi2
            json cmd_msg = {
                {"delivery_id", delivery_id},
                {"destination", destination},
                {"receiver", receiver}
            };
            publish_message("delivery/command/" + vehicle_id, cmd_msg, 1);
            
            std::cout << "  📤 PIN sent (QoS 2): delivery/pin/" << vehicle_id << "/onboard" << std::endl;
            std::cout << "  📤 Command sent (QoS 1): delivery/command/" << vehicle_id << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_order: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 배달 시작
    void handle_delivery_start(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  Delivery started: " << delivery_id << " from " << vehicle_id << std::endl;
            
            // 1. Heartbeat 갱신
            vehicle_heartbeat[vehicle_id] = std::chrono::steady_clock::now();
            
            // 2. delivery_table 업데이트: status='in_transit', start_time
            std::string query = 
                "UPDATE delivery_table SET status='in_transit', start_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 3. event_log에 "delivery_started" 기록
            log_event(vehicle_id, "delivery_started", delivery_id, "info");
            
            // 4. delivery/start/001/offboard (QoS 1) → RPi4 (중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "in_transit"}
            };
            publish_message("delivery/start/" + vehicle_id + "/offboard", msg, 1);
            
            std::cout << "✓ Relayed to RPi4" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_start: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 배달 중 (차량 상태 수집)
    void handle_vehicle_status(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data.value("delivery_id", "");
            
            // 1. Heartbeat 갱신
            vehicle_heartbeat[vehicle_id] = std::chrono::steady_clock::now();
            
            // 2. event_log에 기록 (QoS 0 상태는 로그만)
            log_event(vehicle_id, "status_update", delivery_id, "info");
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_vehicle_status: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 경보 처리
    void handle_vehicle_alert(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string alert_type = data.value("type", "unknown");
            std::string delivery_id = data.value("delivery_id", "");
            
            std::cout << "  ⚠️  Alert: " << alert_type << std::endl;
            
            // Fail-safe: E-Stop 감지
            std::string severity = (alert_type == "e_stop") ? "critical" : "warning";
            log_event(vehicle_id, "alert_" + alert_type, delivery_id, severity);
            
            // delivery/vehicle/001/alert → RPi4 (중계)
            json alert_data = data;
            publish_message("delivery/vehicle/" + vehicle_id + "/alert_offboard", alert_data, 1);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_vehicle_alert: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 도착 신호 처리
    void handle_delivery_arrived(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  📍 Arrived at destination" << std::endl;
            
            // 1. delivery_table 업데이트: status='arrived', arrive_time
            std::string query = 
                "UPDATE delivery_table SET status='arrived', arrive_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 2. event_log에 "delivery_arrived" 기록
            log_event(vehicle_id, "delivery_arrived", delivery_id, "info");
            
            // 3. delivery/arrived/001/offboard (QoS 1) → RPi4 (중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "arrived"}
            };
            publish_message("delivery/arrived/" + vehicle_id + "/offboard", msg, 1);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_arrived: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 배달 완료
    void handle_delivery_complete(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  ✓ Delivery completed" << std::endl;
            
            // 1. delivery_table 업데이트: status='completed', complete_time
            std::string query = 
                "UPDATE delivery_table SET status='completed', complete_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 2. password_table 정리: 사용된 PIN 삭제
            query = "DELETE FROM password_table WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 3. event_log에 "delivery_completed" 기록
            log_event(vehicle_id, "delivery_completed", delivery_id, "info");
            
            // 4. delivery/complete/001/offboard (QoS 1) → RPi4 (중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "completed"}
            };
            publish_message("delivery/complete/" + vehicle_id + "/offboard", msg, 1);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_complete: " << e.what() << std::endl;
        }
    }

    // 공식 요구사항: 인증 실패 로그 (Fail-safe)
    void handle_auth_log(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string result = data["result"].get<std::string>();
            std::string pin_type = data.value("pin_type", "");
            int attempt = data.value("attempt_number", 0);
            
            std::cout << "  Auth: " << result << " (attempt " << attempt << ")" << std::endl;
            
            // Fail-safe: 인증 실패 5회 감지
            if (result == "failed" && attempt >= 5) {
                std::cerr << "  🔒 SECURITY: Authentication failed 5 times!" << std::endl;
                log_event(vehicle_id, "auth_failed_max", delivery_id, "critical");
            } else {
                log_event(vehicle_id, "auth_" + result, delivery_id, "info");
            }
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_auth_log: " << e.what() << std::endl;
        }
    }
};

int main() {
    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi3 Delivery Server (Official Design)      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;
    
    DeliveryServer server;
    if (!server.start()) {
        return 1;
    }
    
    std::cout << "\n✓ Server running (Ctrl+C to stop)\n" << std::endl;
    
    // 메인 루프: Heartbeat 모니터링 (500ms 주기)
    while (true) {
        server.monitor_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return 0;
}
