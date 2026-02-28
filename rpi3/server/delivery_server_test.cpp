#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <unistd.h>

using json = nlohmann::json;

class DeliveryServer : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::map<std::string, std::chrono::steady_clock::time_point> vehicle_heartbeat;

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

        // 연결 완료까지 대기
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

    // Heartbeat 모니터링 (메인 루프에서 호출)
    void monitor_heartbeat() {
        auto now = std::chrono::steady_clock::now();
        
        for (auto& [vehicle_id, last_time] : vehicle_heartbeat) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_time).count();
            
            if (elapsed > 5000) { // 5초 이상 신호 없음
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
            
            // 토픽 구독 (설계 문서 기준)
            subscribe(nullptr, "delivery/order/+", 1);              // RPi4 → RPi3
            subscribe(nullptr, "delivery/start/+/onboard", 1);      // RPi1 → RPi3
            subscribe(nullptr, "delivery/vehicle/+/status", 0);     // RPi1 → RPi3 (상태, QoS 0)
            subscribe(nullptr, "delivery/vehicle/+/alert", 1);      // RPi1 → RPi3 (경보)
            subscribe(nullptr, "delivery/arrived/+/onboard", 1);    // RPi1 → RPi3
            subscribe(nullptr, "delivery/complete/+/onboard", 1);   // RPi1 → RPi3
            subscribe(nullptr, "delivery/log/+", 1);                // RPi1 → RPi3 (인증 로그)
            
            std::cout << "✓ Topics subscribed" << std::endl;
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
    // Prepared Statement로 안전한 쿼리 실행
    bool execute_query(const std::string& query) {
        char* err = nullptr;
        if (sqlite3_exec(db, query.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
            std::cerr << "  ✗ DB Error: " << err << std::endl;
            sqlite3_free(err);
            return false;
        }
        return true;
    }

    // 로그 저장
    void log_event(const std::string& vehicle_id, const std::string& event_type,
                   const std::string& delivery_id, const std::string& severity) {
        std::string query = 
            "INSERT INTO event_log (vehicle_id, event_type, delivery_id, severity) "
            "VALUES ('" + vehicle_id + "', '" + event_type + "', '" + delivery_id + "', '" + severity + "');";
        execute_query(query);
    }

    void handle_order(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string pin_offboard = data["pin_offboard"].get<std::string>();
            std::string pin_onboard = data["pin_onboard"].get<std::string>();
            std::string destination = data.value("destination", "DEST");
            std::string receiver = data.value("receiver", "RECEIVER");
            
            std::cout << "  Delivery: " << delivery_id << ", Vehicle: " << vehicle_id << std::endl;
            
            // DB에 배달 정보 저장
            std::string query = 
                "INSERT INTO delivery_table (delivery_id, vehicle_id, destination, receiver, status) "
                "VALUES ('" + delivery_id + "', '" + vehicle_id + "', '" + destination + 
                "', '" + receiver + "', 'ordered');";
            
            if (!execute_query(query)) return;
            
            // PIN 저장 (offboard - 고객 인증용)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_offboard + 
                   "', 'offboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            // PIN 저장 (onboard - 차량 인증용)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_onboard + 
                   "', 'onboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            log_event(vehicle_id, "order_received", delivery_id, "info");
            std::cout << "✓ Order saved" << std::endl;
            
            // RPi3 → RPi1: PIN 전송 (QoS 2 - 정확히 1회)
            json pin_msg = {
                {"delivery_id", delivery_id},
                {"pin_onboard", pin_onboard}
            };
            std::string pin_payload = pin_msg.dump();
            publish(nullptr, ("delivery/pin/" + vehicle_id + "/onboard").c_str(), pin_payload.length(), (const void*)pin_payload.c_str(), 2, false);
            
            // RPi3 → RPi1: 출동 명령 (QoS 1)
            json cmd_msg = {
                {"delivery_id", delivery_id},
                {"destination", destination},
                {"receiver", receiver}
            };
            std::string cmd_payload = cmd_msg.dump();
            publish(nullptr, ("delivery/command/" + vehicle_id).c_str(), cmd_payload.length(), (const void*)cmd_payload.c_str(), 1, false);
            
            std::cout << "  📤 PIN sent to RPi1 (delivery/pin/" << vehicle_id << "/onboard)" << std::endl;
            std::cout << "  📤 Command sent to RPi1 (delivery/command/" << vehicle_id << ")" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_order: " << e.what() << std::endl;
        }
    }

    void handle_delivery_start(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  Delivery " << delivery_id << " started from " << vehicle_id << std::endl;
            
            // 하트비트 갱신
            vehicle_heartbeat[vehicle_id] = std::chrono::steady_clock::now();
            
            // DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='in_transit', start_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            log_event(vehicle_id, "delivery_started", delivery_id, "info");
            
            // RPi3 → RPi4: 시작 알림 (QoS 1)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "in_transit"}
            };
            std::string start_payload = msg.dump();
            publish(nullptr, ("delivery/start/" + vehicle_id + "/offboard").c_str(), start_payload.length(), (const void*)start_payload.c_str(), 1, false);
            
            std::cout << "✓ Relayed to RPi4" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_start: " << e.what() << std::endl;
        }
    }

    void handle_vehicle_status(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data.value("delivery_id", "");
            
            // 하트비트 갱신
            vehicle_heartbeat[vehicle_id] = std::chrono::steady_clock::now();
            
            log_event(vehicle_id, "status_update", delivery_id, "info");
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_vehicle_status: " << e.what() << std::endl;
        }
    }

    void handle_vehicle_alert(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string alert_type = data.value("type", "unknown");
            std::string delivery_id = data.value("delivery_id", "");
            
            std::cout << "  ⚠️  Alert: " << alert_type << std::endl;
            
            log_event(vehicle_id, "alert_" + alert_type, delivery_id, "warning");
            
            // RPi3 → RPi4: 경보 중계
            std::string alert_payload = data.dump();
            publish(nullptr, ("delivery/vehicle/" + vehicle_id + "/alert_offboard").c_str(), alert_payload.length(), (const void*)alert_payload.c_str(), 1, false);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_vehicle_alert: " << e.what() << std::endl;
        }
    }

    void handle_delivery_arrived(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  📍 Arrived at destination" << std::endl;
            
            // DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='arrived', arrive_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            log_event(vehicle_id, "delivery_arrived", delivery_id, "info");
            
            // RPi3 → RPi4: 도착 알림
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "arrived"}
            };
            std::string arrived_payload = msg.dump();
            publish(nullptr, ("delivery/arrived/" + vehicle_id + "/offboard").c_str(), arrived_payload.length(), (const void*)arrived_payload.c_str(), 1, false);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_arrived: " << e.what() << std::endl;
        }
    }

    void handle_delivery_complete(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  ✓ Delivery completed" << std::endl;
            
            // DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='completed', complete_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            log_event(vehicle_id, "delivery_completed", delivery_id, "info");
            
            // RPi3 → RPi4: 완료 알림
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "completed"}
            };
            std::string complete_payload = msg.dump();
            publish(nullptr, ("delivery/complete/" + vehicle_id + "/offboard").c_str(), complete_payload.length(), (const void*)complete_payload.c_str(), 1, false);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_complete: " << e.what() << std::endl;
        }
    }

    void handle_auth_log(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string result = data["result"].get<std::string>();
            int attempt = data.value("attempt_number", 0);
            
            std::cout << "  Auth: " << result << " (attempt " << attempt << ")" << std::endl;
            
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
    std::cout << "╔════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi3 Delivery Server (Event-driven)   ║" << std::endl;
    std::cout << "╚════════════════════════════════════════╝" << std::endl;

    
    DeliveryServer server;
    if (!server.start()) {
        return 1;
    }
    
    std::cout << "✓ Server running (Ctrl+C to stop)\n" << std::endl;
    
    while (true) {
        server.monitor_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return 0;
}
