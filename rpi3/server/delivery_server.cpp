#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>

using json = nlohmann::json;

class DeliveryServer : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::map<std::string, std::chrono::steady_clock::time_point> vehicle_heartbeat;
    const int HEARTBEAT_TIMEOUT_MS = 5000;

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
            
            // 온보드 통신 구독 (RPi1 → RPi3)
            subscribe(nullptr, "delivery/start/+/1to3", 1);
            subscribe(nullptr, "delivery/vehicle/+/status", 0);
            subscribe(nullptr, "delivery/vehicle/+/alert", 1);
            subscribe(nullptr, "delivery/arrived/+/1to3", 1);
            subscribe(nullptr, "delivery/unlock/+", 1);
            subscribe(nullptr, "delivery/log/+", 1);
            subscribe(nullptr, "delivery/complete/+/1to3", 1);
            
            // 오프보드 통신 구독 (RPi4 → RPi3)
            subscribe(nullptr, "delivery/order/+", 1);
            subscribe(nullptr, "delivery/pin/+/4to3", 1);
            
            std::cout << "✓ Topics subscribed (9 topics)" << std::endl;
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
            
            // 온보드 메시지
            if (topic.find("delivery/start/") != std::string::npos && 
                topic.find("1to3") != std::string::npos) {
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
            else if (topic.find("delivery/arrived/") != std::string::npos && 
                     topic.find("1to3") != std::string::npos) {
                handle_delivery_arrived(data);
            }
            else if (topic.find("delivery/unlock/") != std::string::npos) {
                handle_unlock(data);
            }
            else if (topic.find("delivery/log/") != std::string::npos) {
                handle_auth_log(data);
            }
            else if (topic.find("delivery/complete/") != std::string::npos && 
                     topic.find("1to3") != std::string::npos) {
                handle_delivery_complete(data);
            }
            // 오프보드 메시지
            else if (topic.find("delivery/order/") != std::string::npos) {
                handle_order(data);
            }
            else if (topic.find("delivery/pin/") != std::string::npos && 
                     topic.find("4to3") != std::string::npos) {
                handle_pin_from_customer(data);
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
    void publish_message(const std::string& topic, const json& data, int qos) {
        std::string payload = data.dump();
        int ret = publish(nullptr, topic.c_str(), payload.length(), 
                         (const void*)payload.c_str(), qos, false);
        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ Publish failed to " << topic << std::endl;
        }
    }

    bool execute_query(const std::string& query) {
        char* err = nullptr;
        if (sqlite3_exec(db, query.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
            std::cerr << "  ✗ DB Error: " << err << std::endl;
            sqlite3_free(err);
            return false;
        }
        return true;
    }

    void log_event(const std::string& vehicle_id, const std::string& event_type,
                   const std::string& delivery_id, const std::string& severity) {
        std::string query = 
            "INSERT INTO event_log (vehicle_id, event_type, delivery_id, severity) "
            "VALUES ('" + vehicle_id + "', '" + event_type + "', '" + delivery_id + "', '" + severity + "');";
        execute_query(query);
    }

    // ========== 오프보드 메시지 핸들러 (RPi4) ==========

    void handle_order(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string pin_offboard = data["pin_offboard"].get<std::string>();
            std::string pin_onboard = data["pin_onboard"].get<std::string>();
            std::string destination = data.value("destination", "");
            std::string receiver = data.value("receiver", "");
            
            std::cout << "  Delivery: " << delivery_id << ", Vehicle: " << vehicle_id << std::endl;
            
            // 1. delivery_table에 저장
            std::string query = 
                "INSERT INTO delivery_table (delivery_id, vehicle_id, destination, receiver, status) "
                "VALUES ('" + delivery_id + "', '" + vehicle_id + "', '" + destination + 
                "', '" + receiver + "', 'ordered');";
            if (!execute_query(query)) return;
            
            // 2. PIN 저장 (offboard)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_offboard + 
                   "', 'offboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            // 3. PIN 저장 (onboard)
            query = "INSERT INTO password_table (vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                   "VALUES ('" + vehicle_id + "', '" + delivery_id + "', '" + pin_onboard + 
                   "', 'onboard', datetime('now', '+1 day'));";
            if (!execute_query(query)) return;
            
            log_event(vehicle_id, "order_received", delivery_id, "info");
            std::cout << "✓ Order saved to DB" << std::endl;
            
            // 4. delivery/pin/{id}/3to1 발송 (QoS 2)
            json pin_msg = {
                {"delivery_id", delivery_id},
                {"pin_onboard", pin_onboard}
            };
            publish_message("delivery/pin/" + vehicle_id + "/3to1", pin_msg, 2);
            
            // 5. delivery/command/{id} 발송 (QoS 1)
            json cmd_msg = {
                {"delivery_id", delivery_id},
                {"action", "dispatch"},
                {"destination", destination},
                {"receiver", receiver}
            };
            publish_message("delivery/command/" + vehicle_id, cmd_msg, 1);
            
            std::cout << "  📤 PIN sent (QoS 2): delivery/pin/" << vehicle_id << "/3to1" << std::endl;
            std::cout << "  📤 Command sent (QoS 1): delivery/command/" << vehicle_id << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_order: " << e.what() << std::endl;
        }
    }

    void handle_pin_from_customer(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string pin_offboard = data["pin_offboard"].get<std::string>();
            
            std::cout << "  Customer PIN received: " << delivery_id << std::endl;
            
            // PIN 검증 (password_table과 비교)
            sqlite3_stmt* stmt;
            const char* sql = "SELECT COUNT(*) FROM password_table WHERE delivery_id=? AND pin_code=? AND pin_type='offboard'";
            
            if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
                sqlite3_bind_text(stmt, 1, delivery_id.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_text(stmt, 2, pin_offboard.c_str(), -1, SQLITE_TRANSIENT);
                
                if (sqlite3_step(stmt) == SQLITE_ROW) {
                    int count = sqlite3_column_int(stmt, 0);
                    if (count > 0) {
                        log_event("", "pin_verified", delivery_id, "info");
                        std::cout << "✓ PIN verified" << std::endl;
                    } else {
                        log_event("", "pin_rejected", delivery_id, "warning");
                        std::cout << "✗ PIN incorrect" << std::endl;
                    }
                }
                sqlite3_finalize(stmt);
            }
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_pin_from_customer: " << e.what() << std::endl;
        }
    }

    // ========== 온보드 메시지 핸들러 (RPi1) ==========

    void handle_delivery_start(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  Delivery started: " << delivery_id << " from " << vehicle_id << std::endl;
            
            // 1. Heartbeat 갱신
            vehicle_heartbeat[vehicle_id] = std::chrono::steady_clock::now();
            
            // 2. DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='in_transit', start_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 3. event_log 기록
            log_event(vehicle_id, "delivery_started", delivery_id, "info");
            
            // 4. delivery/start/{id}/3to4 발송 (RPi4로 중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "in_transit"}
            };
            publish_message("delivery/start/" + vehicle_id + "/3to4", msg, 1);
            
            std::cout << "✓ Relayed to RPi4" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_start: " << e.what() << std::endl;
        }
    }

    void handle_vehicle_status(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data.value("delivery_id", "");
            
            // Heartbeat 갱신
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
            
            // Fail-safe: E-Stop 감지
            std::string severity = (alert_type == "e_stop") ? "critical" : "warning";
            log_event(vehicle_id, "alert_" + alert_type, delivery_id, severity);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_vehicle_alert: " << e.what() << std::endl;
        }
    }

    void handle_delivery_arrived(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  📍 Arrived at destination" << std::endl;
            
            // 1. DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='arrived', arrive_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 2. event_log 기록
            log_event(vehicle_id, "delivery_arrived", delivery_id, "info");
            
            // 3. delivery/arrived/{id}/3to4 발송 (RPi4로 중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "arrived"}
            };
            publish_message("delivery/arrived/" + vehicle_id + "/3to4", msg, 1);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_arrived: " << e.what() << std::endl;
        }
    }

    void handle_unlock(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  🔓 Unlock allowed for: " << delivery_id << std::endl;
            
            log_event(vehicle_id, "unlock_allowed", delivery_id, "info");
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_unlock: " << e.what() << std::endl;
        }
    }

    void handle_auth_log(const json& data) {
        try {
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string result = data["result"].get<std::string>();
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

    void handle_delivery_complete(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string vehicle_id = data["vehicle_id"].get<std::string>();
            
            std::cout << "  ✓ Delivery completed" << std::endl;
            
            // 1. DB 업데이트
            std::string query = 
                "UPDATE delivery_table SET status='completed', complete_time=CURRENT_TIMESTAMP "
                "WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 2. PIN 정리
            query = "DELETE FROM password_table WHERE delivery_id='" + delivery_id + "';";
            execute_query(query);
            
            // 3. event_log 기록
            log_event(vehicle_id, "delivery_completed", delivery_id, "info");
            
            // 4. delivery/complete/{id}/3to4 발송 (RPi4로 중계)
            json msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"status", "completed"}
            };
            publish_message("delivery/complete/" + vehicle_id + "/3to4", msg, 1);
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_delivery_complete: " << e.what() << std::endl;
        }
    }
};

int main() {
    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi3 Delivery Server (Final MQTT Structure)   ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;
    
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
