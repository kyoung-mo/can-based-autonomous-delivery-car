#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <cstdlib>  // for getenv()

using json = nlohmann::json;

class DeliveryServer : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::map<std::string, std::chrono::steady_clock::time_point> vehicle_heartbeat;
    const int HEARTBEAT_TIMEOUT_MS = 5000;
    
    // ✅ MQTT 설정 멤버 변수
    std::string broker_host;
    int broker_port;

public:
    // ✅ 개선된 생성자: 호스트/포트를 매개변수로 받음
    DeliveryServer(const std::string& host = "10.42.0.1", 
                   int port = 1883) 
        : mosqpp::mosquittopp("rpi3-server"),
          broker_host(host),
          broker_port(port) {
        
        sqlite3_open("/home/pi/catnip/database/delivery_system.db", &db);
        if (!db) {
            std::cerr << "✗ Database open failed" << std::endl;
        }
        
        // ✅ Mosquitto 인증 정보 설정
        username_pw_set("hoji", "1234");
        std::cout << "✓ MQTT Client created with broker: " << broker_host 
                  << ":" << broker_port << std::endl;
    }

    ~DeliveryServer() {
        if (db) sqlite3_close(db);
    }

    bool start() {
        mosqpp::lib_init();

        // ✅ 환경변수 지원: MQTT_BROKER_HOST 또는 하드코딩된 호스트 사용
        const char* env_host = std::getenv("MQTT_BROKER_HOST");
        if (env_host) {
            broker_host = env_host;
            std::cout << "📝 Using MQTT broker from env: " << broker_host << std::endl;
        }

        std::cout << "🔗 Connecting to MQTT broker " << broker_host 
                  << ":" << broker_port << "..." << std::endl;

        int rc = connect(broker_host.c_str(), broker_port, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ MQTT connection failed (rc=" << rc << ")" << std::endl;
            print_error_code(rc);
            return false;
        }

        if (loop_start() != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ Loop start failed" << std::endl;
            return false;
        }

        // ✅ 연결 대기 (최대 5초)
        for (int i = 0; i < 50; i++) {
            if (connected) {
                std::cout << "✓ Connected to MQTT broker" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << "✗ Connection timeout (waited 5 seconds)" << std::endl;
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
            
            // ✅ 온보드 통신 구독 (RPi1 → RPi3)
            subscribe(nullptr, "delivery/start/+/1to3", 1);
            subscribe(nullptr, "delivery/vehicle/+/status", 0);
            subscribe(nullptr, "delivery/vehicle/+/alert", 1);
            subscribe(nullptr, "delivery/arrived/+/1to3", 1);
            subscribe(nullptr, "delivery/unlock/+", 1);
            subscribe(nullptr, "delivery/log/+", 1);
            subscribe(nullptr, "delivery/complete/+/1to3", 1);
            
            // ✅ 오프보드 통신 구독 (RPi4 → RPi3)
            subscribe(nullptr, "delivery/order/+", 1);
            subscribe(nullptr, "delivery/pin/+/4to3", 1);
            
            std::cout << "✓ Topics subscribed (9 topics)" << std::endl;
        } else {
            std::cerr << "✗ Connection failed: " << rc << std::endl;
            print_error_code(rc);
        }
    }

    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        
        std::cout << "\n[📨] " << topic << std::endl;
        
        try {
            auto data = json::parse(payload);
            
            // ✅ 온보드 메시지
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
            // ✅ 오프보드 메시지
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
    // ✅ 오류 코드 출력 함수
    void print_error_code(int rc) {
        switch(rc) {
            case 1: std::cerr << "  Error: Unacceptable protocol version" << std::endl; break;
            case 2: std::cerr << "  Error: Identifier rejected" << std::endl; break;
            case 3: std::cerr << "  Error: Server unavailable" << std::endl; break;
            case 4: std::cerr << "  Error: Bad username or password" << std::endl; break;
            case 5: std::cerr << "  Error: Not authorised" << std::endl; break;
            default: std::cerr << "  Error code: " << rc << std::endl;
        }
    }

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
        // ✅ SQL 인젝션 방지: prepared statement 사용 권장
        // 현재는 간단한 버전 (프로덕션에서는 prepared statement 필수)
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
            
            // 4. 차량에 PIN 전송 (delivery/pin/{id}/3to1)
            json pin_msg = {
                {"delivery_id", delivery_id},
                {"vehicle_id", vehicle_id},
                {"pin_onboard", pin_onboard},
                {"pin_offboard", pin_offboard}
            };
            publish_message("delivery/pin/" + delivery_id + "/3to1", pin_msg, 1);
            
            // 5. 이벤트 로깅
            log_event(vehicle_id, "order_received", delivery_id, "info");
            
            std::cout << "✓ Order received and PINs sent to vehicle" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_order: " << e.what() << std::endl;
        }
    }

    void handle_pin_from_customer(const json& data) {
        try {
            std::string delivery_id = data["delivery_id"].get<std::string>();
            std::string pin_code = data["pin_code"].get<std::string>();
            
            // ✅ Prepared statement 사용 (SQL 인젝션 방지)
            sqlite3_stmt* stmt = nullptr;
            std::string query = 
                "SELECT id, attempt_count, used FROM password_table "
                "WHERE delivery_id=? AND pin_code=? AND pin_type='offboard'";
            
            if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
                sqlite3_bind_text(stmt, 1, delivery_id.c_str(), -1, SQLITE_STATIC);
                sqlite3_bind_text(stmt, 2, pin_code.c_str(), -1, SQLITE_STATIC);
                
                if (sqlite3_step(stmt) == SQLITE_ROW) {
                    int attempt_count = sqlite3_column_int(stmt, 1);
                    int used = sqlite3_column_int(stmt, 2);
                    
                    // ✅ 이미 사용된 PIN 확인
                    if (used) {
                        log_event("", "pin_already_used", delivery_id, "warning");
                        std::cout << "⚠️  PIN already used" << std::endl;
                        sqlite3_finalize(stmt);
                        return;
                    }
                    
                    // ✅ 시도 횟수 확인
                    if (attempt_count >= 5) {
                        log_event("", "pin_max_attempts", delivery_id, "critical");
                        std::cout << "🔒 Max attempts reached!" << std::endl;
                        sqlite3_finalize(stmt);
                        return;
                    }
                    
                    // ✅ PIN 검증 성공
                    std::string update_query = 
                        "UPDATE password_table SET used=1, attempt_count=0 "
                        "WHERE delivery_id=? AND pin_code=?";
                    
                    sqlite3_stmt* update_stmt = nullptr;
                    if (sqlite3_prepare_v2(db, update_query.c_str(), -1, &update_stmt, nullptr) == SQLITE_OK) {
                        sqlite3_bind_text(update_stmt, 1, delivery_id.c_str(), -1, SQLITE_STATIC);
                        sqlite3_bind_text(update_stmt, 2, pin_code.c_str(), -1, SQLITE_STATIC);
                        sqlite3_step(update_stmt);
                        sqlite3_finalize(update_stmt);
                    }
                    
                    // 언락 신호 발행
                    json unlock_msg = {
                        {"delivery_id", delivery_id},
                        {"status", "unlocked"}
                    };
                    publish_message("delivery/unlock/" + delivery_id, unlock_msg, 1);
                    log_event("", "pin_accepted", delivery_id, "info");
                    std::cout << "✓ PIN accepted" << std::endl;
                } else {
                    // PIN 불일치
                    std::cout << "✗ PIN incorrect" << std::endl;
                    log_event("", "pin_rejected", delivery_id, "warning");
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
            
            std::cout << "  Delivery started: " << delivery_id << std::endl;
            
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

int main(int argc, char* argv[]) {
    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi3 Delivery Server (MQTT Communication)     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;
    
    // ✅ 명령줄 인자 또는 환경변수에서 호스트 가져오기
    std::string broker_host = "10.42.0.1";  // 기본값
    int broker_port = 1883;
    
    // 명령줄 인자 우선순위 높음
    if (argc > 1) {
        broker_host = argv[1];
        std::cout << "📝 Using broker from argument: " << broker_host << std::endl;
    } else {
        // 환경변수 확인
        const char* env_host = std::getenv("MQTT_BROKER_HOST");
        if (env_host) {
            broker_host = env_host;
            std::cout << "📝 Using broker from env MQTT_BROKER_HOST: " << broker_host << std::endl;
        } else {
            std::cout << "📝 Using default broker: " << broker_host << std::endl;
            std::cout << "   (Set MQTT_BROKER_HOST env var or pass as argument)" << std::endl;
        }
    }
    
    DeliveryServer server(broker_host, broker_port);
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
