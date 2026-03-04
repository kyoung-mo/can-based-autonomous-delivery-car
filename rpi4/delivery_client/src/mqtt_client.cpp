#include "mqtt_client.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QDebug>
#include <QTimer>
#include <thread>
#include <mqtt/async_client.h>

// ==================== MQTTCallback 구현 ====================

MQTTClient::MQTTCallback::MQTTCallback(MQTTClient* parent) 
    : m_parent(parent) {}

void MQTTClient::MQTTCallback::connected(const std::string& cause) {
    qDebug() << "✓ MQTT 브로커 연결됨";
    if (m_parent) {
        m_parent->m_connected = true;
        m_parent->subscribe_topics();
        emit m_parent->connection_changed(true);
    }
}

void MQTTClient::MQTTCallback::connection_lost(const std::string& cause) {
    qWarning() << "✗ MQTT 연결 끊김:" << QString::fromStdString(cause);
    if (m_parent) {
        m_parent->m_connected = false;
        emit m_parent->connection_changed(false);
    }
}

void MQTTClient::MQTTCallback::message_arrived(mqtt::const_message_ptr msg) {
    if (!m_parent) return;
    
    try {
        QString topic = QString::fromStdString(msg->get_topic());
        QString message = QString::fromStdString(msg->get_payload());
        
        qDebug() << "📨 MQTT 메시지 수신:" << topic;
        m_parent->process_message(topic, message);
    } catch (const std::exception& e) {
        qWarning() << "✗ 메시지 처리 오류:" << e.what();
    }
}

void MQTTClient::MQTTCallback::delivery_complete(mqtt::delivery_token_ptr token) {
    qDebug() << "✓ 메시지 전송 완료";
}

// ==================== MQTTClient 구현 ====================

MQTTClient::MQTTClient(const QString& broker_address, 
                       const QString& username,
                       const QString& password,
                       QObject* parent)
    : QObject(parent), 
      m_broker_address(broker_address),
      m_username(username),
      m_password(password) {
    
    std::string broker_str = broker_address.toStdString();
    std::string client_id = "rpi4_client_qt6";
    
    try {
        m_mqtt_client = std::make_unique<mqtt::client>(broker_str, client_id);
        m_callback = std::make_unique<MQTTCallback>(this);
        m_mqtt_client->set_callback(*m_callback);
        qDebug() << "✓ MQTT 클라이언트 생성 (Qt6):" << broker_address;
        qDebug() << "  사용자명: " << username;
    } catch (const std::exception& e) {
        qCritical() << "✗ MQTT 클라이언트 생성 실패:" << e.what();
        emit error_occurred(QString("MQTT 클라이언트 생성 실패: %1").arg(e.what()));
    }
}

MQTTClient::~MQTTClient() {
    if (m_connected) {
        try {
            disconnect();
        } catch (...) {
            // 소멸자에서 예외 무시
        }
    }
}

// ✅ 개선된 connect() 함수: 별도 스레드에서 연결
void MQTTClient::connect() {
    if (!m_mqtt_client) return;
    
    // ✅ 별도의 스레드에서 연결 시도 (GUI 블로킹 방지)
    std::thread connection_thread([this]() {
        try {
            mqtt::connect_options conn_opts;
            conn_opts.set_clean_session(true);
            conn_opts.set_automatic_reconnect(1, 30);
            conn_opts.set_connect_timeout(5);  // ✅ 5초 타임아웃
            
            // ✅ 상세한 디버그 로그
            qDebug() << "════════════════════════════════════════════";
            qDebug() << "🔗 MQTT 브로커에 연결 시도 (별도 스레드)";
            qDebug() << "   브로커 주소:" << m_broker_address;
            qDebug() << "   사용자명:" << m_username;
            qDebug() << "   타임아웃: 5초";
            qDebug() << "════════════════════════════════════════════";
            
            // ✅ 인증 정보 설정
            conn_opts.set_user_name(m_username.toStdString());
            conn_opts.set_password(m_password.toStdString());
            
            qDebug() << "⏳ 연결 중...";
            m_mqtt_client->connect(conn_opts);
            qDebug() << "✓ MQTT 연결 함수 호출됨 (콜백 대기 중)";
            
        } catch (const mqtt::exception& e) {
            qCritical() << "✗✗✗ MQTT 연결 실패 ✗✗✗";
            qCritical() << "   오류:" << e.what();
            qCritical() << "   오류 코드:" << e.get_reason_code();
            
            // ✅ 메인 스레드로 신호 전송
            emit error_occurred(QString("MQTT 연결 실패: %1").arg(e.what()));
        }
    });
    
    // ✅ 스레드를 분리하여 백그라운드에서 실행
    connection_thread.detach();
}

void MQTTClient::disconnect() {
    if (!m_mqtt_client) return;
    
    try {
        if (m_connected) {
            m_mqtt_client->disconnect();
            m_connected = false;
            qDebug() << "✓ MQTT 연결 해제";
        }
    } catch (const mqtt::exception& e) {
        qWarning() << "✗ MQTT 연결 해제 오류:" << e.what();
    }
}

bool MQTTClient::is_connected() const {
    return m_connected;
}

void MQTTClient::subscribe_topics() {
    if (!m_mqtt_client || !m_connected) return;
    
    try {
        const int qos = 1;
        
        // ✅ 배송 상태 업데이트 구독 (RPi3 → RPi4)
        m_mqtt_client->subscribe("delivery/start/+/3to4", qos);
        m_mqtt_client->subscribe("delivery/arrived/+/3to4", qos);
        m_mqtt_client->subscribe("delivery/complete/+/3to4", qos);
        
        // ✅ 메뉴 데이터 구독
        m_mqtt_client->subscribe("shop/menu", qos);
        
        // ✅ 오류 응답 구독 (선택사항)
        m_mqtt_client->subscribe("delivery/error/+/4to3", qos);
        
        qDebug() << "✓ MQTT 토픽 구독 완료 (5개 토픽)";
    } catch (const mqtt::exception& e) {
        qWarning() << "✗ 구독 실패:" << e.what();
    }
}

void MQTTClient::publish_order(const QString& order_id,
                               const QStringList& menus,
                               int destination,
                               const QString& pin) {
    if (!m_mqtt_client || !m_connected) {
        emit error_occurred("MQTT 브로커에 연결되지 않았습니다");
        return;
    }
    
    try {
        // JSON 객체 생성
        QJsonObject order_obj;
        order_obj["order_id"] = order_id;
        order_obj["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
        order_obj["destination"] = destination;  // ✅ destination 추가
        
        // 메뉴 배열
        QJsonArray menus_array;
        for (const QString& menu : menus) {
            menus_array.append(menu);
        }
        order_obj["menus"] = menus_array;
        order_obj["pin"] = pin;
        
        // JSON 문서로 변환
        QJsonDocument doc(order_obj);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        
        // ✅ 동적 토픽: order_id를 포함
        std::string topic = "delivery/order/" + order_id.toStdString();
        
        auto msg = mqtt::make_message(topic, payload.toStdString());
        msg->set_qos(1);
        m_mqtt_client->publish(msg);
        
        qDebug() << "✓ 주문 발행:" << order_id;
        qDebug() << "  토픽:" << QString::fromStdString(topic);
        qDebug() << "  메뉴:" << menus;
        qDebug() << "  목적지:" << destination;
    } catch (const std::exception& e) {
        qCritical() << "✗ 주문 발행 실패:" << e.what();
        emit error_occurred(QString("주문 발행 실패: %1").arg(e.what()));
    }
}

void MQTTClient::process_message(const QString& topic, const QString& payload) {
    try {
        QJsonDocument doc = QJsonDocument::fromJson(payload.toUtf8());
        
        // ✅ JSON 유효성 검사
        if (!doc.isObject()) {
            qWarning() << "✗ 유효하지 않은 JSON:" << payload;
            return;
        }
        
        QJsonObject obj = doc.object();
        
        // ✅ 토픽별 메시지 처리
        if (topic == "shop/menu") {
            emit shop_loaded(obj);
            qDebug() << "  ✓ 메뉴 정보 로드됨";
        }
        else if (topic.contains("delivery/start/")) {
            // 토픽 형식: delivery/start/{order_id}/3to4
            QStringList parts = topic.split("/");
            if (parts.size() >= 3) {
                QString order_id = parts[2];
                emit order_started(order_id);
                qDebug() << "  ✓ 배송 시작:" << order_id;
            }
        }
        else if (topic.contains("delivery/arrived/")) {
            QStringList parts = topic.split("/");
            if (parts.size() >= 3) {
                QString order_id = parts[2];
                emit order_arrived(order_id);
                qDebug() << "  ✓ 도착:" << order_id;
            }
        }
        else if (topic.contains("delivery/complete/")) {
            QStringList parts = topic.split("/");
            if (parts.size() >= 3) {
                QString order_id = parts[2];
                emit order_completed(order_id);
                qDebug() << "  ✓ 완료:" << order_id;
            }
        }
        else if (topic.contains("delivery/error/")) {
            QString error_msg = obj["error"].toString("알 수 없는 오류");
            qWarning() << "  ⚠️ 오류:" << error_msg;
        }
        else {
            qDebug() << "  ⚠️ 알 수 없는 토픽:" << topic;
        }
    } catch (const std::exception& e) {
        qWarning() << "✗ 메시지 파싱 오류:" << e.what() << "페이로드:" << payload;
    }
}
