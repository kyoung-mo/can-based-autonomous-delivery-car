#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <QString>
#include <QObject>
#include <QJsonObject>
#include <memory>
#include <mqtt/client.h>

/**
 * @class MQTTClient
 * @brief MQTT 브로커와 통신하는 클래스 (Qt6) - 수정 버전
 *
 * Paho C++ MQTT 라이브러리를 사용하여 비동기 통신
 * ✅ 인증 정보 지원
 * ✅ 동적 토픽 발행
 * ✅ 향상된 에러 처리
 */
class MQTTClient : public QObject {
    Q_OBJECT

public:
    /**
     * @brief MQTT 클라이언트 생성
     * @param broker_address 브로커 주소 (예: "10.42.0.1:1883")
     * @param username 인증 사용자명 (기본: "hoji")
     * @param password 인증 비밀번호 (기본: "1234")
     * @param parent 부모 객체
     */
    explicit MQTTClient(const QString& broker_address = "10.42.0.1:1883",
        const QString& username = "hoji",
        const QString& password = "1234",
        QObject* parent = nullptr);
    ~MQTTClient();

    // 연결 제어
    void connect();
    void disconnect();
    [[nodiscard]] bool is_connected() const;

    // MQTT 발행
    /**
     * @brief 주문 발행 (개선된 버전)
     * @param order_id 주문 ID (토픽에 포함됨)
     * @param menus 선택된 메뉴 목록
     * @param destination 목적지 (1-4)
     * @param pin PIN 코드 (4자리)
     */
    void publish_order(const QString& order_id,
        const QStringList& menus,
        int destination,
        const QString& pin);

    // 토픽 구독
    void subscribe_topics();

signals:
    // 신호 발신
    void shop_loaded(const QJsonObject& shop_data);
    void order_started(const QString& order_id);
    void order_arrived(const QString& order_id);
    void order_completed(const QString& order_id);
    void connection_changed(bool connected);
    void error_occurred(const QString& error_message);

private:
    /**
     * @class MQTTCallback
     * @brief Paho MQTT 콜백 핸들러
     */
    class MQTTCallback : public virtual mqtt::callback {
    public:
        explicit MQTTCallback(MQTTClient* parent);

        void connected(const std::string& cause) override;
        void connection_lost(const std::string& cause) override;
        void message_arrived(mqtt::const_message_ptr msg) override;
        void delivery_complete(mqtt::delivery_token_ptr token) override;

    private:
        MQTTClient* m_parent;
    };

    QString m_broker_address;
    QString m_username;  // ✅ 인증 정보 저장
    QString m_password;
    std::unique_ptr<mqtt::client> m_mqtt_client;
    std::unique_ptr<MQTTCallback> m_callback;
    bool m_connected = false;

    void process_message(const QString& topic, const QString& payload);
};

#endif // MQTT_CLIENT_H
