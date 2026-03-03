#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <QString>
#include <QObject>
#include <QJsonObject>
#include <memory>
#include <mqtt/client.h>

/**
 * @class MQTTClient
 * @brief MQTT 브로커와 통신하는 클래스 (Qt6)
 * 
 * Paho C++ MQTT 라이브러리를 사용하여 비동기 통신
 * 모든 MQTT 이벤트를 Qt Signal로 변환
 */
class MQTTClient : public QObject {
    Q_OBJECT

public:
    /**
     * @brief MQTT 클라이언트 생성
     * @param broker_address 브로커 주소 (예: "127.0.0.1:1883")
     * @param parent 부모 객체
     */
    explicit MQTTClient(const QString& broker_address = "127.0.0.1:1883", 
                        QObject* parent = nullptr);
    ~MQTTClient();

    // 연결 제어
    void connect();
    void disconnect();
    [[nodiscard]] bool is_connected() const;

    // MQTT 발행
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
    std::unique_ptr<mqtt::client> m_mqtt_client;
    std::unique_ptr<MQTTCallback> m_callback;
    bool m_connected = false;

    void process_message(const QString& topic, const QString& payload);
};

#endif // MQTT_CLIENT_H
