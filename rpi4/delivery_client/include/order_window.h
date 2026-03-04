#ifndef ORDER_WINDOW_H
#define ORDER_WINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QMap>
#include <memory>
#include "mqtt_client.h"

/**
 * @class OrderWindow
 * @brief RPi4 배달 주문 인터페이스 (Qt6)
 *
 * 메뉴 선택, 목적지 선택, PIN 입력, 주문 제출 기능 제공
 * MQTT를 통해 배달 상태 추적
 */
class OrderWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit OrderWindow(QWidget* parent = nullptr);
    ~OrderWindow() = default;

protected:
    void closeEvent(QCloseEvent* event) override;
    void changeEvent(QEvent* event) override;

private slots:
    // UI 이벤트 핸들러
    void on_menu_checkbox_toggled(const QString& menu_name, bool checked);
    void on_destination_button_clicked(int destination);
    void on_order_button_clicked();
    void on_reset_button_clicked();

    // MQTT 신호 처리
    void on_shop_loaded(const QJsonObject& shop_data);
    void on_order_started(const QString& order_id);
    void on_order_arrived(const QString& order_id);
    void on_order_completed(const QString& order_id);
    void on_connection_changed(bool connected);
    void on_mqtt_error(const QString& error_message);

private:
    // UI 초기화
    void init_ui();
    void setup_menu_section();
    void setup_destination_section();
    void setup_pin_section();
    void setup_button_section();
    void setup_status_section();
    void create_connections();

    // 비즈니스 로직
    void validate_and_submit_order();
    void reset_form();
    void update_status_message(const QString& message, const QString& color = "#333333");

    // UI 컴포넌트 - 헤더
    QLabel* m_shop_label = nullptr;
    QLabel* m_connection_indicator = nullptr;

    // UI 컴포넌트 - 메뉴
    QMap<QString, QCheckBox*> m_menu_checkboxes;
    QMap<QString, int> m_menu_prices;

    // UI 컴포넌트 - 목적지
    QMap<int, QPushButton*> m_destination_buttons;
    QLabel* m_selected_destination_label = nullptr;

    // UI 컴포넌트 - PIN
    QLineEdit* m_pin_input = nullptr;

    // UI 컴포넌트 - 버튼
    QPushButton* m_order_button = nullptr;
    QPushButton* m_reset_button = nullptr;

    // UI 컴포넌트 - 상태
    QLabel* m_status_label = nullptr;
    QLabel* m_order_info_label = nullptr;

    // 상태 변수
    QStringList m_selected_menus;
    int m_selected_destination = -1;
    int m_order_counter = 1;

    // MQTT 클라이언트
    std::unique_ptr<MQTTClient> m_mqtt_client;
    QString m_last_order_id;
};

#endif // ORDER_WINDOW_H

