#include "order_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QFont>
#include <QStyle>
#include <QApplication>

// Qt6 네임스페이스 추가
using namespace Qt::Literals::StringLiterals;

OrderWindow::OrderWindow(QWidget* parent)
    : QMainWindow(parent),
      m_mqtt_client(std::make_unique<MQTTClient>("127.0.0.1:1883"_L1, this)) {
    
    setWindowTitle(u"배달 주문 시스템 - RPi4 클라이언트"_s);
    setGeometry(100, 100, 600, 900);
    
    // 중앙 위젯 설정
    QWidget* central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    
    // 메인 레이아웃
    QVBoxLayout* main_layout = new QVBoxLayout(central_widget);
    main_layout->setContentsMargins(15, 15, 15, 15);
    main_layout->setSpacing(10);
    
    // UI 초기화
    init_ui();
    
    // 연결 설정
    create_connections();
    
    // MQTT 연결
    m_mqtt_client->connect();
}

void OrderWindow::init_ui() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    // ==================== 1. 헤더 영역 ====================
    QHBoxLayout* header_layout = new QHBoxLayout();
    
    m_shop_label = new QLabel(u" [가게명]"_s);
    QFont shop_font(u"Arial"_s, 18, QFont::Bold);
    m_shop_label->setFont(shop_font);
    
    m_connection_indicator = new QLabel(u" 연결 중..."_s);
    QFont indicator_font(u"Arial"_s, 10);
    m_connection_indicator->setFont(indicator_font);
    m_connection_indicator->setStyleSheet(u"color: #FF6B6B;"_s);
    m_connection_indicator->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    header_layout->addWidget(m_shop_label);
    header_layout->addStretch();
    header_layout->addWidget(m_connection_indicator);
    main_layout->addLayout(header_layout);
    
    // 구분선
    QFrame* line1 = new QFrame();
    line1->setFrameShape(QFrame::HLine);
    line1->setStyleSheet(u"color: #EEEEEE;"_s);
    main_layout->addWidget(line1);
    
    // ==================== 2. 메뉴 선택 ====================
    setup_menu_section();
    
    // 구분선
    QFrame* line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setStyleSheet(u"color: #EEEEEE;"_s);
    main_layout->addWidget(line2);
    
    // ==================== 3. 목적지 선택 ====================
    setup_destination_section();
    
    // 구분선
    QFrame* line3 = new QFrame();
    line3->setFrameShape(QFrame::HLine);
    line3->setStyleSheet(u"color: #EEEEEE;"_s);
    main_layout->addWidget(line3);
    
    // ==================== 4. PIN 입력 ====================
    setup_pin_section();
    
    // 구분선
    QFrame* line4 = new QFrame();
    line4->setFrameShape(QFrame::HLine);
    line4->setStyleSheet(u"color: #EEEEEE;"_s);
    main_layout->addWidget(line4);
    
    // ==================== 5. 버튼 영역 ====================
    setup_button_section();
    
    // 구분선
    QFrame* line5 = new QFrame();
    line5->setFrameShape(QFrame::HLine);
    line5->setStyleSheet(u"color: #EEEEEE;"_s);
    main_layout->addWidget(line5);
    
    // ==================== 6. 상태 표시 ====================
    setup_status_section();
    
    main_layout->addStretch();
}

void OrderWindow::setup_menu_section() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    QLabel* menu_title = new QLabel(u" 메뉴 선택"_s);
    QFont menu_font(u"Arial"_s, 14, QFont::Bold);
    menu_title->setFont(menu_font);
    main_layout->addWidget(menu_title);
    
    // 메뉴 데이터
    const struct Menu {
        const char* name;
        int price;
    } menus[] = {
        {" 비빔밥", 7000},
        {" 김밥", 5000},
        {" 떡볶이", 6000},
        {" 우동", 5500}
    };
    
    for (const auto& menu : menus) {
        QHBoxLayout* menu_hbox = new QHBoxLayout();
        
        QString menu_name = QString::fromUtf8(menu.name);
        QCheckBox* checkbox = new QCheckBox(menu_name);
        QFont checkbox_font(u"Arial"_s, 12);
        checkbox->setFont(checkbox_font);
        checkbox->setMinimumHeight(40);
        
        QLabel* price_label = new QLabel(QString::asprintf("₩%d", menu.price));
        price_label->setStyleSheet(u"color: #FF6B6B; font-weight: bold;"_s);
        price_label->setMinimumWidth(80);
        
        m_menu_checkboxes[menu_name] = checkbox;
        m_menu_prices[menu_name] = menu.price;
        
        connect(checkbox, &QCheckBox::toggled, this, [this, menu_name](bool checked) {
            on_menu_checkbox_toggled(menu_name, checked);
        });
        
        menu_hbox->addWidget(checkbox);
        menu_hbox->addStretch();
        menu_hbox->addWidget(price_label);
        main_layout->addLayout(menu_hbox);
    }
}

void OrderWindow::setup_destination_section() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    QLabel* dest_title = new QLabel(u" 목적지 선택"_s);
    QFont dest_font(u"Arial"_s, 14, QFont::Bold);
    dest_title->setFont(dest_font);
    main_layout->addWidget(dest_title);
    
    QHBoxLayout* dest_hbox = new QHBoxLayout();
    for (int i = 1; i <= 4; ++i) {
        QPushButton* btn = new QPushButton(QString::number(i));
        btn->setFixedSize(70, 70);
        QFont btn_font(u"Arial"_s, 16, QFont::Bold);
        btn->setFont(btn_font);
        btn->setStyleSheet(u"background-color: #F5F5F5; border-radius: 8px; border: 1px solid #E0E0E0;"_s);
        btn->setCursor(Qt::PointingHandCursor);
        
        m_destination_buttons[i] = btn;
        
        connect(btn, &QPushButton::clicked, this, [this, i]() {
            on_destination_button_clicked(i);
        });
        
        dest_hbox->addWidget(btn);
    }
    dest_hbox->addStretch();
    main_layout->addLayout(dest_hbox);
    
    m_selected_destination_label = new QLabel(u"목적지를 선택하세요"_s);
    m_selected_destination_label->setStyleSheet(u"color: #999; font-size: 11px;"_s);
    main_layout->addWidget(m_selected_destination_label);
}

void OrderWindow::setup_pin_section() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    QLabel* pin_title = new QLabel(u" PIN 코드 (4자리)"_s);
    QFont pin_font(u"Arial"_s, 14, QFont::Bold);
    pin_title->setFont(pin_font);
    main_layout->addWidget(pin_title);
    
    m_pin_input = new QLineEdit();
    m_pin_input->setPlaceholderText(u"____"_s);
    m_pin_input->setMaxLength(4);
    m_pin_input->setEchoMode(QLineEdit::Password);
    m_pin_input->setAlignment(Qt::AlignCenter);
    QFont input_font(u"Arial"_s, 24);
    m_pin_input->setFont(input_font);
    m_pin_input->setFixedHeight(60);
    m_pin_input->setStyleSheet(u"border: 2px solid #E0E0E0; border-radius: 5px;"_s);
    
    // Qt6: InputMethodHints 설정
    m_pin_input->setInputMethodHints(Qt::ImhDigitsOnly);
    
    main_layout->addWidget(m_pin_input);
}

void OrderWindow::setup_button_section() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    QHBoxLayout* button_hbox = new QHBoxLayout();
    
    m_order_button = new QPushButton(u"✓ 주문하기"_s);
    QFont order_font(u"Arial"_s, 14, QFont::Bold);
    m_order_button->setFont(order_font);
    m_order_button->setFixedHeight(55);
    m_order_button->setStyleSheet(
        u"background-color: #4CAF50; color: white; border: none; border-radius: 5px; "
        u"font-weight: bold; padding: 10px;"_s
    );
    m_order_button->setCursor(Qt::PointingHandCursor);
    
    connect(m_order_button, &QPushButton::clicked, this, &OrderWindow::on_order_button_clicked);
    
    m_reset_button = new QPushButton(u"↺ 초기화"_s);
    m_reset_button->setFont(order_font);
    m_reset_button->setFixedHeight(55);
    m_reset_button->setStyleSheet(
        u"background-color: #CCCCCC; color: #333; border: none; border-radius: 5px; "
        u"font-weight: bold; padding: 10px;"_s
    );
    m_reset_button->setCursor(Qt::PointingHandCursor);
    
    connect(m_reset_button, &QPushButton::clicked, this, &OrderWindow::on_reset_button_clicked);
    
    button_hbox->addWidget(m_order_button);
    button_hbox->addWidget(m_reset_button);
    main_layout->addLayout(button_hbox);
}

void OrderWindow::setup_status_section() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    m_order_info_label = new QLabel();
    QFont info_font(u"Arial"_s, 11);
    m_order_info_label->setFont(info_font);
    m_order_info_label->setWordWrap(true);
    main_layout->addWidget(m_order_info_label);
    
    m_status_label = new QLabel(u"⏳ 대기 중..."_s);
    QFont status_font(u"Arial"_s, 12);
    m_status_label->setFont(status_font);
    m_status_label->setWordWrap(true);
    m_status_label->setMinimumHeight(40);
    main_layout->addWidget(m_status_label);
}

void OrderWindow::create_connections() {
    // MQTT 신호 연결
    connect(m_mqtt_client.get(), &MQTTClient::shop_loaded,
            this, &OrderWindow::on_shop_loaded);
    connect(m_mqtt_client.get(), &MQTTClient::order_started,
            this, &OrderWindow::on_order_started);
    connect(m_mqtt_client.get(), &MQTTClient::order_arrived,
            this, &OrderWindow::on_order_arrived);
    connect(m_mqtt_client.get(), &MQTTClient::order_completed,
            this, &OrderWindow::on_order_completed);
    connect(m_mqtt_client.get(), &MQTTClient::connection_changed,
            this, &OrderWindow::on_connection_changed);
    connect(m_mqtt_client.get(), &MQTTClient::error_occurred,
            this, &OrderWindow::on_mqtt_error);
}

void OrderWindow::on_menu_checkbox_toggled(const QString& menu_name, bool checked) {
    if (checked) {
        if (!m_selected_menus.contains(menu_name)) {
            m_selected_menus.append(menu_name);
        }
    } else {
        m_selected_menus.removeAll(menu_name);
    }
    
    qDebug() << u"선택된 메뉴:"_s << m_selected_menus;
}

void OrderWindow::on_destination_button_clicked(int destination) {
    m_selected_destination = destination;
    m_selected_destination_label->setText(
        QString(u"✓ 목적지 %1 선택됨"_s).arg(destination)
    );
    
    // 모든 버튼 스타일 업데이트
    for (auto [num, btn] : m_destination_buttons.asKeyValueRange()) {
        if (num == destination) {
            btn->setStyleSheet(
                u"background-color: #2196F3; color: white; border: none; "
                u"border-radius: 8px; font-weight: bold;"_s
            );
        } else {
            btn->setStyleSheet(
                u"background-color: #F5F5F5; border-radius: 8px; border: 1px solid #E0E0E0;"_s
            );
        }
    }
    
    update_status_message(QString(u"✓ 목적지 %1 선택됨"_s).arg(destination));
}

void OrderWindow::on_order_button_clicked() {
    validate_and_submit_order();
}

void OrderWindow::on_reset_button_clicked() {
    reset_form();
    update_status_message(u"초기화되었습니다"_s);
}

void OrderWindow::validate_and_submit_order() {
    if (m_selected_menus.isEmpty()) {
        update_status_message(u"❌ 메뉴를 선택하세요!"_s, u"#D32F2F"_s);
        return;
    }
    
    if (m_selected_destination == -1) {
        update_status_message(u"❌ 목적지를 선택하세요!"_s, u"#D32F2F"_s);
        return;
    }
    
    QString pin = m_pin_input->text();
    if (pin.isEmpty()) {
        update_status_message(u"❌ PIN을 입력하세요!"_s, u"#D32F2F"_s);
        return;
    }
    
    if (pin.length() != 4 || !pin.at(0).isDigit()) {
        update_status_message(u"❌ PIN은 4자리 숫자여야 합니다!"_s, u"#D32F2F"_s);
        return;
    }
    
    m_last_order_id = QString(u"ORD%1"_s).arg(m_order_counter, 3, 10, QChar('0'));
    m_mqtt_client->publish_order(m_last_order_id, m_selected_menus, m_selected_destination, pin);
    
    m_order_counter++;
    
    m_order_info_label->setText(
        QString(u"주문 번호: <b>%1</b> | 주문 상태: 처리 중..."_s).arg(m_last_order_id)
    );
    m_order_info_label->setStyleSheet(u"color: #388E3C;"_s);
    
    update_status_message(
        QString(u"✅ 주문 %1 제출됨! 배달을 기다리세요..."_s).arg(m_last_order_id),
        u"#388E3C"_s
    );
    
    reset_form();
}

void OrderWindow::reset_form() {
    for (auto [menu_name, checkbox] : m_menu_checkboxes.asKeyValueRange()) {
        checkbox->setChecked(false);
    }
    
    for (auto [num, btn] : m_destination_buttons.asKeyValueRange()) {
        btn->setStyleSheet(
            u"background-color: #F5F5F5; border-radius: 8px; border: 1px solid #E0E0E0;"_s
        );
    }
    
    m_pin_input->clear();
    m_selected_menus.clear();
    m_selected_destination = -1;
    m_selected_destination_label->setText(u"목적지를 선택하세요"_s);
}

void OrderWindow::update_status_message(const QString& message, const QString& color) {
    m_status_label->setText(message);
    m_status_label->setStyleSheet(QString(u"color: %1;"_s).arg(color));
    qDebug() << message;
}

void OrderWindow::on_shop_loaded(const QJsonObject& shop_data) {
    QString shop_name = shop_data[u"shop_name"_s].toString(u"우리 가게"_s);
    m_shop_label->setText(QString(u" %1"_s).arg(shop_name));
    
    update_status_message(u"✓ 가게 정보 로드 완료"_s, u"#388E3C"_s);
}

void OrderWindow::on_order_started(const QString& order_id) {
    if (order_id == m_last_order_id) {
        update_status_message(
            QString(u" [%1] 배달 시작됨!"_s).arg(order_id),
            u"#F57C00"_s
        );
        m_order_info_label->setText(
            QString(u"주문 번호: <b>%1</b> | 주문 상태: 배달 중..."_s).arg(order_id)
        );
        m_order_info_label->setStyleSheet(u"color: #F57C00;"_s);
    }
}

void OrderWindow::on_order_arrived(const QString& order_id) {
    if (order_id == m_last_order_id) {
        update_status_message(
            QString(u" [%1] 목적지에 도착했습니다!"_s).arg(order_id),
            u"#1976D2"_s
        );
        m_order_info_label->setText(
            QString(u"주문 번호: <b>%1</b> | 주문 상태: 목적지 도착"_s).arg(order_id)
        );
        m_order_info_label->setStyleSheet(u"color: #1976D2;"_s);
    }
}

void OrderWindow::on_order_completed(const QString& order_id) {
    if (order_id == m_last_order_id) {
        update_status_message(
            QString(u"✅ [%1] 배달 완료!"_s).arg(order_id),
            u"#388E3C"_s
        );
        m_order_info_label->setText(
            QString(u"주문 번호: <b>%1</b> | 주문 상태: 완료"_s).arg(order_id)
        );
        m_order_info_label->setStyleSheet(u"color: #388E3C;"_s);
    }
}

void OrderWindow::on_connection_changed(bool connected) {
    if (connected) {
        m_connection_indicator->setText(u" 연결됨"_s);
        m_connection_indicator->setStyleSheet(u"color: #4CAF50;"_s);
    } else {
        m_connection_indicator->setText(u" 연결 끊김"_s);
        m_connection_indicator->setStyleSheet(u"color: #D32F2F;"_s);
    }
}

void OrderWindow::on_mqtt_error(const QString& error_message) {
    update_status_message(QString(u"⚠️ 오류: %1"_s).arg(error_message), u"#D32F2F"_s);
}

void OrderWindow::closeEvent(QCloseEvent* event) {
    m_mqtt_client->disconnect();
    QMainWindow::closeEvent(event);
}

void OrderWindow::changeEvent(QEvent* event) {
    if (event->type() == QEvent::LanguageChange) {
        // Qt6: 언어 변경 처리
    }
    QMainWindow::changeEvent(event);
}
