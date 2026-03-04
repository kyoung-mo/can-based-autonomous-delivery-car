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
#include <QJsonDocument>
#include <QFile>
#include <QDebug>
#include <QFont>
#include <QStyle>
#include <QApplication>
#include <QStandardPaths>
#include <QTimer>
#include <thread>  // ✅ 추가: 스레드 사용

// Qt6 네임스페이스 추가
using namespace Qt::Literals::StringLiterals;

// ✅ 설정 파일 로드 헬퍼 함수
QString load_config_value(const QString& key, const QString& default_value) {
    QString config_path = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QString config_file_path = config_path + "/config.json";
    
    // config.json이 없으면 현재 디렉토리에서 찾기
    QFile config_file(config_file_path);
    if (!config_file.exists()) {
        config_file.setFileName("config.json");
    }
    
    if (config_file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(config_file.readAll());
        QJsonObject obj = doc.object();
        config_file.close();
        
        // key는 dot notation 형식 (예: "mqtt.broker_host")
        QStringList keys = key.split(".");
        QJsonValue value = obj;
        
        for (const QString& k : keys) {
            if (value.isObject()) {
                value = value.toObject()[k];
            } else {
                return default_value;
            }
        }
        
        if (value.isString()) {
            return value.toString();
        }
    }
    
    return default_value;
}

// ==================== OrderWindow 생성자 ====================

OrderWindow::OrderWindow(QWidget* parent)
    : QMainWindow(parent) {
    
    // ✅ config.json에서 MQTT 설정 로드
    QString broker_host = load_config_value(u"mqtt.broker_host"_s, u"10.42.0.1"_s);
    QString broker_port = load_config_value(u"mqtt.broker_port"_s, u"1883"_s);
    QString username = load_config_value(u"mqtt.username"_s, u"hoji"_s);
    QString password = load_config_value(u"mqtt.password"_s, u"1234"_s);
    
    // 브로커 주소 생성
    QString broker_address = QString(u"%1:%2"_s).arg(broker_host, broker_port);
    
    // ✅ 개선된 MQTT 클라이언트 생성 (인증 정보 포함)
    m_mqtt_client = std::make_unique<MQTTClient>(
        broker_address,  // 10.42.0.1:1883
        username,        // hoji
        password,        // 1234
        this);
    
    qDebug() << "✓ MQTT 클라이언트 생성됨";
    qDebug() << "  브로커:" << broker_host << ":" << broker_port;
    qDebug() << "  사용자명:" << username;
    
    // 윈도우 설정
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
    
    // ✅ 지연된 연결 (윈도우 표시 후 MQTT 연결)
    QTimer::singleShot(500, [this]() {
        m_mqtt_client->connect();
    });
    
    qDebug() << "✓ OrderWindow 초기화 완료";
}

// ==================== UI 초기화 ====================

void OrderWindow::init_ui() {
    QWidget* central = qobject_cast<QWidget*>(centralWidget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(central->layout());
    
    // ==================== 1. 헤더 영역 ====================
    QHBoxLayout* header_layout = new QHBoxLayout();
    
    m_shop_label = new QLabel(u" [가게명]"_s);
    QFont shop_font(u"Arial"_s, 18, QFont::Bold);
    m_shop_label->setFont(shop_font);

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

// ==================== 섹션 설정 ====================

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

// ==================== 신호 연결 ====================

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

// ==================== 슬롯 함수 ====================

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

// ==================== 주문 검증 및 제출 ====================

void OrderWindow::validate_and_submit_order() {
    // 메뉴 선택 확인
    if (m_selected_menus.isEmpty()) {
        update_status_message(u"❌ 메뉴를 선택하세요!"_s, u"#D32F2F"_s);
        qDebug() << "✗ 메뉴 미선택";
        return;
    }
    
    // 목적지 선택 확인
    if (m_selected_destination == -1) {
        update_status_message(u"❌ 목적지를 선택하세요!"_s, u"#D32F2F"_s);
        qDebug() << "✗ 목적지 미선택";
        return;
    }
    
    // PIN 입력 확인
    QString pin = m_pin_input->text();
    if (pin.isEmpty()) {
        update_status_message(u"❌ PIN을 입력하세요!"_s, u"#D32F2F"_s);
        qDebug() << "✗ PIN 미입력";
        return;
    }
    
    // PIN 형식 확인 (4자리 숫자)
    if (pin.length() != 4 || !pin.at(0).isDigit()) {
        update_status_message(u"❌ PIN은 4자리 숫자여야 합니다!"_s, u"#D32F2F"_s);
        qDebug() << "✗ PIN 형식 오류:" << pin;
        return;
    }
    
    // 주문 ID 생성 (ORD001, ORD002, ...)
    m_last_order_id = QString(u"ORD%1"_s).arg(m_order_counter, 3, 10, QChar('0'));
    
    // ✅ UI 먼저 업데이트 (즉시 표시)
    m_order_info_label->setText(
        QString(u"주문 번호: <b>%1</b> | 주문 상태: 처리 중..."_s).arg(m_last_order_id)
    );
    m_order_info_label->setStyleSheet(u"color: #388E3C;"_s);
    
    update_status_message(
        QString(u"✅ 주문 %1 제출됨! 배달을 기다리세요..."_s).arg(m_last_order_id),
        u"#388E3C"_s
    );
    
    qDebug() << "✓ 주문 제출됨:" << m_last_order_id;
    qDebug() << "  메뉴:" << m_selected_menus;
    qDebug() << "  목적지:" << m_selected_destination;
    qDebug() << "  PIN:" << "****" << "(숨김)";
    
    m_order_counter++;
    
    // ✅ MQTT 발행을 별도 스레드에서 실행 (메인 스레드 블로킹 방지)
    std::thread publish_thread([this]() {
        try {
            qDebug() << "📡 별도 스레드에서 주문 발행 중...";
            m_mqtt_client->publish_order(
                m_last_order_id,
                m_selected_menus,
                m_selected_destination,
                m_pin_input->text()
            );
            qDebug() << "✓ 주문 발행 완료";
        } catch (const std::exception& e) {
            qCritical() << "✗ 주문 발행 실패:" << e.what();
        }
    });
    publish_thread.detach();  // 백그라운드에서 실행
    
    // ✅ 폼 초기화 (MQTT 발행과 동시에 진행)
    reset_form();
}

// ==================== 폼 초기화 ====================

void OrderWindow::reset_form() {
    // 메뉴 체크박스 해제
    for (auto [menu_name, checkbox] : m_menu_checkboxes.asKeyValueRange()) {
        checkbox->setChecked(false);
    }
    
    // 목적지 버튼 스타일 리셋
    for (auto [num, btn] : m_destination_buttons.asKeyValueRange()) {
        btn->setStyleSheet(
            u"background-color: #F5F5F5; border-radius: 8px; border: 1px solid #E0E0E0;"_s
        );
    }
    
    // 입력 필드 초기화
    m_pin_input->clear();
    m_selected_menus.clear();
    m_selected_destination = -1;
    m_selected_destination_label->setText(u"목적지를 선택하세요"_s);
    
    qDebug() << "✓ 폼 초기화 완료";
}

// ==================== 상태 메시지 ====================

void OrderWindow::update_status_message(const QString& message, const QString& color) {
    m_status_label->setText(message);
    m_status_label->setStyleSheet(QString(u"color: %1;"_s).arg(color));
    qDebug() << message;
}

// ==================== MQTT 이벤트 핸들러 ====================

void OrderWindow::on_shop_loaded(const QJsonObject& shop_data) {
    QString shop_name = shop_data[u"shop_name"_s].toString(u"우리 가게"_s);
    m_shop_label->setText(QString(u" %1"_s).arg(shop_name));
    
    update_status_message(u"✓ 가게 정보 로드 완료"_s, u"#388E3C"_s);
    qDebug() << "✓ 가게 정보 로드됨:" << shop_name;
}

void OrderWindow::on_order_started(const QString& order_id) {
    // 해당 주문의 상태 업데이트만 수행
    if (order_id == m_last_order_id) {
        update_status_message(
            QString(u" [%1] 배달 시작됨!"_s).arg(order_id),
            u"#F57C00"_s
        );
        m_order_info_label->setText(
            QString(u"주문 번호: <b>%1</b> | 주문 상태: 배달 중..."_s).arg(order_id)
        );
        m_order_info_label->setStyleSheet(u"color: #F57C00;"_s);
        
        qDebug() << "✓ 배송 시작됨:" << order_id;
    } else {
        qDebug() << "ℹ️ 다른 주문의 배송 시작:" << order_id << "(현재 주문:" << m_last_order_id << ")";
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
        
        qDebug() << "✓ 목적지 도착:" << order_id;
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
        
        qDebug() << "✓ 배달 완료:" << order_id;
    }
}

void OrderWindow::on_connection_changed(bool connected) {
    if (connected) {
        m_connection_indicator->setText(u" 연결됨"_s);
        m_connection_indicator->setStyleSheet(u"color: #4CAF50;"_s);
        qDebug() << "✓ MQTT 연결됨";
    } else {
        m_connection_indicator->setText(u" 연결 끊김"_s);
        m_connection_indicator->setStyleSheet(u"color: #D32F2F;"_s);
        qDebug() << "✗ MQTT 연결 끊김";
    }
}

void OrderWindow::on_mqtt_error(const QString& error_message) {
    update_status_message(QString(u"⚠️ 오류: %1"_s).arg(error_message), u"#D32F2F"_s);
    qCritical() << "✗ MQTT 오류:" << error_message;
}

// ==================== 윈도우 이벤트 ====================

void OrderWindow::closeEvent(QCloseEvent* event) {
    qDebug() << "⏹️  애플리케이션 종료 중...";
    m_mqtt_client->disconnect();
    QMainWindow::closeEvent(event);
}

void OrderWindow::changeEvent(QEvent* event) {
    if (event->type() == QEvent::LanguageChange) {
        // Qt6: 언어 변경 처리
        qDebug() << "ℹ️ 언어 변경됨";
    }
    QMainWindow::changeEvent(event);
}
