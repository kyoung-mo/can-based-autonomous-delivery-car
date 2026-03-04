#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include <QColor>
#include "order_window.h"

// Qt6 문자열 리터럴 연산자 사용을 위한 네임스페이스
using namespace Qt::Literals::StringLiterals;

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    // Qt6 스타일 설정
    app.setStyle(QStyleFactory::create(u"Fusion"_s));

    // 팔레트 설정 (다크 모드 대신 라이트 모드)
    QPalette dark_palette;
    dark_palette.setColor(QPalette::Window, QColor(245, 245, 245));
    dark_palette.setColor(QPalette::WindowText, QColor(0, 0, 0));
    app.setPalette(dark_palette);

    // 윈도우 생성 및 표시
    OrderWindow window;
    window.show();

    return app.exec();
}
