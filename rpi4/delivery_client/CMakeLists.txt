cmake_minimum_required(VERSION 3.21)
project(DeliveryClient CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS 
    Core 
    Gui 
    Widgets 
    Network
    REQUIRED
)

set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake" ${CMAKE_PREFIX_PATH})

set(SOURCES
    src/main.cpp
    src/order_window.cpp
    src/mqtt_client.cpp
)

set(HEADERS
    include/order_window.h
    include/mqtt_client.h
)

set(UI_FILES
    src/order_window.ui
)

# ✅ config.json 복사 추가
set(CONFIG_FILES
    config.json
)

add_executable(delivery_client
    ${SOURCES} 
    ${HEADERS}
    ${UI_FILES}
)

target_link_libraries(delivery_client
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    /usr/local/lib/libpaho-mqttpp3.so
    /usr/local/lib/libpaho-mqtt3as.so
)

target_include_directories(delivery_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include
)

# ✅ config.json 설치
install(TARGETS delivery_client DESTINATION /home/pi/delivery_system)
install(FILES ${CONFIG_FILES} DESTINATION /home/pi/delivery_system)
