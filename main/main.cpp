#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"

#define TAG "Application"
#define ROOT "/mnt"

Application app;

// メッセージ種別 (メッセージキュー用)
enum class AppMessage {
    UpdateDisplay,      // ディスプレイに現在状態表示
    WIFIConnection,     // Wi-Fi接続
    WIFIDisconnection,  // Wi-Fi切断
    Quit                // 終了
};

Application::Application() {
    m_LedState = false;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_isWiFi = false;
    m_30sec_off = false;
}

// 初期化
void Application::init() {
    ESP_LOGI(TAG, "Init(S)");

    // LED初期化
    gpio_reset_pin((gpio_num_t)CONFIG_LED_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
    led(0);

    // ボタン初期化(GPIO0)
    gpio_reset_pin((gpio_num_t)0);
    gpio_set_intr_type((gpio_num_t)0, GPIO_INTR_NEGEDGE);
    gpio_set_direction((gpio_num_t)0, GPIO_MODE_INPUT);
    gpio_pulldown_dis((gpio_num_t)0);
    gpio_pullup_en((gpio_num_t)0);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)0, btn0HandlerFunc, (void*)this);

    // タスク作成
    xTaskCreate(Application::app_task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(AppMessage));

    // SDカード初期化
    m_sd_card.init(ROOT);
    m_sd_card.setMountCallback(mountFunc, this);

    // 保存データ初期化
    m_save_data.init(ROOT);

    // モーターコントローラー
    m_motor.init((gpio_num_t)CONFIG_INA_PIN, (gpio_num_t)CONFIG_INB_PIN, (gpio_num_t)CONFIG_PWM_PIN);

    // OLED(SSD1306)ディスプレイ初期化
    m_oled.init(dispInitCompFunc, this);

    // Wi-Fi初期化
    m_wifi.init(wifiConnectFunc, this);

    // Webサーバー初期化
    m_web.init();
    m_web.addHandler(HTTP_GET, "get_data", getData, this);
    m_web.addHandler(HTTP_POST, "set_data", setData, this);
    m_web.addHandler(HTTP_POST, "save", save, this);
    m_web.setWebSocketHandler(sebSocketFunc, this);

    ESP_LOGI(TAG, "Init(E)");
}

// タスク
void Application::app_task(void* arg) {
    Application* pThis = (Application*)arg;
    AppMessage msg;
    bool loop = true;
    while(loop) {
        // メッセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case AppMessage::UpdateDisplay:     // ディスプレイに現在状態表示
                    pThis->updateDisplay();
                    break;
                case AppMessage::WIFIConnection:    // Wi-Fi接続
                    pThis->wifiConnection();
                    break;
                case AppMessage::WIFIDisconnection: // Wi-Fi切断
                    pThis->wifiDisconnection();
                    break;
                case AppMessage::Quit:              // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    vTaskDelete(NULL);
}

// LED点灯制御
//  state   : 1=点灯, 0=消灯
void Application::led(int state) {
    m_LedState = state;
    gpio_set_level((gpio_num_t)CONFIG_LED_PIN, m_LedState);
}

// SDカードマウントコールバック
void Application::mountFunc(bool isMount, void* context) {
    ESP_LOGI(TAG, "SD Card mount : %d", isMount);
    Application* pThis = (Application*)context;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    if (isMount && pThis->getConfig(ROOT)) {
        AppMessage msg = AppMessage::WIFIConnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    } else {
        AppMessage msg = AppMessage::WIFIDisconnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    }
    // 保存データ読み込み
    pThis->m_save_data.read();
}

// ファイル一覧コールバック
bool Application::fileFunc(bool isFile, const char* name, void* context) {
    ESP_LOGI(TAG, "%s : %s", isFile ? "File" : "Directory", name);
    return true;
}

// ディスプレイ初期化完了コールバック
void Application::dispInitCompFunc(void* context) {
    Application* pThis = (Application*)context;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// Wi-Fi接続コールバック
void Application::wifiConnectFunc(bool isConnect, void* context) {
    Application* pThis = (Application*)context;
    if (isConnect) {
        pThis->m_30sec_off = pThis->m_isWiFi = true;
        const char* ipAddress = pThis->m_wifi.getIPAddress();
        ESP_LOGI(TAG, "IP Address: %s", ipAddress);
        pThis->led(0);
        pThis->m_web.start(ipAddress, ROOT);   // Webサーバー開始

        // 30秒後に画面を消灯するためにタイマ設定
        pThis->timer30secStart();

        // pThis->m_sd_card.fileLists("/document/", fileFunc, pThis);
    } else {
        pThis->m_30sec_off = pThis->m_isWiFi = false;
    }
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// 30秒タイマ開始
void Application::timer30secStart() {
    TimerHandle_t xTimer = xTimerCreate(
        "30SecTimer",
        pdMS_TO_TICKS(30000),
        pdFALSE,
        this,
        timer30secFunc
    );
    if (xTimer != NULL)
        xTimerStart(xTimer, 0);
}

// 30秒タイマ
void Application::timer30secFunc(TimerHandle_t xTimer) {
    Application* pThis = (Application*)pvTimerGetTimerID(xTimer);
    pThis->m_30sec_off = false;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// 基盤上のボタン押下ハンドラ (GPIO0)
void IRAM_ATTR Application::btn0HandlerFunc(void* context) {
    Application* pThis = (Application*)context;
    pThis->m_30sec_off = true;
    pThis->timer30secStart();
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// ディスプレイに現在状態表示
void Application::updateDisplay() {
    if (!m_oled.isInitialize())
        return;
    if (m_isWiFi) {
        if (m_30sec_off) {
            const char* ipAddress = m_wifi.getIPAddress();
            char url[256];
            sprintf(url, "http://%s/", ipAddress);
            m_oled.dispQRCode(url);
        } else {
            // 画面消灯
            m_oled.dispClear();
        }
    } else if (m_sd_card.isMount()) {
        m_oled.dispString("Wi-Fi Connecting");
        led(1);
    } else {
        m_oled.dispString("No File");
        led(0);
    }
}

// Wi-Fi接続
void Application::wifiConnection() {
    wifiDisconnection();
    if (m_configMap.find("ssid") == m_configMap.end() || m_configMap.find("pass") == m_configMap.end())
        return; // CONFIGファイルにssidまたはpassの設定がない
    const char* ssid = m_configMap["ssid"].c_str();
    const char* pass = m_configMap["pass"].c_str();
    m_wifi.connect(ssid, pass);
}

// Wi-Fi切断
void Application::wifiDisconnection() {
    m_web.stop();
    m_wifi.disconnect();
}

// WebAPI GET /API/get_data
// {
//   "ip_address": "192.168.0.1",
// }
void Application::getData(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "getData");
    Application* pThis = (Application*)context;
    char resp[100];
    const char* ip_address = pThis->m_wifi.getIPAddress();
    ip_address = ip_address == NULL ? "" : ip_address;
    sprintf(resp, "{\"ip_address\": \"%s\" }", ip_address);
    ESP_LOGI(TAG, "%s", resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
}

// WebAPI POST /API/set_data
// {
//   "servo_trim": 5
// }
void Application::setData(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "setData");
    Application* pThis = (Application*)context;
    int ret, remaining = req->content_len;
    char body[100];
    if (remaining >= sizeof(body)) {
        ESP_LOGE(TAG, "oversize request body");
        httpd_resp_send_500(req);
        return;
    }
    ret = httpd_req_recv(req, body, remaining);
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "timeout");
        httpd_resp_send_408(req);
        return;
    }
    body[ret] = '\0';
    ESP_LOGI(TAG, "body : %s", body);
    cJSON* json = cJSON_Parse(body);
    if (json == NULL) {
        ESP_LOGI(TAG, "json null");
        httpd_resp_send_500(req);
        return;
    }
    // cJSON* servo_trim = cJSON_GetObjectItemCaseSensitive(json, "servo_trim");
    // if (!cJSON_IsNumber(servo_trim)) {
    //     ESP_LOGI(TAG, "json format error");
    //     httpd_resp_send_500(req);
    //     return;
    // }
    // double trim = cJSON_GetNumberValue(servo_trim);
    // pThis->m_save_data.set("servo_trim", std::to_string(trim).c_str());
    cJSON_Delete(json);
    httpd_resp_send(req, NULL, 0);
}

// WebAPI POST /API/save
void Application::save(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "save");
    Application* pThis = (Application*)context;
    pThis->m_save_data.save();
    httpd_resp_send(req, NULL, 0);
}

// WebSocketコールバック
char* Application::sebSocketFunc(const char* data, void* context) {
    Application* pThis = (Application*)context;
    std::string str = data;
    int speed = std::stoi(str);
    MotorDirection md = MotorDirection::Stop;
    if (speed == 0) {
        md = MotorDirection::Stop;
    } else if (speed > 0) {
        md = MotorDirection::Foward;
    } else if (speed < 0) {
        md = MotorDirection::Back;
        speed *= -1;
    }
    pThis->m_motor.setDirection(md);
    pThis->m_motor.setSpeed(speed);
    return NULL;
}

extern "C" void app_main(void)
{
    nvs_flash_init();     // Flash初期化  (お約束のようなもの)

    app.init();
}
