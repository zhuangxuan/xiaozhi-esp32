#include "car_link.h"
#include "config.h"

#include "board.h"
#include "display/display.h"
#include "settings.h"
#include "system_info.h"

#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <esp_netif.h>

#include <atomic>
#include <string>

namespace {
constexpr const char* TAG = "CarLink";
constexpr const char* kDefaultUrl = "ws://10.23.172.204:8090/ws/car";
constexpr int64_t kFailSafeUs = 300 * 1000;
constexpr int kServoHz = 50;
constexpr int kPulseCenterUs = 1500;
constexpr int kPulseSpanUs = 5;
constexpr uint32_t kDutyMax = (1 << 14) - 1;

std::atomic<int> g_throttle{0};
std::atomic<int> g_steer{0};
std::atomic<int64_t> g_last_seq{0};
std::atomic<int64_t> g_last_rx_us{0};
std::atomic<bool> g_ws_up{false};

int ClampAxis(int value) {
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return value;
}

void SetPulse(ledc_channel_t channel, int axis) {
    int pulse = kPulseCenterUs + axis * kPulseSpanUs;
    if (pulse < 1000) {
        pulse = 1000;
    }
    if (pulse > 2000) {
        pulse = 2000;
    }
    uint32_t duty = static_cast<uint32_t>(pulse) * kDutyMax / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void InitPwm() {
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_14_BIT;
    timer.timer_num = LEDC_TIMER_1;
    timer.freq_hz = kServoHz;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    auto config_channel = [](ledc_channel_t channel, gpio_num_t pin) {
        ledc_channel_config_t cfg = {};
        cfg.gpio_num = pin;
        cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        cfg.channel = channel;
        cfg.intr_type = LEDC_INTR_DISABLE;
        cfg.timer_sel = LEDC_TIMER_1;
        cfg.duty = 0;
        cfg.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&cfg));
    };
    config_channel(LEDC_CHANNEL_1, CAR_THROTTLE_GPIO);
    config_channel(LEDC_CHANNEL_2, CAR_STEER_GPIO);
    SetPulse(LEDC_CHANNEL_1, 0);
    SetPulse(LEDC_CHANNEL_2, 0);
}

void ApplyCommand(const char* data, size_t len) {
    cJSON* root = cJSON_ParseWithLength(data, len);
    if (root == nullptr) {
        return;
    }
    cJSON* type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type)) {
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type->valuestring, "hello") == 0) {
        cJSON* code = cJSON_GetObjectItem(root, "pairCode");
        if (cJSON_IsString(code) && code->valuestring != nullptr) {
            std::string text = std::string("配对码 ") + code->valuestring;
            auto* display = Board::GetInstance().GetDisplay();
            if (display != nullptr) {
                display->SetStatus(code->valuestring);
                display->SetChatMessage("system", text.c_str());
            }
            ESP_LOGI(TAG, "pair code %s", code->valuestring);
        }
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type->valuestring, "drive") != 0 && strcmp(type->valuestring, "estop") != 0) {
        cJSON_Delete(root);
        return;
    }

    cJSON* seq_item = cJSON_GetObjectItem(root, "seq");
    int64_t seq = cJSON_IsNumber(seq_item) ? static_cast<int64_t>(seq_item->valuedouble) : 0;
    int64_t last = g_last_seq.load();
    if (seq <= last) {
        cJSON_Delete(root);
        return;
    }

    int throttle = 0;
    int steer = 0;
    if (strcmp(type->valuestring, "drive") == 0) {
        cJSON* throttle_item = cJSON_GetObjectItem(root, "throttle");
        cJSON* steer_item = cJSON_GetObjectItem(root, "steer");
        if (cJSON_IsNumber(throttle_item)) {
            throttle = ClampAxis(throttle_item->valueint);
        }
        if (cJSON_IsNumber(steer_item)) {
            steer = ClampAxis(steer_item->valueint);
        }
    }
    g_throttle.store(throttle);
    g_steer.store(steer);
    g_last_seq.store(seq);
    int64_t now = esp_timer_get_time();
    int64_t logged = g_last_rx_us.exchange(now);
    if (throttle == 0 && steer == 0 || now - logged >= 500 * 1000) {
        ESP_LOGI(TAG, "drive seq=%lld throttle=%d steer=%d", static_cast<long long>(seq), throttle, steer);
    }
    cJSON_Delete(root);
}

bool WifiHasIp() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) {
        return false;
    }
    esp_netif_ip_info_t ip = {};
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) {
        return false;
    }
    return ip.ip.addr != 0;
}

int WifiRssi() {
    wifi_ap_record_t ap = {};
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return 0;
    }
    return ap.rssi;
}

std::string BuildHello() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddStringToObject(root, "role", "car");
    cJSON_AddStringToObject(root, "deviceId", SystemInfo::GetMacAddress().c_str());
    char* printed = cJSON_PrintUnformatted(root);
    std::string text = printed != nullptr ? printed : "";
    cJSON_free(printed);
    cJSON_Delete(root);
    return text;
}

std::string BuildHeartbeat() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "heartbeat");
    int level = -1;
    bool charging = false;
    bool discharging = false;
    if (Board::GetInstance().GetBatteryLevel(level, charging, discharging)) {
        cJSON_AddNumberToObject(root, "battery", level);
    } else {
        cJSON_AddNumberToObject(root, "battery", -1);
    }
    cJSON_AddNumberToObject(root, "rssi", WifiRssi());
    cJSON_AddNumberToObject(root, "throttle", g_throttle.load());
    cJSON_AddNumberToObject(root, "steer", g_steer.load());
    cJSON_AddStringToObject(root, "fault", "");
    char* printed = cJSON_PrintUnformatted(root);
    std::string text = printed != nullptr ? printed : "";
    cJSON_free(printed);
    cJSON_Delete(root);
    return text;
}

void PwmTask(void* /*arg*/) {
    while (true) {
        int64_t now = esp_timer_get_time();
        int64_t last = g_last_rx_us.load();
        int throttle = 0;
        int steer = 0;
        if (last > 0 && now - last <= kFailSafeUs) {
            throttle = g_throttle.load();
            steer = g_steer.load();
        } else if (last > 0) {
            g_throttle.store(0);
            g_steer.store(0);
        }
        SetPulse(LEDC_CHANNEL_1, throttle);
        SetPulse(LEDC_CHANNEL_2, steer);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void NetTask(void* /*arg*/) {
    while (true) {
        if (!WifiHasIp()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        Settings settings("car", false);
        std::string url = settings.GetString("url", kDefaultUrl);
        if (url.empty()) {
            url = kDefaultUrl;
        }

        auto network = Board::GetInstance().GetNetwork();
        if (network == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        auto ws = network->CreateWebSocket(1);
        if (ws == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        g_ws_up.store(false);
        ws->OnData([](const char* data, size_t len, bool binary) {
            if (!binary && data != nullptr && len > 0) {
                ApplyCommand(data, len);
            }
        });
        ws->OnDisconnected([]() { g_ws_up.store(false); });

        ESP_LOGI(TAG, "connecting %s", url.c_str());
        if (auto connected = ws->Connect(url.c_str()); !connected) {
            ESP_LOGE(TAG, "connect failed: %s", connected.error().ToString().c_str());
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        g_ws_up.store(true);
        g_last_rx_us.store(0);
        if (!ws->Send(BuildHello())) {
            ESP_LOGE(TAG, "hello failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        while (g_ws_up.load() && ws->IsConnected()) {
            if (!ws->Send(BuildHeartbeat())) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        g_ws_up.store(false);
        g_last_rx_us.store(0);
        g_throttle.store(0);
        g_steer.store(0);
        ESP_LOGW(TAG, "link down, outputs centered");
        auto* display = Board::GetInstance().GetDisplay();
        if (display != nullptr) {
            display->SetStatus("遥控断开");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

}  // namespace

void StartCarLink() {
    InitPwm();
    xTaskCreate(PwmTask, "car_pwm", 3072, nullptr, 6, nullptr);
    xTaskCreate(NetTask, "car_net", 8192, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "throttle gpio=%d steer gpio=%d", CAR_THROTTLE_GPIO, CAR_STEER_GPIO);
}
