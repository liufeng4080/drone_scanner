#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "src/ui_src/generated/gui_guider.h"

// ====== BLE 相关头文件 ======
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

static lv_ui init_ui;
DisplayPort RlcdPort(12, 11, 5, 40, 41, 400, 300);

#define SCREEN_WIDTH  400
#define SCREEN_HEIGHT 300

// ====== 日志/列表显示相关 ======
#define MAX_LINES 10
static lv_obj_t * log_container;
static lv_obj_t * log_labels[MAX_LINES];
static int log_write_index = 0;

// ====== BLE 数据结构 ======
// 通过队列传递的原始数据，避免在 BLE 回调中做字符串拼接
struct BeaconData {
    char address[18];   // MAC 地址，如 "AA:BB:CC:DD:EE:FF"
    char name[32];      // 设备名称，可能为空
    int rssi;           // 信号强度
};

// ====== FreeRTOS 通信 ======
static QueueHandle_t beacon_queue = NULL;

// ====== BLE 扫描相关 ======
BLEScan* pBLEScan;
int scanTime = 3; // 扫描时间（秒）

// ====== LVGL 刷屏回调 ======
static void Lvgl_FlushCallback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map) {
    uint16_t *buffer = (uint16_t *)color_map;
    for (int y = area->y1; y <= area->y2; y++) {
        for (int x = area->x1; x <= area->x2; x++) {
            uint8_t color = (*buffer < 0x7fff) ? ColorBlack : ColorWhite;
            RlcdPort.RLCD_SetPixel(x, y, color);
            buffer++;
        }
    }
    RlcdPort.RLCD_Display();
    lv_display_flush_ready(drv);
}

// ====== BLE 回调：只做最轻量的数据拷贝，不做字符串拼接 ======
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        BeaconData data;
        memset(&data, 0, sizeof(data));

        // 获取 MAC 地址
        String addrStr = advertisedDevice.getAddress().toString();
        strncpy(data.address, addrStr.c_str(), sizeof(data.address) - 1);
        data.address[sizeof(data.address) - 1] = '\0';

        // 获取设备名称（可能为空）
        if (advertisedDevice.haveName()) {
            String nameStr = advertisedDevice.getName();
            strncpy(data.name, nameStr.c_str(), sizeof(data.name) - 1);
            data.name[sizeof(data.name) - 1] = '\0';
        }

        // 获取信号强度
        data.rssi = advertisedDevice.getRSSI();

        // 入队（非阻塞，队列满则丢弃）
        if (beacon_queue != NULL) {
            xQueueSend(beacon_queue, &data, 0);
        }
    }
};

// ====== LVGL 定时器：从队列取数据，拼接字符串并更新界面 ======
void update_beacon_list_from_queue(lv_timer_t * timer) {
    if (beacon_queue == NULL) {
        return;
    }

    BeaconData data;
    int processed = 0;

    // 每次最多处理 3 条，避免单次 LVGL 任务负载过高
    while (xQueueReceive(beacon_queue, &data, 0) == pdTRUE && processed < 3) {
        // 拼接显示字符串
        char buffer[96];
        if (data.name[0] != '\0') {
            snprintf(buffer, sizeof(buffer), "%s %s %d", data.name, data.address, data.rssi);
        } else {
            snprintf(buffer, sizeof(buffer), "[NoName] %s %d", data.address, data.rssi);
        }

        // 去重：检查当前 10 个 Label 中是否已有相同内容
        bool duplicate = false;
        for (int i = 0; i < MAX_LINES; i++) {
            const char * existing = lv_label_get_text(log_labels[i]);
            if (existing != NULL && strcmp(existing, buffer) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        // 写入当前索引的 Label（环形覆盖）
        lv_label_set_text(log_labels[log_write_index], buffer);
        log_write_index = (log_write_index + 1) % MAX_LINES;
        processed++;
    }

    // 有新数据时才滚动到底部
    if (processed > 0) {
        lv_obj_scroll_to_y(log_container, lv_obj_get_scroll_bottom(log_container), LV_ANIM_OFF);
    }
}

// ====== 日志列表初始化：一次性创建 10 个 Label ======
void log_init(void) {
    log_container = lv_obj_create(lv_scr_act());
    if (log_container == NULL) {
        Serial.println("ERROR: log_container creation failed!");
        return;
    }
    lv_obj_set_size(log_container, 390, 290);
    lv_obj_align(log_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(log_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(log_container, LV_SCROLLBAR_MODE_AUTO);

    for (int i = 0; i < MAX_LINES; i++) {
        log_labels[i] = lv_label_create(log_container);
        if (log_labels[i] == NULL) {
            Serial.print("ERROR: log_labels[");
            Serial.print(i);
            Serial.println("] creation failed!");
            return;
        }
        lv_label_set_text(log_labels[i], "");
    }
    log_write_index = 0;
}

// ====== 初始化 ======
void setup() {
    Serial.begin(115200);
    delay(500);

    RlcdPort.RLCD_Init();
    Lvgl_PortInit(400, 300, Lvgl_FlushCallback);

    // 所有 LVGL 操作必须加锁
    if (Lvgl_lock(-1)) {
        lv_tick_set_cb(millis);
        log_init();
        lv_timer_create(update_beacon_list_from_queue, 200, NULL);
        Lvgl_unlock();
    }

    Serial.println("LVGL init done");

    // 队列创建（与 LVGL 无关，不需要锁）
    beacon_queue = xQueueCreate(20, sizeof(BeaconData));
    if (beacon_queue == NULL) {
        Serial.println("ERROR: Failed to create beacon_queue!");
    } else {
        Serial.println("beacon_queue created OK");
    }

    // BLE 初始化
    Serial.println("Initializing BLE...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    Serial.printf("Free heap: %d\n", esp_get_free_heap_size());
    Serial.println("Setup done");
}

void loop() {
    // 执行 BLE 扫描（阻塞 scanTime 秒）
    BLEScanResults *foundDevices = pBLEScan->start(scanTime, false);
    Serial.printf("Devices found: %d\n", foundDevices->getCount());

    pBLEScan->clearResults(); // 释放扫描结果内存

    // 短暂延时后再次扫描
    delay(1000);
}