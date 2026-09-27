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

// ====== FreeRTOS 通信 ======
// 用于在 BLE 回调（任务上下文）和 LVGL 任务之间安全传递字符串
static QueueHandle_t beacon_queue = NULL;

// BLE 扫描相关
BLEScan* pBLEScan;
int scanTime = 3; // 扫描时间（秒）


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


// ====== LVGL 部分（保持不变，略作调整以支持队列输入） ======
void generate_random_string(char * buffer, size_t max_len) {
    // 此函数不再用于生成日志，但保留以防万一
    const char charset[] = "0123456789";
    size_t charset_size = sizeof(charset) - 1;
    size_t len = 1;
    for (size_t i = 0; i < len; i++) {
        buffer[i] = charset[random(charset_size)];
    }
    buffer[len] = '\0';
}

// ====== 新增：在 BLE 回调中收集信标信息，放入队列 ======
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        String addrStr = advertisedDevice.getAddress().toString();
        const char * address = addrStr.c_str();
        int rssi = advertisedDevice.getRSSI();

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%s %d", address, rssi);

        if (beacon_queue != NULL) {
            xQueueSend(beacon_queue, buffer, 0);
        }

        Serial.printf("Beacon: %s RSSI: %d\n", address, rssi);
    }
};


// ====== 新增：从队列取出数据并更新 LVGL 列表 ======
// 在 LVGL 定时器回调中调用，确保在 LVGL 任务上下文中执行
void update_beacon_list_from_queue(lv_timer_t * timer) {
    char buffer[64];
    // 每次最多处理 3 条，避免单次 LVGL 任务负载过高
    int processed = 0;
    while (xQueueReceive(beacon_queue, buffer, 0) == pdTRUE && processed < 3) {
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
            Serial.print("Duplicate, skipped: ");
            Serial.println(buffer);
            continue;
        }
        
        // 写入当前索引的 Label（环形覆盖）
        lv_label_set_text(log_labels[log_write_index], buffer);
        log_write_index = (log_write_index + 1) % MAX_LINES;
        processed++;
    }
    
    // 有新数据时才滚动
    if (processed > 0) {
        lv_obj_scroll_to_y(log_container, lv_obj_get_scroll_bottom(log_container), LV_ANIM_OFF);
    }
}

// ====== 初始化 ======
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
        lv_label_set_text(log_labels[i], "");
    }
    log_write_index = 0;
}

// 原有的 log_add 不再需要，因为列表更新改由队列驱动
void setup() {
    Serial.begin(115200);
    delay(500);

    // 1. 先初始化 BLE（内存大户）
    Serial.println("Initializing BLE...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    // 2. 再创建队列（确保内存充足）
    beacon_queue = xQueueCreate(20, 64);
    if (beacon_queue == NULL) {
        Serial.println("ERROR: Failed to create beacon_queue!");
    } else {
        Serial.println("beacon_queue created OK");
    }

    // 3. 再初始化 LVGL 显示
    RlcdPort.RLCD_Init();
    Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
    if (Lvgl_lock(-1)) {
        Lvgl_unlock();
    }
    lv_tick_set_cb(millis);

    log_init();
    lv_timer_create(update_beacon_list_from_queue, 200, NULL);

    Serial.println("Setup done");
}


void loop() {
    // 执行 BLE 扫描（此函数是阻塞的，会等待 scanTime 秒）
    BLEScanResults *foundDevices = pBLEScan->start(scanTime, false);
    Serial.printf("Devices found: %d\n", foundDevices->getCount());

    pBLEScan->clearResults(); // 释放扫描结果内存
    
    // 短暂延时后再次扫描
    delay(1000);
}