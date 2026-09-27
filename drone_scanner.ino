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
static lv_obj_t * log_container = NULL;
static lv_obj_t * log_labels[MAX_LINES];
static int log_write_index = 0;

// ====== RID 数据结构 ======
// 解析 GB 42590 / ASTM F3411 后的无人机信息
struct RIDData {
    char mac[18];           // 蓝牙 MAC 地址（用于关联同一设备的多条报文）
    int rssi;               // 信号强度
    uint8_t ua_type;        // 无人机类型（0-15）
    uint8_t id_type;        // ID 类型
    char uas_id[21];        // UAS ID（序列号/会话ID），最长 20 字符
    double latitude;        // 纬度（度）
    double longitude;       // 经度（度）
    float altitude;         // 海拔高度（米）
    float height;           // 相对起飞点高度（米）
    float speed;            // 速度（m/s）
    float heading;          // 航迹角（度）
    uint8_t op_status;      // 运行状态
    bool has_basic;         // 是否已解析到基本ID
    bool has_location;      // 是否已解析到位置信息
};

// ====== FreeRTOS 通信 ======
static QueueHandle_t rid_queue = NULL;

// ====== BLE 扫描相关 ======
BLEScan* pBLEScan = NULL;
int scanTime = 3; // 扫描时间（秒）

// ====== 前置声明 ======
static void Lvgl_FlushCallback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map);

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

// ====== GB 42590 / ASTM F3411 RID 解析 ======
// 广播数据封装在 OUI (FA:0B:BC) + Vendor Type (0x0D) 之后
// 每条报文 25 字节，报头高4位为类型，低4位固定为 0x1
// 参考：民航局《民用微轻小型无人驾驶航空器运行识别最低性能要求》
bool parse_gb42590_rid(uint8_t *payload, int len, RIDData *out) {
    if (len < 5) return false;

    // 检查 ASTM/国标 OUI (FA:0B:BC) 和 Vendor Type (0x0D)
    if (payload[0] != 0xFA || payload[1] != 0x0B || payload[2] != 0xBC) return false;
    if (payload[3] != 0x0D) return false;

    // payload[4] 是 Message Counter，跳过
    int offset = 5;

    while (offset + 25 <= len) {
        uint8_t *msg = payload + offset;
        uint8_t msg_type = (msg[0] >> 4) & 0x0F;
        uint8_t version = msg[0] & 0x0F;

        // 协议版本必须为 0x1
        if (version != 0x1) {
            offset += 25;
            continue;
        }

        switch (msg_type) {
            case 0x0:  // 基本 ID 报文
                // msg[1]: 高4位 ID类型, 低4位 UA类型
                out->id_type = (msg[1] >> 4) & 0x0F;
                out->ua_type = msg[1] & 0x0F;
                // UAS ID 从 msg[2] 开始，20 字节 ASCII
                memcpy(out->uas_id, msg + 2, 20);
                out->uas_id[20] = '\0';
                out->has_basic = true;
                break;

            case 0x1:  // 位置向量报文
                // msg[1]: 高4位运行状态, 低4位保留
                out->op_status = (msg[1] >> 4) & 0x0F;

                // 经度/纬度：int32 小端序，单位 1e-7 度
                // 偏移量参考 ASTM F3411 报文格式
                int32_t lat_raw, lon_raw;
                memcpy(&lat_raw, msg + 5, 4);
                memcpy(&lon_raw, msg + 9, 4);
                out->latitude = lat_raw * 1e-7;
                out->longitude = lon_raw * 1e-7;

                // 高度：uint16 小端序，单位 0.5 米（偏移 500 米）
                uint16_t alt_raw;
                memcpy(&alt_raw, msg + 13, 2);
                out->altitude = (alt_raw * 0.5f) - 1000.0f;

                // 速度：uint16 小端序，单位 0.25 m/s
                uint16_t spd_raw;
                memcpy(&spd_raw, msg + 15, 2);
                out->speed = spd_raw * 0.25f;

                // 航迹角：uint16 小端序，单位 0.01 度
                uint16_t hdg_raw;
                memcpy(&hdg_raw, msg + 17, 2);
                out->heading = hdg_raw * 0.01f;

                out->has_location = true;
                break;

            case 0x4:  // 系统报文（操作员位置等）
                // 可扩展解析操作员位置
                break;

            default:
                break;
        }
        offset += 25;
    }

    return out->has_basic || out->has_location;
}

// ====== BLE 回调：解析 RID 数据并入队 ======
class MyRIDCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        // 只处理有 manufacturer data 的设备
        if (!advertisedDevice.haveManufacturerData()) return;

        String mfgData = advertisedDevice.getManufacturerData();
        int len = mfgData.length();
        if (len < 5) return;

        uint8_t *payload = (uint8_t *)mfgData.c_str();

        RIDData rid;
        memset(&rid, 0, sizeof(rid));

        // 解析 GB 42590 格式
        if (!parse_gb42590_rid(payload, len, &rid)) {
            return;  // 不是 RID 广播，忽略
        }

        // 填充 MAC 和 RSSI
        String addrStr = advertisedDevice.getAddress().toString();
        strncpy(rid.mac, addrStr.c_str(), sizeof(rid.mac) - 1);
        rid.rssi = advertisedDevice.getRSSI();

        // 入队（非阻塞）
        if (rid_queue != NULL) {
            xQueueSend(rid_queue, &rid, 0);
        }
    }
};

// ====== LVGL 定时器：从队列取 RID 数据并更新界面 ======
void update_rid_display(lv_timer_t * timer) {
    if (rid_queue == NULL || log_container == NULL) {
        return;
    }

    RIDData rid;
    int processed = 0;

    // 每次最多处理 3 条，避免单次 LVGL 任务负载过高
    while (xQueueReceive(rid_queue, &rid, 0) == pdTRUE && processed < 3) {
        // 拼接显示字符串
        // 格式：ID类型 UAS_ID 纬度 经度 RSSI
        char buffer[96];
        const char *id_str = (rid.uas_id[0] != '\0') ? rid.uas_id : rid.mac;
        const char *ua_type_str = "";
        switch (rid.ua_type) {
            case 0: ua_type_str = "None"; break;
            case 1: ua_type_str = "Aeroplane"; break;
            case 2: ua_type_str = "Helicopter"; break;
            case 3: ua_type_str = "Gyroplane"; break;
            case 4: ua_type_str = "Hybrid"; break;
            case 5: ua_type_str = "Ornithopter"; break;
            case 6: ua_type_str = "Glider"; break;
            case 7: ua_type_str = "Kite"; break;
            case 8: ua_type_str = "FreeBalloon"; break;
            case 9: ua_type_str = "CaptiveBalloon"; break;
            case 10: ua_type_str = "Airship"; break;
            case 11: ua_type_str = "FreeFall"; break;
            case 12: ua_type_str = "Rocket"; break;
            case 13: ua_type_str = "Tethered"; break;
            case 14: ua_type_str = "Glider2"; break;
            case 15: ua_type_str = "Other"; break;
            default: ua_type_str = "?"; break;
        }

        if (rid.has_location && rid.has_basic) {
            snprintf(buffer, sizeof(buffer), "%.8s %s %.4f,%.4f %d",
                     id_str, ua_type_str,
                     rid.latitude, rid.longitude,
                     rid.rssi);
        } else if (rid.has_basic) {
            snprintf(buffer, sizeof(buffer), "%.8s %s %d",
                     id_str, ua_type_str, rid.rssi);
        } else {
            snprintf(buffer, sizeof(buffer), "%s ? %d",
                     rid.mac, rid.rssi);
        }

        // 去重：检查当前 10 个 Label 中是否已有相同内容
        bool duplicate = false;
        for (int i = 0; i < MAX_LINES; i++) {
            if (log_labels[i] == NULL) continue;
            const char *existing = lv_label_get_text(log_labels[i]);
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
    Serial.println("[INIT] log_init start");

    log_container = lv_obj_create(lv_scr_act());
    if (log_container == NULL) {
        Serial.println("[INIT] ERROR: log_container creation failed!");
        return;
    }
    lv_obj_set_size(log_container, 390, 290);
    lv_obj_align(log_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(log_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(log_container, LV_SCROLLBAR_MODE_AUTO);

    for (int i = 0; i < MAX_LINES; i++) {
        log_labels[i] = lv_label_create(log_container);
        if (log_labels[i] == NULL) {
            Serial.printf("[INIT] ERROR: log_labels[%d] creation failed!\n", i);
            return;
        }
        lv_label_set_text(log_labels[i], "");
    }
    log_write_index = 0;
    Serial.println("[INIT] log_init done");
}

// ====== 初始化 ======
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n===== BOOT =====");

    // ===== 阶段 1：RLCD 硬件初始化 =====
    Serial.println("[SETUP] RLCD_Init...");
    RlcdPort.RLCD_Init();
    Serial.println("[SETUP] RLCD_Init done");

    // ===== 阶段 2：LVGL 初始化 =====
    Serial.println("[SETUP] Lvgl_PortInit...");
    Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
    Serial.println("[SETUP] Lvgl_PortInit done");

    // ===== 阶段 3：LVGL 对象创建（必须加锁）=====
    Serial.println("[SETUP] acquiring LVGL lock...");
    if (Lvgl_lock(-1)) {
        Serial.println("[SETUP] lock acquired, creating UI...");
        lv_tick_set_cb(millis);
        log_init();
        lv_timer_create(update_rid_display, 500, NULL);
        Lvgl_unlock();
        Serial.println("[SETUP] UI created, lock released");
    } else {
        Serial.println("[SETUP] ERROR: failed to acquire LVGL lock!");
    }

    // ===== 阶段 4：创建 RID 队列 =====
    Serial.println("[SETUP] creating rid_queue...");
    rid_queue = xQueueCreate(20, sizeof(RIDData));
    if (rid_queue == NULL) {
        Serial.println("[SETUP] ERROR: Failed to create rid_queue!");
    } else {
        Serial.println("[SETUP] rid_queue created OK");
    }

    // ===== 阶段 5：BLE 初始化 =====
    Serial.println("[SETUP] initializing BLE...");
    BLEDevice::init("");
    Serial.println("[SETUP] BLEDevice::init done");

    pBLEScan = BLEDevice::getScan();
    Serial.println("[SETUP] getScan done");

    pBLEScan->setAdvertisedDeviceCallbacks(new MyRIDCallbacks());
    Serial.println("[SETUP] setAdvertisedDeviceCallbacks done");

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    Serial.println("[SETUP] scan config done");

    Serial.printf("[SETUP] Free heap: %d\n", esp_get_free_heap_size());
    Serial.println("===== SETUP DONE =====\n");
}

void loop() {
    // 执行 BLE 扫描（阻塞 scanTime 秒）
    BLEScanResults *foundDevices = pBLEScan->start(scanTime, false);
    Serial.printf("[LOOP] scan done, found %d devices\n", foundDevices->getCount());

    pBLEScan->clearResults(); // 释放扫描结果内存

    // 短暂延时后再次扫描
    delay(1000);
}