#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "src/ui_src/generated/gui_guider.h"

#include <WiFi.h>
#include "esp_wifi.h"

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
struct RIDData {
    char mac[18];
    int rssi;
    uint8_t ua_type;
    uint8_t id_type;
    char uas_id[21];
    double latitude;
    double longitude;
    float altitude;
    float height;
    float speed;
    float heading;
    uint8_t op_status;
    bool has_basic;
    bool has_location;
};

// ====== FreeRTOS 通信 ======
static QueueHandle_t rid_queue = NULL;

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

// ====== Wi-Fi 混杂模式回调：解析 GB 42590 RID ======
// 注意：此函数运行在 Wi-Fi 驱动上下文中，必须用 xQueueSendFromISR
// void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
//     if (type != WIFI_PKT_MGMT) return;

//     wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buff;
//     uint8_t *frame = pkt->payload;
//     int len = pkt->rx_ctrl.sig_len;


//     // 802.11 Beacon 帧最小长度检查
//     // [0-1] Frame Control, [2-3] Duration, [4-9] DA, [10-15] SA, [16-21] BSSID,
//     // [22-23] Seq, [24-31] Timestamp, [32-33] Beacon Interval, [34-35] Capability
//     if (len < 36) return;


//     // 只处理 Beacon 帧（Frame Control 的 subtype 为 0x8）
//     uint8_t frame_type = (frame[0] >> 2) & 0x03;
//     uint8_t frame_subtype = (frame[0] >> 4) & 0x0F;
//     if (frame_type != 0x00 || frame_subtype != 0x08) return;


//     int offset = 36;

//     // 遍历所有 Information Elements
//     while (offset + 2 <= len) {
//         uint8_t ie_id = frame[offset];
//         uint8_t ie_len = frame[offset + 1];

//         if (offset + 2 + ie_len > len) break;


//         // 查找 Vendor Specific IE (221 = 0xDD)
//         if (ie_id == 0xDD && ie_len >= 7) {
//             uint8_t *ie_data = frame + offset + 2;


//             // 检查 OUI (FA:0B:BC) 和 OUI Type (0x0D)
//             if (ie_data[0] == 0xFA && ie_data[1] == 0x0B &&
//                 ie_data[2] == 0xBC && ie_data[3] == 0x0D) {
//                 Serial.println("[SNIFFER] 6");

//                 RIDData rid;
//                 memset(&rid, 0, sizeof(rid));

//                 // 提取源 MAC 地址
//                 snprintf(rid.mac, sizeof(rid.mac), "%02X:%02X:%02X:%02X:%02X:%02X",
//                          frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
//                 rid.rssi = pkt->rx_ctrl.rssi;

//                 // Message Pack 从 ie_data[5] 开始（ie_data[4] 是 Message Counter）
//                 uint8_t *msg_pack = ie_data + 5;
//                 int msg_len = ie_len - 5;

//                 int msg_offset = 0;
//                 while (msg_offset + 25 <= msg_len) {
//                     uint8_t *msg = msg_pack + msg_offset;
//                     uint8_t msg_type = (msg[0] >> 4) & 0x0F;
//                     uint8_t version = msg[0] & 0x0F;
//                     Serial.println("[SNIFFER] 7");

//                     if (version == 0x1) {
//                         if (msg_type == 0x0) {  // 基本 ID 报文
//                             rid.id_type = (msg[1] >> 4) & 0x0F;
//                             rid.ua_type = msg[1] & 0x0F;
//                             memcpy(rid.uas_id, msg + 2, 20);
//                             rid.uas_id[20] = '\0';
//                             rid.has_basic = true;
//                         }
//                         else if (msg_type == 0x1) {  // 位置向量报文
//                             rid.op_status = (msg[1] >> 4) & 0x0F;

//                             int32_t lat, lon;
//                             memcpy(&lat, msg + 5, 4);
//                             memcpy(&lon, msg + 9, 4);
//                             rid.latitude = lat * 1e-7;
//                             rid.longitude = lon * 1e-7;

//                             uint16_t alt;
//                             memcpy(&alt, msg + 13, 2);
//                             rid.altitude = (alt * 0.5f) - 1000.0f;

//                             uint16_t spd;
//                             memcpy(&spd, msg + 15, 2);
//                             rid.speed = spd * 0.25f;

//                             uint16_t hdg;
//                             memcpy(&hdg, msg + 17, 2);
//                             rid.heading = hdg * 0.01f;

//                             rid.has_location = true;
//                         }
//                     }
//                     msg_offset += 25;
//                 }
//                 Serial.println("[SNIFFER] 8");

//                 // 入队（必须用 FromISR 版本）
//                 if (rid_queue != NULL && (rid.has_basic || rid.has_location)) {
//                     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//                     Serial.println("[SNIFFER] 9");
//                     xQueueSendFromISR(rid_queue, &rid, &xHigherPriorityTaskWoken);
//                     if (xHigherPriorityTaskWoken) {
//                         portYIELD_FROM_ISR();
//                     }
//                 }
//             }
//         }

//         offset += 2 + ie_len;
//     }
// }
// ====== Wi-Fi 混杂模式回调：解析 GB 42590 RID ======
void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buff;
    uint8_t *frame = pkt->payload;
    int len = pkt->rx_ctrl.sig_len;

    // 只处理 Beacon 帧（subtype = 0x08）
    uint8_t subtype = (frame[0] >> 4) & 0x0F;
    if (subtype != 0x08) return;
    if (len < 36) return;

    // 遍历 Information Elements
    int offset = 36;
    while (offset + 2 <= len) {
        uint8_t ie_id = frame[offset];
        uint8_t ie_len = frame[offset + 1];

        if (offset + 2 + ie_len > len) break;

        // 只关心 Vendor Specific IE (0xDD)
        if (ie_id == 0xDD && ie_len >= 8) {
            uint8_t *d = frame + offset + 2;

            // 判断 OUI 是否为 FA:0B:BC，且 Type 为 0x0D
            if (d[0] == 0xFA && d[1] == 0x0B && d[2] == 0xBC && d[3] == 0x0D) {

                // 解析 RID 数据
                RIDData rid;
                memset(&rid, 0, sizeof(rid));

                // 提取源 MAC 地址
                snprintf(rid.mac, sizeof(rid.mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                         frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
                rid.rssi = pkt->rx_ctrl.rssi;

                // 【关键修正】跳过 OUI(3) + Type(1) + MessageCounter(1) + PackHeader(3)
                // d[0..2] = OUI, d[3] = Type, d[4] = MessageCounter
                // d[5..7] = Pack Header (F1 xx xx)
                // 真正的消息从 d[8] 开始
                uint8_t *msg_pack = d + 8;
                int msg_len = ie_len - 8;

                int msg_offset = 0;
                while (msg_offset + 25 <= msg_len) {
                    uint8_t *msg = msg_pack + msg_offset;
                    uint8_t msg_type = (msg[0] >> 4) & 0x0F;
                    uint8_t version = msg[0] & 0x0F;

                    if (version == 0x1) {
                        if (msg_type == 0x0) {
                            // 基本 ID 报文
                            rid.id_type = (msg[1] >> 4) & 0x0F;
                            rid.ua_type = msg[1] & 0x0F;
                            memcpy(rid.uas_id, msg + 2, 20);
                            rid.uas_id[20] = '\0';
                            rid.has_basic = true;

                            Serial.printf("[RID] BASIC ID: %s (id_type=%d ua_type=%d)\n",
                                          rid.uas_id, rid.id_type, rid.ua_type);
                        }
                        else if (msg_type == 0x1) {
                            // 位置向量报文
                            rid.op_status = (msg[1] >> 4) & 0x0F;

                            // 经纬度：int32 小端序，单位 1e-7 度
                            int32_t lat, lon;
                            memcpy(&lat, msg + 2, 4);
                            memcpy(&lon, msg + 6, 4);
                            rid.latitude = lat * 1e-7;
                            rid.longitude = lon * 1e-7;

                            // 高度：uint16 小端序，单位 0.5 米（偏移 -1000 米）
                            uint16_t alt;
                            memcpy(&alt, msg + 10, 2);
                            rid.altitude = (alt * 0.5f) - 1000.0f;

                            // 速度：uint16 小端序，单位 0.25 m/s
                            uint16_t spd;
                            memcpy(&spd, msg + 14, 2);
                            rid.speed = spd * 0.25f;

                            // 航迹角：uint16 小端序，单位 0.01 度
                            uint16_t hdg;
                            memcpy(&hdg, msg + 16, 2);
                            rid.heading = hdg * 0.01f;

                            rid.has_location = true;

                            Serial.printf("[RID] LOCATION: %.4f, %.4f alt=%.1fm spd=%.1fm/s hdg=%.1f\n",
                                          rid.latitude, rid.longitude,
                                          rid.altitude, rid.speed, rid.heading);
                        }
                    }
                    msg_offset += 25;
                }

                // 打印完整原始数据（调试用）
                Serial.printf("[RID] MAC=%s RSSI=%d\n", rid.mac, rid.rssi);
                Serial.print("[RID] Raw: ");
                for (int i = 0; i < ie_len; i++) {
                    Serial.printf("%02X ", d[i]);
                }
                Serial.println();

                // 入队（必须用 FromISR 版本）
                if (rid_queue != NULL && (rid.has_basic || rid.has_location)) {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    xQueueSendFromISR(rid_queue, &rid, &xHigherPriorityTaskWoken);
                    if (xHigherPriorityTaskWoken) {
                        portYIELD_FROM_ISR();
                    }
                }
            }
        }

        offset += 2 + ie_len;
    }
}


// ====== LVGL 定时器：从队列取 RID 数据并更新界面 ======
void update_rid_display(lv_timer_t * timer) {
    if (rid_queue == NULL || log_container == NULL) {
        return;
    }

    RIDData rid;
    int processed = 0;

    while (xQueueReceive(rid_queue, &rid, 0) == pdTRUE && processed < 3) {
        char buffer[96];
        const char *id_str = (rid.uas_id[0] != '\0') ? rid.uas_id : rid.mac;

        if (rid.has_location && rid.has_basic) {
            snprintf(buffer, sizeof(buffer), "%.8s %.4f,%.4f %.0fm %d",
                     id_str, rid.latitude, rid.longitude,
                     rid.altitude, rid.rssi);
        } else if (rid.has_basic) {
            snprintf(buffer, sizeof(buffer), "%.8s ID %d",
                     id_str, rid.rssi);
        } else {
            snprintf(buffer, sizeof(buffer), "%s %.4f,%.4f %d",
                     rid.mac, rid.latitude, rid.longitude, rid.rssi);
        }

        // 去重
        bool duplicate = false;
        for (int i = 0; i < MAX_LINES; i++) {
            if (log_labels[i] == NULL) continue;
            const char *existing = lv_label_get_text(log_labels[i]);
            if (existing != NULL && strcmp(existing, buffer) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        lv_label_set_text(log_labels[log_write_index], buffer);
        log_write_index = (log_write_index + 1) % MAX_LINES;
        processed++;
    }

    if (processed > 0) {
        lv_obj_scroll_to_y(log_container, lv_obj_get_scroll_bottom(log_container), LV_ANIM_OFF);
    }
}

// ====== 日志列表初始化 ======
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

    // ===== 阶段 3：LVGL 对象创建（加锁）=====
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

    // ===== 阶段 5：Wi-Fi 混杂模式 =====
    Serial.println("[SETUP] starting Wi-Fi promiscuous mode...");
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

    Serial.printf("[SETUP] Free heap: %d\n", esp_get_free_heap_size());
    Serial.println("===== SETUP DONE =====\n");
}

void loop() {
    // 在 loop() 里轮流切换信道
    static uint8_t ch = 1;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    ch = (ch % 13) + 1;
    delay(500);
}