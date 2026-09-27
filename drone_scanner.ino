#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "src/ui_src/generated/gui_guider.h"
#include "i2c_bsp.h"
#include "codec_bsp.h"

#include <WiFi.h>
#include "esp_wifi.h"
#include <math.h>

static lv_ui init_ui;
DisplayPort RlcdPort(12, 11, 5, 40, 41, 400, 300);

// ====== 音频（ES8311） ======
I2cMasterBus I2cbus(14, 13, 0);
CodecPort *codecport = NULL;

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
    float speed;
    float heading;
    uint8_t op_status;
    bool has_basic;
    bool has_location;
};

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

// ====== 蜂鸣：播放一声 1kHz、100ms 的 "嘟" ======
void beep_once() {
    if (codecport == NULL) return;

    const int sample_rate = 16000;
    const int freq = 1000;
    const int duration_ms = 100;
    const int sample_count = sample_rate * duration_ms / 1000;

    int16_t *buffer = (int16_t *)malloc(sample_count * sizeof(int16_t));
    if (buffer == NULL) return;

    for (int i = 0; i < sample_count; i++) {
        buffer[i] = (int16_t)(sin(2 * PI * freq * i / sample_rate) * 10000);
    }

    codecport->CodecPort_PlayWrite(buffer, sample_count * sizeof(int16_t));
    free(buffer);
}

// ====== UA 类型转字符串 ======
const char* ua_type_to_string(uint8_t t) {
    switch (t) {
        case 0: return "None";
        case 1: return "Aeroplane";
        case 2: return "Helicopter/Multirotor";
        case 3: return "Gyroplane";
        case 4: return "Hybrid";
        case 5: return "Ornithopter";
        case 6: return "Glider";
        case 7: return "Kite";
        case 8: return "FreeBalloon";
        case 9: return "CaptiveBalloon";
        case 10: return "Airship";
        case 11: return "FreeFall";
        case 12: return "Rocket";
        case 13: return "Tethered";
        case 14: return "Glider2";
        case 15: return "Other";
        default: return "Unknown";
    }
}

// ====== Wi-Fi 混杂模式回调：解析 GB 42590 RID ======
void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buff;
    uint8_t *frame = pkt->payload;
    int len = pkt->rx_ctrl.sig_len;

    uint8_t subtype = (frame[0] >> 4) & 0x0F;
    if (subtype != 0x08) return;
    if (len < 36) return;

    int offset = 36;
    while (offset + 2 <= len) {
        uint8_t ie_id = frame[offset];
        uint8_t ie_len = frame[offset + 1];

        if (offset + 2 + ie_len > len) break;

        if (ie_id == 0xDD && ie_len >= 8) {
            uint8_t *d = frame + offset + 2;

            if (d[0] == 0xFA && d[1] == 0x0B && d[2] == 0xBC && d[3] == 0x0D) {

                RIDData rid;
                memset(&rid, 0, sizeof(rid));

                snprintf(rid.mac, sizeof(rid.mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                         frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
                rid.rssi = pkt->rx_ctrl.rssi;

                uint8_t *msg_pack = d + 8;
                int msg_len = ie_len - 8;

                int msg_offset = 0;
                while (msg_offset + 25 <= msg_len) {
                    uint8_t *msg = msg_pack + msg_offset;
                    uint8_t msg_type = (msg[0] >> 4) & 0x0F;
                    uint8_t version = msg[0] & 0x0F;

                    if (version == 0x1) {
                        if (msg_type == 0x0) {
                            rid.id_type = (msg[1] >> 4) & 0x0F;
                            rid.ua_type = msg[1] & 0x0F;
                            memcpy(rid.uas_id, msg + 2, 20);
                            rid.uas_id[20] = '\0';
                            rid.has_basic = true;
                        }
                        else if (msg_type == 0x1) {
                            rid.op_status = (msg[1] >> 4) & 0x0F;

                            int32_t lat, lon;
                            memcpy(&lat, msg + 5, 4);
                            memcpy(&lon, msg + 9, 4);
                            rid.latitude = lat * 1e-7;
                            rid.longitude = lon * 1e-7;

                            uint16_t alt;
                            memcpy(&alt, msg + 13, 2);
                            rid.altitude = (alt * 0.5f) - 1000.0f;

                            uint16_t spd;
                            memcpy(&spd, msg + 15, 2);
                            rid.speed = spd * 0.25f;

                            uint16_t hdg;
                            memcpy(&hdg, msg + 17, 2);
                            rid.heading = hdg * 0.01f;

                            rid.has_location = true;
                        }
                    }
                    msg_offset += 25;
                }

                // ===== 每 10 秒最多打印一次 + 蜂鸣 =====
                static uint32_t last_print_time = 0;
                uint32_t now = millis();
                bool should_print = (now - last_print_time >= 10000);

                if ((rid.has_basic || rid.has_location) && should_print) {
                    last_print_time = now;

                    // 蜂鸣
                    beep_once();

                    Serial.println("========== RID ==========");
                    Serial.printf("MAC       : %s\n", rid.mac);
                    Serial.printf("RSSI      : %d dBm\n", rid.rssi);
                    if (rid.has_basic) {
                        Serial.printf("UAS ID    : %s\n", rid.uas_id);
                        Serial.printf("ID Type   : %d\n", rid.id_type);
                        Serial.printf("UA Type   : %d (%s)\n", rid.ua_type, ua_type_to_string(rid.ua_type));
                    }
                    if (rid.has_location) {
                        Serial.printf("Latitude  : %.7f\n", rid.latitude);
                        Serial.printf("Longitude : %.7f\n", rid.longitude);
                        Serial.printf("Altitude  : %.1f m\n", rid.altitude);
                        Serial.printf("Speed     : %.2f m/s\n", rid.speed);
                        Serial.printf("Heading   : %.1f deg\n", rid.heading);
                        Serial.printf("Status    : %d\n", rid.op_status);
                    }

                    Serial.printf("RAW Len   : %d\n", ie_len);
                    Serial.print("RAW Data  : ");
                    for (int i = 0; i < ie_len; i++) {
                        Serial.printf("%02X ", d[i]);
                        if ((i + 1) % 16 == 0) Serial.print("\n            ");
                    }
                    Serial.println();
                    Serial.println("==========================");
                }

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

// ====== LVGL 定时器：只显示 UAS ID ======
void update_rid_display(lv_timer_t * timer) {
    if (rid_queue == NULL || log_container == NULL) return;

    RIDData rid;
    int processed = 0;

    while (xQueueReceive(rid_queue, &rid, 0) == pdTRUE && processed < 3) {
        if (!rid.has_basic || rid.uas_id[0] == '\0') continue;

        const char *display_str = rid.uas_id;

        bool duplicate = false;
        for (int i = 0; i < MAX_LINES; i++) {
            if (log_labels[i] == NULL) continue;
            const char *existing = lv_label_get_text(log_labels[i]);
            if (existing != NULL && strcmp(existing, display_str) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        lv_label_set_text(log_labels[log_write_index], display_str);
        log_write_index = (log_write_index + 1) % MAX_LINES;
        processed++;
    }

    if (processed > 0) {
        lv_obj_scroll_to_y(log_container, lv_obj_get_scroll_bottom(log_container), LV_ANIM_OFF);
    }
}

// ====== 日志列表初始化 ======
void log_init(void) {
    log_container = lv_obj_create(lv_scr_act());
    if (log_container == NULL) return;
    lv_obj_set_size(log_container, 390, 290);
    lv_obj_align(log_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(log_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(log_container, LV_SCROLLBAR_MODE_AUTO);

    for (int i = 0; i < MAX_LINES; i++) {
        log_labels[i] = lv_label_create(log_container);
        lv_obj_set_width(log_labels[i], 380);
        lv_label_set_long_mode(log_labels[i], LV_LABEL_LONG_WRAP);
        lv_label_set_text(log_labels[i], "");
    }
    log_write_index = 0;
}

// ====== 初始化 ======
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n===== BOOT =====");

    // 1. 音频（ES8311）
    codecport = new CodecPort(I2cbus, "S3_RLCD_4_2");
    codecport->CodecPort_SetInfo("es8311", 1, 16000, 1, 16);
    codecport->CodecPort_SetSpeakerVol(80);

    // 2. RLCD
    RlcdPort.RLCD_Init();

    // 3. LVGL
    Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
    if (Lvgl_lock(-1)) {
        lv_tick_set_cb(millis);
        log_init();
        lv_timer_create(update_rid_display, 500, NULL);
        Lvgl_unlock();
    }

    // 4. RID 队列
    rid_queue = xQueueCreate(20, sizeof(RIDData));

    // 5. Wi-Fi 混杂模式
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

    Serial.println("Setup done");
}

void loop() {
    delay(1000);
}