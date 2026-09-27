#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "i2c_bsp.h"
#include "codec_bsp.h"
#include "sdcard_bsp.h"
#include "adc_bsp.h"

#include <WiFi.h>
#include "esp_wifi.h"
#include <math.h>

DisplayPort RlcdPort(12, 11, 5, 40, 41, 400, 300);

// ====== 音频（ES8311） ======
I2cMasterBus I2cbus(14, 13, 0);
CodecPort *codecport = NULL;

// ====== SD 卡 ======
CustomSDPort *sdcardPort = NULL;
#define RID_LOG_FILE "/sdcard/rid_log.txt"

// ====== 电池电压 ======
static lv_obj_t * bat_label = NULL;

// ====== 启动画面 ======
static lv_obj_t * splash_label = NULL;

// ====== 左侧：UAS ID 列表 ======
#define MAX_LINES 10
static lv_obj_t * log_container = NULL;
static lv_obj_t * log_labels[MAX_LINES];
static int log_write_index = 0;

// ====== 右侧：最新无人机详情（14 行） ======
#define DETAIL_LINES 14
static lv_obj_t * detail_container = NULL;
static lv_obj_t * detail_labels[DETAIL_LINES];
static const char *detail_titles[DETAIL_LINES] = {
    "ID:", "MAC:", "RSSI:", "UA:", "IDT:", "MsgCnt:",
    "Cord:", "Spd/VSpd:", "Hdg:", "HRef:", "Acc(H/V/S):",
    "Time:", "OpCord:", "OpID:"
};

// ====== RID 数据结构 ======
struct RIDData {
    char mac[18];
    int rssi;
    uint8_t ua_type;
    uint8_t id_type;
    char uas_id[21];
    uint8_t msg_counter;
    uint8_t pack_count;

    double latitude;
    double longitude;
    float altitude;
    float speed;
    float heading;
    float speed_vertical;
    uint8_t height_ref;
    uint8_t horiz_acc;
    uint8_t vert_acc;
    uint8_t speed_acc;
    uint16_t timestamp;

    double op_latitude;
    double op_longitude;
    float op_altitude;
    uint8_t op_loc_type;
    uint16_t area_radius;
    float area_ceiling;
    float area_floor;

    char self_id[24];
    char operator_id[24];

    uint8_t op_status;
    bool has_basic;
    bool has_location;
    bool has_system;
    bool has_self_id;
    bool has_operator_id;
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

// ====== 蜂鸣 ======
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
        case 2: return "Heli/Multirotor";
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

// ====== 同时输出到串口和 SD 卡（追加模式） ======
void log_output(const char *msg) {
    Serial.print(msg);
    if (sdcardPort == NULL) return;

    esp_err_t ret = sdcardPort->SDPort_AppendFile(RID_LOG_FILE,
                                                   (char *)msg,
                                                   strlen(msg));
    if (ret != ESP_OK) {
        Serial.println("[SD] append failed!");
    }
}

// ====== 电池电压显示初始化 ======
void bat_label_init(void) {
    bat_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(bat_label, &lv_font_montserrat_12, 0);
    lv_obj_align(bat_label, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_label_set_text(bat_label, "BAT: --.--V");
}

// ====== 电池电压刷新定时器（30 秒一次） ======
void bat_update_timer(lv_timer_t * timer) {
    if (bat_label == NULL) return;
    int data;
    float vol = Adc_GetBatteryVoltage(&data);

    static float last_vol = 0;
    if (fabs(vol - last_vol) < 0.05f) return;
    last_vol = vol;

    char buf[24];
    snprintf(buf, sizeof(buf), "BAT: %.2fV", vol);
    lv_label_set_text(bat_label, buf);
}

// ====== 启动画面 ======
void splash_init(void) {
    splash_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(splash_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(splash_label, "DRONE SCANNER");
    lv_obj_align(splash_label, LV_ALIGN_CENTER, 0, 0);
}

// ====== 左侧列表初始化 ======
void log_init(void) {
    log_container = lv_obj_create(lv_scr_act());
    if (log_container == NULL) return;
    lv_obj_set_size(log_container, 199, 299);
    lv_obj_align(log_container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_flex_flow(log_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(log_container, LV_SCROLLBAR_MODE_AUTO);

    for (int i = 0; i < MAX_LINES; i++) {
        log_labels[i] = lv_label_create(log_container);
        lv_obj_set_width(log_labels[i], 190);
        lv_label_set_long_mode(log_labels[i], LV_LABEL_LONG_WRAP);
        lv_label_set_text(log_labels[i], "");
    }
    log_write_index = 0;
}

// ====== 右侧详情初始化 ======
void detail_init(void) {
    detail_container = lv_obj_create(lv_scr_act());
    if (detail_container == NULL) return;
    lv_obj_set_size(detail_container, 200, 280);
    lv_obj_align(detail_container, LV_ALIGN_TOP_RIGHT, 0, 10);
    lv_obj_set_flex_flow(detail_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(detail_container, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_set_style_pad_row(detail_container, 2, 0);
    lv_obj_set_style_pad_all(detail_container, 3, 0);

    for (int i = 0; i < DETAIL_LINES; i++) {
        detail_labels[i] = lv_label_create(detail_container);
        lv_obj_set_width(detail_labels[i], 190);
        lv_label_set_long_mode(detail_labels[i], LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(detail_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_pad_all(detail_labels[i], 0, 0);

        char buf[64];
        snprintf(buf, sizeof(buf), "%s --", detail_titles[i]);
        lv_label_set_text(detail_labels[i], buf);
    }

    lv_obj_add_flag(detail_container, LV_OBJ_FLAG_HIDDEN);
}

// ====== 更新右侧详情 ======
void detail_update(RIDData *rid) {
    if (detail_container == NULL) return;
    char buf[96];

    snprintf(buf, sizeof(buf), "ID: %s", rid->uas_id[0] ? rid->uas_id : "--");
    lv_label_set_text(detail_labels[0], buf);

    snprintf(buf, sizeof(buf), "MAC: %s", rid->mac);
    lv_label_set_text(detail_labels[1], buf);

    snprintf(buf, sizeof(buf), "RSSI: %d dBm", rid->rssi);
    lv_label_set_text(detail_labels[2], buf);

    snprintf(buf, sizeof(buf), "UA: %s", ua_type_to_string(rid->ua_type));
    lv_label_set_text(detail_labels[3], buf);

    snprintf(buf, sizeof(buf), "IDT: %d", rid->id_type);
    lv_label_set_text(detail_labels[4], buf);

    snprintf(buf, sizeof(buf), "MsgCnt: %d", rid->msg_counter);
    lv_label_set_text(detail_labels[5], buf);

    // 纬度 / 经度 / 高度
    snprintf(buf, sizeof(buf), "Cord: %.4f/%.4f/%.0fm",
             rid->latitude, rid->longitude, rid->altitude);
    lv_label_set_text(detail_labels[6], buf);

    // 水平速度 / 垂直速度
    snprintf(buf, sizeof(buf), "Spd/VSpd: %.2f/%.1f",
             rid->speed, rid->speed_vertical);
    lv_label_set_text(detail_labels[7], buf);

    // 航向角
    snprintf(buf, sizeof(buf), "Hdg: %.1f deg", rid->heading);
    lv_label_set_text(detail_labels[8], buf);

    snprintf(buf, sizeof(buf), "HRef: %d", rid->height_ref);
    lv_label_set_text(detail_labels[9], buf);

    // 水平 / 垂直 / 速度精度
    snprintf(buf, sizeof(buf), "Acc(H/V/S): %d/%d/%d",
             rid->horiz_acc, rid->vert_acc, rid->speed_acc);
    lv_label_set_text(detail_labels[10], buf);

    snprintf(buf, sizeof(buf), "Time: %u", rid->timestamp);
    lv_label_set_text(detail_labels[11], buf);

    // 操作员纬度 / 经度 / 高度
    snprintf(buf, sizeof(buf), "OpCord: %.4f/%.4f/%.0fm",
             rid->op_latitude, rid->op_longitude, rid->op_altitude);
    lv_label_set_text(detail_labels[12], buf);

    snprintf(buf, sizeof(buf), "OpID: %s", rid->operator_id[0] ? rid->operator_id : "--");
    lv_label_set_text(detail_labels[13], buf);
}

// ====== Wi-Fi 混杂模式回调 ======
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
                rid.msg_counter = d[4];
                rid.pack_count = d[7];

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

                            rid.speed_vertical = (int8_t)msg[16] * 0.5f;
                            rid.height_ref = (msg[17] >> 4) & 0x0F;
                            rid.horiz_acc = msg[18] & 0x0F;
                            rid.vert_acc = msg[19] & 0x0F;
                            rid.speed_acc = msg[20] & 0x0F;
                            memcpy(&rid.timestamp, msg + 21, 2);

                            rid.has_location = true;
                        }
                        else if (msg_type == 0x3) {
                            memcpy(rid.self_id, msg + 2, 23);
                            rid.self_id[23] = '\0';
                            rid.has_self_id = true;
                        }
                        else if (msg_type == 0x4) {
                            rid.op_loc_type = (msg[1] >> 4) & 0x0F;

                            int32_t op_lat, op_lon;
                            memcpy(&op_lat, msg + 2, 4);
                            memcpy(&op_lon, msg + 6, 4);
                            rid.op_latitude = op_lat * 1e-7;
                            rid.op_longitude = op_lon * 1e-7;

                            uint16_t op_alt;
                            memcpy(&op_alt, msg + 10, 2);
                            rid.op_altitude = (op_alt * 0.5f) - 1000.0f;

                            memcpy(&rid.area_radius, msg + 14, 2);

                            uint16_t ceiling, floor;
                            memcpy(&ceiling, msg + 16, 2);
                            memcpy(&floor, msg + 18, 2);
                            rid.area_ceiling = ceiling * 0.5f;
                            rid.area_floor = floor * 0.5f;

                            rid.has_system = true;
                        }
                        else if (msg_type == 0x5) {
                            memcpy(rid.operator_id, msg + 2, 20);
                            rid.operator_id[20] = '\0';
                            rid.has_operator_id = true;
                        }
                    }
                    msg_offset += 25;
                }

                static uint32_t last_print_time = 0;
                uint32_t now = millis();
                bool should_print = (now - last_print_time >= 10000);

                if ((rid.has_basic || rid.has_location) && should_print) {
                    last_print_time = now;

                    beep_once();

                    char out[512];
                    int pos = 0;
                    pos += snprintf(out + pos, sizeof(out) - pos, "========== RID ==========\n");
                    pos += snprintf(out + pos, sizeof(out) - pos, "MAC       : %s\n", rid.mac);
                    pos += snprintf(out + pos, sizeof(out) - pos, "RSSI      : %d dBm\n", rid.rssi);
                    if (rid.has_basic) {
                        pos += snprintf(out + pos, sizeof(out) - pos, "UAS ID    : %s\n", rid.uas_id);
                        pos += snprintf(out + pos, sizeof(out) - pos, "ID Type   : %d\n", rid.id_type);
                        pos += snprintf(out + pos, sizeof(out) - pos, "UA Type   : %d (%s)\n", rid.ua_type, ua_type_to_string(rid.ua_type));
                    }
                    if (rid.has_location) {
                        pos += snprintf(out + pos, sizeof(out) - pos, "Latitude  : %.7f\n", rid.latitude);
                        pos += snprintf(out + pos, sizeof(out) - pos, "Longitude : %.7f\n", rid.longitude);
                        pos += snprintf(out + pos, sizeof(out) - pos, "Altitude  : %.1f m\n", rid.altitude);
                        pos += snprintf(out + pos, sizeof(out) - pos, "Speed     : %.2f m/s\n", rid.speed);
                        pos += snprintf(out + pos, sizeof(out) - pos, "Heading   : %.1f deg\n", rid.heading);
                        pos += snprintf(out + pos, sizeof(out) - pos, "Status    : %d\n", rid.op_status);
                    }
                    pos += snprintf(out + pos, sizeof(out) - pos, "RAW Len   : %d\n", ie_len);
                    pos += snprintf(out + pos, sizeof(out) - pos, "RAW Data  : ");
                    for (int i = 0; i < ie_len && pos < (int)sizeof(out) - 4; i++) {
                        pos += snprintf(out + pos, sizeof(out) - pos, "%02X ", d[i]);
                        if ((i + 1) % 16 == 0 && pos < (int)sizeof(out) - 2) {
                            pos += snprintf(out + pos, sizeof(out) - pos, "\n            ");
                        }
                    }
                    pos += snprintf(out + pos, sizeof(out) - pos, "\n==========================\n");

                    log_output(out);
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

// ====== LVGL 定时器：左侧列表 + 右侧详情 ======
void update_rid_display(lv_timer_t * timer) {
    if (rid_queue == NULL || log_container == NULL) return;

    RIDData rid;
    int processed = 0;

    while (xQueueReceive(rid_queue, &rid, 0) == pdTRUE && processed < 3) {
        if (!rid.has_basic || rid.uas_id[0] == '\0') continue;

        if (splash_label != NULL) {
            lv_obj_del(splash_label);
            splash_label = NULL;

            if (detail_container != NULL) {
                lv_obj_clear_flag(detail_container, LV_OBJ_FLAG_HIDDEN);
            }
        }

        detail_update(&rid);

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

// ====== 初始化 ======
void setup() {
    setCpuFrequencyMhz(80);

    Serial.begin(115200);
    delay(500);
    Serial.println("\n===== BOOT =====");
    Serial.printf("CPU freq: %d MHz\n", getCpuFrequencyMhz());

    sdcardPort = new CustomSDPort("/sdcard");

    codecport = new CodecPort(I2cbus, "S3_RLCD_4_2");
    codecport->CodecPort_SetInfo("es8311", 1, 16000, 1, 16);
    codecport->CodecPort_SetSpeakerVol(80);

    RlcdPort.RLCD_Init();
    Adc_PortInit();

    Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
    if (Lvgl_lock(-1)) {
        lv_tick_set_cb(millis);
        log_init();
        detail_init();
        bat_label_init();
        splash_init();
        lv_timer_create(update_rid_display, 3000, NULL);
        lv_timer_create(bat_update_timer, 30000, NULL);
        Lvgl_unlock();
    }

    rid_queue = xQueueCreate(20, sizeof(RIDData));

    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

    Serial.println("Setup done");
    beep_once();
}

void loop() {
    delay(1000);
}