#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "src/ui_src/generated/gui_guider.h"

static lv_ui init_ui;

DisplayPort RlcdPort(12, 11, 5, 40, 41, 400, 300);

// ====== 屏幕分辨率定义 ======
#define SCREEN_WIDTH  400
#define SCREEN_HEIGHT 300

#define LOG_LINES 10                          // 固定 10 行
static lv_obj_t * log_container;              // 日志容器
static lv_obj_t * log_labels[LOG_LINES];      // 固定 10 个 Label，永不删除
static int log_write_index = 0;               // 下一个要写入的 Label 索引（环形）

// 全局变量：屏幕上的 Label 对象
static lv_obj_t * random_label;

// 生成随机字符串的函数
// 将随机字符写入传入的缓冲区
void generate_random_string(char * buffer, size_t max_len) {
    // 可选的字符集：字母和数字
    const char charset[] = "0123456789ABCDEF"; //ABCDEFGHIJKLMNOPQRSTUVWXYZ
    size_t charset_size = sizeof(charset) - 1;

    // 随机生成长度在 5 到 max_len-1 之间的字符串
    size_t len = 2; //random(5, max_len - 1);

    for (size_t i = 0; i < len; i++) {
        buffer[i] = charset[random(charset_size)];
    }
    buffer[len] = '\0'; // 字符串结束符
}

// 定时器回调函数：定期更新 Label 文本
void update_label_timer(lv_timer_t * timer) {
    // 创建一个缓冲区存放随机字符串
    char buffer[64];
    generate_random_string(buffer, sizeof(buffer));

    // 1. 更新 Label 的显示内容
    //lv_label_set_text(random_label, buffer);
    log_add(buffer);

    // 2. 同时通过串口输出相同内容
    Serial.print("Log: ");
    Serial.println(buffer);
}

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

void log_init(void) {
    // 创建容器，高度根据屏幕和字体大小调整
    log_container = lv_obj_create(lv_scr_act());
    if (log_container == NULL) {
        Serial.println("ERROR: log_container creation failed!");
        return;
    }
    lv_obj_set_size(log_container, 390, 290);
    lv_obj_align(log_container, LV_ALIGN_TOP_MID, 0, 0);
    
    // 关键：设置 Flex 布局为垂直方向
    lv_obj_set_flex_flow(log_container, LV_FLEX_FLOW_COLUMN);
    
    // 设置滚动条模式
    lv_obj_set_scrollbar_mode(log_container, LV_SCROLLBAR_MODE_AUTO);
    // 一次性创建 10 个 Label，初始为空
    for (int i = 0; i < LOG_LINES; i++) {
        log_labels[i] = lv_label_create(log_container);
        if (log_labels[i] == NULL) {
            Serial.print("ERROR: log_labels[");
            Serial.print(i);
            Serial.println("] creation failed!");
            return;
        }
        lv_label_set_text(log_labels[i], "drone x...");
    }
    log_write_index = 0;
}

void log_add(const char * msg) {
  if (log_container == NULL) {
      Serial.println("ERROR: log_container is NULL, skipping log add");
      return;
  }

  // 去重：检查当前 10 个 Label 中是否已有相同内容
  for (int i = 0; i < LOG_LINES; i++) {
      const char * existing = lv_label_get_text(log_labels[i]);
      if (existing != NULL && strcmp(existing, msg) == 0) {
          Serial.print("Duplicate, skipped: ");
          Serial.println(msg);
          return;
      }
  }

  // 写入当前索引的 Label
  lv_label_set_text(log_labels[log_write_index], msg);

  // 环形推进索引
  log_write_index = (log_write_index + 1) % LOG_LINES;

  // 滚动到底部（最后一行）
  // 注意：由于是环形写入，最后写入的位置不一定是屏幕最下方
  // 这里直接滚到容器底部即可
  lv_obj_scroll_to_y(log_container, lv_obj_get_scroll_bottom(log_container), LV_ANIM_OFF);
}

void setup() {
  Serial.begin(115200);

  RlcdPort.RLCD_Init();
  Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
  if (Lvgl_lock(-1)) {
  //  setup_ui(&init_ui);
    Lvgl_unlock();
  }

    // 关键：将 Arduino 的 millis() 作为 LVGL 的时间源
  lv_tick_set_cb(millis);  // 注意：部分旧版库可能不支持此函数

  // 注意：屏幕分辨率需要在显示驱动初始化时指定
  // 不同驱动库的写法不同，通常类似下面这样：
  // lv_disp_drv_t disp_drv;
  // disp_drv.hor_res = SCREEN_WIDTH;
  // disp_drv.ver_res = SCREEN_HEIGHT;
  //
  // 如果你用的是 lvgl_arduino 或 TFT_eSPI 等库，
  // 分辨率通常在库的配置文件中设置，或作为参数传入。
  // my_disp_init(SCREEN_WIDTH, SCREEN_HEIGHT);

  // // --- 2. 创建 Label 并设置为循环滚动模式 ---
  // random_label = lv_label_create(lv_scr_act());

  // // 设置长文本模式为循环滚动
  // lv_label_set_long_mode(random_label, LV_LABEL_LONG_SCROLL_CIRCULAR);

  // // 设置 Label 宽度（在滚动模式下，宽度决定滚动区域）
  // // 这里用屏幕宽度减去两边留白
  // lv_obj_set_width(random_label, SCREEN_WIDTH - 20);

  // // 让 Label 内部文本居中对齐（可选）
  // lv_obj_set_style_text_align(random_label, LV_TEXT_ALIGN_CENTER, 0);

  // // 将 Label 放在屏幕中央
  // lv_obj_align(random_label, LV_ALIGN_CENTER, 0, 0);

  // // 设置初始文本
  // lv_label_set_text(random_label, "Initializing...");
  log_init();
  //log_add("Initializing...");


  // 初始化时也输出一次串口信息
  Serial.println("Log: Initializing...");

  // --- 3. 创建定时器，每 1 秒更新一次随机文本 ---
  lv_timer_create(update_label_timer, 1000, NULL);
}

void loop() {
  // lv_obj_clear_flag(init_ui.screen_img_1, LV_OBJ_FLAG_HIDDEN);
  // lv_obj_add_flag(init_ui.screen_img_2, LV_OBJ_FLAG_HIDDEN);
  // vTaskDelay(pdMS_TO_TICKS(1500));
  // lv_obj_clear_flag(init_ui.screen_img_2, LV_OBJ_FLAG_HIDDEN);
  // lv_obj_add_flag(init_ui.screen_img_1, LV_OBJ_FLAG_HIDDEN);
  // vTaskDelay(pdMS_TO_TICKS(1500));
  //lv_timer_handler();
  //delay(5);
}