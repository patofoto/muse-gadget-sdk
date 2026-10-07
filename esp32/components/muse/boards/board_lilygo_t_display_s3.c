/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * LilyGO T-Display-S3 (no touch): ESP32-S3R8 (8 MB octal PSRAM) with 16 MB quad
 * flash, a 1.9" 170x320 ST7789 LCD on an 8-bit I80 (parallel) bus, an AW9364
 * backlight driver, two buttons under the screen (BOOT on GPIO0, IO14), a
 * battery connector read through a 1:2 divider on GPIO4, and no audio. GPIO15
 * switches the LCD's power, which it needs on battery. Pins and the panel setup
 * are from LilyGO's T-Display-S3 repo (examples/factory/pin_config.h and
 * factory.ino) and its schematic (schematic/T_Display_S3.pdf).
 */
#include "driver/rtc_io.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_board.h"
#include "muse_mem.h"

static const char *TAG = "board";

#define LCD_W 170
#define LCD_H 320
#define LCD_GAP_X 35               /* the 170 columns sit in the middle of the controller's 240 */
#define LCD_PCLK_HZ (16 * 1000 * 1000)  /* LilyGO's clock; faster can garble the picture */
#define LCD_D0 GPIO_NUM_39
#define LCD_D1 GPIO_NUM_40
#define LCD_D2 GPIO_NUM_41
#define LCD_D3 GPIO_NUM_42
#define LCD_D4 GPIO_NUM_45
#define LCD_D5 GPIO_NUM_46
#define LCD_D6 GPIO_NUM_47
#define LCD_D7 GPIO_NUM_48
#define LCD_WR GPIO_NUM_8
#define LCD_RD GPIO_NUM_9          /* unused, held high */
#define LCD_DC GPIO_NUM_7
#define LCD_CS GPIO_NUM_6
#define LCD_RST GPIO_NUM_5
#define LCD_BL GPIO_NUM_38         /* the AW9364's enable */
#define LCD_POWER GPIO_NUM_15      /* high: LCD powered when running on battery */
#define DRAW_BUF_LINES 32

#define TALK_GPIO GPIO_NUM_0       /* BOOT, bottom left under the screen */
#define AUX_GPIO GPIO_NUM_14       /* IO14, bottom right */

#define BATT_ADC ADC_CHANNEL_3     /* GPIO4, behind a 1:2 divider */

/* The AW9364 dims in 16 steps counted on its enable pin: driven high it starts
 * at full brightness, each low pulse of 0.5 to 500 us steps one level down
 * (from the dimmest it wraps to full), and low for over 2.5 ms turns it off.
 * PWM would count as a stream of pulses, so this follows setBrightness() in
 * LilyGO's factory firmware instead. */
#define BL_STEPS 16

static esp_lcd_panel_handle_t s_panel;
static muse_gpio_button_t s_talk, s_aux;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static int s_bl_level;             /* 0 off, else 1..BL_STEPS */
static portMUX_TYPE s_bl_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t init(void)
{
    /* LCD power on, the backlight off until the panel has a picture, and the
     * I80 bus's unused read strobe idle. */
    const gpio_config_t out = {
        .pin_bit_mask = BIT64(LCD_POWER) | BIT64(LCD_BL) | BIT64(LCD_RD),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out), TAG, "lcd gpios");
    gpio_set_level(LCD_BL, 0);
    gpio_set_level(LCD_RD, 1);
    gpio_set_level(LCD_POWER, 1);

    const adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc");
    const adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BATT_ADC, &ch_cfg), TAG, "battery adc");
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BATT_ADC,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali), TAG, "adc calibration");

    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_talk, TALK_GPIO), TAG, "talk button");
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_aux, AUX_GPIO), TAG, "aux button");
    /* A button that woke the board from power-off is still down; don't count it. */
    s_talk.pressed = gpio_get_level(TALK_GPIO) == 0;
    s_aux.pressed = gpio_get_level(AUX_GPIO) == 0;
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    (void)touch;
    esp_lcd_i80_bus_handle_t bus;
    const esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num = LCD_DC,
        .wr_gpio_num = LCD_WR,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = { LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7 },
        .bus_width = 8,
        .max_transfer_bytes = LCD_W * DRAW_BUF_LINES * 2,
        .dma_burst_size = 64,
    };
    if (esp_lcd_new_i80_bus(&bus_cfg, &bus) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
    };
    if (esp_lcd_new_panel_io_i80(bus, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    /* Portrait, the USB-C port and both buttons below the screen. */
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, 0);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_W,
            .ver_res = LCD_H,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    int level = pct <= 0 ? 0 : (pct * BL_STEPS + 99) / 100;
    if (level > BL_STEPS) {
        level = BL_STEPS;
    }
    if (level == s_bl_level) {
        return;
    }
    if (level == 0) {
        gpio_set_level(LCD_BL, 0);
        esp_rom_delay_us(3000);     /* long enough to count as off, not as a step */
        s_bl_level = 0;
        return;
    }
    if (s_bl_level == 0) {
        gpio_set_level(LCD_BL, 1);
        esp_rom_delay_us(30);
        s_bl_level = BL_STEPS;
    }
    int pulses = (s_bl_level - level + BL_STEPS) % BL_STEPS;
    /* An interrupt stretching a low pulse past 500 us would lose the count. */
    portENTER_CRITICAL(&s_bl_lock);
    for (int i = 0; i < pulses; i++) {
        gpio_set_level(LCD_BL, 0);
        esp_rom_delay_us(1);
        gpio_set_level(LCD_BL, 1);
        esp_rom_delay_us(1);
    }
    portEXIT_CRITICAL(&s_bl_lock);
    s_bl_level = level;
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);   /* SLPIN/SLPOUT; GRAM is kept */
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

static unsigned poll_buttons(void)
{
    return muse_gpio_button_poll(&s_talk) | muse_gpio_button_poll(&s_aux) << 2;
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_talk, &s_aux }, 2, timeout_ms);
}

/* Voltage only: nothing tells the ESP32 whether USB is in or the battery is
 * charging. With no battery, GPIO4 sees the charger's output, over 4.3 V
 * (LilyGO's GetBatteryVoltage example). */
static esp_err_t read_power(muse_power_t *out)
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int raw;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, BATT_ADC, &raw), TAG, "adc read");
        sum += raw;
    }
    int mv;
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, sum / 8, &mv), TAG, "adc calibration");
    int v = mv * 2;
    out->charging = false;
    if (v > 4300) {
        out->usb = true;
        out->battery_mv = 0;
        out->battery_pct = -1;
        return ESP_OK;
    }
    out->usb = false;
    out->battery_mv = v;
    if (v < 2500) {
        out->battery_pct = -1;
        return ESP_OK;
    }
    /* The Watcher's LiPo curve, as on the StickS3. */
    int pct = (-v * v + 9016 * v - 19189000) / 10000;
    out->battery_pct = pct < 0 ? 0 : pct > 100 ? 100 : pct;
    return ESP_OK;
}

/*
 * Backlight and panel off and the LCD unpowered, then deep sleep until either
 * button is pressed. On battery there's no power switch: the battery feeds the
 * regulator directly, so this is as off as the board gets.
 */
static esp_err_t power_off(void)
{
    set_brightness(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    esp_lcd_panel_disp_sleep(s_panel, true);
    gpio_set_level(LCD_POWER, 0);
    while (gpio_get_level(TALK_GPIO) == 0 || gpio_get_level(AUX_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    /* Keep the buttons' pull-ups powered through deep sleep, as the StopWatch
     * does; LilyGO's sleep example relies on the board's own. */
    const gpio_num_t keys[] = { TALK_GPIO, AUX_GPIO };
    for (int i = 0; i < 2; i++) {
        rtc_gpio_pullup_en(keys[i]);
        rtc_gpio_pulldown_dis(keys[i]);
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup_io(BIT64(TALK_GPIO) | BIT64(AUX_GPIO), ESP_EXT1_WAKEUP_ANY_LOW),
                        TAG, "button wake");
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "LilyGO T-Display-S3",
    .width = LCD_W,
    .height = LCD_H,
    .round = false,
    .touch = false,
    .diagonal_in = 1.9f,
    .talk_button = "boot",
    .aux_button = "right",
    /* Both buttons sit just under the screen, BOOT on the left and IO14 on
     * the right, either side of the USB-C port. */
    .talk_hint = { LV_ALIGN_BOTTOM_LEFT, 6, -4 },
    .aux_hint = { LV_ALIGN_BOTTOM_RIGHT, -6, -4 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
    .read_power = read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
