#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <cstdlib>
#include "i2c_bus.h"
#include <driver/ledc.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"
#include <esp_timer.h>
#include "esp_io_expander_tca9554.h"
#include <cJSON.h>
#include <vector>
#include <cstring>

#include "esp_private/sdmmc_common.h"
#include <esp_vfs_fat.h>
#include <driver/sdspi_host.h>

#include "audio_player.h"
#include <dirent.h>
#include <sys/stat.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "power_manager.h"
#include "power_save_timer.h"
#include "mcp_server.h"

#define TAG "waveshare_lcd_1_85c"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

static const st77916_lcd_init_cmd_t vendor_specific_init_new[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){0x00}, 1, 0},
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0x29, (uint8_t []){0x00}, 1, 0},  
};

class CustomLcdDisplay : public SpiLcdDisplay
{
    lv_obj_t* music_label_ = nullptr;
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width,
                     int height,
                     int offset_x,
                     int offset_y,
                     bool mirror_x,
                     bool mirror_y,
                     bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy)
    {
        // Note: UI customization should be done in SetupUI(), not in constructor
        // to ensure lvgl objects are created before accessing them
    }

    virtual void SetupUI() override {
        // Call parent SetupUI() first to create all base LVGL objects.
        // It handles its own locking, so do not lock before this call.
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        lv_display_set_default(display_);

        /*
            auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
            auto text_font = lvgl_theme->text_font()->font();
            auto icon_font = lvgl_theme->icon_font()->font();

            lv_obj_set_size(top_bar_, LV_HOR_RES, text_font->line_height);
            lv_obj_set_style_layout(top_bar_, LV_LAYOUT_NONE, 0);
            lv_obj_set_style_pad_top(top_bar_, 10, 0);
            lv_obj_set_style_pad_bottom(top_bar_, 1, 0);

            lv_obj_set_size(status_bar_, LV_HOR_RES, text_font->line_height);
            lv_obj_set_style_layout(status_bar_, LV_LAYOUT_NONE, 0);
            lv_obj_set_style_pad_top(status_bar_, 10, 0);
            lv_obj_set_style_pad_bottom(status_bar_, 1, 0);
            lv_obj_set_y(status_bar_, text_font->line_height);
            lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_IGNORE_LAYOUT);

            // Reparent mute and battery labels to top_bar_ to allow absolute positioning
            lv_obj_set_parent(mute_label_, top_bar_);
            lv_obj_set_parent(battery_label_, top_bar_);
            lv_obj_set_style_margin_left(battery_label_, 0, 0);

            // 针对圆形屏幕调整位置
            //      network  mute  battery     //
            //               status            //
            lv_obj_align(network_label_, LV_ALIGN_TOP_MID, -1.5 * icon_font->line_height, 0);
            lv_obj_align(mute_label_, LV_ALIGN_TOP_MID, 1.0 * icon_font->line_height, 0);
            lv_obj_align(battery_label_, LV_ALIGN_TOP_MID, 2.5 * icon_font->line_height, 0);
            
            lv_obj_align(status_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_flex_grow(status_label_, 0);
            lv_obj_set_width(status_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            lv_obj_align(notification_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_width(notification_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(notification_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            lv_obj_align(low_battery_popup_, LV_ALIGN_BOTTOM_MID, 0, -20);
            lv_obj_set_style_bg_color(low_battery_popup_, lv_color_hex(0xFF0000), 0);
            lv_obj_set_width(low_battery_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(low_battery_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            // 针对圆形屏幕调整底部对话框位置，避免被圆角遮挡
            lv_obj_set_style_pad_bottom(bottom_bar_, 30, 0);
            lv_obj_set_width(chat_message_label_, LV_HOR_RES * 0.75); // 限制宽度，避免文字贴边
        */

        // 状态栏容器适配
        if (top_bar_ != nullptr) {
            lv_obj_set_style_pad_left(top_bar_, LV_HOR_RES * 0.33, 0);  // 左侧填充12%
            lv_obj_set_style_pad_right(top_bar_, LV_HOR_RES * 0.33, 0); // 右侧填充12%
        }

        // 创建播放状态图标
        music_label_ = lv_label_create(top_bar_);
        lv_label_set_text(music_label_, "");
        lv_obj_add_flag(music_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(music_label_, LV_ALIGN_CENTER, -40, 0);

        // 表情容器上移适配
        if (emoji_box_ != nullptr) {
            lv_obj_align(emoji_box_, LV_ALIGN_CENTER, 0, -10);          // 向上偏移30
        }

        // 消息栏适配
        if (bottom_bar_ != nullptr) {
            lv_obj_align(bottom_bar_, LV_ALIGN_BOTTOM_MID, 0, 60);     // 向上偏移20
        }
    }

    void UpdateMusicStatus(bool playing, bool paused) {
        DisplayLockGuard lock(this);
        if (music_label_ == nullptr) return;
        
        if (!playing) {
            lv_obj_add_flag(music_label_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(music_label_, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(music_label_, paused ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
            lv_obj_set_style_text_color(music_label_, paused ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN), 0);
        }
    }
};


class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;

        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE(TAG, "Failed to create touch semaphore");
        }
    }

    ~Cst816s()
    {
        delete[] read_buffer_;

        // Delete semaphore if it exists
        if (touch_isr_mux_ != NULL) {
            vSemaphoreDelete(touch_isr_mux_);
            touch_isr_mux_ = NULL;
        }
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched && !was_touched_) {
            // Press event (transition from not touched to touched)
            press_count_++;
            event = TOUCH_PRESS;
            ESP_LOGI(TAG, "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            // Release event (transition from touched to not touched)
            event = TOUCH_RELEASE;
            ESP_LOGI(TAG, "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            // Continuous touch (hold)
            event = TOUCH_HOLD;
            ESP_LOGD(TAG, "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
        }

        // Update previous state
        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

    // Semaphore management methods
    SemaphoreHandle_t GetTouchSemaphore()
    {
        return touch_isr_mux_;
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY)
    {
        if (touch_isr_mux_ != NULL) {
            return xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent()
    {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;

    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_;
};

// 用于保存原有的 SD SPI 事务处理函数
static esp_err_t (*orig_sd_do_transaction)(int slot, sdmmc_command_t *cmdinfo) = nullptr;

class CustomBoard : public WifiBoard {
private:
    Button boot_button_;
    i2c_master_bus_handle_t i2c_bus_;
    esp_io_expander_handle_t io_expander = NULL;
    Display* display_ = nullptr;
    Cst816s* cst816s_;
    esp_lcd_touch_handle_t tp;   // LCD touch handle
    bool is_sdcard_found = false;
    TaskHandle_t touch_task_handle_ = nullptr;
    esp_timer_handle_t emotion_reset_timer_ = nullptr;
    PowerManager* power_manager_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;
    static CustomBoard* instance_;

    // 静态成员函数，用于控制 SD 卡片选
    static esp_err_t sd_cs_set_level(int level) {
        if (instance_ && instance_->io_expander) {
            return esp_io_expander_set_level(instance_->io_expander, IO_EXPANDER_PIN_NUM_2, level);
        }
        return ESP_FAIL;
    }

    // 包装后的事务处理函数，在操作前后切换 CS
    static esp_err_t sd_do_transaction(int slot, sdmmc_command_t *cmdinfo) {
        sd_cs_set_level(0); // 选中 SD 卡
        esp_err_t ret = orig_sd_do_transaction(slot, cmdinfo);
        sd_cs_set_level(1); // 释放 SD 卡
        return ret;
    }

    // Audio player callbacks
    static esp_err_t audio_mute_callback(AUDIO_PLAYER_MUTE_SETTING setting) {
        return ESP_OK;
    }

    static esp_err_t audio_reconfig_callback(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t slot_mode) {
        ESP_LOGI(TAG, "Audio reconfig: rate=%lu, bits=%lu, slot_mode=%d", rate, bits_cfg, (int)slot_mode);
        return ESP_OK;
    }

    static esp_err_t audio_write_callback(void *audio_buffer, size_t size, size_t *bytes_written, uint32_t timeout_ms) {
        auto codec = Board::GetInstance().GetAudioCodec();
        if (codec) {
            // 避免在循环中创建 std::vector，这会导致频繁的堆内存分配导致卡顿
            // 直接调用 codec 的原始输出方法（如果 AudioCodec 支持指针接口）
            // 如果项目中的 AudioCodec 只接受 vector，请确保该 vector 是预先分配好的
            std::vector<int16_t> data((int16_t*)audio_buffer, (int16_t*)audio_buffer + size / 2);
            codec->OutputData(data);
            *bytes_written = size;
            return ESP_OK;
        }
        return ESP_FAIL;
    }

    static void emotion_reset_timer_callback(void* arg)
    {
        auto* self = static_cast<CustomBoard*>(arg);
        if (self && self->display_ != nullptr) {
            self->display_->SetEmotion("neutral");
        }
    }

    void ShowTemporaryEmotion(const char* emotion, uint32_t duration_ms)
    {
        if (display_ == nullptr || emotion == nullptr) {
            return;
        }
        display_->SetEmotion(emotion);
        if (emotion_reset_timer_ != nullptr) {
            esp_timer_stop(emotion_reset_timer_);
            esp_timer_start_once(emotion_reset_timer_, static_cast<uint64_t>(duration_ms) * 1000ULL);
        }
    }

    void ShowHappyTouchFeedback()
    {
        static int64_t s_last_us = 0;
        constexpr int64_t kCooldownUs = 1200000;
        const int64_t now = esp_timer_get_time();
        if ((now - s_last_us) < kCooldownUs) {
            return;
        }
        s_last_us = now;
        ShowTemporaryEmotion("happy", 2000);
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(BATTERY_CHARGING_PIN, BATTERY_ADC_PIN, BATTERY_EN_PIN);
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            auto display = GetDisplay();
            display->SetChatMessage("system", "");
            display->SetEmotion("sleepy");
            GetBacklight()->SetBrightness(20);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            auto display = GetDisplay();
            display->SetChatMessage("system", "");
            display->SetEmotion("neutral");
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = I2C_SDA_IO,
            .scl_io_num = I2C_SCL_IO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }
    
    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                // 缩短超时时间，避免扫描太慢
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(100));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
        ESP_LOGI(TAG, "I2C scan finished");
    }


    void InitializeTca9554(void)
    {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, I2C_ADDRESS, &io_expander);
        if(ret != ESP_OK)
            ESP_LOGE(TAG, "TCA9554 create returned error");        

        // 设置 EXIO0, EXIO1, EXIO2 为输出模式
        ret = esp_io_expander_set_dir(io_expander, 
                                      IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2, 
                                      IO_EXPANDER_OUTPUT);
        ESP_ERROR_CHECK(ret);

        // 初始时将 EXIO0, EXIO1, EXIO2 都设置为高电平。
        // EXIO0/1 用于 LCD/TouchPad 复位，EXIO2 用于 SD CS (非激活状态)
        ret = esp_io_expander_set_level(io_expander, 
                                      IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2, 
                                      1);
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(300));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(300));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 1);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
    }

    static void touch_isr_callback(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad != nullptr) {
            touchpad->NotifyTouchEvent();
        }
    }

    static void touch_event_task(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &app = Application::GetInstance();
                auto &board = (CustomBoard &)Board::GetInstance();

                ESP_LOGD(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (app.GetDeviceState() == kDeviceStateStarting) {
                        board.EnterWifiConfigMode();
                    } else {
                        audio_player_pause(); // 触摸屏幕开始/切换对话时暂停音乐
                        app.ToggleChatState();
                    }
                }
            }
        }
    }

    void InitializeCst816sTouchPad()
    {
        cst816s_ = new Cst816s(i2c_bus_, 0x15);

        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, cst816s_, 5, &touch_task_handle_, 1);

        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            // .intr_type = GPIO_INTR_NEGEDGE
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&int_gpio_config);
        gpio_install_isr_service(0);
        gpio_intr_enable(TP_PIN_NUM_INT);
        gpio_isr_handler_add(TP_PIN_NUM_INT, CustomBoard::touch_isr_callback, cst816s_);
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");

        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                        QSPI_PIN_NUM_LCD_DATA0,
                                                                        QSPI_PIN_NUM_LCD_DATA1,
                                                                        QSPI_PIN_NUM_LCD_DATA2,
                                                                        QSPI_PIN_NUM_LCD_DATA3,
                                                                        QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");

        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = QSPI_PIN_NUM_LCD_CS,               
            .dc_gpio_num = -1,                  
            .spi_mode = 0,                     
            .pclk_hz = 3 * 1000 * 1000,      
            .trans_queue_depth = 10,            
            .on_color_trans_done = NULL,                            
            .user_ctx = NULL,                   
            .lcd_cmd_bits = 32,                 
            .lcd_param_bits = 8,                
            .flags = {                          
            .dc_low_on_data = 0,            
            .octal_mode = 0,                
            .quad_mode = 1,                 
            .sio_mode = 0,                  
            .lsb_first = 0,                 
            .cs_high_active = 0,            
            },                                  
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install ST77916 panel driver");
        
        st77916_vendor_config_t vendor_config = {
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        
        printf("-------------------------------------- Version selection -------------------------------------- \r\n");
        esp_err_t ret;
        int lcd_cmd = 0x04;
        uint8_t register_data[4]; 
        size_t param_size = sizeof(register_data);
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_READ_CMD << 24;  // Use the read opcode instead of write
        ret = esp_lcd_panel_io_rx_param(panel_io, lcd_cmd, register_data, param_size); 
        if (ret == ESP_OK) {
            printf("Register 0x04 data: %02x %02x %02x %02x\n", register_data[0], register_data[1], register_data[2], register_data[3]);
        } else {
            printf("Failed to read register 0x04, error code: %d\n", ret);
        } 
        // panel_io_spi_del(io_handle);
        esp_lcd_panel_io_del(panel_io);
        // 降低 SPI 频率以提高稳定性
        io_config.pclk_hz = 40 * 1000 * 1000;
        if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io) != ESP_OK) {
            printf("Failed to set LCD communication parameters -- SPI\r\n");
            return ;
        }
        printf("LCD communication parameters are set successfully -- SPI\r\n");
        
        // Check register values and configure accordingly
        if (register_data[0] == 0x00 && register_data[1] == 0x7F && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            // Handle the case where the register data matches this pattern
            printf("Vendor-specific initialization for case 1.\n");
        }
        else if (register_data[0] == 0x00 && register_data[1] == 0x02 && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            // Provide vendor-specific initialization commands if register data matches this pattern
            vendor_config.init_cmds = vendor_specific_init_new;
            vendor_config.init_cmds_size = sizeof(vendor_specific_init_new) / sizeof(st77916_lcd_init_cmd_t);
            printf("Vendor-specific initialization for case 2.\n");
        }
        printf("------------------------------------- End of version selection------------------------------------- \r\n");
 
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18)
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new CustomLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeSDcardSpi() {
        spi_bus_config_t bus_cnf = {
            .mosi_io_num = SD_CMD,
            .miso_io_num = SD_DATA0,
            .sclk_io_num = SD_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 400000,
        };

        esp_err_t err = spi_bus_initialize(SD_SPI_HOST, &bus_cnf, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI总线初始化失败: %s", esp_err_to_name(err));
            return;
        }
        
        static sdspi_device_config_t slot_cnf = {
            .host_id = SD_SPI_HOST,
            .gpio_cs = SD_CS,
            .gpio_cd = SDSPI_SLOT_NO_CD,
            .gpio_wp = GPIO_NUM_NC,
            .gpio_int = GPIO_NUM_NC,
        };
        
        esp_vfs_fat_sdmmc_mount_config_t mount_cnf = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
        };
        
        sdmmc_card_t* card = NULL;

        // 分配自定义的 CS 控制函数
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = SD_SPI_HOST; // 必须指定为 SPI3_HOST，否则默认为 SPI2 会与 LCD 冲突
        // 挂钩 do_transaction 以便在每次操作时自动切换 CS 引脚
        orig_sd_do_transaction = host.do_transaction;
        host.do_transaction = sd_do_transaction;

        err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cnf, &mount_cnf, &card);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SD卡挂载失败: %s", esp_err_to_name(err));
            is_sdcard_found = false;
            return;
        } else if (err == ESP_OK) {
            ESP_LOGI(TAG, "SD卡挂载成功");
            is_sdcard_found = true;
        }
        // sdmmc_card_print_info(stdout, card); // 打印SD卡信息
        ESP_LOGI(TAG, "SD card initialization finished");
    }


    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            audio_player_pause(); // 按键开始/切换对话时暂停音乐
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        mcp_server.AddTool("self.music.list_songs",
            "列出SD卡中所有的MP3歌曲文件。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return ListSongs();
            });

        mcp_server.AddTool("self.music.play_song",
            "从SD卡播放指定的MP3歌曲。请提供歌曲名称。",
            PropertyList({
                Property("name", kPropertyTypeString, "要播放的歌曲名称")
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto name = properties["name"].value<std::string>();
                return PlayMp3(name);
            });

        mcp_server.AddTool("self.music.resume_song",
            "恢复已暂停的音乐播放。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return ResumeMp3();
            });

        mcp_server.AddTool("self.music.stop_song",
            "停止当前的音乐播放。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return StopMp3();
            });
    }

    ReturnValue ListSongs() {
        if (!is_sdcard_found) {
            return "SD卡未挂载。";
        }

        DIR* dir = opendir(SD_MOUNT_POINT);
        if (!dir) {
            return "无法打开SD卡目录。";
        }

        cJSON* array = cJSON_CreateArray();
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) {
                size_t len = strlen(entry->d_name);
                if (len >= 4) {
                    const char* ext = entry->d_name + len - 4;
                    // 不区分大小写检查后缀是否为 .mp3
                    if (strcasecmp(ext, ".mp3") == 0) {
                        cJSON_AddItemToArray(array, cJSON_CreateString(entry->d_name));
                    }
                }
            }
        }
        closedir(dir);

        char* json_str = cJSON_PrintUnformatted(array);
        std::string result = json_str ? json_str : "[]";
        free(json_str);
        cJSON_Delete(array);
        return result;
    }

    ReturnValue PlayMp3(const std::string& name) {
        if (!is_sdcard_found) {
            return "SD卡未就绪。";
        }

        std::string file_path = std::string(SD_MOUNT_POINT) + "/" + name;
        struct stat st;
        
        // 检查文件是否存在，如果不存在尝试加上 .mp3 后缀
        if (stat(file_path.c_str(), &st) != 0) {
            if (name.find(".mp3") == std::string::npos) {
                file_path += ".mp3";
                if (stat(file_path.c_str(), &st) != 0) {
                    return "没有找到文件: " + name;
                }
            } else {
                return "没有找到文件: " + name;
            }
        }

        ESP_LOGI(TAG, "Playing MP3: %s", file_path.c_str());
        FILE *fp = fopen(file_path.c_str(), "rb");
        if (fp) {
            audio_player_play(fp); // 先启动播放器，确保状态变为 PLAYING

            auto& app = Application::GetInstance();
            app.SetDeviceState(kDeviceStateIdle); // 这里内部会自动根据播放状态关闭唤醒词

            if (display_) {
                auto lcd = static_cast<CustomLcdDisplay*>(display_);
                lcd->UpdateMusicStatus(true, false);
                lcd->SetChatMessage("system", ("正在播放: " + name).c_str());
            }
            return "正在播放: " + name;
        }
        return "无法打开文件进行播放。";
    }

    ReturnValue ResumeMp3() {
        audio_player_resume();
        // 恢复播放后自动切换 ChatState（回到 Idle 状态）
        Application::GetInstance().SetDeviceState(kDeviceStateIdle);
        if (display_) {
            static_cast<CustomLcdDisplay*>(display_)->UpdateMusicStatus(true, false);
        }
        return "继续播放。";
    }

    ReturnValue StopMp3() {
        audio_player_stop();
        // 停止音乐后，重新开启唤醒词检测
        auto& app = Application::GetInstance();
        app.GetAudioService().EnableWakeWordDetection(true);
        // 如果在播放期间强制进入了 Idle，这里确保状态一致
        if (app.GetDeviceState() == kDeviceStateIdle) {
            app.GetAudioService().EnableVoiceProcessing(false);
        }

        if (display_) {
            auto lcd = static_cast<CustomLcdDisplay*>(display_);
            lcd->UpdateMusicStatus(false, false);
            lcd->SetChatMessage("system", "");
        }
        return "已停止播放。";
    }

public:

    ~CustomBoard() {
        audio_player_delete();

        // Delete objects
        delete cst816s_;
        delete display_;
        // Note: backlight_ (PwmBacklight) and camera_ (EspVideo) are not deleted here
        // because their base classes (Backlight, Camera) don't have virtual destructors.
        // Since EspVocat is a singleton that lives for the device lifetime, this is acceptable.

        // Remove GPIO ISR handler
        gpio_isr_handler_remove(TP_PIN_NUM_INT);
        if (emotion_reset_timer_ != nullptr) {
            esp_timer_stop(emotion_reset_timer_);
            esp_timer_delete(emotion_reset_timer_);
            emotion_reset_timer_ = nullptr;
        }
    }

    CustomBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        instance_ = this; // 尽早初始化 instance 指针以供回调函数使用

        const esp_timer_create_args_t emotion_timer_args = {
            .callback = &CustomBoard::emotion_reset_timer_callback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "emotion_rst",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&emotion_timer_args, &emotion_reset_timer_));

        InitializePowerManager();
        InitializePowerSaveTimer();

        audio_player_config_t config = {
            .mute_fn = audio_mute_callback,
            .clk_set_fn = audio_reconfig_callback,
            .write_fn = audio_write_callback,
        };
        audio_player_new(config);

        InitializeI2c();
        I2cDetect();
        InitializeTca9554();
        ESP_LOGI(TAG, "Initializing SPI and Display...");
        InitializeSpi();
        Initializest77916Display();
        ESP_LOGI(TAG, "Initializing Touch and Buttons...");
        InitializeCst816sTouchPad();
        InitializeButtons();
        ESP_LOGI(TAG, "Initializing SD Card and Tools...");
        InitializeSDcardSpi();
        InitializeTools();
        ESP_LOGI(TAG, "Board initialization complete");
        GetBacklight()->RestoreBrightness();
    }

    #ifdef CONFIG_VERSION_1_0
    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_LEFT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_RIGHT); // I2S_STD_SLOT_LEFT / I2S_STD_SLOT_RIGHT / I2S_STD_SLOT_BOTH

        return &audio_codec;
    }
    #endif

    #ifdef CONFIG_VERSION_2_0
    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
            return &audio_codec;
    }
    #endif

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    Cst816s* GetTouchpad()
    {
        return cst816s_;
    }


    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        // 自动检测播放状态并更新图标（利用电池电量获取的周期性调用）
        if (display_) {
            audio_player_state_t audio_state = audio_player_get_state();
            // 状态看门狗：如果正在播放音乐，持续确保唤醒词处于关闭状态
            if (audio_state == AUDIO_PLAYER_STATE_PLAYING) {
                Application::GetInstance().GetAudioService().EnableWakeWordDetection(false);
                Application::GetInstance().GetAudioService().EnableVoiceProcessing(false);
            }
            
            auto app_state = Application::GetInstance().GetDeviceState();
            bool is_chatting = (app_state == kDeviceStateListening || app_state == kDeviceStateSpeaking);
            static_cast<CustomLcdDisplay*>(display_)->UpdateMusicStatus(audio_state != AUDIO_PLAYER_STATE_IDLE, is_chatting || audio_state == AUDIO_PLAYER_STATE_PAUSE);
        }

        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(CustomBoard);

CustomBoard* CustomBoard::instance_ = nullptr;
