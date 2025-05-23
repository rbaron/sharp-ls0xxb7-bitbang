#include <lvgl.h>
#include <lvgl_display.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_DISPLAY_LOG_LEVEL);

#define DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define WIDTH DT_PROP(DISPLAY_NODE, width)
#define HEIGHT DT_PROP(DISPLAY_NODE, height)

static const struct device *display_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static const struct gpio_dt_spec vddio_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vddio_en), gpios);
static const struct gpio_dt_spec vbus_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vbus_en), gpios);

static const struct gpio_dt_spec led0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

// The display does support partial updates, but only at full rows. I.e. we can
// start drawing at arbitrary y up to arbitrary height, but only at x0 = x0 and
// x1 = WIDTH - 1.
static void my_lvgl_rounder_cb(lv_event_t *e) {
  lv_area_t *area = lv_event_get_param(e);
  area->x1 = 0;
  area->x2 = WIDTH - 1;
}

int main(void) {
  gpio_pin_configure_dt(&vddio_en, GPIO_OUTPUT_HIGH);
  gpio_pin_configure_dt(&vbus_en, GPIO_OUTPUT_HIGH);
  gpio_pin_configure_dt(&led0, GPIO_OUTPUT);

  // Set up the area rounder.
  lv_display_t *display = lv_disp_get_default();
  lv_display_add_event_cb(display, my_lvgl_rounder_cb, LV_EVENT_INVALIDATE_AREA,
                          display);

  // Power up.
  gpio_pin_set_dt(&vddio_en, 1);
  k_busy_wait(1000);
  gpio_pin_set_dt(&vbus_en, 1);
  k_busy_wait(1000);

  if (!device_is_ready(display_dev)) {
    LOG_ERR("Device not ready, aborting test");
    return 0;
  }

  lv_obj_t *scr = lv_scr_act();

  static lv_style_t style_bg_white;
  lv_style_init(&style_bg_white);
  lv_style_set_bg_color(&style_bg_white, lv_color_white());
  lv_style_set_bg_opa(&style_bg_white, LV_OPA_COVER);
  lv_obj_add_style(scr, &style_bg_white, LV_PART_MAIN);

  // Square.
  lv_obj_t *square = lv_obj_create(scr);
  lv_obj_set_style_bg_opa(square, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_size(square, WIDTH, 40);
  lv_obj_set_style_border_color(square, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(square, LV_ALIGN_TOP_MID, 0, 0);

  // Text.
  lv_obj_t *label = lv_label_create(scr);
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(label, "Hello, world");
  lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -64);

#if CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_MONOCHROME
  lv_obj_set_style_bg_color(square, lv_color_black(), LV_PART_MAIN);
#elif CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_COLOR
  // Paint background white.

  // Square.
  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_bg_color(&style, lv_color_hex(0x00ff00));
  lv_obj_add_style(square, &style, LV_PART_MAIN);

  // Paint text red.
  static lv_style_t style_red;
  lv_style_init(&style_red);
  lv_style_set_text_color(&style_red, lv_color_hex(0xff0000));
  lv_obj_add_style(label, &style_red, LV_PART_MAIN);
#endif  // CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_COLOR

  int y = 0;
  while (1) {
    lv_task_handler();
    k_msleep(1);

    lv_obj_set_y(square, y++ % (HEIGHT - 40));
  }

  return 0;
}
