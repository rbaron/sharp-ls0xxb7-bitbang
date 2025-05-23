/*
 *  Bit-banging driver for Sharp LS0XXB7 displays. It currently only works with
 *  the LS014B7DD01 display.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sharp_mip_parallel, CONFIG_DISPLAY_LOG_LEVEL);

#include <stdint.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// Look-up table.
struct lut_entry {
  gpio_port_value_t lsb_val;
  gpio_port_value_t msb_val;
};

struct rgb_port {
  const struct device *port;
  // Bitmask of GPIO indexed into config->rgb that are connected to this port.
  uint8_t rgb_idx_mask;
  gpio_port_pins_t port_mask;

  // [odd/even][color]
  struct lut_entry lut_entries[2][256];
};

// Private mutable data for the driver.
struct sharp_mip_data {
  K_KERNEL_STACK_MEMBER(vcom_thread_stack, /*size=*/256);
  struct k_thread vcom_thread;

  // Optimization for RGB pins.
  struct rgb_port rgb_ports[2];
};

// Private const data for the driver.
struct sharp_mip_config {
  uint16_t height;
  uint16_t width;

  struct gpio_dt_spec intb;
  struct gpio_dt_spec gsp;
  struct gpio_dt_spec gck;
  struct gpio_dt_spec gen;
  struct gpio_dt_spec bsp;
  struct gpio_dt_spec bck;
  struct gpio_dt_spec rgb[6];

  struct gpio_dt_spec vcom_vb;
  struct gpio_dt_spec va;
  int vcom_freq;
};

static void vcom_thread(void *config, void *unused1, void *unused2) {
  const struct sharp_mip_config *cfg = config;

  // Ensure out of phase.
  gpio_pin_set_dt(&cfg->vcom_vb, 1);
  gpio_pin_set_dt(&cfg->va, 0);

  while (1) {
    gpio_pin_toggle_dt(&cfg->vcom_vb);
    gpio_pin_toggle_dt(&cfg->va);
    k_msleep(1000 / cfg->vcom_freq);
  }
}

static inline uint8_t get_color_for_pin(uint8_t rgb222, int rgb_idx) {
  switch (rgb_idx) {
    case 0:
    case 1:
      return (rgb222 >> 4) & 0x03;  // R
    case 2:
    case 3:
      return (rgb222 >> 2) & 0x03;  // G
    case 4:
    case 5:
      return (rgb222 >> 0) & 0x03;  // B
    default:
      LOG_ERR("Invalid RGB index: %d", rgb_idx);
      __ASSERT(false, "Invalid RGB index");
      return 0;
  }
}

#define SET_GPIO_OUTPUT(ret, name)                       \
  do {                                                   \
    if (!device_is_ready(name.port)) {                   \
      LOG_ERR(#name " GPIO port not ready");             \
      return -ENODEV;                                    \
    }                                                    \
    ret = gpio_pin_configure_dt(&name, GPIO_OUTPUT);     \
    if (ret < 0) {                                       \
      LOG_ERR(#name " GPIO pin config failed: %d", ret); \
      return ret;                                        \
    }                                                    \
  } while (0)

static int sharp_mip_init(const struct device *dev) {
  const struct sharp_mip_config *config = dev->config;
  struct sharp_mip_data *data = dev->data;

  int ret;

  SET_GPIO_OUTPUT(ret, config->intb);
  SET_GPIO_OUTPUT(ret, config->gsp);
  SET_GPIO_OUTPUT(ret, config->gck);
  SET_GPIO_OUTPUT(ret, config->gen);
  SET_GPIO_OUTPUT(ret, config->bsp);
  SET_GPIO_OUTPUT(ret, config->bck);
  for (int i = 0; i < sizeof(config->rgb) / sizeof(config->rgb[0]); i++) {
    SET_GPIO_OUTPUT(ret, config->rgb[i]);
  }

  SET_GPIO_OUTPUT(ret, config->vcom_vb);
  SET_GPIO_OUTPUT(ret, config->va);
  k_thread_create(&data->vcom_thread, data->vcom_thread_stack,
                  K_KERNEL_STACK_SIZEOF(data->vcom_thread_stack),
                  (k_thread_entry_t)vcom_thread, /*arg1=*/(void *)config,
                  /*arg2=*/NULL, /*arg3=*/NULL, K_PRIO_PREEMPT(7),
                  /*options=*/0, K_NO_WAIT);

  // Collect RGB gpios into data->rgb_ports.
  for (int gpio_idx = 0;
       gpio_idx < sizeof(config->rgb) / sizeof(config->rgb[0]); gpio_idx++) {
    for (int port_idx = 0;
         port_idx < sizeof(data->rgb_ports) / sizeof(data->rgb_ports[0]);
         port_idx++) {
      if (data->rgb_ports[port_idx].port == config->rgb[gpio_idx].port ||
          data->rgb_ports[port_idx].port == NULL) {
        data->rgb_ports[port_idx].port = config->rgb[gpio_idx].port;
        data->rgb_ports[port_idx].rgb_idx_mask |= (1 << gpio_idx);
        data->rgb_ports[port_idx].port_mask |= (1 << config->rgb[gpio_idx].pin);
        break;
      }
    }
  }

  // Populate look up table.
  for (int port_idx = 0;
       port_idx < sizeof(data->rgb_ports) / sizeof(data->rgb_ports[0]);
       port_idx++) {
    // Not needed?
    memset(data->rgb_ports[port_idx].lut_entries, 0,
           sizeof(data->rgb_ports[port_idx].lut_entries));

    // Pins on this port.
    const gpio_port_pins_t mask = data->rgb_ports[port_idx].port_mask;

    for (int rgb_idx = 0; rgb_idx < 6; rgb_idx++) {
      // Even or odd??
      uint8_t oe = rgb_idx % 2;

      // If this pin is not on this port, continue.
      if ((mask & BIT(config->rgb[rgb_idx].pin)) == 0) {
        LOG_DBG("Skipping pin %d on port %d (mask: 0x%02x)", rgb_idx, port_idx,
                mask);
        continue;
      }

      // For each possible color, precompute the val on this port.
      for (int16_t color = 0; color < 256; color++) {
        // Which color for this pin?
        uint8_t pin_color = get_color_for_pin(color, rgb_idx);

        // Set lsb.
        data->rgb_ports[port_idx].lut_entries[oe][color].lsb_val |=
            ((pin_color >> 1) << config->rgb[rgb_idx].pin);

        // Set msb.
        data->rgb_ports[port_idx].lut_entries[oe][color].msb_val |=
            ((pin_color & 1) << config->rgb[rgb_idx].pin);
      }
    }
  }

  LOG_INF("Sharp MIP display initialized. Resolution: %dx%d", config->width,
          config->height);

  for (int i = 0; i < sizeof(data->rgb_ports) / sizeof(data->rgb_ports[0]);
       i++) {
    LOG_DBG("RGB port %d: %p, mask: 0x%02x", i, data->rgb_ports[i].port,
            data->rgb_ports[i].rgb_idx_mask);
  }

  return 0;
}

// Convert from RGB565 to RGB222 by dropping the least significant bits.
static inline uint8_t rgb222(uint16_t rgb565) {
  return ((((rgb565 >> 14) & 0x3) << 4) |  // R
          (((rgb565 >> 9) & 0x3) << 2) |   // G
          (((rgb565 >> 3) & 0x3) << 0));   // B
}

static inline void set_rgb(bool is_msb, int x0, const uint8_t *buf,
                           const struct sharp_mip_config *cfg,
                           const struct sharp_mip_data *data) {
#if CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_COLOR
  // Offset into buf for 16-bit RGB565 data at column x0.
  const uint8_t *b = buf + (x0 << 1);

  for (int port_idx = 0;
       port_idx < sizeof(data->rgb_ports) / sizeof(data->rgb_ports[0]);
       port_idx++) {
    // Skip port if it's not connected to any RGB pins.
    if (data->rgb_ports[port_idx].port == NULL) {
      continue;
    }

    gpio_port_value_t val;
    uint8_t color_b0 = rgb222((uint16_t)b[1] << 8 | b[0]);
    uint8_t color_b2 = rgb222((uint16_t)b[3] << 8 | b[2]);
    if (is_msb) {
      val = data->rgb_ports[port_idx].lut_entries[0][color_b0].msb_val |
            data->rgb_ports[port_idx].lut_entries[1][color_b2].msb_val;
    } else {
      val = data->rgb_ports[port_idx].lut_entries[0][color_b0].lsb_val |
            data->rgb_ports[port_idx].lut_entries[1][color_b2].lsb_val;
    }

    gpio_port_set_masked_raw(data->rgb_ports[port_idx].port,
                             data->rgb_ports[port_idx].port_mask, val);
  }

#elif CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_MONOCHROME

#define GET_BUF_BIT(buf, x) (((uint8_t *)buf)[(x) / 8] >> ((x) % 8) & 0x1)

#define VAL_BIT_IF_ON_PORT(port, rgb_idx, buf, x)             \
  ((data->rgb_ports[(port)].rgb_idx_mask & BIT(rgb_idx))      \
       ? (GET_BUF_BIT((buf), (x)) << cfg->rgb[(rgb_idx)].pin) \
       : 0)

  // for (int port_idx = 0;
  //      port_idx < sizeof(data->rgb_ports) / sizeof(data->rgb_ports[0]);
  //      port_idx++) {
  //   // Skip port if it's not connected to any RGB pins.
  //   if (data->rgb_ports[port_idx].port == NULL) {
  //     continue;
  //   }

  const int port_idx = 0;

  // const gpio_port_value_t val = VAL_BIT_IF_ON_PORT(port_idx, 0, buf, x0 + 0)
  // |
  //                               VAL_BIT_IF_ON_PORT(port_idx, 1, buf, x0 + 1)
  //                               | VAL_BIT_IF_ON_PORT(port_idx, 2, buf, x0 +
  //                               0) | VAL_BIT_IF_ON_PORT(port_idx, 3, buf, x0
  //                               + 1) | VAL_BIT_IF_ON_PORT(port_idx, 4, buf,
  //                               x0 + 0) | VAL_BIT_IF_ON_PORT(port_idx, 5,
  //                               buf, x0 + 1);

  // uint8_t state = GET_BUF_BIT(buf, x0) | (GET_BUF_BIT(buf, x0 + 1) << 1);

  // const gpio_port_value_t val = lut_masks[port_idx][state].val;
  // const gpio_port_value_t val = 0;

  // gpio_port_set_masked(data->rgb_ports[port_idx].port,
  //                      data->rgb_ports[port_idx].port_mask, val);
  // NRF_P1->OUTCLR = data->rgb_ports[0].port_mask;
  // NRF_P1->OUTSET = 0x00;

  const gpio_port_value_t val = 0xaa;
  const gpio_port_pins_t mask = data->rgb_ports[port_idx].port_mask;
  NRF_P1->OUT = (NRF_P1->OUT & ~mask) | (val & mask);
  // }

#endif  // CONFIG_SHARP_LS0XXB7_DISPLAY_MODE
}

static inline void set(const struct gpio_dt_spec *gpio) {
  gpio_pin_set_raw(gpio->port, gpio->pin, 1);
}
static inline void clear(const struct gpio_dt_spec *gpio) {
  gpio_pin_set_raw(gpio->port, gpio->pin, 0);
}
static inline void send_half_line(bool is_msb, const void *buf,
                                  const struct sharp_mip_config *cfg,
                                  const struct sharp_mip_data *data) {
  clear(&cfg->bsp);
  set(&cfg->bsp);

  for (int i = 1; i <= 144; i++) {
    // Toggle BCK: 1 on odd, 0 on even.
    gpio_pin_set_raw(cfg->bck.port, cfg->bck.pin, i & 1);

    if (i == 2) {
      clear(&cfg->bsp);
    }
    if (i >= 1 && i <= 140) {
      // Prepare RGB pins for the next BCK edge.
      set_rgb(is_msb, /*x0=*/(i - 1) << 1, buf, cfg, data);
    }
  }
}

static int sharp_mip_write(const struct device *dev, const uint16_t x,
                           const uint16_t y,
                           const struct display_buffer_descriptor *desc,
                           const void *buf) {
  const struct sharp_mip_config *cfg = dev->config;

  // The display only supports partial updates at full rows. I.e.: we can
  // start drawing at (y = 10, x = 0), but _not_ at (y, x = 10).
  if (x != 0) {
    LOG_ERR(
        "Full screen or partial updates must be at full rows! x: %d, y = %d, "
        "buf size: %d",
        x, y, desc->buf_size);
    return -ENOTSUP;
  }

  // Offset for GCK. For full-screen updates, this is 2. For partial updates,
  // it's 2 plus the number of half-lines to skip.
  const int gck_offset = 2 + y * 2;

  // The last GCK index of the last sent half-line.
  const int gck_last_half_line = gck_offset + 2 * desc->height - 1;

  LOG_DBG(
      "Sharp MIP display write. x: %d, y: %d; buf size: %d (buf height: "
      "%d,buf "
      "width: %d). Offset: %d, last: %d",
      x, y, desc->buf_size, desc->height, desc->width, gck_offset,
      gck_last_half_line);

  clear(&cfg->gck);
  set(&cfg->intb);
  set(&cfg->gsp);

  // 1-indexed to match the datasheet and improve debugging.
  for (int i = 1; i <= 568; i++) {
    gpio_pin_set_raw(cfg->gck.port, cfg->gck.pin, i & 1);

    if (i == 2) {
      clear(&cfg->gsp);
    }

    if (i == gck_offset) {
      send_half_line(/*is_msb=*/true, buf, cfg, dev->data);
    } else if (i >= gck_offset + 1 && i <= gck_last_half_line) {
      set(&cfg->gen);

      // This display sends MSB for a full line at a GCK edge and then the LSB
      // for the same line at the next GCK edge.
      const uint16_t display_line = (i - gck_offset) / 2;
      const bool is_msb = (i - gck_offset) % 2 == 0;

#if CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_COLOR
      // Color buffer, 16 bits per pixel.
      // Each line has buf_width pixels, which is buf_width * 2.
      uint8_t *line_buf = (uint8_t *)buf + display_line * desc->width * 2;
#elif CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_MONOCHROME
      // Monochrome buffer, 1 bit per pixel.
      // Each line has 280 pixels, which is 280 / 8 = 35 bytes.
      uint8_t *line_buf = (uint8_t *)buf + display_line * 35;
#endif  // CONFIG_SHARP_LS0XXB7_DISPLAY_MODE

      send_half_line(is_msb, line_buf, cfg, dev->data);

      clear(&cfg->gen);
    } else if (i == gck_last_half_line + 1) {
      set(&cfg->gen);
      clear(&cfg->gen);
    } else if (i == 566) {
      clear(&cfg->intb);
    }
  }

  return 0;
}

static void sharp_mip_get_capabilities(
    const struct device *dev, struct display_capabilities *capabilities) {
  const struct sharp_mip_config *config = dev->config;

  capabilities->x_resolution = config->width;
  capabilities->y_resolution = config->height;

#if CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_COLOR
  capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
  capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
#elif CONFIG_SHARP_LS0XXB7_DISPLAY_MODE_MONOCHROME
  capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO10;
  capabilities->current_pixel_format = PIXEL_FORMAT_MONO10;
#endif  // CONFIG_SHARP_LS0XXB7_DISPLAY_MODE

  // We need partial updates to contain full rows. Note that as of writing,
  // this flag only takes effect for monochrome pixel formats.
  capabilities->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
  capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

struct display_driver_api sharp_mip_driver_api = {
    .write = sharp_mip_write,
    .get_capabilities = sharp_mip_get_capabilities,
};

#define SHARP_MIP_DEFINE(node_id)                                  \
  static struct sharp_mip_data data_##node_id = {                  \
      .rgb_ports =                                                 \
          {                                                        \
              {.port = NULL, .rgb_idx_mask = 0, .port_mask = 0},   \
              {.port = NULL, .rgb_idx_mask = 0, .port_mask = 0},   \
          },                                                       \
  };                                                               \
  static const struct sharp_mip_config config_##node_id = {        \
      .height = DT_PROP(node_id, height),                          \
      .width = DT_PROP(node_id, width),                            \
      .intb = GPIO_DT_SPEC_GET(node_id, intb_gpios),               \
      .gsp = GPIO_DT_SPEC_GET(node_id, gsp_gpios),                 \
      .gck = GPIO_DT_SPEC_GET(node_id, gck_gpios),                 \
      .gen = GPIO_DT_SPEC_GET(node_id, gen_gpios),                 \
      .bsp = GPIO_DT_SPEC_GET(node_id, bsp_gpios),                 \
      .bck = GPIO_DT_SPEC_GET(node_id, bck_gpios),                 \
      .rgb = {GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 0),      \
              GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 1),      \
              GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 2),      \
              GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 3),      \
              GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 4),      \
              GPIO_DT_SPEC_GET_BY_IDX(node_id, rgb_gpios, 5)},     \
      .vcom_vb = GPIO_DT_SPEC_GET(node_id, vb_gpios),              \
      .va = GPIO_DT_SPEC_GET(node_id, va_gpios),                   \
      .vcom_freq = DT_PROP(node_id, vcom_freq),                    \
  };                                                               \
  DEVICE_DT_DEFINE(node_id, sharp_mip_init, NULL, &data_##node_id, \
                   &config_##node_id, POST_KERNEL,                 \
                   CONFIG_DISPLAY_INIT_PRIORITY, &sharp_mip_driver_api);

DT_FOREACH_STATUS_OKAY(sharp_ls0xxb7_bitbang, SHARP_MIP_DEFINE);
