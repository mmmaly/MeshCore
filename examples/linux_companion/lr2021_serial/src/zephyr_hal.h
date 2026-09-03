/*
 * RadioLibHal for Zephyr: the four LR2021 control pins come from the
 * `zephyr,user` devicetree node, SPI from spi00. Raw GPIO ops keep RadioLib's
 * LOW/HIGH equal to physical levels, as on Arduino. Adapted from the
 * xiao_nrf54l15 companion port proposed to MeshCore (CampusIoT, PR #2944).
 *
 * Opaque pin ids handed to Module(hal, cs, irq, rst, gpio): NSS=0 IRQ=1 RESET=2 BUSY=3.
 */
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <RadioLib.h>

#define LR_PIN_NSS   0
#define LR_PIN_IRQ   1
#define LR_PIN_RESET 2
#define LR_PIN_BUSY  3
#define LR_PIN_COUNT 4

class ZephyrHal : public RadioLibHal {
public:
  ZephyrHal()
    : RadioLibHal(/*INPUT*/0, /*OUTPUT*/1, /*LOW*/0, /*HIGH*/1, /*RISING*/1, /*FALLING*/2) {}

  void init() override { spiBegin(); }
  void term() override {}

  void pinMode(uint32_t pin, uint32_t mode) override {
    if (pin >= LR_PIN_COUNT) return;
    const gpio_dt_spec* g = &gpios[pin];
    gpio_pin_configure(g->port, g->pin, (mode == 1) ? GPIO_OUTPUT : GPIO_INPUT);
  }
  void digitalWrite(uint32_t pin, uint32_t value) override {
    if (pin >= LR_PIN_COUNT) return;
    const gpio_dt_spec* g = &gpios[pin];
    gpio_pin_set_raw(g->port, g->pin, value ? 1 : 0);
  }
  uint32_t digitalRead(uint32_t pin) override {
    if (pin >= LR_PIN_COUNT) return 0;
    const gpio_dt_spec* g = &gpios[pin];
    return gpio_pin_get_raw(g->port, g->pin);
  }

  void attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) override {
    if (pin >= LR_PIN_COUNT) return;
    const gpio_dt_spec* g = &gpios[pin];
    user_cb = cb;
    gpio_pin_configure(g->port, g->pin, GPIO_INPUT);
    if (!cb_added) {
      gpio_init_callback(&cb_data, gpioIsr, BIT(g->pin));
      gpio_add_callback(g->port, &cb_data);
      cb_added = true;
    }
    gpio_pin_interrupt_configure(g->port, g->pin,
        (mode == 2) ? GPIO_INT_EDGE_FALLING :
        (mode == 1) ? GPIO_INT_EDGE_RISING  : GPIO_INT_EDGE_BOTH);
  }
  void detachInterrupt(uint32_t pin) override {
    if (pin >= LR_PIN_COUNT) return;
    const gpio_dt_spec* g = &gpios[pin];
    gpio_pin_interrupt_configure(g->port, g->pin, GPIO_INT_DISABLE);
    user_cb = nullptr;
  }

  void delay(RadioLibTime_t ms) override { k_msleep((int32_t)ms); }
  void delayMicroseconds(RadioLibTime_t us) override { k_busy_wait((uint32_t)us); }
  RadioLibTime_t millis() override { return (RadioLibTime_t)k_uptime_get(); }
  RadioLibTime_t micros() override { return (RadioLibTime_t)k_ticks_to_us_floor64(k_uptime_ticks()); }
  long pulseIn(uint32_t, uint32_t, RadioLibTime_t) override { return 0; }

  void spiBegin() override {
    spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi00));
    spi_cfg.frequency = 8000000;      // LR2021 SPI max is 16 MHz, mode 0, MSB first
#ifdef SPI_OP_MODE_CONTROLLER
    spi_cfg.operation = SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg.peripheral = 0;
#else
    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg.slave = 0;
#endif
    spi_cfg.cs.gpio.port = NULL;      // RadioLib drives NSS
  }
  void spiBeginTransaction() override {}
  void spiEndTransaction() override {}
  void spiEnd() override {}

  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
    spi_buf txb = { .buf = out, .len = len };
    spi_buf rxb = { .buf = in,  .len = len };
    spi_buf_set tx = { .buffers = &txb, .count = 1 };
    spi_buf_set rx = { .buffers = &rxb, .count = 1 };
    spi_transceive(spi_dev, &spi_cfg, out ? &tx : NULL, in ? &rx : NULL);
  }

private:
  static void (*user_cb)(void);
  static gpio_callback cb_data;
  static bool cb_added;
  static void gpioIsr(const device*, gpio_callback*, uint32_t) { if (user_cb) user_cb(); }

  const device* spi_dev = nullptr;
  spi_config spi_cfg = {};

  const gpio_dt_spec gpios[LR_PIN_COUNT] = {   // index = LR_PIN_*
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nss_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), irq_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), reset_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), busy_gpios),
  };
};

inline void (*ZephyrHal::user_cb)(void) = nullptr;
inline gpio_callback ZephyrHal::cb_data;
inline bool ZephyrHal::cb_added = false;
