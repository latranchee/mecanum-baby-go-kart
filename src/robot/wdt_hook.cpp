// Task-watchdog ISR hook: stop the motors before the panic reboot.
//
// Kept in its own translation unit on purpose. esp_task_wdt.h declares
// esp_task_wdt_isr_user_handler() __attribute__((weak)); a definition in any file
// that includes that header would itself be emitted weak and could lose to the
// framework's empty default at link time. Nothing here includes it, so this
// definition is strong and always wins.
#include <stdint.h>
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"

// Bit masks of the motor direction pins (INA/INB of all four channels), split by
// GPIO bank. Filled once in setup() by the robot firmware before the watchdog arms.
volatile uint32_t g_motorDirMaskLo = 0;   // GPIO 0..31
volatile uint32_t g_motorDirMaskHi = 0;   // GPIO 32..39, bit = pin - 32

// Runs in the TWDT interrupt when loop() stopped feeding the watchdog. LEDC keeps
// outputting the last duty through a hang, so instead of touching it, drive every
// INA/INB LOW: per the driver truth table L/L = brake/coast whatever the PWM is.
// Raw register writes only (ISR context). The panic that follows reboots the chip.
extern "C" void IRAM_ATTR esp_task_wdt_isr_user_handler(void) {
  REG_WRITE(GPIO_OUT_W1TC_REG,  g_motorDirMaskLo);
  REG_WRITE(GPIO_OUT1_W1TC_REG, g_motorDirMaskHi);
}
