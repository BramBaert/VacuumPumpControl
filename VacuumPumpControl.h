#ifndef VACUUM_PUMP_CONTROL_H
#define VACUUM_PUMP_CONTROL_H
#include <pins_arduino.h>

#define RELAY_ON                HIGH
#define RELAY_OFF               LOW
#define STATE_PUMP_OFF          1
#define STATE_PUMP_ON           2
#define STATE_PUMP_HOLD         3
#define STATE_PUMP_THERMAL_PROT 4

#define PIN_TOUCH_IRQ           PIN_D4
#define PIN_RELAY               PIN_D5
#define PIN_DISP_BACKLIGHT      PIN_D6
#define PIN_TOUCH_CS            PIN_D7
#define PIN_TFT_RST             PIN_D8
#define PIN_TFT_CMD             PIN_D9
#define PIN_TFT_CS              PIN_D10
#define PIN_TFT_MOSI            PIN_D11
#define PIN_TFT_MISO            PIN_D12
#define PIN_TFT_CLK             PIN_D13

#endif /* VACUUM_PUMP_CONTROL_H */