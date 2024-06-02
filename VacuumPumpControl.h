#ifndef VACUUM_PUMP_CONTROL_H
#define VACUUM_PUMP_CONTROL_H
#include <pins_arduino.h>

// Step 1: Determine the variant
#define VACUUM_PUMP_CONTROL_VARIANT
//#define CONTROLLER_VARIANT

// Step 2a: Run the touch calibration
//#define RUN_CALIBRATION

// Step 2b: Update the touch screen limits if needed (generaly it is not) and re-run the calibration
#define XPT2046_X_LO 3963
#define XPT2046_X_HI 338
#define XPT2046_Y_LO 4031
#define XPT2046_Y_HI 211

// Step 3: Fill in the obtained touchscreen calibration values
//         When done:
//            * Comment out step 2a
//            * Uncomment step 4
static TS_Point _touchPoint[] = {
  TS_Point(3785, 3631), // point A
  TS_Point(537,  2186), // point B
  TS_Point(2040, 617), // point C
};

// Step4: Verify the touch calibration
//         When done:
//            * comment this step out as well
//#define VERIFY_CALIBRATION

#define RELAY_ON                HIGH
#define RELAY_OFF               LOW
#define STATE_PUMP_OFF          1
#define STATE_PUMP_ON           2
#define STATE_PUMP_HOLD         3
#define STATE_PUMP_THERMAL_PROT 4

#ifdef CONTROLLER_VARIANT
  #ifdef VACUUM_PUMP_CONTROL_VARIANT
    #error "only one variant can be defined at once"
  #endif

  #define PIN_RIGHT_BUTTON        PIN_D2
  #define PIN_LEFT_BUTTON         PIN_D3
  #define PIN_TOUCH_IRQ           PIN_D4
#elif defined(VACUUM_PUMP_CONTROL_VARIANT)
  #define PIN_TOUCH_IRQ           PIN_D3
#else
  #error "Unknown or no variant defined."
#endif
#define PIN_RELAY               PIN_D5
#define PIN_DISP_BACKLIGHT      PIN_D6
#define PIN_TOUCH_CS            PIN_D7
#define PIN_TFT_RST             PIN_D8
#define PIN_TFT_CMD             PIN_D9
#define PIN_TFT_CS              PIN_D10
#define PIN_TFT_MOSI            PIN_D11
#define PIN_TFT_MISO            PIN_D12
#define PIN_TFT_CLK             PIN_D13

enum class PointID { NONE = -1, A, B, C, COUNT };

static uint16_t const SCREEN_WIDTH    = 320;
static uint16_t const SCREEN_HEIGHT   = 240;
static uint8_t  const SCREEN_ROTATION = 1U;


#if defined(VERIFY_CALIBRATION) || defined(RUN_CALIBRATION)
// source points used for calibration
static TS_Point _screenPoint[] = {
  TS_Point( 15,  15), // point A
  TS_Point(305, 113), // point B
  TS_Point(167, 214)  // point C
};
#endif

#ifdef VERIFY_CALIBRATION
static TS_Calibration cal(
    _screenPoint[(int)PointID::A], _touchPoint[(int)PointID::A],
    _screenPoint[(int)PointID::B], _touchPoint[(int)PointID::B],
    _screenPoint[(int)PointID::C], _touchPoint[(int)PointID::C],
    SCREEN_WIDTH,
    SCREEN_HEIGHT
);
#endif

#endif /* VACUUM_PUMP_CONTROL_H */