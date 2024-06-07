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

// Step 4: Verify the touch calibration
//         When done:
//            * comment this step out as well
//#define VERIFY_CALIBRATION


// Step 5: Configure the profiles for your use-case(s)


#define RELAY_ON                          HIGH
#define RELAY_OFF                         LOW
#define STATE_PUMP_IDLE                   0
#define STATE_PUMP_OFF                    1
#define STATE_PUMP_ON                     2
#define STATE_PUMP_HOLD                   3
#define STATE_PUMP_THERMAL_PROT           4
#define STATE_RUN_TIME                    1000      // Statemachine run period in ms
#define PUMP_OFF_ACTIVE_TIME_DECREMENTER  0.333     // Amount of seconds to decrement from the active time per STATE_RUN_TIME_PERIOD. Note that this is intentionaly not 1, this allows for a longer cooldown than run time
#define PUMP_THERMAL_PROT_TIME            (1 * 60) // Time after which we going into thermal protection
#define PUMP_THERMAL_PROT_RELEASE_TIME    (0.666 * 60) // Time after which we release thermal protection
#define PUMP_POST_RUN_TIME                (1 * 60)

#define STATE_SCREEN_MENU_MAIN    0
#define STATE_SCREEN_MENU_PROFILE 1
#define STATE_SCREEN_PROFILE      2
#define STATE_SCREEN_SET_POINT    3

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

#define BUTTON_BLUE             0x54FF // 51 153 255
#define BUTTON_TEXT_ORANGE      0xFA86 // 255 85 51
#define BUTTON_TEXT_RED         0xF98C // 255 51 201
#define BUTTON_COLOR            BUTTON_BLUE

#define BUTTON_1X2_X_1          (320/2)
#define BUTTON_1X2_Y_1          90
#define BUTTON_1X2_Y_2          175
#define BUTTON_1X2_WIDTH        200
#define BUTTON_1X2_HEIGTH       50

#define BUTTON_2X2_X_1          ((320/4))
#define BUTTON_2X2_X_2          (((320/4)*3))
#define BUTTON_2X2_Y_1          90
#define BUTTON_2X2_Y_2          175
#define BUTTON_2X2_WIDTH        140
#define BUTTON_2X2_HEIGTH       50

/*
typedef struct {
    uint32_t time;   // Time field
    float pressure;  // Pressure field
} timePressure_t;

typedef struct {
  size_t                length;
  const timePressure_t* profileData[];
} profile_t;

const timePressure_t gv_a_timePressure_p1[9] = {{0, 0.9f},
                                                {60, 0.8f},
                                                {120, 0.7f},
                                                {180, 0.6f},
                                                {240, 0.5f},
                                                {300, 0.4f},
                                                {360, 0.3f},
                                                {420, 0.2f},
                                                {480, 0.1f}};

const profile_t gv_profile1 = {
    .length = sizeof(gv_a_timePressure_p1),
    .profileData = gv_a_timePressure_p1
};
*/

enum class PointID { NONE = -1, A, B, C, COUNT };

static uint16_t const SCREEN_WIDTH    = 320;
static uint16_t const SCREEN_HEIGHT   = 240;
static uint8_t  const SCREEN_ROTATION = 1U;

static inline char* state_pump_string(uint8_t state){
  switch(state){
    case STATE_PUMP_IDLE          : return "IDLE"; break;
    case STATE_PUMP_OFF           : return "PUMP OFF"; break;
    case STATE_PUMP_ON            : return "PUMP ON"; break;
    case STATE_PUMP_HOLD          : return "POST RUN"; break;
    case STATE_PUMP_THERMAL_PROT  : return "THERMAL PROTECTION"; break;
    default                       : return "UNKNOWN"; break;
  }
}

// source points used for calibration
static TS_Point _screenPoint[] = {
  TS_Point( 15,  15), // point A
  TS_Point(305, 113), // point B
  TS_Point(167, 214)  // point C
};

#ifndef RUN_CALIBRATION
static TS_Calibration cal(
    _screenPoint[(int)PointID::A], _touchPoint[(int)PointID::A],
    _screenPoint[(int)PointID::B], _touchPoint[(int)PointID::B],
    _screenPoint[(int)PointID::C], _touchPoint[(int)PointID::C],
    SCREEN_WIDTH,
    SCREEN_HEIGHT
);
#endif

#endif /* VACUUM_PUMP_CONTROL_H */