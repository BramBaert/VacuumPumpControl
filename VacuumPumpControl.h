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
#define PUMP_THERMAL_PROT_TIME            (15 * 60) // Time after which we going into thermal protection
#define PUMP_THERMAL_PROT_RELEASE_TIME    (10 * 60) // Time after which we release thermal protection
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

typedef struct {
    uint32_t time;   // Time field
    float pressure;  // Pressure field
} timePressure_t;

typedef struct {
  size_t                length;
  const timePressure_t* profileData;
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

const timePressure_t gv_a_timePressure_p2[82] = {{0, 0.95f},
                                                {1, 0.94f},
                                                {2, 0.93f},
                                                {3, 0.92f},
                                                {4, 0.91f},
                                                {5, 0.90f},
                                                {6, 0.89f},
                                                {7, 0.88f},
                                                {8, 0.87f},
                                                {9, 0.86f},
                                                {10, 0.85f},
                                                {11, 0.84f},
                                                {12, 0.83f},
                                                {13, 0.82f},
                                                {14, 0.81f},
                                                {15, 0.80f},
                                                {16, 0.79f},
                                                {17, 0.78f},
                                                {18, 0.77f},
                                                {19, 0.76f},
                                                {20, 0.75f},
                                                {21, 0.74f},
                                                {22, 0.73f},
                                                {23, 0.72f},
                                                {24, 0.71f},
                                                {25, 0.70f},
                                                {26, 0.69f},
                                                {27, 0.68f},
                                                {28, 0.67f},
                                                {29, 0.66f},
                                                {30, 0.65f},
                                                {31, 0.64f},
                                                {32, 0.63f},
                                                {33, 0.62f},
                                                {34, 0.61f},
                                                {35, 0.60f},
                                                {36, 0.59f},
                                                {37, 0.58f},
                                                {38, 0.57f},
                                                {39, 0.56f},
                                                {40, 0.55f},
                                                {41, 0.54f},
                                                {42, 0.53f},
                                                {43, 0.52f},
                                                {44, 0.51f},
                                                {45, 0.50f},
                                                {46, 0.49f},
                                                {47, 0.48f},
                                                {48, 0.47f},
                                                {49, 0.46f},
                                                {50, 0.45f},
                                                {51, 0.44f},
                                                {52, 0.43f},
                                                {53, 0.42f},
                                                {54, 0.41f},
                                                {55, 0.40f},
                                                {56, 0.39f},
                                                {57, 0.38f},
                                                {58, 0.37f},
                                                {59, 0.36f},
                                                {60, 0.35f},
                                                {61, 0.34f},
                                                {62, 0.33f},
                                                {63, 0.32f},
                                                {64, 0.31f},
                                                {65, 0.30f},
                                                {66, 0.29f},
                                                {67, 0.28f},
                                                {68, 0.27f},
                                                {69, 0.26f},
                                                {70, 0.25f},
                                                {71, 0.24f},
                                                {72, 0.23f},
                                                {73, 0.22f},
                                                {74, 0.21f},
                                                {75, 0.20f},
                                                {76, 0.19f},
                                                {77, 0.18f},
                                                {78, 0.17f},
                                                {79, 0.16f},
                                                {80, 0.15f},
                                                {150, 0.15f}};

const profile_t gv_profile1 = {
    .length = 9,
    .profileData = gv_a_timePressure_p1
};

const profile_t gv_profile2 = {
    .length = 82,
    .profileData = gv_a_timePressure_p2
};

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