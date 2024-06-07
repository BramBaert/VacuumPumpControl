
#include <Wire.h>

#include "SparkFun_MicroPressure.h"
#include "Arduino_LED_Matrix.h"
/* In the below library version 1.11.9:
    * To make the drawCircleHelper work you need to add:
      * startWrite(); before the while function
      * endWrite(); after the while function
#include "Adafruit_GFX.h"

/* Due to a HW bug, the SPI frequency in XPT2046_Touchscreen.cpp must be lowered to 50kHz 
    in XPT2046_Touchscreen.cpp change the SPI_SETTING line to
       #define SPI_SETTING     SPISettings(50000, MSBFIRST, SPI_MODE0)
*/
//#include "XPT2046_Touchscreen.h"
#include "XPT2046_Calibrated.h"

/* To support the arduino R4 change the following lines in the cpp file
    #ifndef RASPI
    #include "wiring_private.h"
    #endif
  to this:
    #if defined(__has_include)
    #if __has_include("wiring_private.h")
    #include "wiring_private.h"
    #endif
    #else
    #include "wiring_private.h"
    #endif
*/
#include "Adafruit_ILI9341.h"
#include "VacuumPumpControl.h"

byte gv_frame_heart[8][12] = {
  { 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 },
  { 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

byte gv_frame_full[8][12] = {
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
};

Adafruit_ILI9341        tft = Adafruit_ILI9341(PIN_TFT_CS, PIN_TFT_CMD, PIN_TFT_RST);
#ifdef CONTROLLER_VARIANT
XPT2046_Calibrated      ts(PIN_TOUCH_CS,255);           // Apparently the R4 only supports IRQ's on PIN_D2 and PIN_D3, to bad TOUCH_IRQ is on PIN_D4 
#else
XPT2046_Calibrated      ts(PIN_TOUCH_CS,PIN_TOUCH_IRQ); // To get this to work, short PIN_D3 and PIN_D4
#endif
SparkFun_MicroPressure  gv_mpr; // Use default values with reset and EOC pins unused
ArduinoLEDMatrix        gv_matrix;
uint8_t                 gv_u8_startStopState = 0;               // TODO I should make a class for the start/stop button

void setup() {
  // Configure the I/Os
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_TOUCH_IRQ, INPUT);

  // Set default values for I/Os
  digitalWrite(PIN_RELAY, RELAY_OFF);
  analogWrite(PIN_DISP_BACKLIGHT, 128);

  // Configure the UART
  Serial.begin(115200);
  while (!Serial && (millis() <= 1000))
  delay(100); // Still waiting a bit more to ensure we can print
  Serial.println("Welcome to VacuumPressureControl");

  // Configure the I2C
  Wire1.begin();
  if(!gv_mpr.begin(DEFAULT_ADDRESS,Wire1)){
    Serial.println("Cannot connect to MicroPressure sensor.");
    while(1);
  }

  // Configure  the LED matrix
  gv_matrix.begin();  
  gv_matrix.renderBitmap(gv_frame_full, 8, 12);

  // Configure the touch
  ts.begin();
  ts.setRotation(1);
  // Do a first transaction towards the touch, to free up the peripheral.
  // I don't know why it is needed, but not doing so slows down the TFT
  // by a factor of 6 until you do.
  ts.getPoint();

  // Configure the TFT and run the benchmark as startup annimation
  tft.begin(12000000);
  tft.setRotation(1);
#if !defined(VERIFY_CALIBRATION) && !defined(RUN_CALIBRATION)
  splashScreen();
  drawMainMenu();
#endif

#ifndef RUN_CALIBRATION
  ts.calibrate(cal);
#endif

#if defined(VERIFY_CALIBRATION) || defined(RUN_CALIBRATION)
  for (int i = (int)PointID::NONE + 1; i < (int)PointID::COUNT; ++i)
  { crosshair(_screenPoint[i]); }
#endif
}

void loop() {

  static uint8_t  fv_u8_backlight_brightness  = 250;
  static uint8_t  fv_u8_screen_state          = STATE_SCREEN_MENU_MAIN;
  static bool     fv_b_not_touched            = true;
  static uint32_t fv_u32_lastRun              = 0; // last time the state machine was executed, expressed in ms

  uint32_t        fv_u32_time_backlight       = 0;
  TS_Point        fv_touch_point;


#ifdef CONTROLLER_VARIANT
  // Note: both ts.touched() and ts.getPoint() do an actual read-out of the touch driver
  //       thus reading out the point directly and checking the pressure our selfs is
  //       faster than using ts.touched(). On an arduion R4 Wifi using 50kHz SPI speed
  //       4 ms i.s.o. 8 ms

  // TODO reading out the INPUT might be even more efficient and equally responsive
  fv_touch_point = ts.getPoint();
  if (500 < fv_touch_point.z) {    
#else
  if (ts.tirqTouched()) {
    fv_touch_point = ts.getPoint();
#endif

#if defined(VERIFY_CALIBRATION) || defined(RUN_CALIBRATION)
    /*Serial.print("Pressure = ");
    Serial.print(fv_touch_point.z);
    Serial.print(", x = ");
    Serial.print(fv_touch_point.x);
    Serial.print(", y = ");
    Serial.print(fv_touch_point.y);
    Serial.println();*/

#ifdef RUN_CALIBRATION
    // Determine the touch limits dimension 
    updateScreenEdges(fv_touch_point);
#endif

    // determine which screen point is closest to this touch event
    PointID n = nearestScreenPoint(fv_touch_point);

    // update the corresponding line mapping
    drawMapping(n, fv_touch_point);

    delay(333);
  }
}
#else /* NORMAL TOUCH OPERATION */

    if(fv_b_not_touched){
      Serial.print("x = ");
      Serial.print(fv_touch_point.x);
      Serial.print(", y = ");
      Serial.print(fv_touch_point.y);
      Serial.println();
      switch(fv_u8_screen_state){

        case STATE_SCREEN_MENU_MAIN:
          if (checkPointInSquare(fv_touch_point, BUTTON_1X2_X_1, BUTTON_1X2_Y_1, BUTTON_1X2_WIDTH, BUTTON_1X2_HEIGTH)){
              drawSetPointInitial();
              fv_u8_screen_state = STATE_SCREEN_SET_POINT;
          }
          else if (checkPointInSquare(fv_touch_point, BUTTON_1X2_X_1, BUTTON_1X2_Y_2, BUTTON_1X2_WIDTH, BUTTON_1X2_HEIGTH)){
            drawProfileMenu();
            fv_u8_screen_state = STATE_SCREEN_MENU_PROFILE;
          }
          break;
        case STATE_SCREEN_MENU_PROFILE:

          if (backButtonLocation(fv_touch_point)){
            drawMainMenu();
            fv_u8_screen_state = STATE_SCREEN_MENU_MAIN;
          }
          else{
            if (checkPointInSquare(fv_touch_point, BUTTON_2X2_X_1, BUTTON_2X2_Y_1, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH)){
              drawProfileInitial();//gv_profile1);
              fv_u8_screen_state = STATE_SCREEN_PROFILE;
            }
            else if (checkPointInSquare(fv_touch_point, BUTTON_2X2_X_1, BUTTON_2X2_Y_2, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH)){

            }
            else if (checkPointInSquare(fv_touch_point, BUTTON_2X2_X_2, BUTTON_2X2_Y_1, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH)){

            }
            else if (checkPointInSquare(fv_touch_point, BUTTON_2X2_X_2, BUTTON_2X2_Y_2, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH)){

            }
          }

          break;
        case STATE_SCREEN_PROFILE:
          if (backButtonLocation(fv_touch_point)){
            fv_u8_screen_state = STATE_SCREEN_MENU_MAIN;
            setPointStateMachine(0,0,2);
            drawMainMenu();
          }
          break;
        case STATE_SCREEN_SET_POINT:
          if (backButtonLocation(fv_touch_point)){
            fv_u8_screen_state = STATE_SCREEN_MENU_MAIN;
            setPointStateMachine(0,0,2);
            drawMainMenu();
          }
          else{
            // High pressure decrement
            if (checkPointInSquare(fv_touch_point, 40, 120, 40 , 40)){
              setPointStateMachine(2,0,0);
            }
            // High pressure increment
            else if (checkPointInSquare(fv_touch_point, 170, 120, 40, 40)){
              setPointStateMachine(1,0,0);
            }
            // Low pressure decrement
            else if (checkPointInSquare(fv_touch_point, 40, 190, 40, 40)){
              setPointStateMachine(0,2,0);
            }
            // High pressure increment
            else if (checkPointInSquare(fv_touch_point, 170, 190, 40, 40)){
              setPointStateMachine(0,1,0);
            }
            // Start / Stop
            else if (checkPointInSquare(fv_touch_point, tft.width()-23,23,40,40)){
              if(1 == gv_u8_startStopState){
                setPointStateMachine(0,0,1);
                drawStartStopButton(2);
              }
              else{
                setPointStateMachine(0,0,2);
                drawStartStopButton(1);
              }
            }
          }
        default:
          break;
      }
    }
    fv_b_not_touched = false;
    delay(100);
  }
  else{
    fv_b_not_touched = true;
  }

  // Only execute the state machine every "STATE_RUN_TIME" period. This is more than fast enough and reduces
  // the screen flicker.
  if (fv_u32_lastRun + STATE_RUN_TIME < millis()){   
    fv_u32_lastRun = millis();

    switch(fv_u8_screen_state){
      case STATE_SCREEN_MENU_MAIN:
        break;
      case STATE_SCREEN_MENU_PROFILE:
        break;
      case STATE_SCREEN_PROFILE:
      case STATE_SCREEN_SET_POINT:
        setPointStateMachine(0, 0, 0);
      default:
        break;
    }
  }

}
#endif

/**
 * @brief This function controls the pump relay in a fix set point manner
 * @param [in] changeHighPress: 0 -> no, 1 -> increment, 2 -> decrement
 * @param [in] changeLowPress:  0 -> no, 1 -> increment, 2 -> decrement
 * @param [in] changeStartStop: 0 -> no, 1 -> start, 2 -> stop
 */
void setPointStateMachine(uint8_t changeHighPress, uint8_t changeLowPress, uint8_t changeStartStop){

  static float    fv_f32_thresh_high          = 0.950;  // expressed in Bar
  static float    fv_f32_thresh_low           = 0.900;  // expressed in Bar
  static uint8_t  fv_u8_state                 = STATE_PUMP_IDLE;
  static uint8_t  fv_u8_state_d1              = STATE_PUMP_IDLE;
  static uint8_t  fv_u8_state_d2              = STATE_PUMP_IDLE;
  static float    fv_f32_time_active          = 0.0;
  static float    fv_f32_time_postRun         = 0.0;
         float    fv_f32_pressure;
         float    fv_f32_time_coolDown        = 0.0;

  if(1 == changeHighPress){
    if(0.951 >= (fv_f32_thresh_high + 0.025)){
      fv_f32_thresh_high += 0.025;
    }
  }
  if(2 == changeHighPress){
    if(0.051 < (fv_f32_thresh_high - fv_f32_thresh_low)){
      fv_f32_thresh_high -= 0.025;
    }
  }
  if(1 == changeLowPress) {
    if(0.051 < (fv_f32_thresh_high - fv_f32_thresh_low)){
      fv_f32_thresh_low  += 0.025;
    }
  }
  if(2 == changeLowPress) {
    if(0.099 <= (fv_f32_thresh_low - 0.025)){
      fv_f32_thresh_low -= 0.025;
    }
  }

  fv_f32_pressure = gv_mpr.readPressure(BAR);
  //Serial.print(fv_f32_pressure,3);
  //Serial.println(" bar");

  if((1 == changeStartStop) && (STATE_PUMP_IDLE == fv_u8_state)){
    if(fv_f32_thresh_low > fv_f32_pressure){
      Serial.println("Starting pump control with pump off following user request.");
      fv_u8_state = STATE_PUMP_OFF;
      digitalWrite(PIN_RELAY, RELAY_OFF);
    }
    else{
      Serial.println("Starting pump control with pump on following user request.");
      fv_u8_state = STATE_PUMP_ON;
      digitalWrite(PIN_RELAY, RELAY_ON); 
    }
  }

  if(2 == changeStartStop){
    Serial.println("Turning pump off following user request.");
    digitalWrite(PIN_RELAY, RELAY_OFF);
    if (STATE_PUMP_THERMAL_PROT != fv_u8_state) {
      fv_u8_state                 = STATE_PUMP_IDLE;
      fv_u8_state_d1              = STATE_PUMP_IDLE;
      fv_u8_state_d2              = STATE_PUMP_IDLE;
    }
  }

  switch(fv_u8_state){
    case STATE_PUMP_OFF:
      if(fv_f32_thresh_high < fv_f32_pressure){
        Serial.println("Too high pressure, turning pump on.");
        fv_u8_state = STATE_PUMP_ON;
        digitalWrite(PIN_RELAY, RELAY_ON);      
      }
      else{fv_f32_time_active -= PUMP_OFF_ACTIVE_TIME_DECREMENTER;}
      break;
    case STATE_PUMP_ON:
      if(fv_f32_thresh_low > fv_f32_pressure){
        Serial.println("Reached target pressure, going over to post-run time.");
        fv_f32_time_postRun = 0.0;
        fv_u8_state         = STATE_PUMP_HOLD;
      }
      else if (PUMP_THERMAL_PROT_TIME <= fv_f32_time_active){
        Serial.println("Turning pump off as thermal protection.");
        digitalWrite(PIN_RELAY, RELAY_OFF);    
        fv_u8_state = STATE_PUMP_THERMAL_PROT;
      }
      else{fv_f32_time_active += ((float) STATE_RUN_TIME)/1000;}
      break;
    case STATE_PUMP_HOLD:
      if (PUMP_THERMAL_PROT_TIME <= fv_f32_time_active){
        Serial.println("Turning pump off as thermal protection.");
        digitalWrite(PIN_RELAY, RELAY_OFF);    
        fv_u8_state = STATE_PUMP_THERMAL_PROT;
      }
      else if(PUMP_POST_RUN_TIME <= fv_f32_time_postRun){
        Serial.println("Post-run time finished, turning pump off.");
        fv_u8_state = STATE_PUMP_OFF;
        digitalWrite(PIN_RELAY, RELAY_OFF);
      }
      else{
        fv_f32_time_active  += ((float) STATE_RUN_TIME)/1000;
        fv_f32_time_postRun += ((float) STATE_RUN_TIME)/1000;
      }
      break;
    case STATE_PUMP_THERMAL_PROT:
      if(fv_u8_state_d2 != fv_u8_state){
        // Intentional duplication of the off event, making sure the pump is off
        Serial.println("Turning pump off as thermal protection.");
        digitalWrite(PIN_RELAY, RELAY_OFF);
      }

      if(PUMP_THERMAL_PROT_RELEASE_TIME > fv_f32_time_active){
        if (2 == gv_u8_startStopState){
          Serial.println("Turning pump on after releasing thermal protection.");
          fv_u8_state = STATE_PUMP_ON;
          digitalWrite(PIN_RELAY, RELAY_ON);
        }
        else {
          Serial.println("Going to idle after releasing thermal protection.");
          fv_u8_state = STATE_PUMP_IDLE;
        }
      }
      else{
        fv_f32_time_active  -= PUMP_OFF_ACTIVE_TIME_DECREMENTER;
        fv_f32_time_coolDown = (fv_f32_time_active - PUMP_THERMAL_PROT_RELEASE_TIME)/PUMP_OFF_ACTIVE_TIME_DECREMENTER;
      }
      break;
    case STATE_PUMP_IDLE:
      fv_f32_time_active -= PUMP_OFF_ACTIVE_TIME_DECREMENTER;
      break;
    default:
      Serial.println("Unknown state. Turning pump off going to idle");
      fv_u8_state = STATE_PUMP_IDLE;
      digitalWrite(PIN_RELAY, RELAY_OFF);
      break;
  }

  drawSetPointUpdate(fv_f32_thresh_high,
                    fv_f32_thresh_low,
                    fv_f32_pressure,
                    state_pump_string(fv_u8_state),
                    (uint16_t) fv_f32_time_active,
                    (uint16_t) fv_f32_time_coolDown);
  fv_u8_state_d2 = fv_u8_state_d1;
  fv_u8_state_d1 = fv_u8_state;
}

void splashScreen(void){

  int16_t   fv_x1, fv_y1;
  uint16_t  fv_width, fv_height;
  uint16_t  fv_x = tft.width();
  uint16_t  fv_y = 2*(tft.height()/3);
  int i = 0;

  tft.setTextWrap(false);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.getTextBounds("VacuumPumpControl", 0, 0, &fv_x1, &fv_y1, &fv_width, &fv_height);

  for(;fv_y>0;fv_y -= 2){
    tft.fillRect(fv_x, fv_y+10, fv_width, fv_height, ILI9341_BLACK);
    fv_x = (int16_t) (0.0260596*(float)(fv_y * fv_y) - 2.563434*(float) fv_y + 63.02263);
    tft.setCursor(fv_x,fv_y+10);
    tft.setTextColor(ILI9341_BLUE);
    tft.print("Vacuum");
    tft.setTextColor(ILI9341_YELLOW);
    tft.print("Pump");
    tft.setTextColor(ILI9341_RED);
    tft.print("Control");
    delay(5); 
  }
  delay(3000);
  tft.fillRect(50, 28, 270, 15, ILI9341_BLACK);
}

void drawMainMenu(void){
  int16_t   fv_x1, fv_y1;
  uint16_t  fv_width, fv_height;

  drawBackButton(false);
  drawStartStopButton(0);
  tft.fillRect(0, 43, tft.width(), tft.height(), ILI9341_BLACK);
  drawButton(BUTTON_1X2_X_1, BUTTON_1X2_Y_1, BUTTON_1X2_WIDTH, BUTTON_1X2_HEIGTH, "Set Point");
  drawButton(BUTTON_1X2_X_1, BUTTON_1X2_Y_2, BUTTON_1X2_WIDTH, BUTTON_1X2_HEIGTH, "Profile");
}

void drawProfileMenu(void){
  int16_t   fv_x1, fv_y1;
  uint16_t  fv_width, fv_height;

  tft.fillRect(0, 43, tft.width(), tft.height(), ILI9341_BLACK);
  drawStartStopButton(0);
  drawButton(BUTTON_2X2_X_1, BUTTON_2X2_Y_1, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH, "De-gas");
  drawButton(BUTTON_2X2_X_1, BUTTON_2X2_Y_2, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH, "Profile 2");
  drawButton(BUTTON_2X2_X_2, BUTTON_2X2_Y_1, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH, "Profile 3");
  drawButton(BUTTON_2X2_X_2, BUTTON_2X2_Y_2, BUTTON_2X2_WIDTH, BUTTON_2X2_HEIGTH, "Profile 4");
  drawBackButton(true);
}

/**
 * @brief draws a button with centered text and configurable colors
 * @param [in] x:           the x coordinate centered in the button
 * @param [in] y:           the y coordinate centered in the button
 * @param [in] width:       width of the button button
 * @param [in] height:      height of in the button
 * @param [in] text:        Text to be centered inside the button
 * @param [in] textSize:    Size of the text (0 = no text)
 * @param [in] buttonColor: Color to be used to draw the rectangle of the button
 * @param [in] textColor:   Color to be used to draw the text of the button
 */
void drawButtonColor (uint16_t x,
                      uint16_t y,
                      uint16_t width,
                      uint16_t heigth,
                      const char* text,
                      uint8_t  textSize,
                      uint16_t buttonColor,
                      uint16_t textColor){
  int16_t   fv_x1, fv_y1;
  uint16_t  fv_width, fv_height;

  tft.fillRoundRect(x-(width/2), y-(heigth/2), width, heigth, 3, buttonColor);
  if(textSize){
    tft.setTextSize(textSize);
    tft.setTextColor(textColor);
    tft.getTextBounds(text, 0, 0, &fv_x1, &fv_y1, &fv_width, &fv_height);
    tft.setCursor(x - (fv_width/2), y - (fv_height/2));
    tft.print(text);
  }
}

/**
 * @brief draws a button with centered text
 * @param [in] x:       the x coordinate centered in the button
 * @param [in] y:       the y coordinate centered in the button
 * @param [in] width:   width of the button button
 * @param [in] height:  height of in the button
 * @param [in] text:    Text to be centered inside the button
 */
void drawButton (uint16_t x, uint16_t y, uint16_t width, uint16_t heigth, const char* text){
  drawButtonColor(x, y, width, heigth, text, 2, BUTTON_BLUE, ILI9341_BLACK);
}

/**
 * @brief Draws/Clears the back button in the top left corner
 * @param [in] visible: Whether to draw or clear the button
 */
void drawBackButton(bool visible){
  if(!visible){
    tft.fillRect(3,3,40,40,ILI9341_BLACK);
  }
  else{
    tft.fillRoundRect(3,3, 40, 40, 3, BUTTON_BLUE);
    tft.drawRect(13, 15, 15, 2, ILI9341_BLACK);
    tft.drawRect(13, 30, 15, 2, ILI9341_BLACK);
    tft.drawCircleHelper(25, 23, 8, 0x6, ILI9341_BLACK);
    tft.drawCircleHelper(25, 23, 7, 0x6, ILI9341_BLACK);
    tft.drawPixel(19,12,ILI9341_BLACK);
    tft.drawLine(17,13,19,13,ILI9341_BLACK);
    tft.drawLine(15,14,19,14,ILI9341_BLACK);
    tft.drawLine(15,17,19,17,ILI9341_BLACK);
    tft.drawLine(17,18,19,18,ILI9341_BLACK);
    tft.drawPixel(19,19,ILI9341_BLACK);
  }  
}

/**
 * @brief Draws/Clears the back button in the top left corner
 * @param [in] state: 0 -> black, 1 -> green, 2 -> red
 */
void drawStartStopButton(uint8_t state){
  gv_u8_startStopState = state;
  switch(state){    
    case 1:
      drawButtonColor(tft.width()-23,23,40,40,"Start",1,ILI9341_GREEN,ILI9341_BLACK);
      break;
    case 2:
      drawButtonColor(tft.width()-23,23,40,40,"Stop",1,ILI9341_RED,ILI9341_BLACK);
      break;
    case 0:
      // Intentional fall-through
    default:
      drawButtonColor(tft.width()-23,23,40,40,"",0,ILI9341_BLACK,ILI9341_BLACK);
      gv_u8_startStopState = 0;
      break;
  }    
}

/**
 * @brief Check if a given point falls within the square located around the center
 * @param [in] point:   the point in question
 * @param [in] x:       the center x coordinate
 * @param [in] y:       the center y coordinate
 * @param [in] width:   width of the square
 * @param [in] height:  height of in square
 */
bool checkPointInSquare(TS_Point point, uint16_t x, uint16_t y, uint16_t width, uint16_t height){
  if (((x - width/2) <= point.x) && ( (x + width/2) >= point.x)){
    if (((y - height/2) <= point.y) && ( (y + height/2) >= point.y)){
      return true;
    }
  }
  return false;
}

/**
 * @brief Check if a given point falls within the back button location
 * @param [in] point: the point in question
 */
bool backButtonLocation(TS_Point point){
  return checkPointInSquare(point, 25, 25, 50, 50);
}

/**
 * @brief Draws a rounded square 40 by 40 with a + or - symbol in it
 * @param [in] incDec:  true -> +, false -> -
 * @param [in] x:       x coordinate of the center
 * @param [in] y:       y coordinate of the center
 */
void drawIncDecButton(bool incDec, uint16_t x, uint16_t y){
  tft.drawRoundRect(x-20, y-20, 40, 40, 3, ILI9341_WHITE);
  tft.fillRect(x-10,y-2,20,4,ILI9341_WHITE);
  if(incDec){
    tft.fillRect(x-2,y-10,4,20,ILI9341_WHITE);
  }
}

/** 
 * @brief Draws out the static parts of the Set point operation screen
 */
void drawSetPointInitial(){
  drawBackButton(true);
  drawStartStopButton(1);
  tft.fillRect(0, 43, tft.width(), tft.height(), ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20,55);
  tft.print("State:");
  tft.setTextSize(1);
  tft.setCursor(205,80);
  tft.print("Current pressure:");
  tft.setCursor(205,130);
  tft.print("Active time:");
  tft.setCursor(205,180);
  tft.print("Cooldown time:");
  drawIncDecButton(false, 40, 120);
  drawIncDecButton(true, 170, 120);
  drawIncDecButton(false, 40, 190);
  drawIncDecButton(true, 170, 190);
}

void drawSetPointUpdate(float pressHigh, float pressLow, float press, const char* state, uint16_t onTime, uint16_t coolDownTime){
  char fv_a6_str[6];

  tft.fillRect(100,55,220,20,ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(100,55);
  tft.print(state);

  sprintf(fv_a6_str, "%.03f",pressHigh);
  tft.fillRect(75,112,60,18,ILI9341_BLACK);
  tft.setCursor(75,112);
  tft.print(fv_a6_str);

  sprintf(fv_a6_str, "%.03f",pressLow);
  tft.fillRect(75,182,60,18,ILI9341_BLACK);
  tft.setCursor(75,182);
  tft.print(fv_a6_str);

  sprintf(fv_a6_str, "%.03f",press);
  tft.fillRect(220,100,60,18,ILI9341_BLACK);
  tft.setCursor(220,100);
  tft.print(fv_a6_str);

  tft.fillRect(220,150,100,18,ILI9341_BLACK);
  tft.setCursor(220,150);
  tft.print(onTime);
  tft.print(" s");

  tft.fillRect(220,200,100,18,ILI9341_BLACK);
  tft.setCursor(220,200);
  tft.print(coolDownTime);
  tft.print(" s");
}

/** 
 * @brief Draws out the static parts of the Set point operation screen
 */
void drawProfileInitial(void){//profile_t* profile){

  char fv_a100_str[100];

  drawBackButton(true);
  drawStartStopButton(1);
  tft.fillRect(0, 43, tft.width(), tft.height(), ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20,55);
  tft.print("State:");
  tft.setTextSize(1);
  tft.setCursor(205,80);
  tft.print("Current pressure:");
  tft.setCursor(205,130);
  tft.print("Active time:");
  tft.setCursor(205,180);
  tft.print("Cooldown time:");

/*
  for(int i=0;i<profile.len;i++){
    sprintf(fv_a100_str,"Point %d on time %lu go to pressure %.03f",i ,profile.array[i].time, profile.array][i].array);
  }*/
}

//TODO these probably should be moved outside the main ino file
#if defined(VERIFY_CALIBRATION) || defined(RUN_CALIBRATION)
void crosshair(TS_Point p) {
  tft.drawCircle   (p.x,     p.y,     6, ILI9341_WHITE);
  tft.drawFastHLine(p.x - 4, p.y,     9, ILI9341_WHITE);
  tft.drawFastVLine(p.x,     p.y - 4, 9, ILI9341_WHITE);
}

uint16_t distance(TS_Point a, TS_Point b) {
  // calculate the distance between points a and b in whatever 2D coordinate
  // system they both exist. returns an integer distance with naive rounding.
  static uint16_t const MAX = ~((uint16_t)0U);
  int32_t dx = b.x - a.x;
  int32_t dy = b.y - a.y;
  uint32_t dsq = (uint32_t)sq(dx) + (uint32_t)sq(dy);
  double d = sqrt(dsq); // add 1/2 for rounding
  if (d > ((double)MAX - 0.5))
    { return MAX; }
  else
    { return (uint16_t)(d + 0.5); } // poor-man's rounding
}

void drawMapping(PointID n, TS_Point tp) {
  static uint8_t const BUF_LEN = 64;
  static char buf[BUF_LEN] = { '\0' };

  static uint16_t lineHeight = (uint16_t)(1.5F * 8.0F + 0.5F);
  static uint16_t lineSpace  = 1U;

  int16_t posX, posY;
  uint16_t sizeW, sizeH;
  uint16_t posLeft = 6U;
  uint16_t posTop =
    SCREEN_HEIGHT - (3U - (uint16_t)n) * (lineHeight + lineSpace);

  TS_Point sp = _screenPoint[(int)n];

  // construct the line buffer
  snprintf(buf, BUF_LEN, "%c (%u,%u) = (%u,%u)",
    (uint8_t)n + 'A', sp.x, sp.y, tp.x, tp.y);

  // print the current line to serial port for debugging
  //Serial.println("%s\n", buf);
  Serial.println(buf);

  // erase the previous line
  tft.getTextBounds(buf, posLeft, posTop, &posX, &posY, &sizeW, &sizeH);
  tft.fillRect(posX, posY, sizeW, sizeH, ILI9341_BLACK);

  // draw the current line
  tft.setCursor(posLeft, posTop);
  //tft.printf("%s", buf);
  tft.print(buf);
}

// -- NOT FOR GENERAL USAGE -- IGNORE THESE --------------------------- BEGIN --
// approximate calibration only used for determining distance from touch to
// crosshair while calibrating. you can determine these values by using the
// updateScreenEdges() routine below -- just call it with the position of every
// touch event while scrubbing all 4 edges of the screen with a stylus.

#define MAP_2D_PORTRAIT(x, y)                                        \
  TS_Point(                                                          \
    (int16_t)map((x), XPT2046_X_LO, XPT2046_X_HI, 0,  SCREEN_WIDTH), \
    (int16_t)map((y), XPT2046_Y_LO, XPT2046_Y_HI, 0, SCREEN_HEIGHT)  \
  )
#define MAP_2D_LANDSCAPE(x, y)                                       \
  TS_Point(                                                          \
    (int16_t)map((x), XPT2046_Y_LO, XPT2046_Y_HI, 0,  SCREEN_WIDTH), \
    (int16_t)map((y), XPT2046_X_LO, XPT2046_X_HI, 0, SCREEN_HEIGHT)  \
  )
void updateScreenEdges(TS_Point p) {
  static uint16_t xHi = 0xFFFF;
  static uint16_t yHi = 0xFFFF;
  static uint16_t xLo = 0x0;
  static uint16_t yLo = 0x0;
  if (p.x < xHi) { xHi = p.x; }
  if (p.x > xLo) { xLo = p.x; }
  if (p.y < yHi) { yHi = p.y; }
  if (p.y > yLo) { yLo = p.y; }
  /*Serial.printf("[X_LO, X_HI] = [%u, %u], [Y_LO, Y_HI] = [%u, %u]\n",
      xLo, xHi, yLo, yHi);*/
  Serial.print("[X_LO, X_HI] = [");
  Serial.print(xLo);
  Serial.print(", ");
  Serial.print(xHi);
  Serial.print("], [Y_LO, Y_HI] = [");
  Serial.print(yLo);
  Serial.print(", ");
  Serial.print(yHi);
  Serial.println("]");
}
// -- NOT FOR GENERAL USAGE -- IGNORE THESE ----------------------------- END --

PointID nearestScreenPoint(TS_Point touch) {

#ifdef VERIFY_CALIBRATION
  // the input point is already in screen coordinates because the touchscreen
  // has been calibrated. no need to perform translation.
  TS_Point tp = touch;
#else
  // translate the input point (in touch coordinates) to screen coordinates
  // using the hardcoded ranges defined in these macros. not particularly
  // accurate, but it doesn't need to be.
  TS_Point tp = (SCREEN_ROTATION & 1U)
    ? MAP_2D_LANDSCAPE(touch.x, touch.y)
    : MAP_2D_PORTRAIT(touch.x, touch.y);
  /*Serial.printf(
      "Touch{%u, %u} => Screen{%u, %u}\n", touch.x, touch.y, tp.x, tp.y);*/
    Serial.print("Touch{");
    Serial.print(touch.x);
    Serial.print(", ");
    Serial.print(touch.y);
    Serial.print("} => Screen{");
    Serial.print(tp.x);
    Serial.print(", ");
    Serial.print(tp.y);
    Serial.println("}");
#endif

  PointID  near = PointID::NONE;
  uint16_t dist = 0U;

  for (int id = (int)PointID::NONE + 1; id < (int)PointID::COUNT; ++id) {
    // compute the distance from our (translated) input touch point to each
    // screen point to determine minimum distance.
    uint16_t d = distance(tp, _screenPoint[id]);
    if ((near == PointID::NONE) || (d < dist)) {
      // new minimum distance, this is the nearest point to our touch (so far)
      near = (PointID)id;
      dist = d;
    }
  }

  return near;
}
#endif

void tft_benchmark(void){

  // Reduce the SPI speed for reading due to HW bug
  tft.setSPISpeed(50000);
  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
  
  //Restore the SPI speed to the imperical determined maximum
  tft.setSPISpeed(12000000);

  Serial.println(F("Benchmark                Time (microseconds)"));
  delay(10);
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(ILI9341_CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(ILI9341_RED, ILI9341_BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(ILI9341_GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, ILI9341_MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, ILI9341_WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  Serial.println(F("Done!"));

}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.fillScreen(ILI9341_RED);
  yield();
  tft.fillScreen(ILI9341_GREEN);
  yield();
  tft.fillScreen(ILI9341_BLUE);
  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}