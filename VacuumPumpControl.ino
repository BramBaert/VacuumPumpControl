/*
  Basic test of the Qwiic MicroPressure Sensor
  By: Alex Wende
  SparkFun Electronics
  Date: July 2020
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16476
  
  This example demonstrates how to get started with the Qwiic MicroPressure Sensor board, and read pressures in various units.
*/

// Include the SparkFun MicroPressure library.
// Click here to get the library: http://librarymanager/All#SparkFun_MicroPressure

#include<Wire.h>
#include <SparkFun_MicroPressure.h>

#define RELAY_ON  HIGH
#define RELAY_OFF LOW
#define STATE_PUMP_OFF          1
#define STATE_PUMP_ON           2
#define STATE_PUMP_HOLD         3
#define STATE_PUMP_THERMAL_PROT 4

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr; // Use default values with reset and EOC pins unused

void setup() {
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_D5, OUTPUT);
  digitalWrite(PIN_D5, RELAY_OFF);
  Serial.begin(115200);
  Serial.println("Hello world!");
  Wire1.begin();

  /* The micropressure sensor uses default settings with the address 0x18 using Wire.

     The mircropressure sensor has a fixed I2C address, if another address is used it
     can be defined here. If you need to use two micropressure sensors, and your
     microcontroller has multiple I2C buses, these parameters can be changed here.

     E.g. mpr.begin(ADDRESS, Wire1)

     Will return true on success or false on failure to communicate. */
  if(!mpr.begin(DEFAULT_ADDRESS,Wire1))
  {
    Serial.println("Cannot connect to MicroPressure sensor.");
    while(1);
  }
}

void loop() {

  static float    fv_f32_thresh_high = 0.25;
  static float    fv_f32_thresh_low  = 0.2;
  float           fv_f32_pressure    = 0.0;
  static uint8_t  fv_u8_state        = STATE_PUMP_OFF;

  /* The micropressure sensor outputs pressure readings in pounds per square inch (PSI).
     Optionally, if you prefer pressure in another unit, the library can convert the
     pressure reading to: pascals, kilopascals, bar, torr, inches of murcury, and
     atmospheres.
   */
  fv_f32_pressure = mpr.readPressure(BAR);
  Serial.print(fv_f32_pressure,6);
  Serial.println(" bar");

  switch(fv_u8_state){
    case STATE_PUMP_OFF:
      if(fv_f32_thresh_high < fv_f32_pressure){
        Serial.println("Turning pump on.");
        fv_u8_state = STATE_PUMP_ON;
        digitalWrite(PIN_D5, RELAY_ON);      
      }
      break;
    case STATE_PUMP_ON:
      if(fv_f32_thresh_low > fv_f32_pressure){
        Serial.println("Turning pump off.");
        fv_u8_state = STATE_PUMP_OFF;
        digitalWrite(PIN_D5, RELAY_OFF);
      }
      break;
    case STATE_PUMP_HOLD:
    case STATE_PUMP_THERMAL_PROT:
    default:
      Serial.println("Turning pump off.");
      fv_u8_state = STATE_PUMP_OFF;
      digitalWrite(PIN_D5, RELAY_OFF);
      break;
  }

  delay(1000);
}