// Carpet CANTroller II  Source Code  - For Arduino Mega 2560 with Adafruit 2.8inch Captouch TFT shield.

// Libraries to include.  Note all these have example code when installed into arduino ide
#include <Servo.h>   // Makes PWM output to control motors (for our gas and steering)
#include <FastPID.h>  // Fixed-point math based PID loop (for our brakes and maybe cruise control)
#include <SPI.h>   // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>      // Contains I2C serial bus, needed to talk to touchscreen chip
#include <Adafruit_ILI9341.h>   // For interfacing with the TFT LCD controller chip
#include <Adafruit_FT6206.h>   // For interfacing with the cap touchscreen controller chip
#include <SdFat.h>        // SD card & FAT filesystem library
#include <Adafruit_GFX.h>    // For drawing pictures & text on the screen
//#include <Adafruit_ImageReader.h>  // We don't really need thihs but can use it to test that SD flash is working
//#include "TouchScreen.h"  // Touchscreen library (I don't think we need it? Maybe for resistive TP we're not using)

// Some human readable integers for array indexing
#define STR 0   // indexer for steering values
#define BRK 1   // indexer for brake values
#define GAS 2   // indexer for throttle values
#define V 0   // indexer for vertical values
#define H 1   // indexer for horizontal values
#define BOOT -1
#define SHUTDOWN 0
#define BASIC 1
#define STALL 2
#define HOLD 3
#define FLY 4
#define CRUISE 5

// Defines for all the GPIO pins we're using
#define ENC_B_PIN 2  // Int input, Encoder is user input knob for the UI.  This is its B quadrature output, active low (needs pullup)
#define ENC_A_PIN 3  // Int input, Encoder is user input knob for the UI.  This is its A quadrature output, active low (needs pullup)
#define USD_CS_PIN 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
#define TFT_LEDK_PIN 5   // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define BRAKE_PWM_PIN 6  // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define TP_IRQ_PIN 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
#define STEERING_PWM_PIN 8  // Output, PWM signal duty cycle sets speed of steering motor from full speed left, to full speed right, (50% is stopped)
#define TFT_DC_PIN 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
#define TFT_CS_PIN 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define GAS_PWM_PIN 13 // Output, PWM signal duty cycle controls throttle angle
#define IGNITION_PIN 15  // Input tells us if ignition signal is on or off, active high (no pullup)
#define NEUTRAL_PIN 16  // Input, active low, is asserted when car is in neutral, i.e. out of gear.  (needs pullup)
#define BASICMODE_SW_PIN 17  // Input, active low, asserted to tell us to run in basic mode.   (needs pullup)
#define TACH_PULSE_PIN 18  // Int Input, active high, asserted when magnet is in range of sensor. 4 pulses per engine rotation. (no pullup)
#define SPEEDO_PULSE_PIN 19  // Int Input, active high, asserted when magnet is in range of sensor. 8 pulses per driven pulley rotation. (no pullup)
#define ENC_SW_PIN 20  // Int input, Encoder is user input knob for the UI.  This is its pushbutton output, active low (needs pullup)
#define JOY_VERT_PIN A0  // Analog input, tells us up-down position of joystick
#define JOY_HORZ_PIN A1  // Analog input, tells us left-right position of joystick
#define BRAKE_PRES_PIN A2  // Analog input, tells us brake fluid pressure (5V = 1000psi)
#define BRAKE_POS_PIN A6  // Analog input, tells us linear position of brake actuator 
#define MULE_VOLT_PIN A7  // Analog input, mule battery voltage level, full scale is 5V * 3.1276 = 15.638V

// Initialization of global variables, tunable parameters, etc.
#define TACH_TIMEOUT_MS 400  // Time in ms to wait for a magnet pulse before declaring engine has stopped
#define SPEEDO_TIMEOUT_MS 400  // Time in ms to wait for a magnet pulse before declaring vehicle has stopped

const int GAS_MIN_IDLE_BYTE = 35; // What duty cycle (out of 255) is the throttle fully closed
const int ADC_MID_SCALE = 512;
const int ADC_FULL_SCALE = 1024;
const int JOY_RAW_DEADBAND_SIZE = 140;
const int JOY_RAW_DEADBAND_MIN = (ADC_MID_SCALE-(JOY_RAW_DEADBAND_SIZE/2));
const int JOY_RAW_DEADBAND_MAX = (ADC_MID_SCALE+(JOY_RAW_DEADBAND_SIZE/2));
const int SERVO_MAX_ANGLE_IN_DEGREES = 180;
//const int BRAKE_FULL_EXTEND_ANGLE = ;
const int GAS_IDLE_MIN_ANGLE = 45;
float brake_kp = 0.1, brake_ki = 0.5, brake_kd = 0, brake_hz = 10;
int output_bits = 8;
bool output_signed = false;

unsigned long tach_old_time, speedo_old_time, tach_time, speedo_time;

// Instantiate objects 
Adafruit_FT6206 captouch_panel = Adafruit_FT6206();
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_PIN, TFT_DC_PIN);
static Servo steering_motor;
static Servo gas_servo;
FastPID brake_pid(brake_kp, brake_ki, brake_kd, brake_hz, output_bits, output_signed);
SdFat SD;         // SD card filesystem

// Setup code runs once at boot time
void setup() {
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(BRAKE_PWM_PIN, OUTPUT);
    pinMode(STEERING_PWM_PIN, OUTPUT);
    pinMode(TFT_DC_PIN, OUTPUT);
    pinMode(ENC_SW_PIN, INPUT_PULLUP);
    pinMode(GAS_PWM_PIN, OUTPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(NEUTRAL_PIN, INPUT_PULLUP);
    pinMode(BASICMODE_SW_PIN, INPUT_PULLUP);
    pinMode(TACH_PULSE_PIN, INPUT);
    pinMode(SPEEDO_PULSE_PIN, INPUT);
    pinMode(JOY_VERT_PIN, INPUT);
    pinMode(JOY_HORZ_PIN, INPUT);
    pinMode(BRAKE_PRES_PIN, INPUT);
    pinMode(BRAKE_POS_PIN, INPUT);
    pinMode(MULE_VOLT_PIN, INPUT);
    pinMode(USD_CS_PIN, OUTPUT);
    pinMode(TFT_CS_PIN, OUTPUT);
    //pinMode(TFT_LEDK_PIN, OUTPUT);
    //pinMode(TP_IRQ_PIN, INPUT);

    // Set all outputs to known sensible values
    digitalWrite(TFT_CS_PIN, HIGH);   // Prevent bus contention
    digitalWrite(USD_CS_PIN, HIGH);   // Prevent bus contention
    digitalWrite(TFT_DC_PIN, LOW);
    analogWrite(BRAKE_PWM_PIN, 128);   // Write values range from 0 to 255
    analogWrite(STEERING_PWM_PIN, 128);
    analogWrite(GAS_PWM_PIN, GAS_MIN_IDLE_BYTE);

    // Timers T0 (8b): pins 4,13 - T1 (16b): pins 11,12 - T2 (8b): pins 9,10 - T3 (16b): pins 2,3,5 - T4 (16b): pins 6,7,8 - T5 (16b): pins 44,45,46
    // Registers: TCCRx = Timer control, prescalar, TCNTx = Timer value, OCRx = Output compare, ICRx = Input Capture, TIMSKx = Int mask, TIFRx = Int Flags
    TCCR0B = TCCR0B & B11111000 | B00000101; // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz (Gas Servo)
    TCCR4B = TCCR4B & B11111000 | B00000100; // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz (Brake & Steering)
    
    //while (!Serial);     // needed for debugging?!
    Serial.begin(115200);
    tft.begin();
    
    Serial.println(F("Captouch initialization"));
    if (! captouch_panel.begin(40)) {  // pass in 'sensitivity' coefficient
        Serial.println("Couldn't start FT6206 touchscreen controller");
        while (1);
    }
    Serial.println("Capacitive touchscreen started");
    tft.fillScreen(ILI9341_BLACK);
    
    Serial.print(F("Initializing filesystem..."));  // SD card is pretty straightforward, a single call. 
    if (! SD.begin(USD_CS_PIN, SD_SCK_MHZ(25))) {   // ESP32 requires 25 MHz limit
        Serial.println(F("SD begin() failed"));
        for(;;); // Fatal error, do not continue
    }
    Serial.println(F("Filesystem started"));
    tft.fillScreen(ILI9341_BLUE);  // Fill screen blue. 

    steering_motor.attach(STEERING_PWM_PIN);
    gas_servo.attach(GAS_PWM_PIN);

    // Set up our interrupts
    bool encb_isr_flag = false;
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), EncoderB_ISR, CHANGE);
    bool enca_isr_flag = false;
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), EncoderA_ISR, CHANGE);
    bool encsw_isr_flag = false;
    attachInterrupt(digitalPinToInterrupt(ENC_SW_PIN), EncoderSw_ISR, CHANGE);
    bool tach_isr_flag = false;
    attachInterrupt(digitalPinToInterrupt(TACH_PULSE_PIN), Tach_ISR, RISING);
    bool speedo_isr_flag = false;
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), Speedo_ISR, RISING);
}

void EncoderB_ISR(void) {
    encb_isr_flag = true;
}
void EncoderA_ISR(void) {
    enca_isr_flag = true;
}
void EncoderSw_ISR(void) {
    encsw_isr_flag = true;
}
void Tach_ISR(void) {
    tach_time = millis();  // This might screw up things.  Anders would remember
    tach_isr_flag = true;
}
void Speedo_ISR(void) {
    speedo_time = millis();  // This might screw up things.  Anders would remember
    speedo_isr_flag = true;
}

void set_steering_pwm_output(int angle_in_degrees) {
    steering_motor.write(angle_in_degrees);
}

void set_gas_pwm_output(int angle_in_degrees) {
    gas_servo.write(angle_in_degrees);
}

int* read_joy_values(void) {
    int joy_raw_values[2];
    joy_raw_values[0] = analogRead(JOY_VERT_PIN);
    joy_raw_values[1] = analogRead(JOY_HORZ_PIN);

    int joy_values[2];  

    for (axis=V; axis>H; axis++) {
        //check if the value is in the deadband, return 0 input if it is
        if ((joy_raw_values[axis] > JOY_RAW_DEADBAND_MIN) && (joy_raw_values[axis] < JOY_RAW_DEADBAND_MAX)) {
            joy_values[axis] = ADC_MID_SCALE;
        }
        else {
            joy_values[axis] = ADC_FULL_SCALE-joy_raw_values[axis];
        }
    return joy_values;
}

int *joy_to_actuator_angles(int *joy_values) {
    long servo_max_angle_deg_long = (long)SERVO_MAX_ANGLE_IN_DEGREES;
    long adc_mid_scale_long = (long)ADC_MID_SCALE;
    long adc_full_scale_long = (long)ADC_FULL_SCALE;

    int actuator_angles[3];
    long joy_values_long[2];
    
    joy_values_long[V] = (long)joy_values[V];
    joy_values_long[H] = (long)joy_values[H];  
    
    actuator_angles[STR] = (int)((joy_values_long[H]*servo_max_angle_deg_long)/adc_full_scale_long);
    
    //actuator_angles[BRK] = BRAKE_FULL_EXTEND_ANGLE;
    //if (joy_values[V] < ADC_MID_SCALE)
    //{
    //  actuator_angles[BRK] = (int)((adc_mid_scale_long-joy_values_long[V])*servo_max_angle_deg_long/adc_mid_scale_long);
    //}

    actuator_angles[GAS] = GAS_IDLE_MIN_ANGLE;
    if (joy_values[V] > ADC_MID_SCALE) {
        actuator_angles[GAS] = (int)((joy_values_long[V]-adc_mid_scale_long)/adc_mid_scale_long);
    }

    return actuator_angles;
}

// Loop code runs repeatedly and forever, directly after setup finishes
void loop() {
    int joy_values[2];     // 0 = vert, 1 = horz
    int actuator_angles[3];    // 0 = steering, 1 = brake, 2 = gas

    int brake_pressure = analogRead(BRAKE_PRES_PIN);
    int pwmtimer
    
    // Update engine rpm and car speed values
    int now = millis()
    if (tach_isr_flag == true) {
        float engine_speed_rpm = 60*1000/(4*(tach_time-tach_old_time))
        tach_old_time = tach_time;
        tach_isr_flag = false
    } 
    else if (now-tach_old_time > TACH_TIMEOUT_MS) {
        engine_speed_rpm = 0;
    }
    if (speedo_isr_flag == true) {  
        float car_speed_arb = 1000/(8*(speedo_time-speedo_old_time))
        speedo_old_time = speedo_time;
        tach_isr_flag = false
    }  // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
    else if (now-speedo_old_time > SPEEDO_TIMEOUT_MS) {
        car_speed_arb = 0;
    }

    joy_values = read_joy_values();
    
    actuator_angles = joy_to_actuator_angles(joy_values);

    now = millis()
    if (now-pwmtimer > 15)  {}  // Update gas and steering output only every 15ms
        set_steering_pwm_output(actuator_angles[STR]);
        set_gas_pwm_output(actuator_angles[GAS]);
        pwmtimer = now;
   }
    
    before = micros();
    uint8_t brake_output = brake_pid.step(actuator_angles[BRK], brake_pressure);
    after = micros();
    analogWrite(BRAKE_PWM_PIN, brake_output);
    Serial.print("runtime: "); 
    Serial.print(after - before);
    Serial.print(" sp: "); 
    Serial.print(setpoint); 
    Serial.print(" fb: "); 
    Serial.print(feedback);
    Serial.print(" out: ");
    Serial.println(output);

    if (captouch_panel.touched())  {
        TS_Point touch_point = captouch_panel.getPoint();  // Retrieve a point  
        touch_point.x = map(touch_point.x, 0, 240, 240, 0);   // flip it around to match the screen.
        touch_point.y = map(touch_point.y, 0, 320, 320, 0);   // flip it around to match the screen.
    }
}
