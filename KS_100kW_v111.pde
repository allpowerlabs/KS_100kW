// KS_100kW
// Library used to run APL 100kW PowerTainer
// Developed for the APL GCU/PCU: http://gekgasifier.pbworks.com/Gasifier-Control-Unit

#include <EEPROM.h>         // included with Arduino, can read/writes to non-volatile memory
#include <Servo.h>          // Arduino's native servo library
#include <PID_Beta6.h>      // http://www.arduino.cc/playground/Code/PIDLibrary, http://en.wikipedia.org/wiki/PID_controller
#include <adc.h>            // part of KSlibs, for reading analog inputs
#include <display.h>        // part of KSlibs, write to display
#include <fet.h>            // part of KSlibs, control FETs (field effect transitor) to drive motors, solenoids, etc
#include <keypad.h>         // part of KSlibs, read buttons and keypad
#include <pressure.h>       // part of KSlibs, read pressure sensors
#include <servos.h>         // part of KSlibs, not implemented
#include <temp.h>           // part of KSlibs, read thermocouples
#include <timer.h>          // part of KSlibs, not implemented
#include <ui.h>             // part of KSlibs, menu
#include <util.h>           // part of KSlibs, utility functions, GCU_Setup
#include <avr/io.h>         // advanced: provides port definitions for the microcontroller (ATmega1280, http://www.atmel.com/dyn/resources/prod_documents/doc2549.PDF)   
#include <SD.h>

//#include <Canbus.h>
//#include <defaults.h>
//#include <global.h>
//#include <mcp2515.h>
//#include <mcp2515_defs.h>

//constant definitions
#define ABSENT -500

#define CODE_VERSION "100kW v1.11"

// Analog Input Mapping
#define ANA_EO2 ANA0
#define ANA_FO2 ANA1
#define ANA_FUEL_SWITCH ANA2
#define ANA_POT1 ANA3
#define ANA_POT2 ANA4
#define ANA_BOOT_LOWER ANA5
#define ANA_BOOT_UPPER ANA6
#define ANA_7 ABSENT

// FET Mapping
#define FET_0 ABSENT
#define FET_GRATE FET1
#define FET_2 ABSENT
#define FET_STARTER ABSENT
#define FET_FLARE_IGNITOR FET4
#define FET_O2_RESET ABSENT
#define FET_ALARM FET6
#define FET_ASHOUT FET3

//Servo Mapping
//TODO: Use these define
#define SERVO_MIXTURE SERVO0
#define SERVO_1 SERVO1
#define SERVO_2 SERVO2

Servo Servo_Mixture;
Servo Servo_Calib;
Servo Servo_Throttle;

//Thermocouple Mappings
#define T_REST1 0
#define T_RED1 1
#define T_RED2 2
#define T_RED3 3
//#define T_? 4  // Thermocouple 4 unused as of now
#define T_FLARE 9
#define T_AUGER_GAS_OUT 10
#define T_AUGER_FUEL_IN 11
//#define T_? 8  // Thermocouple 8 unused as of now
#define T_RED4 4
#define T_RED5 5
#define T_BGRATE 6
#define T_GAS_OUT 7
#define T_TAUGER 8

//Pressure Mapping
#define P_REACTOR 0 //Labeled Line 4
#define P_FILTER 1  //Labeled Line 5
#define P_COMB 2    //Labeled Line 2
#define P_GAS_OUT 3 //Labeled Line 1
#define P_BGRATE 4  //Labeled Line 3
//#define P_NC 5  //not in use yet


// Grate Shaking States
#define GRATE_SHAKE_OFF 0
#define GRATE_SHAKE_ON 1
#define GRATE_SHAKE_PRATIO 2
#define GRATE_SHAKE_POT_INTERVAL 3
unsigned long pot_period;
unsigned long ash_on = 0;
#define ASH_SCRAPER_PERIOD 1000*16


// Grate Motor States
#define GRATE_OFF 0
#define GRATE_ON 1
#define GRATE_LOW 2
#define GRATE_HIGH 3
#define GRATE_PRATIO_THRESHOLD 180 //number of seconds until we use high shaking mode

// Grate Shaking
#define GRATE_SHAKE_CROSS 5000
#define GRATE_SHAKE_INIT 32000

//Control States
#define CONTROL_OFF 0
#define CONTROL_START 1
#define CONTROL_ON 2

//Engine States
#define ENGINE_OFF 0
#define ENGINE_ON 1
#define ENGINE_OVERSPEED 2

//Flare States
#define FLARE_OFF 0
#define FLARE_USER_SET 1
#define FLARE_LOW 2
#define FLARE_HIGH 3
#define FLARE_MAX 4

//Mixture States
#define MIXTURE_POT_CONTROL 0
#define MIXTURE_P_COMB 1
#define MIXTURE_RATE_BASED 2
#define MIXTURE_OVERSPEED 3
#define MIXTURE_OPEN 4

//Display States
#define DISPLAY_SPLASH 0
#define DISPLAY_REACTOR 1
#define DISPLAY_ENGINE 2
#define DISPLAY_TEST 3
#define DISPLAY_MIXTURE 4
#define DISPLAY_PRESSURE_PID 5
#define DISPLAY_GRATE 6
#define DISPLAY_TESTING 7
#define DISPLAY_SERVO 8
#define DISPLAY_CONFIG 9

String display_string = "";

//Testing States
#define TESTING_OFF 0
#define TESTING_FET0 1
#define TESTING_FET1 2
#define TESTING_FET2 3
#define TESTING_FET3 4
#define TESTING_FET4 5
#define TESTING_FET5 6
#define TESTING_FET6 7
#define TESTING_FET7 8
#define TESTING_ANA_EO2 9
#define TESTING_ANA_FO2 10
#define TESTING_ANA_FUEL_SWITCH 11
#define TESTING_ANA_POT1 12
#define TESTING_ANA_POT2 13
#define TESTING_SERVO 14

//Configuration Variables
int config_var;
byte config_changed = false;
static char *Configuration[] = { "CANbus speed   ", "PR_LOW (.01)   ", "PR_HIGH (.01)  ", "Servo Control  " };  //15 character Display prompt
static char *Config_Choices[] = {"+5   -5 ","+    -  ","+    -  ", "POT PID "}; //8 char options for last two buttons
int defaults[] = {50,90,30,0};  //default values to be saved to EEPROM for the following getConfig variables
int config_min[] = {0,70, 9, 0};  //minimum values allowed 
int config_max[] = {254,99, 69, 254}; //maximum values allowed  

//Don't forget to add the following to update_config_var in Display!
int CANbus_speed = 5*getConfig(1); 
float PR_LOW_boundary = getConfig(2)/100.0;
float PR_HIGH_boundary = getConfig(3)/100.0;
int PID_Control = getConfig(4);

//Test Variables
int testing_state = TESTING_OFF;
unsigned long testing_state_entered = 0;
static char *TestingStateName[] = { "Off","FET0","FET1","FET2","FET3","FET4","FET5","FET6","FET7","ANA_EO2","ANA_FO2","ANA_Fuel_Switch","ANA_POT1","ANA_POT2"};
// Datalogging variables
int lineCount = 0;
int data_log_num;

// Grate turning variables
int grateMode = GRATE_SHAKE_POT_INTERVAL; //set default starting state
int grate_state; //changed to indicate state (for datalogging, etc)
int grate_val = GRATE_SHAKE_INIT; //variable that is changed and checked
int grate_pratio_accumulator = 0; // accumulate high pratio to trigger stronger shaking past threshhold
int grate_max_interval = 5*60; //longest total interval in seconds
int grate_min_interval = 60;
int grate_on_interval = 3;
//define these in init, how much to remove from grate_val each cycle [1 second] (slope)
int m_grate_bad; 
int m_grate_good;
int m_grate_on;
unsigned long next_shake;
unsigned long grate_on;
unsigned long grate_shake_start;
int shake_num;
int PULSE=4000;

// Reactor pressure ratio
float pRatioReactor;
enum pRatioReactorLevels { PR_HIGH = 0, PR_CORRECT = 1, PR_LOW = 2} pRatioReactorLevel;
static char *pRatioReactorLevelName[] = { "High", "Correct","Low" };
//float pRatioReactorLevelBoundary[3][2] = { { 0.9, 1.0 }, { 0.3, 0.6 }, {0.0, 0.3} };
float pRatioReactorLevelBoundary[3][2] = {{PR_HIGH_boundary, 1.0}, {PR_LOW_boundary, 0.6 }, {0.0, PR_LOW_boundary} };

// Filter pressure ratio
float pRatioFilter;
boolean pRatioFilterHigh;
int filter_pratio_accumulator;

// Temperature Levels
#define TEMP_LEVEL_COUNT 5
enum TempLevels { COLD = 0,COOL = 1,WARM = 2 ,HOT = 3, EXCESSIVE = 4} TempLevel;
TempLevels T_rest1Level;
static char *TempLevelName[] = { "Cold", "Cool", "Warm", "Hot", "Too Hot" };
int T_rest1LevelBoundary[TEMP_LEVEL_COUNT][2] = { { 0, 40 }, {50, 80}, {300,790}, {800,950}, {1000,1250} };

TempLevels T_red1Level;
int T_red1LevelBoundary[TEMP_LEVEL_COUNT][2] = { { 0, 40 }, {50, 80}, {300,740}, {750,900}, {950,1250} };

//Pressure Levels
#define P_REACTOR_LEVEL_COUNT 4
enum P_reactorLevels { OFF = 0, LITE = 1, MEDIUM = 2 , EXTREME = 3} P_reactorLevel;
static char *P_reactorLevelName[] = { "Off", "Low", "Medium", "High"};
int P_reactorLevelBoundary[4][2] = { { -100, 0 }, {-250, -125}, {-2000,-500}, {-4000,-2000} };

//Fuel Switch Levels
#if ANA_FUEL_SWITCH != ABSENT
int FuelSwitchValue = 0;
enum FuelSwitchLevels { SWITCH_OFF = 0, SWITCH_ON = 1} FuelSwitchLevel;
static char *FuelSwitchLevelName[] = { "Off","On"};
//int FuelSwitchLevelBoundary[2][2] = {{ 0, 200 }, {800, 1024}}; //not currently used
#endif

//Auger Current Levels
#if ANA_AUGER_CURRENT != ABSENT
int AugerCurrentValue = 0; // current level in mA
enum AugerCurrentLevels { AUGER_OFF = 0, AUGER_ON = 1, AUGER_HIGH = 2} AugerCurrentLevel;
static char *AugerCurrentLevelName[] = { "Off","On", "High"};
int AugerCurrentLevelBoundary[3][2] = { { 0, 1200}, {1200, 5000}, {5000,20000} };
#endif

#if ANA_OIL_PRESSURE != ABSENT
int EngineOilPressureValue;
enum EngineOilPressureLevels { OIL_P_LOW = 0, OIL_P_HIGH = 1} EngineOilPressureLevel;
int EngineOilPressureLevelBoundary[2][2] = { { 0, 500}, {600, 1024} };
#endif

// Loop variables - 0 is longest, 3 is most frequent, place code at different levels in loop() to execute more or less frequently
//TO DO: move loops to hardware timer and interrupt based control, figure out interrupt prioritization
int loopPeriod0 = 5000;
unsigned long nextTime0;
int loopPeriod1 = 1000;
unsigned long nextTime1;
int loopPeriod2 = 100;
unsigned long nextTime2;
int loopPeriod3 = 10;
unsigned long nextTime3;

//Control
int control_state = CONTROL_OFF;
unsigned long control_state_entered;

//Flare
int flare_state = FLARE_USER_SET;
boolean ignitor_on;
int blower_dial = 0;
double blower_setpoint;
double blower_input;
double blower_output;
double blower_value;
double blower_P[1] = {2}; //Adjust P_Param to get more aggressive or conservative control, change sign if moving in the wrong direction
double blower_I[1] = {.2}; //Make I_Param about the same as your manual response time (in Seconds)/4 
double blower_D[1] = {0.0}; //Unless you know what it's for, don't use D
PID blower_PID(&blower_input, &blower_output, &blower_setpoint,blower_P[0],blower_I[0],blower_D[0]);

//Engine
int engine_state = ENGINE_OFF;
unsigned long engine_state_entered;
unsigned long engine_end_cranking;
int engine_crank_period = 10000; //length of time to crank engine before stopping (milliseconds)
double battery_voltage;

//Display 
int display_state = DISPLAY_SPLASH;
unsigned long display_state_entered;
unsigned long transition_entered;
String transition_message;
int item_count,cur_item;

//Keypad
int key = -1;

//Pressure PID
byte servo_min,servo_max;
double throttle_valve_open = 123; //calibrated angle for servo valve open
double throttle_valve_closed = 48; //calibrated angle for servo valve closed (must be smaller value than open)
double idle_setpoint;
double pressure_setpoint;
double pressure_input;
double pressure_output;
double pressure_value;
//double pressure_setpoint_mode[1] = {1.05};
double pressure_P[1] = {13.0}; //Adjust P_Param to get more aggressive or conservative control, change sign if moving in the wrong direction
double pressure_I[1] = {100.0}; //Make I_Param about the same as your manual response time (in Seconds)/4 
double pressure_D[1] = {0.0}; //Unless you know what it's for, don't use D
PID pressure_PID(&pressure_input, &pressure_output, &pressure_setpoint,pressure_P[0],pressure_I[0],pressure_D[0]);
unsigned long pressure_updated_time;
boolean write_pressure = false;
String pressure_state_name;
int pressure_state;
unsigned long pressure_state_entered;

//Engine Parameters
double fuel_consumption;
double engine_speed;
double engine_torque;
double total_fuel_consumption;
double engine_boost;
int coolant_temperature;
double total_engine_hours;

// Fuel PID variables
double premix_valve_open = 133; 
double premix_valve_closed = 68;
double premix_valve_max = 1.0;  //minimum of range for closed loop operation (percent open)
double premix_valve_min = 0.00; //maximum of range for closed loop operation (percent open)
double premix_valve_center = 0.00; //initial value when entering closed loop operation (percent open)
double mixture_setpoint;
double mixture_input;
double mixture_output;
double mixture_value;
double mixture_setpoint_mode[1] = {1.05};
double mixture_P[1] = {0.13}; //Adjust P_Param to get more aggressive or conservative control, change sign if moving in the wrong direction
double mixture_I[1] = {1.0}; //Make I_Param about the same as your manual response time (in Seconds)/4 
double mixture_D[1] = {0.0}; //Unless you know what it's for, don't use D
PID mixture(&mixture_input, &mixture_output, &mixture_setpoint,mixture_P[0],mixture_I[0],mixture_D[0]);
unsigned long lamba_updated_time;
boolean write_mixture = false;
String mixture_state_name;
int mixture_state;
unsigned long mixture_state_entered;

////Governor
//double governor_setpoint;
//double governor_input;
//double governor_output;
//double governor_value;
//double governor_P[1] = {2}; //Adjust P_Param to get more aggressive or conservative control, change sign if moving in the wrong direction
//double governor_I[1] = {.2}; //Make I_Param about the same as your manual response time (in Seconds)/4 
//double governor_D[1] = {0.0}; //Unless you know what it's for, don't use D
//PID governor_PID(&governor_input, &governor_output, &governor_setpoint,governor_P[0],governor_I[0],governor_D[0]);

// Pressure variables
int Press_Calib[6];
int Press[6]; //values corrected for sensor offset (calibration)

//Serial
char serial_last_input = '\0'; // \0 is the ABSENT character

//Air Butterfly Servo Position
int air_butterfly_position;


//Drive Reset
unsigned long last_drive_reset = 0;

//Ash  Out
unsigned long ashout_state_entered = 0;
int ashout_state = 0;
#define ASHOUT_RUNNING 2
#define ASHOUT_STOPPED 3

//Ash  Grate
unsigned long ashgrate_state_entered = 0;
int ashgrate_state = 0;
#define ASHGRATE_RUNNING 0
#define ASHGRATE_STOPPED 1
#define ASHGRATE_REVERSING 2

//Auger and Fuel demand
boolean fuel_demand = false;
boolean upper_boot_detect = false;
boolean lower_boot_detect = false;
int fuel_demand_on_length = 0;
int fuel_demand_off_length = 0;
unsigned int fuel_demand_on_alarm_point = 300;
unsigned int fuel_demand_off_alarm_point = 900;
unsigned long auger_state_entered = 0;
int auger_state = 0;
#define AUGER_STOPPED 0
#define AUGER_RUNNING 1
#define AUGER_REVERSING 2

//Hopper agitator parameters
unsigned long hopper_agitator_period = 5000; //milliseconds
float hopper_agitator_duty = 0.4; // 0 to 1
unsigned long hopper_agitator_state_entered = 0;
int hopper_agitator_state = 0;
#define HOPPER_AGITATOR_OFF 0
#define HOPPER_AGITATOR_ON_PULSE 1 //air on
#define HOPPER_AGITATOR_ON_PAUSE 2 //air off

//Rotary Valve
int rotaryvalve_state = 0;
#define ROTARYVALVE_OFF 0
#define ROTARYVALVE_ON 1
#define ROTARYVALVE_STARTING 2
#define ROTARYVALVE_STOPPING 3
#define ROTARYVALVE_OVERCURRENT 4
#define ROTARYVALVE_REVERSING 5
unsigned long rotaryvalve_state_entered = 0;
int rotary_valve_starting_pause = 2*1000;  //#define ???
int rotary_valve_stopping_pause = 2*1000;  //#define ???

//Conveyor
int conveyor_state = 0;
unsigned long conveyor_state_entered = 0;
#define CONVEYOR_OFF 0
#define CONVEYOR_ON 1
#define CONVEYOR_REVERSING 2

//Boot States
int boot_state = 0;
int boot_empty_length = 0;
int boot_unknown_length = 0;
#define EMPTY 0
#define MID 1
#define FULL 2
#define UNKNOWN 3

int dataPin = 50;  //To SRIN on Relay Board, Bottom Right Pin on Relay Board when XR IN at top.
int latchPin = 51; //To RCK on Relay Board, Second Pin from Bottom on Right hand side
int clockPin = 52; //To SRCLK on Relay Board, Second Pin from Bottom on Left hand side

int sec = 1000;

//Relay Pins
int ROTARY_PIN = 0;
int ROTARY_DIRECTION_PIN = 1; //depreciated for air solenoid control on 1
int HOPPER_AGITATOR_SOLENOID_PIN = 1;
int MAIN_AUGER_PIN = 2;
int MAIN_AUGER_DIRECTION_PIN = 3;
int CONVEYER_PIN = 4;
int CONVEYER_DIRECTION_PIN = 5; 

int ASH_GRATE_PIN = 6;
int ASH_GRATE_DIRECTION_PIN = 7;

byte shiftRegister = 0;  //Holder for all 8 relay states (8 bits, initialized to B00000000, all relays off)

// Alarm
int alarm;
int alarm_interval = 5; // in seconds
int pressureRatioAccumulator = 0;
#define ALARM_NONE 0 //no alarm
#define ALARM_FUEL_DEMAND_ON_LONG 1
#define ALARM_FUEL_DEMAND_OFF_LONG 2
#define ALARM_BAD_REACTOR 3
#define ALARM_BAD_FILTER 4
#define ALARM_LOW_FUEL_REACTOR 5
#define ALARM_LOW_TREST 6
#define ALARM_HIGH_RED1 7
#define ALARM_BAD_OIL_PRESSURE 8
#define ALARM_O2_NO_SIG 9
#define ALARM_BOOT_EMPTY_LONG 10
#define ALARM_BOOT_UNKNOWN_STATE 11
#define ALARM_CAN_ERROR 12
#define ALARM_OVERSPEED 13
char* display_alarm[] = {
  "No alarm            ",
  "Fuel demand on long ",
  "Fuel demand off long",
  "Bad Reactor P_ratio",
  "Bad Filter P_ratio ",
  "Reactor Fuel Low   ",
  "trest low for eng. ",
  "tred1 high for eng.",
  "Check Oil Pressure ",
  "No O2 Sensor Signal",
  "Auger boot empty   ",
  "Auger boot unknown ",
  "CANbus error       ",
  "Engine Overspeed   "
}; //20 char message for 4x20 display

//// SD Card
boolean sd_loaded;
Sd2Card sd_card;
SdVolume sd_volume;
SdFile sd_root;
SdFile sd_file;
char sd_data_file_name[] = "datalog1.txt";     //Create an array that contains the name of our datalog file, updated upon 
char sd_in_char=0;
int sd_index=0;  

//Datalogging Buffer
String data_buffer = "";
char float_buf[15] = "";

char* OBDPGNHex[] = {
  "B3 FE 00", //PGN_FUEL - 1 line response
  "04 F0 00", //PGN_SPEED - streams out
  "EE FE 00", //PGN_TEMP -  1 line response
  "E5 FE 00", //PGN_HOURS - 1 line response
  "F5 FE 00", //PGN_AMBIENT - 2 line response (lines the same)
  "F6 FE 00", //PGN_EXHAUST - 2 line response (lines the same)
  "E9 FE 00", //PGN_FUEL_ACCUM - 1 line response
  "F2 FE 00", //PGN_FUEL_ECON - stream, 0.1s
  "6C FE 00", //PGN_TACH - ?
  "04 F0 00", //PGN_EEC1 - ?
  };

//Data from Charles Vessely (Cummins), cummins-J1939.pdf, and http://www.ddcsn.com/cps/rde/xbcr/ddcsn/ch6_mbe_ec_v13.pdf

//PGN 65203 / 00FEB3 - Fuel Information (Liquid)
//Trip Fuel Rate trip - bytes 5 & 6 [.05 L/hour]
#define PGN_FUEL 0 

// PGN 61444 / 00F004 - Electronic Engine Controller #1
// Engine Speed - bytes 4 & 5 [0.125 rpm/bit]
// Percent Engine Torque - byte 3 [1 percent/bit?]
#define PGN_SPEED 1 

//PGN 65262 / 00FEEE - Engine Temperature
// Engine Coolant Temperature - byte 1 [1C/bit, -40 C offset]
#define PGN_TEMP 2

//PGN 65253 / 00FEE5 - Engine Hours, Revolutions
//Total Engine Hours - bytes 1-4 [0.05 hr/bit]
#define PGN_HOURS 3

//PGN 65269 / 00FEF5 - Ambient Conditions
// ?
#define PGN_AMBIENT 4

//PGN 65270 / 00FEF6 - Inlet/Exhaust Conditions
//Boost Pressure - byte 2 - [2 kPa/bit]
#define PGN_EXHAUST 5

//PGN 65257 / 00FEE9 - Fuel Consumption
//Trip Fuel Consumption - bytes 1-4 [.5L/bit]
//Total Fuel Consumption - bytes 5-8 [.5L/bit]
#define PGN_FUEL_ACCUM 6

//PGN 65266 / 00FEF2 - Fuel Economy - 0.1s
//Fuel Rate - bytes 1,2 [0.05 L/h]
#define PGN_FUEL_ECON 7

//Tachograph
//Tach Shaft Speed - bytes 5,6 - [0.125 rpm/bit]
//Overspeed - byte 2 ?
#define PGN_TACH 8

//Electronic Engine Controller #1 - Rate: 10 ms
//Engine Speed - bytes 4,5 - [0.125 rpm/bit]
#define PGN_EEC1 9

//CANbus 
//#define CANDIAGNOSTICS 1
char can_data_file_name[] = "can00001.txt";
boolean can_init = false; 
#define OBDDATAINDEXSIZE 100
char OBDData[OBDDATAINDEXSIZE] = ""; //Serial2's buffer is 128, although our incomming feed should be smaller.
int OBDDataIndex = 0;
int lastPGNrequest = 0;
boolean lastPGNPollSuccess = true;
int lastReceivedPGNMessage = 0;
byte lastParsedPGNMessageBytes[8] = {0,0,0,0,0,0,0,0};
unsigned long lastPGNPollTime = millis();
unsigned long lastReceivedOBDMsgTime = millis();

//OBD PGN Polling
#define PGNPollCount 6
int PGNPollIndex = 0;
int PGNSeries[PGNPollCount] = {PGN_FUEL_ECON,PGN_SPEED,PGN_FUEL_ACCUM, PGN_EXHAUST, PGN_HOURS,PGN_EEC1}; //
unsigned long PGNTimeout[PGNPollCount];
unsigned long PGNLastSampleTime[PGNPollCount] = {millis(), millis(), millis(), millis(), millis(), millis()};
unsigned long PGNSamplePeriod[PGNPollCount] = {100,1000,1000,1000,1000,1000};

int PGNPollAttempts = 0;

void setup() {
  GCU_Setup(V3,FULLFILL,P777722);
  //
  DDRJ |= 0x80;      
  PORTJ |= 0x80;
  
  //TODO: Check attached libraries, FET6 seemed to be set to non-OUTPUT mode
  //set all FET pins to output
  pinMode(FET0,OUTPUT);
  pinMode(FET1,OUTPUT);
  pinMode(FET2,OUTPUT);
  pinMode(FET3,OUTPUT);
  pinMode(FET4,OUTPUT);
  pinMode(FET5,OUTPUT);
  pinMode(FET6,OUTPUT);
  pinMode(FET7,OUTPUT);
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  //pinMode(FET_BLOWER,OUTPUT); //TODO: Move into library (set PE0 to output)
  //digitalWrite(FET_BLOWER,HIGH);
  //delay(50);	
  
  // timer initialization
  nextTime0 = millis() + loopPeriod0;
  nextTime1 = millis() + loopPeriod1;
  nextTime2 = millis() + loopPeriod2;
  nextTime3 = millis() + loopPeriod3;
  
  LoadPressureSensorCalibration();
  //LoadFuel(); - must save Fuel data first?
  Serial.begin(57600);
  InitSD();
  LoadServo();
  InitOBD();

  Disp_Init();
  Kpd_Init();
  UI_Init();
  ADC_Init();
  Temp_Init();
  Press_Init();
  Fet_Init();
  //Servo_Init();
  Timer_Init();

  Disp_Reset();
  Kpd_Reset();
  UI_Reset();
  ADC_Reset();
  Temp_Reset();
  Press_Reset();
  Fet_Reset();
  //Servo_Reset();
  Timer_Reset();
  
  InitMixture();
  InitServos();
  InitGrate();  
  InitDriveReset();
  
  TransitionEngine(ENGINE_ON); //default to engine on. if PCU resets, don't shut a running engine off. in the ENGINE_ON state, should detect and transition out of engine on.
  //TransitionMixture(FUEL_P_COMB);
  TransitionDisplay(DISPLAY_SPLASH);
  
  TransitionAshOut(ASHOUT_STOPPED);
  TransitionAuger(AUGER_OFF);
  TransitionRotaryValve(ROTARYVALVE_OFF);
  TransitionConveyor(CONVEYOR_OFF);
  
  //InitCAN();
}

void loop() {
  if (millis() >= nextTime3) {
    nextTime3 += loopPeriod3;
    if (testing_state == TESTING_OFF) {
      // first, read all KS's sensors
      Temp_ReadAll();  // reads into array Temp_Data[], in deg C
      Press_ReadAll(); // reads into array Press_Data[], in hPa
      Timer_ReadAll(); // reads pulse timer into Timer_Data, in RPM ??? XXX
      DoPressure();
      DoSerialIn();
      //ReadRS232();
      DoMixture();
      //DoGovernor();
      DoAuger();
      DoRotaryValve();
      DoConveyor();
      DoEngine();
      DoGrate();
      DoAshOut();
      DoAshGrate();
      DoHopperAgitator();
      //DoServos();
      DoFlare();
      DoReactor();
      DoControlInputs();
      DoDriveReset(); //reset drive commands to correctly drive AC motors
      //TODO: Add OpenEnergyMonitor Library
    }
    DoKeyInput();
    DoHeartBeat(); // blink heartbeat LED
    ReadOBD();
    if (millis() >= nextTime2) {
      nextTime2 += loopPeriod2;
      DoDisplay();
      if (millis() >= nextTime1) {
        nextTime1 += loopPeriod1;
        SendOBDPGNRequestSeries();
        if (testing_state == TESTING_OFF) {
          DoFilter();
          DoDatalogging();
          DoAlarmUpdate();
          //getMixture();
        }
        if (millis() >= nextTime0) {
          if (testing_state == TESTING_OFF) {
            nextTime0 += loopPeriod0;
            DoAlarm();
          }
        }
      }
    }
  }
}

