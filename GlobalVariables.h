


// *****************  Analog input defines  **************** //

// **** WiteSpy hardware **** //

#define voltagePin A2          // Batt 1
#define vidvoltagePin A0       // Batt 2
#define amperagePin A1         // Amperage
#define rssiPin A3             // TXRSSI
#define PWMrssiPin A3          // PWM RSSI uses same pin of analog RSSI A3

// ********************************************************** //
/*
int16_t test1=0; // NEB per prove, eliminare
int16_t test2=0; // NEB per prove, eliminare
int16_t test3=0; // NEB per prove, eliminare
int16_t test4=0; // NEB per prove, eliminare
int16_t test5=0; // NEB per prove, eliminare
int16_t test6=0; // NEB per prove, eliminare
int16_t test7=0; // NEB per prove, eliminare
int16_t test8=0; // NEB per prove, eliminare
*/

#define METRIC 0
#define IMPERIAL 1

const uint8_t rssiSample=30;

//General use variables
//int tenthSec=0;
int TempBlinkAlarm=0;                       // Temporary for blink alarm 
int BlinkAlarm=0;                           // This is turning on and off at selected freq. (alarm)
int Blink10hz=0;                            // This is turning on and off at 10hz
uint16_t lastCallSign=0;                    // callsign_timer
uint8_t rssiTimer=0;

unsigned int allSec=0;

// Per MSP message
uint8_t nextCharToRequest;
uint8_t lastCharToRequest;
uint8_t retransmitQueue;
uint8_t fontData[54];
uint8_t fontMode = 0;
bool  mspmessage = false;

// Mode bits
/*
uint32_t mode_armed;
uint32_t mode_angle;
uint32_t mode_horizon;
uint32_t mode_nav_althold;
uint32_t mode_mag;
uint32_t mode_nav_rth;
uint32_t mode_nav_poshold;
uint32_t mode_osd_switch;
uint32_t mode_headfree;
uint32_t mode_failsafe;
//uint32_t mode_passthru;
//uint32_t mode_llights;
uint32_t mode_nav_wp;
uint32_t mode_gpsland;  //Solo per MWii, non usato da iNAV
*/

  
// Settings Locations
enum Setting_ {
  S_CHECK_,		// used for check
  S_RSSIMIN,
  S_RSSIMAX,
  S_RSSI_ALARM,
  S_MWRSSI,
  S_PWMRSSI,
  S_PWMRSSIDIVIDER,
  S_VOLTAGEMIN,
  S_DIVIDERRATIO,
  S_MAINVOLTAGE_VBAT,
  S_VIDDIVIDERRATIO,
  S_VIDVOLTAGE_VBAT,
  S_BOARDTYPE,
  S_DISPLAYGPS,
  S_COORDINATES,
  S_HEADING360,
  S_UNITSYSTEM,
  S_VIDEOSIGNALTYPE,
  S_RESETSTATISTICS,
  S_ENABLEADC,
  S_BLINKINGHZ,    // selectable alarm blink freq
  S_MWAMPERAGE,
  S_CURRSENSSENSITIVITY,
  S_CURRSENSOFFSET_H,
  S_CURRSENSOFFSET_L,
  S_CLIMB_RATE_ALARM,
  S_VOLUME_DIST_MAX,
  S_VOLUME_ALT_MAX,
  S_VOLUME_ALT_MIN,
  S_VIDVOLTAGEMIN,
  S_PITCH_WARNING,
  S_SCREEN_COUNT,
//  S_Spare0,      // NEB aggiunto riserve
//  S_Spare1,      // NEB aggiunto riserve
//  S_Spare2,
//  S_Spare3,
//  S_Spare4,
//  S_Spare5,
//  S_Spare6,
  S_CS0,      // 10 callsign char locations
  S_CS1,
  S_CS2,
  S_CS3,
  S_CS4,
  S_CS5,
  S_CS6,
  S_CS7,
  S_CS8,
  S_CS9,      // 48
  // EEPROM_SETTINGS must be last for H/W settings!
  EEPROM_SETTINGS,  // 49
  
// Screen item Locations
// ********* EEProm enum data position and display On/Off option for all items on screen ****************
// Enum valid for both PAL/NTSC  
  L_GPS_NUMSATPOSITIONROW,    // 50
  L_GPS_NUMSATPOSITIONCOL,
  L_GPS_NUMSATPOSITIONDSPL,
  L_GPS_DIRECTIONTOHOMEPOSROW,
  L_GPS_DIRECTIONTOHOMEPOSCOL,
  L_GPS_DIRECTIONTOHOMEPOSDSPL,
  L_GPS_DISTANCETOHOMEPOSROW,
  L_GPS_DISTANCETOHOMEPOSCOL,
  L_GPS_DISTANCETOHOMEPOSDSPL,
  L_SPEEDPOSITIONROW,
  L_SPEEDPOSITIONCOL,
  L_SPEEDPOSITIONDSPL,
  L_GPS_ANGLETOHOMEPOSROW,
  L_GPS_ANGLETOHOMEPOSCOL,
  L_GPS_ANGLETOHOMEPOSDSPL,
  L_SENSORPOSITIONROW,
  L_SENSORPOSITIONCOL,
  L_SENSORPOSITIONDSPL,
  L_MODEPOSITIONROW,
  L_MODEPOSITIONCOL, 
  L_MODEPOSITIONDSPL,
  L_MW_HEADINGPOSITIONROW,
  L_MW_HEADINGPOSITIONCOL,
  L_MW_HEADINGPOSITIONDSPL,
  L_MW_HEADINGGRAPHPOSROW,
  L_MW_HEADINGGRAPHPOSCOL,
  L_MW_HEADINGGRAPHPOSDSPL,
  L_MW_ALTITUDEPOSITIONROW,
  L_MW_ALTITUDEPOSITIONCOL,
  L_MW_ALTITUDEPOSITIONDSPL,
  L_CLIMBRATEPOSITIONROW,
  L_CLIMBRATEPOSITIONCOL,
  L_CLIMBRATEPOSITIONDSPL,
  L_HORIZONPOSITIONROW,
  L_HORIZONPOSITIONCOL,
  L_HORIZONPOSITIONDSPL,
  L_CURRENTTHROTTLEPOSITIONROW,
  L_CURRENTTHROTTLEPOSITIONCOL,
  L_CURRENTTHROTTLEPOSITIONDSPL,
  L_FLYTIMEPOSITIONROW,
  L_FLYTIMEPOSITIONCOL,
  L_FLYTIMEPOSITIONDSPL,
  L_ONTIMEPOSITIONROW,
  L_ONTIMEPOSITIONCOL,
  L_ONTIMEPOSITIONDSPL,
  L_MW_GPS_LATPOSITIONROW,
  L_MW_GPS_LATPOSITIONCOL,
  L_MW_GPS_LATPOSITIONDSPL,
  L_MW_GPS_LONPOSITIONROW,
  L_MW_GPS_LONPOSITIONCOL,
  L_MW_GPS_LONPOSITIONDSPL,
  L_RSSIPOSITIONROW,
  L_RSSIPOSITIONCOL,
  L_RSSIPOSITIONDSPL,
  L_VOLTAGEPOSITIONROW,
  L_VOLTAGEPOSITIONCOL,
  L_VOLTAGEPOSITIONDSPL,   
  L_VIDVOLTAGEPOSITIONROW,
  L_VIDVOLTAGEPOSITIONCOL,
  L_VIDVOLTAGEPOSITIONDSPL,
  L_AMPERAGEPOSITIONROW,
  L_AMPERAGEPOSITIONCOL,
  L_AMPERAGEPOSITIONDSPL,
  L_PMETERSUMPOSITIONROW,
  L_PMETERSUMPOSITIONCOL,
  L_PMETERSUMPOSITIONDSPL,
  L_CALLSIGNPOSITIONROW,
  L_CALLSIGNPOSITIONCOL,
  L_CALLSIGNPOSITIONDSPL,  // 118
  
  // EEPROM_ITEM_LOCATION must be last for Items location!
  EEPROM_ITEM_LOCATION
};

uint8_t Settings[EEPROM_ITEM_LOCATION];

// For Settings Defaults
uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
EEPROM_VERSION,  // used for check  ?   // Valore definito in Config.h
0,   // S_RSSIMIN                   1
255, // S_RSSIMAX                   2
60,  // S_RSSI_ALARM                3
1,   // S_MWRSSI                    4
0,   // S_PWMRSSI                   5
8,   // S_PWMRSSIDIVIDER            6   // PWM Freq 500Hz=8, 1KHz=4 (Divider to avoid value >255)
105, // S_VOLTAGEMIN                7
74, // S_DIVIDERRATIO               8   // Per MinimOSD(VbattPin)=101(NAZA), per MicroOSD=102(NAZA), per MinimOSD+Baro=74(NAZA),  Attenzione: Calibrare con alimentazione da FC Velivolo.
0,   // S_MAINVOLTAGE_VBAT          9   // for NAZA S_MAINVOLTAGE_VBAT=0 Enabled
100, // S_VIDDIVIDERRATIO           10
1,   // S_VIDVOLTAGE_VBAT           11  // for NAZA S_VIDVOLTAGE_VBAT=1 Disabled
1,   // S_BOARDTYPE                 12
1,   // S_DISPLAYGPS                13
1,   // S_COORDINATES               14
1,   // S_HEADING360                15  // 360 default
0,   // S_UNITSYSTEM                16
1,   // S_VIDEOSIGNALTYPE           17
0,   // S_RESETSTATISTICS           18
1,   // S_ENABLEADC                 19  // for NAZA S_ENABLEADC=1 Enabled
5,   // S_BLINKINGHZ                20   // 10=1Hz, 9=1.1Hz, 8=1,25Hz, 7=1.4Hz, 6=1.6Hz, 5=2Hz, 4=2,5Hz, 3=3,3Hz, 2=5Hz, 1=10Hz
1,   // S_MWAMPERAGE                21
17,  // S_CURRSENSSENSITIVITY       22   // May vary from 17 to 40mV/A (Sensor type) 
2,   // S_CURRSENSOFFSET_H          23   // offset(H/L) =0 for unidir sensors or =512 for bidirectional sensors, may be changed only of few units.
0,   // S_CURRSENSOFFSET_L          24   // 2H+0L=512
2,   // S_CLIMB_RATE_ALARM          25
6,   // S_VOLUME_DIST_MAX           26   // Flying Volume Warning (Distance value in meters x100) by default is 600m
10,  // S_VOLUME_ALT_MAX            27   //   "     "       "   (Altitude Max "    "    "   x10 )  "     "   "   100m 
0,   // S_VOLUME_ALT_MIN            28   //   "     "       "   (Altitude Min "    "    "   ___    "     "   "    0m
105, // S_VIDVOLTAGEMIN             29
30,  // S_PITCH_WARNING             30   // Warning message at given angle in degrees positive and negative (default 30°)
2,   // S_SCREEN_COUNT              31   // Screen number (selected from 0 to 3), per NAZA default=2 -> page 3 per adesso.
//0,   // S_Spare0,                   32   // NEB aggiunto riserve
//0,   // S_Spare1,                   33   // NEB aggiunto riserve
//0,   // S_Spare2,                   34
//0,   // S_Spare3,                   35
//0,   // S_Spare4,                   36
//0,   // S_Spare5,                   37
//0,   // S_Spare6,                   38
'C',   // S_CS0,                    32    // 10 callsign char locations
'A',   // S_CS1,
'R',   // S_CS2,
'L',   // S_CS3,
'O',   // S_CS4,
'N',   // S_CS5,
'B',   // S_CS6,
'1',   // S_CS7,
'2',   // S_CS8,
'3',   // S_CS9,                    41
};

// PAL item position Defaults
uint8_t EEPROM_PAL_DEFAULT[EEPROM_ITEM_LOCATION-EEPROM_SETTINGS] = {
// ROW= Row position on screen (255= no action)
// COL= Column position on screen (255= no action)
// DSPL= Display item on 4 screens (Examples : 0000=Not display, 0001=only on screen 1, 0100=only on screen 3, 1101=screens 1,3,4)

3,   // L_GPS_NUMSATPOSITIONROW         50
2,  // L_GPS_NUMSATPOSITIONCOL
0b0111,   // L_GPS_NUMSATPOSITIONDSPL

4,   // L_GPS_DIRECTIONTOHOMEPOSROW 
15,   // L_GPS_DIRECTIONTOHOMEPOSCOL
0b0111,   // L_GPS_DIRECTIONTOHOMEPOSDSPL

7,   // L_GPS_DISTANCETOHOMEPOSROW 
23,   // L_GPS_DISTANCETOHOMEPOSCOL
0b0111,   // L_GPS_DISTANCETOHOMEPOSDSPL

9,  // L_SPEEDPOSITIONROW 
24,  // L_SPEEDPOSITIONCOL 
0b0111,   // L_SPEEDPOSITIONDSPL

4,   // L_GPS_ANGLETOHOMEPOSROW 
19,   // L_GPS_ANGLETOHOMEPOSCOL
0b0100,   // L_GPS_ANGLETOHOMEPOSDSPL

6,   // L_SENSORPOSITIONROW
2,  // L_SENSORPOSITIONCOL
0b0110,   // L_SENSORPOSITIONDSPL

4,   // L_MODEPOSITIONROW
2,   // L_MODEPOSITIONCOL
0b0110,   // L_MODEPOSITIONDSPL

3,   // L_MW_HEADINGPOSITIONROW 
20,   // L_MW_HEADINGPOSITIONCOL
0b0110,   // L_MW_HEADINGPOSITIONDSPL

3,   // L_MW_HEADINGGRAPHPOSROW
13,  // L_MW_HEADINGGRAPHPOSCOL
0b0110,   // L_MW_HEADINGGRAPHPOSDSPL

7,   // L_MW_ALTITUDEPOSITIONROW
4,  // L_MW_ALTITUDEPOSITIONCOL
0b0111,   // L_MW_ALTITUDEPOSITIONDSPL

8,   // L_CLIMBRATEPOSITIONROW
5,   // L_CLIMBRATEPOSITIONCOL
0b0111,   // L_CLIMBRATEPOSITIONDSPL

6,   // L_HORIZONPOSITIONROW
8,   // L_HORIZONPOSITIONCOL
0b0111,   // L_HORIZONPOSITIONDSPL

7,   // L_CURRENTTHROTTLEPOSITIONROW 
21,  // L_CURRENTTHROTTLEPOSITIONCOL
0b0110,   // L_CURRENTTHROTTLEPOSITIONDSPL

13,  // L_FLYTIMEPOSITIONROW
13,   // L_FLYTIMEPOSITIONCOL  // NEB default era 22 sovrapposto a onTime, ora sempre visibile per NAZA
0b1111,   // L_FLYTIMEPOSITIONDSPL

13,  // L_ONTIMEPOSITIONROW 
22,   // L_ONTIMEPOSITIONCOL  // NEB era sovrapposto a flytime, ora sempre visibile per NAZA
0b1111,   // L_ONTIMEPOSITIONDSPL

2,  // L_MW_GPS_LATPOSITIONROW
2,   // L_MW_GPS_LATPOSITIONCOL
0b1111,   // L_MW_GPS_LATPOSITIONDSPL

2,  // L_MW_GPS_LONPOSITIONROW
17,  // L_MW_GPS_LONPOSITIONCOL
0b1111,   // L_MW_GPS_LONPOSITIONDSPL

12,   // L_RSSIPOSITIONROW
4,    // L_RSSIPOSITIONCOL
0b0100,    // L_RSSIPOSITIONDSPL

13,  // L_VOLTAGEPOSITIONROW
3,  // L_VOLTAGEPOSITIONCOL
0b1111,   // L_VOLTAGEPOSITIONDSPL

12,  // L_VIDVOLTAGEPOSITIONROW 
23,  // L_VIDVOLTAGEPOSITIONCOL
0b0100,   // L_VIDVOLTAGEPOSITIONDSPL

12,  // L_AMPERAGEPOSITIONROW
10,  // L_AMPERAGEPOSITIONCOL
0b0100,   // L_AMPERAGEPOSITIONDSPL

12,  // L_PMETERSUMPOSITIONROW 
17,  // L_PMETERSUMPOSITIONCOL
0b0100,   // L_PMETERSUMPOSITIONDSPL

11,  // L_CALLSIGNPOSITIONROW 13
11,   // L_CALLSIGNPOSITIONCOL 10
0b0111,   // L_CALLSIGNPOSITIONDSPL  // 118
};

/*
// NTSC item position Defaults
uint8_t EEPROM_NTSC_DEFAULT[EEPROM_ITEM_LOCATION-EEPROM_SETTINGS] = {
// ROW= Row position on screen (255= no action)
// COL= Column position on screen (255= no action)
// DSPL= Display item on 4 screens (Examples : 0000=Not display, 0001=only on screen 1, 0100=only on screen 3, 1101=screens 1,3,4)

2,   // L_GPS_NUMSATPOSITIONROW         50
19,  // L_GPS_NUMSATPOSITIONCOL
0b0011,   // L_GPS_NUMSATPOSITIONDSPL

4,   // L_GPS_DIRECTIONTOHOMEPOSROW 
21,   // L_GPS_DIRECTIONTOHOMEPOSCOL
0b0111,   // L_GPS_DIRECTIONTOHOMEPOSDSPL

3,   // L_GPS_DISTANCETOHOMEPOSROW 
19,   // L_GPS_DISTANCETOHOMEPOSCOL
0b0011,   // L_GPS_DISTANCETOHOMEPOSDSPL

8,  // L_SPEEDPOSITIONROW 
21,  // L_SPEEDPOSITIONCOL 
0b0011,   // L_SPEEDPOSITIONDSPL

4,   // L_GPS_ANGLETOHOMEPOSROW 
13,   // L_GPS_ANGLETOHOMEPOSCOL
0b0010,   // L_GPS_ANGLETOHOMEPOSDSPL

6,   // L_SENSORPOSITIONROW
2,  // L_SENSORPOSITIONCOL
0b0011,   // L_SENSORPOSITIONDSPL

3,   // L_MODEPOSITIONROW
7,   // L_MODEPOSITIONCOL
0b0011,   // L_MODEPOSITIONDSPL

2,   // L_MW_HEADINGPOSITIONROW 
12,   // L_MW_HEADINGPOSITIONCOL
0b0010,   // L_MW_HEADINGPOSITIONDSPL

3,   // L_MW_HEADINGGRAPHPOSROW
12,  // L_MW_HEADINGGRAPHPOSCOL
0b0011,   // L_MW_HEADINGGRAPHPOSDSPL

7,   // L_MW_ALTITUDEPOSITIONROW
3,  // L_MW_ALTITUDEPOSITIONCOL
0b0011,   // L_MW_ALTITUDEPOSITIONDSPL

7,   // L_CLIMBRATEPOSITIONROW
4,   // L_CLIMBRATEPOSITIONCOL
0b0011,   // L_CLIMBRATEPOSITIONDSPL

6,   // L_HORIZONPOSITIONROW
8,   // L_HORIZONPOSITIONCOL
0b0011,   // L_HORIZONPOSITIONDSPL

6,   // L_CURRENTTHROTTLEPOSITIONROW 
20,  // L_CURRENTTHROTTLEPOSITIONCOL
0b0011,   // L_CURRENTTHROTTLEPOSITIONDSPL

12,  // L_FLYTIMEPOSITIONROW
5,   // L_FLYTIMEPOSITIONCOL
0b0011,   // L_FLYTIMEPOSITIONDSPL

12,  // L_ONTIMEPOSITIONROW 
5,   // L_ONTIMEPOSITIONCOL
0b0011,   // L_ONTIMEPOSITIONDSPL

1,  // L_MW_GPS_LATPOSITIONROW
2,   // L_MW_GPS_LATPOSITIONCOL
0b0010,   // L_MW_GPS_LATPOSITIONDSPL

1,  // L_MW_GPS_LONPOSITIONROW
17,  // L_MW_GPS_LONPOSITIONCOL
0b0010,   // L_MW_GPS_LONPOSITIONDSPL

13,   // L_RSSIPOSITIONROW
6,    // L_RSSIPOSITIONCOL
0b0011,    // L_RSSIPOSITIONDSPL

13,  // L_VOLTAGEPOSITIONROW
19,  // L_VOLTAGEPOSITIONCOL
0b0011,   // L_VOLTAGEPOSITIONDSPL

13,  // L_VIDVOLTAGEPOSITIONROW 
24,  // L_VIDVOLTAGEPOSITIONCOL
0b0010,   // L_VIDVOLTAGEPOSITIONDSPL

12,  // L_AMPERAGEPOSITIONROW
24,  // L_AMPERAGEPOSITIONCOL
0b0010,   // L_AMPERAGEPOSITIONDSPL

12,  // L_PMETERSUMPOSITIONROW 
19,  // L_PMETERSUMPOSITIONCOL
0b0011,   // L_PMETERSUMPOSITIONDSPL

2,  // L_CALLSIGNPOSITIONROW
1,  // L_CALLSIGNPOSITIONCOL
0b0010,   // L_CALLSIGNPOSITIONDSPL  // 118
};
*/

//static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

//static uint8_t rcRate8,rcExpo8;
//static uint8_t rollPitchRate;
//static uint8_t yawRate;
//static uint8_t dynThrPID;
//static uint8_t thrMid8;
//static uint8_t thrExpo8;

int32_t  MwAltitude=0;          // This hold barometric value from MWii/iNAV MSP

int MwAngle[2]={0,0};           // Those will hold Accelerator Angle

static uint16_t MwRcData[8]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500} ;

uint16_t MwSensorPresent=0;
uint32_t MwSensorActive=0;
uint8_t MwVersion=0;
uint8_t MwVBat=255;             // NEB Inizializzo al max per non avere voltagemin=zero in statistics
int16_t MwVario=0;
uint8_t armed=0;
uint8_t previousarmedstatus=0;  // for statistics after disarming
float GPS_distanceToHome=0;     // per NAZA
uint8_t GPS_fix=0;
float GPS_latitude=0;           // Per Calcoli
float GPS_longitude=0;
float GPS_latitude_Dsply=0;   // Per Display
float GPS_longitude_Dsply=0;
//int16_t GPS_altitude=0;
uint16_t GPS_speed=0;
uint16_t GPS_hdop;
float GPS_directionToHome=0;
uint8_t GPS_numSat=0;
uint8_t NAV_waypoint=0;         // NEB
uint8_t NAV_state;              // NEB
uint8_t GPS_Mode;               // NEB 11/12/2016 implementato INAV
int16_t I2CError=0;
uint16_t cycleTime=0;
uint16_t MWpMeterSum=0;
uint16_t MwRssi=0;
uint16_t MWAmperage=0;


// For Time
uint16_t onTime=0;
uint16_t flyTime=0;

// For Heading
const char headGraph[] PROGMEM = {
//   0XFB,0XFE,0XFA,0XFE,0XFC,0XFE,0XF9,0XFE,0XFB,0XFE,0XFA,0XFE,0XFC,0};  // NEB Originale per MWii ma ????
//   0XFB,0XFE,0XFA,0XFE,0XFC,0XFE,0XF9,0XFE,0XFB,0XFE,0XFA,0XFE,0XFC,0XFE,0XF9,0XFE,0XFB,0};  // NEB prima mod per NAZA per 8 punti 5 digit
//   0XFD,0XFE,0XF9,0XFE,0XFD,0XFE,0XFB,0XFE,0XFD,0XFE,0XFA,0XFE,0XFD,0XFE,0XFC,0XFE,0XFD,0XFE,0XF9,0XFE,0XFD,0};  // NEB mod per NAZA per 16 punti 5 digit (car. normali)
//   0XFC,0XFE,0XFD,0XFE,0XF9,0XFE,0XFD,0XFE,0XFB,0XFE,0XFD,0XFE,0XFA,0XFE,0XFD,0XFE,0XFC,0XFE,0XFD,0XFE,0XF9,0XFE,0XFD,0XFE,0XFB,0};  // NEB mod per NAZA e per 16 punti 9 digit (car. normali)
//   0XFA,0XFB,0XFD,0XF0,0XF1,0XF2,0XFD,0XF3,0XF4,0XF5,0XFD,0XF6,0XF7,0XF8,0XFD,0XF9,0XFA,0XFB,0XFD,0XF0,0XF1,0XF2,0XFD,0XF3,0XF4,0};  // NEB mod per NAZA e per 16 punti 5 digit (mezzo carattere)   
   0XFA,0XFC,0XFD,0XF0,0XF1,0XF2,0XFD,0XF3,0XF4,0XF5,0XFD,0XF6,0XF7,0XF8,0XFD,0XF9,0XFA,0XFC,0XFD,0XF0,0XF1,0XF2,0XFD,0XF3,0XF4,0};  // NEB mod per NAZA e per 16 punti 5 digit (mezzo carattere) per MinimOSD difettoso FC invece di FB  

static int16_t MwHeading=0;

// For Amperage
float amperageADC =0;
int amperage_Int=0;  
float amperage = 0;                      // its the real value x10
float amperagesum = 0;

// Rssi
int rssi =0;
int rssiADC=0;
int rssiMin;
int rssiMax;
int rssi_Int=0;

// For Voltage
uint16_t voltage=0;                      // its the value x10
uint16_t vidvoltage=0;                   // its the value x10



// For Statistics
uint16_t speedMAX=0;
int16_t altitudeMAX=0;
int16_t distanceMAX=0;
float trip50ms=0;
uint16_t flyingTime=0;
uint16_t voltagemin=65535;      // NEB Inizializzo al max per non avere zero in statistics

// For OSD-SWITCH change screen
uint8_t osd_switch_off=false;
int8_t screen_number;
int8_t screen_counter;
int8_t screen_number_show_timer=0;
int8_t screen_saved=false;
#define MAX_OSDSWITCH_COUNT 3   // Max screen N.

// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
/*
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_NAV_STATUS           121   //out message	     Returns navigation status  // NEB added

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1333
*/
// Duplicati per simulatore configuratore OSD
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
// ---------------------------------------------------------------------------------------

// Private MSP for use with the GUI
#define MSP_OSD                  220   //in message          starts epprom send to OSD GUI
// Subcommands
#define OSD_NULL                 0  // Non usato
#define OSD_READ_CMD             1
#define OSD_WRITE_CMD            2
#define OSD_GET_FONT             3
#define OSD_SERIAL_SPEED         4  // Non usato
#define OSD_RESET                5

// End private MSP for use with the GUI


// For Intro
const char HUD_vers[] PROGMEM = "HUD 05-08 NAZA";  // NEB NAZA

// For Config menu common
/*
const char configMsgON[] PROGMEM = "ON";
const char configMsgOFF[] PROGMEM = "OFF";
const char configMsgNoAct[] PROGMEM = "--";
const char configMsgEXIT[] PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE-EXIT";
const char configMsgPGS[] PROGMEM = "<PAGE>";
const char configMsgNTSC[] PROGMEM = "NTSC";
const char configMsgPAL[] PROGMEM = "PAL";
*/
const char configMsgSCREEN[] PROGMEM = "SCRN";
const char configMsgWaitHome[] PROGMEM = "WAIT HOME";

// For Config pages
/*
//-----------------------------------------------------------Page1
const char configMsg10[] PROGMEM = "1/9 PID CONFIG";
const char configMsg11[] PROGMEM = "ROLL";
const char configMsg12[] PROGMEM = "PITCH";
const char configMsg13[] PROGMEM = "YAW";
const char configMsg14[] PROGMEM = "ALT";
const char configMsg15[] PROGMEM = "GPS";
const char configMsg16[] PROGMEM = "LEVEL";
const char configMsg17[] PROGMEM = "MAG";
//-----------------------------------------------------------Page2
const char configMsg20[] PROGMEM = "2/9 RC TUNING";
const char configMsg21[] PROGMEM = "RC RATE";
const char configMsg22[] PROGMEM = "EXPONENTIAL";
const char configMsg23[] PROGMEM = "ROLL PITCH RATE";
const char configMsg24[] PROGMEM = "YAW RATE";
const char configMsg25[] PROGMEM = "THROTTLE PID ATT";
const char configMsg26[] PROGMEM = "MWCYCLE TIME";
const char configMsg27[] PROGMEM = "MWI2C ERRORS";
//-----------------------------------------------------------Page3
const char configMsg30[] PROGMEM = "3/9 SUPPLY & ALARM";
const char configMsg31[] PROGMEM = "VOLTAGE ALARM";
const char configMsg33[] PROGMEM = "BLINKING FREQ";
//-----------------------------------------------------------Page4
const char configMsg40[] PROGMEM = "4/9 RSSI";
const char configMsg41[] PROGMEM = "ACTUAL RSSI RAW";
const char configMsg42[] PROGMEM = "ACTUAL RSSI %";
const char configMsg43[] PROGMEM = "SET RSSI MIN";
const char configMsg44[] PROGMEM = "SET RSSI MAX";
//-----------------------------------------------------------Page5
const char configMsg50[] PROGMEM = "5/9 CALIBRATION";
const char configMsg51[] PROGMEM = "ACC CALIBRATION";
const char configMsg52[] PROGMEM = "ACC ROLL";
const char configMsg53[] PROGMEM = "ACC PITCH";
const char configMsg54[] PROGMEM = "ACC Z";
const char configMsg55[] PROGMEM = "MAG CALIBRATION";
const char configMsg56[] PROGMEM = "HEADING";
const char configMsg57[] PROGMEM = "MW EEPROM WRITE";
//-----------------------------------------------------------Page6
const char configMsg60[] PROGMEM = "6/9 GPS";
const char configMsg61[] PROGMEM = "DISPLAY GPS DATA";
const char configMsg62[] PROGMEM = "GPS COORDINATES";
const char configMsg63[] PROGMEM = "CALLSIGN";
//-----------------------------------------------------------Page7
const char configMsg70[] PROGMEM = "7/9 ADV SETUP";
const char configMsg71[] PROGMEM = "RESET STATISTICS";
const char configMsg72[] PROGMEM = "HEADING 0-360";
const char configMsg73[] PROGMEM = "UNIT SYSTEM";
const char configMsg74[] PROGMEM = "METRIC";
const char configMsg75[] PROGMEM = "IMPERL";
const char configMsg76[] PROGMEM = "VIDEO SYSTEM";
//const char configMsg77[] PROGMEM = "CLF/INAV";  // NEB Aggiunto per selezione CleanFlight/iNAV o MWii

//-----------------------------------------------------------Page8
const char configMsg80[] PROGMEM = "8/9 SCREEN CONFIG";
const char configMsg81[] PROGMEM = "ITEM      DSP LINE COL";
const char configMsg82[] PROGMEM = "DEFAULT-EXIT";
*/
//-----------------------------------------------------------Page9
const char configMsg90[] PROGMEM = "9/9 STATISTICS";
const char configMsg91[] PROGMEM = "TRIP";
const char configMsg92[] PROGMEM = "MAX DISTANCE";
const char configMsg93[] PROGMEM = "MAX ALTITUDE";
const char configMsg94[] PROGMEM = "MAX SPEED";
const char configMsg95[] PROGMEM = "FLYING TIME";
const char configMsg96[] PROGMEM = "AMPS DRAINED";
const char configMsg97[] PROGMEM = "MIN BATT";


// Variables for items pos change on screen
/*
//-----------------------------------------------------------
int8_t screenitemselect=0; // pointer for item text strings
int8_t screen_pos_item_pointer=EEPROM_SETTINGS+1;  // pointer for first item display/row/col positions
#define MAXSCREENITEMS 22

// Strings for item select on screen
//-----------------------------------------------------------
const char screen_item_00[] PROGMEM = "NUM SAT/HDOP";
const char screen_item_01[] PROGMEM = "DIR TO HOME";
const char screen_item_02[] PROGMEM = "DIST TO HOME";
const char screen_item_03[] PROGMEM = "GPS SPEED";
const char screen_item_04[] PROGMEM = "ANGLE TO HOM";
const char screen_item_05[] PROGMEM = "SENSORS";
const char screen_item_06[] PROGMEM = "FLIGHT MODE";
const char screen_item_07[] PROGMEM = "HEADING";
const char screen_item_08[] PROGMEM = "HEAD GRAPH";
const char screen_item_09[] PROGMEM = "BARO ALTIT";
const char screen_item_10[] PROGMEM = "CLIMB RATE";
const char screen_item_11[] PROGMEM = "HORIZON";
const char screen_item_12[] PROGMEM = "THROTTLE";
const char screen_item_13[] PROGMEM = "FLY TIME";
const char screen_item_14[] PROGMEM = "ON TIME";
const char screen_item_15[] PROGMEM = "GPS LATIT";
const char screen_item_16[] PROGMEM = "GPS LONGIT";
const char screen_item_17[] PROGMEM = "RSSI";
const char screen_item_18[] PROGMEM = "MAIN BATT";
const char screen_item_19[] PROGMEM = "VIDEO BATT";
const char screen_item_20[] PROGMEM = "AMPERAGE";
const char screen_item_21[] PROGMEM = "MA/H CONSUM";  
const char screen_item_22[] PROGMEM = "CALLSIGN";


//PROGMEM const char *item_table[] =    // NEB per uso con Arduino vers. 1.0.2
const char * const item_table[]PROGMEM =    // NEB per uso con Arduino vers. 1.6.8
{   
screen_item_00,
screen_item_01,
screen_item_02,
screen_item_03,
screen_item_04,
screen_item_05,
screen_item_06,
screen_item_07,
screen_item_08,
screen_item_09,
screen_item_10,
screen_item_11,
screen_item_12,
screen_item_13,
screen_item_14,
screen_item_15,
screen_item_16,
screen_item_17,
screen_item_18,
screen_item_19,
screen_item_20,
screen_item_21,
screen_item_22,
};
*/
// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const char MultiWiiLogoL1Add[17] PROGMEM = {
  0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0};
const char MultiWiiLogoL2Add[17] PROGMEM = {
  0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0};
const char KVTeamVersionPosition = 34+2*LINE;
/*
#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_RAW_GPS   (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  6)
#define REQ_MSP_ALTITUDE  (1 <<  7)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
*/
#define REQ_MSP_FONT      (1 << 12)
/*
#define REQ_MSP_NAV_STATUS 32768 // (1 << 15) // 32768
*/


// +++++++++++++
// NAZA var
// +++++++++++++
uint8_t      osd_alt_cnt = 0;                // counter for stable osd_alt
float        osd_alt_prev = 0;               // previous GPS altitude
float        GPS_alt = 0;                    // GPS altitude
float        osd_home_lat = 0;               // home latitude
float        osd_home_lon = 0;               // home longitude
float        osd_home_alt = 0;               // home altitude
bool         osd_got_home = false;           // tells if got home position or not
bool         Fly_Time_Started = false;       // Flag start fly timer, una volta avviato si arresta allo spegnimento dell'OSD
float        heading_from_naza_packet = 0;
float        heading_from_gps_calcs = 0;



