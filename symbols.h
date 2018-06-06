/*
 *  symbols.h
 *  
 *
 *
 */


// Satellite Graphics
#define SYM_SAT 0XCE
#define SYM_SAT_1 0XCF

// Degrees Icon for HEADING/DIRECTION HOME
#define SYM_DEGREES 0X7C

// Direction arrows 
#define SYM_ARROW_NORTH 0X60
//#define SYM_ARROW_SOUTH_1 0X61
//#define SYM_HEADING_POINTER 0X11

// GPS Coordinates and Altitude
#define SYM_LAT 0xCA
#define SYM_LON 0XCB
//#define SYM_GPS 0XA3

// Altitude
#define SYM_ALT 0XCC

// GPS Mode
//#define SYM_READY 0XA3
#define SYM_HOLD 0X1E
#define SYM_G_HOME 0X10
#define SYM_LAND 0X7F

// Sensor´s Presence
#define SYM_ACC_OFF 0X02
#define SYM_MAG_OFF 0X12
#define SYM_BAR_OFF 0X13
#define SYM_GPS_OFF 0X03

// Sensor´s Active
#define SYM_ACC_ON 0XA0
#define SYM_MAG_ON 0XA1
#define SYM_BAR_ON 0XA2
#define SYM_GPS_ON 0XA3
//#define SYM_GPS_SEARCH 0XA3

// AH Center screen Graphics
#define SYM_AH_CENTER 0XC5
//#define SYM_AH_RIGHT 0XC6
//#define SYM_AH_LEFT 0XC7
#define SYM_AH_DECORATION_RIGHT 0XB9
#define SYM_AH_DECORATION_LEFT 0XBA
//#define SYM_AH_SIDE_L_TOP 0X02
//#define SYM_AH_SIDE_R_TOP 0X03
//#define SYM_AH_SIDE_BAR_L 0X11
//#define SYM_AH_SIDE_BAR_R 0X01
//#define SYM_AH_SIDE_L_BOT 0X12
//#define SYM_AH_SIDE_R_BOT 0X13


// AH Bars
#define SYM_AH_BAR_0 0x80  // Per vecchio AH compatibile cleanflight
#define SYM_AH_BAR_9 0x89
//#define SYM_AH_BAR18_0 0x80  // Per nuovo AH int77 (cleanflight non ha questa patch)
//#define SYM_AH_BAR18_18 0x92
//#define SYM_AH_BAR12_0 0x93
//#define SYM_AH_BAR12_12 0x9F

//AH Side Bars
#define SYM_AH_DEG 0X70
#define SYM_AH_UP_15DG_L 0X15
#define SYM_AH_UP_5_10DG_L 0X05
#define SYM_AH_UP_5_10DG_R 0X04
#define SYM_AH_UP_15DG_R 0X14
#define SYM_AH_DOWN_15DG_L 0XEF
#define SYM_AH_DOWN_5_10DG_L 0XDF
#define SYM_AH_DOWN_5_10DG_R 0XDE
#define SYM_AH_DOWN_15DG_R 0XEE

// Batt Icon´s
#define SYM_MAIN_BATT 0X97
#define SYM_VID_BAT 0X98

// Voltage and amperage 
#define SYM_VOLTS 0XC9
#define SYM_AMPS 0XC8
//#define SYM_CURRENT 0XA4
#define SYM_CURRENT 0XBA

// Flying Mode
#define SYM_ACRO 0XAE
#define SYM_ACRO_1 0XAF
#define SYM_ANGLE 0XAC
#define SYM_ANGLE_1 0XAD
#define SYM_HORIZON 0XA9
#define SYM_HORIZON_1 0XAA
#define SYM_HORIZON_2 0xAB

// Time
#define SYM_ON 0X9B
#define SYM_FLY 0X9C

// Throttle 
#define SYM_THR_POINTER_DIV_HIGH 0X0A
#define SYM_THR_POINTER_DIV_LOW 0X0B  
#define SYM_THR_POINTER_TOP 0X09  
#define SYM_THR_POINTER_4 0X08  
#define SYM_THR_POINTER_3 0X1B
#define SYM_THR_POINTER_2 0X1A
#define SYM_THR_POINTER_1 0X19
#define SYM_THR_POINTER_BOT 0X18
#define SYM_THR_SCALE_LOW 0X07
#define SYM_THR_SCALE_MED 0X17
#define SYM_THR_SCALE_MAX 0X16

// Stall Warning
#define SYM_THR_STALL 0X7D
#define SYM_THR_STALL1 0X7E

//Warn sign
#define SYM_WARN 0XA5
#define SYM_WARN_1 0XA6
#define SYM_WARN_2 0XA7

//Failsafe sign
#define SYM_FSAFE 0X8A
#define SYM_FSAFE_1 0X8B
#define SYM_FSAFE_2 0X8C

// RSSI
#define SYM_RSSI 0XBB

// Menu cursor
#define SYM_CURSOR 0XCD

//Autopilot
#define SYM_AUTOPILOT 0X1C
#define SYM_AUTOPILOT_1 0X1D
#define SYM_WAYPOINT 0X77
#define SYM_WAYPOINT_1 0X78
#define SYM_MISSION 0X22
#define SYM_MISSION_1 0X23
#define SYM_MISSION_2 0X24

//Armed/Disarmed
#define SYM_MOTOR_DIS 0X0C
#define SYM_MOTOR_AR 0X0D
#define SYM_MOTOR_ME 0X0E
#define SYM_MOTOR_D 0X0F

//LOS
#define SYM_LOS 0X1F

//SPEED
#define SYM_SPEED 0X79
#define SYM_SPEED_1 0X7A

//NAV
#define SYM_NAV 0X8E
#define SYM_NAV_1 0X8F

// Vertical Speed
#define SYM_CLIMB_CENTER 0XBB
#define SYM_CLIMB_D_1 0XBA
#define SYM_CLIMB_D_2 0XB9
#define SYM_CLIMB_D_3 0XB8
#define SYM_CLIMB_D_4_T 0XB7
#define SYM_CLIMB_D_5_T 0XB6
#define SYM_CLIMB_D_6_DOT_DOWN 0XB5

#define SYM_CLIMB_U_1 0XC0
#define SYM_CLIMB_U_2 0XB0
#define SYM_CLIMB_U_3_T 0XB1
#define SYM_CLIMB_U_4_T 0XB2
#define SYM_CLIMB_U_5_T 0XB3
#define SYM_CLIMB_U_6_DOT_UP 0XB4

#define SYM_CLIMB_SCALE_UP 0XC3
#define SYM_CLIMB_SCALE_CENTER 0XC2
#define SYM_CLIMB_SCALE_DOWN 0XC1

#define SYM_CLIMB_FULL 0XB0
#define SYM_CLIMB_EMPTY 0XB1
#define SYM_CLIMB_0P_EDGE 0XC0
#define SYM_CLIMB_0N_EDGE 0XC1
#define SYM_CLIMB_0P 0XB2
#define SYM_CLIMB_3P 0XB3
#define SYM_CLIMB_6P 0XB4
#define SYM_CLIMB_9P 0XB5
#define SYM_CLIMB_12P 0XB6
#define SYM_CLIMB_15P 0XB7
#define SYM_CLIMB_18P 0XB8
#define SYM_CLIMB_0N 0XB9
#define SYM_CLIMB_3N 0XBA
#define SYM_CLIMB_6N 0XBB
#define SYM_CLIMB_9N 0XBC
#define SYM_CLIMB_12N 0XBD
#define SYM_CLIMB_15N 0XBE
#define SYM_CLIMB_18N 0XBF










