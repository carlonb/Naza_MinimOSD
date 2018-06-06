 
 
 /*--------------------------------------------------       Configurable parameters      ----------------------------------------------------*/ 
 
/**********************          Serial port Speed       *************************/

#define SPEED 115200 // 115200,57600,38400,28800,19200,14400,9600

/**********    Here you can define time out for Mag calibration and EEProm write (mostly useful for mag calibration)    ***********/
//#define CALIBRATION_DELAY 6
#define EEPROM_WRITE_DELAY 3
#define EEPROM_VERSION 12     // Max valore=255 - Cambiare valore per aggiornamento defaults


/*----------------------------------------------     GPS NAZA 16/11/2017    ------------------------------------*/
#define START_ALTITUDE 3000                 // Altitudine oltre la quale parte il timer di volo (cm)
#define START_SPEED   50                    // Velocità oltre la quale parte il timer di volo (cm/sec)
#define USE_CALC_COMPASS_MIN_SPEED  100     // Velocità oltre la quale vengono calcolati i gradi bussola da dati vel GPS (cm/sec)
//#define STATISTICS_TIME 30                // Tempo visualizzazione statistiche di volo     NON USATO
#define BARO_BMP085                         // Commentare se Baro non installato
/*----------------------------------------------     GPS NAZA 16/11/2017    ------------------------------------*/



#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000


#define LINE      30
#define LINE01    0
#define LINE02    30
#define LINE03    60
#define LINE04    90
#define LINE05    120
#define LINE06    150
#define LINE07    180
#define LINE08    210
#define LINE09    240
#define LINE10    270
#define LINE11    300
#define LINE12    330
#define LINE13    360
#define LINE14    390
#define LINE15    420
#define LINE16    450

// DEFINE CONFIGURATION MENU PAGES
#define MINPAGE 1
#define MAXPAGE 9

#define PIDITEMS 10

// RX CHANEL IN MwRcData table
#define ROLLSTICK        0
#define PITCHSTICK       1
#define YAWSTICK         2
#define THROTTLESTICK    3

// STICK POSITION
#define MAXSTICK         1900
#define MINSTICK         1100
#define MINTROTTLE       1000

// FOR POSITION OF PID CONFIG VALUE
#define ROLLT 93
#define ROLLP 101
#define ROLLI 107
#define ROLLD 113
#define PITCHT 93+(30*1)
#define PITCHP 101+(30*1)
#define PITCHI 107+(30*1)
#define PITCHD 113+(30*1)
#define YAWT 93+(30*2)
#define YAWP 101+(30*2)
#define YAWI 107+(30*2)
#define YAWD 113+(30*2)
#define ALTT 93+(30*3)
#define ALTP 101+(30*3)
#define ALTI 107+(30*3)
#define ALTD 113+(30*3)
#define VELT 93+(30*4)
#define VELP 101+(30*4)
#define VELI 107+(30*4)
#define VELD 113+(30*4)
#define LEVT 93+(30*5)
#define LEVP 101+(30*5)
#define LEVI 107+(30*5)
#define LEVD 113+(30*5)
#define MAGT 93+(30*6)
#define MAGP 101+(30*6)
#define MAGI 107+(30*6)
#define MAGD 113+(30*6)

#define SAVEP 93+(30*9)



