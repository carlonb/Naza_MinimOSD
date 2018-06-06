
#include <avr/pgmspace.h>
#include <EEPROM.h> //Needed to access eeprom read/write functions
#include "symbols.h"
#include "Config.h"
#include "GlobalVariables.h"


boolean gps_process(void);

// Screen is the Screen buffer between program an MAX7456 that will be writen to the screen at 10hz
char screen[480];
// ScreenBuffer is an intermediary buffer to created Strings to send to Screen buffer
char screenBuffer[20];

uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;

//-------------- Timed Service Routine variables
unsigned long previous_millis_calcs =0;
unsigned long previous_millis_screen=0;
unsigned long previous_millis_MSP =0;
unsigned long previous_millis_1s =0;
unsigned long previous_millis_300ms =0;
unsigned int compass_cycle_time = 300;  //ms
unsigned int screen_cycle_time = 50;    //ms    Note: Do not go faster than 50ms (20Hz)
unsigned int MSP_cycle_time = 5;        //ms
unsigned int calcs_cycle_time = 50;     //ms    Note: Do not change this cycle time or will result in erratic calcs for statistics.
//----------------



void setup()
{
  Serial.begin(SPEED);
  
  delay(1000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.flush();
  
  //PWM RSSI
  pinMode(PWMrssiPin, INPUT);
 
  checkEEPROM();
  readEEPROM();
  MAX7456Setup();

  analogReference(DEFAULT);
  
  ScreenSelSetup();  // Init saved/selected screen
  
#ifdef BARO_BMP085
// NEB Baro setup
  i2c_init();
  delay(100);
  i2c_Baro_init();
#endif // Baro
}

void (* resetFunc)(void)=0; // NEB Chiamata a funzione e punta all'indirizzo =0 (Con 0 Punta al reset del processore, quindi resetta tutto e riparte)

void setMspRequests() {
     modeMSPRequests = REQ_MSP_FONT;

  // so we do not send requests that are not needed.
  queuedMSPRequests &= modeMSPRequests;
}

void loop()
{
  // Process AI   
  if (Settings[S_ENABLEADC]){
    //temperature=(analogRead(temperaturePin)-102)/2.048; // Does someone use this ATM??
    if (!Settings[S_MAINVOLTAGE_VBAT]){
      static uint16_t ind = 0;
      static uint32_t voltageRawArray[8];
      voltageRawArray[(ind++)%8] = analogRead(voltagePin);                  
      uint16_t voltageRaw = 0;
      for (uint16_t i=0;i<8;i++)
        voltageRaw += voltageRawArray[i];
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] /1023;
    }
    if (!Settings[S_VIDVOLTAGE_VBAT]) {     
      static uint16_t ind = 0;
      static uint32_t voltageRawArray[8];
      voltageRawArray[(ind++)%8] = analogRead(vidvoltagePin);                  
      uint16_t voltageRaw = 0;
      for (uint16_t i=0;i<8;i++)
        voltageRaw += voltageRawArray[i];
      vidvoltage = float(voltageRaw) * Settings[S_VIDDIVIDERRATIO] /1023;  
    }
    if (!Settings[S_MWRSSI] && !Settings[S_PWMRSSI]) {
      rssiADC = analogRead(rssiPin)/4;  // RSSI Readings, rssiADC=0 to 1023/4 (avoid a number > 255)
    }
    if (!Settings[S_MWAMPERAGE]) {
      int currsensOffSet=(Settings[S_CURRSENSOFFSET_L] | (Settings[S_CURRSENSOFFSET_H] << 8));  // Read OffSetH/L
      amperageADC = analogRead(amperagePin);
      if (amperageADC > currsensOffSet) amperageADC=((amperageADC-currsensOffSet)*4.8828)/Settings[S_CURRSENSSENSITIVITY]; // [A] Positive Current flow (512...1023) or Unidir (0...1023)
      else amperageADC=((currsensOffSet-amperageADC)*4.8828)/Settings[S_CURRSENSSENSITIVITY]; // [A] Negative Current flow (0...512)
      }
}
   if (Settings[S_MWAMPERAGE]) {
     amperagesum = MWpMeterSum;
     amperage = MWAmperage /100;
    }
  if (Settings[S_MWRSSI]) {
      rssiADC = MwRssi/4;  // RSSI from MWii, rssiADC=0 to 1023/4 (avoid a number > 255)
    } 
   if (Settings[S_PWMRSSI] && !Settings[S_MWRSSI]){
   rssiADC = pulseIn(PWMrssiPin, HIGH,15000)/Settings[S_PWMRSSIDIVIDER]; // Reading W/time out (microseconds to wait for pulse to start: 15000=0.015sec)
    }
    
  // ****************************************************************************
  //                    Start Timed Service Routines
  // ****************************************************************************


  // ********************  Start of Timed Service Routine 5ms (MSP requests)  ************************
  unsigned long currentMillis = millis();
  
  if((currentMillis - previous_millis_MSP) >= MSP_cycle_time)  // 200Hz
  {
    previous_millis_MSP = currentMillis;
/*
       uint8_t MSPcmdsend=0;
      if(queuedMSPRequests == 0)
        queuedMSPRequests = modeMSPRequests;
      uint32_t req = queuedMSPRequests & -queuedMSPRequests;
      queuedMSPRequests &= ~req;
      switch(req) {
      case REQ_MSP_FONT:
        MSPcmdsend = MSP_OSD;
      break;      
    }
*/    
  }
  // ********************  End of Timed Service Routine 5ms (MSP requests)  ************************



  // ********************  Start of Timed Service Routine 50ms (Screen refresh)  *******************
  currentMillis = millis();
  if((currentMillis - previous_millis_screen) >= screen_cycle_time)  // 20Hz
  {
   previous_millis_screen = currentMillis;

   Naza_setHomeAltitudeDirHome();
  
  if( allSec < 5 ){     // NEB 5-2 di boot
  displayIntro();
  lastCallSign = onTime;
  }   
//  else if (enable_statistics==true && timer_statistics <= STATISTICS_TIME) displayStatistics(); // Statistics per x secondi in GlobalVariables
  
  else {
    displayWaitHome();
    
//    display_Test_Data();  // NEB solo per test, da commentare dopo l'uso.
         
    displayVoltage();           // voltage
    
#ifdef BARO_BMP085
    displayTemperature();       // Temperatura da Baro (a fianco di V batt)
#endif // Baro

//    displayVidVoltage();        // vidvoltage
//    displayRSSI();              // rssi
    displayTime();              // flyTime, onTime, Distanza percorsa

    displayHorizon(MwAngle[0],MwAngle[1]);    // Solo per centro HUD
    displayHeadingGraph();
    displayHeading();
    displayAltitude();
    displayClimbRate();

    if(Settings[S_DISPLAYGPS]) {
      displayNumberOfSat();
      displayDirectionToHome();
      displayDistanceToHome();
//      displayAngleToHome();
      displayGPS_speed();
      displayGPSPosition();
    }              

//    displayAmperage();          // amperage
//    displaypMeterSum();         // amperagesum

    if ((onTime > (lastCallSign+300)) || (onTime < (lastCallSign+4))){
       // Displays 4 sec every 5min (no blink during flight)
      if ( onTime > (lastCallSign+300)) lastCallSign = onTime; 
        displayCallsign();
      }

    if(Blink10hz && screen_number_show_timer) displayscreenpage();
    }   

    MAX7456_DrawScreen();
  }
  // ********************  End of Timed Service Routine 50ms (Screen refresh)  ************************
  


  // ********************  Start of Timed Service Routine 50ms (Calculations)  ***********************
  currentMillis = millis();

  if((currentMillis - previous_millis_calcs) >= calcs_cycle_time)  // 20 Hz
  {
    previous_millis_calcs = currentMillis;   

    TempBlinkAlarm++;
    Blink10hz=!Blink10hz;
    
    calculateTrip50ms();      // Speed integration on 50msec
    
#ifdef BARO_BMP085
    i2c_Baro_update(); // NEB
#endif // Baro

    if (!Settings[S_MWAMPERAGE]) calculateAmperage();  // Amperage and amperagesum integration on 50msec
    if(Settings[L_RSSIPOSITIONDSPL]) calculateRssi();
  }
  // ********************  End of Timed Service Routine 50ms (Calculations)  ************************





 // ********************  Start of Timed Service Routine 300ms (Compass data and compass mode select)  ************************
  currentMillis = millis();
  
  if((currentMillis - previous_millis_300ms) >= compass_cycle_time)  // 3.3Hz
  {
    previous_millis_300ms = currentMillis;


// Refresh ogni 300ms per evitare sfarfallamento eccessivo indicazioni bussola e Dir-to-home
// Selezione tipo utilizzo dati BUSSOLA e DIR-to-HOME // NEB vers. 08
  // Gradi bussola calcolati con velE e velN dal GPS usati se la velocità >=3.6Km/h (100cm/sec)
  if(GPS_speed >= USE_CALC_COMPASS_MIN_SPEED) MwHeading = heading_from_gps_calcs;
  // Gradi bussola ricavati da pacchetto dati NAZA usati se la velocità <3.6Km/h (100cm/sec)
  else MwHeading = heading_from_naza_packet; 
  }
// ********************  End of Timed Service Routine 300ms (Compass data and compass mode select)  ************************





  // ********************  Start of Timed Service Routine 1000ms (1 sec timers)  **********************
  currentMillis = millis();
  if(currentMillis - previous_millis_1s >= 1000)     // this execute 1 time a second
  {
    previous_millis_1s = currentMillis;

    onTime++;
    allSec++;
   
    if (screen_number_show_timer > 0) screen_number_show_timer--;

    
 // flyTime parte velocità >1.8Km/h (50cm/sec) e Home fatta
   if(osd_got_home && GPS_speed > START_SPEED && Fly_Time_Started==false) {
     flyTime=0;                     // reset flyTime
     Fly_Time_Started=true;
     }
 // flyTime viene incrementato se Fly_Time_Started =true    
   if(Fly_Time_Started==true){
     flyTime++;                     // flyTime per ogni sessione
     flyingTime++;                  // flyingTime totale sessioni (Visualizzato solo in Statistics)
     }
  }
  // ********************  End of Timed Service Routine 1000ms (1 sec timers)  ************************
    
  // ****************************************************************************
  //                    End of Timed Service Routines
  // ****************************************************************************

  
  if(TempBlinkAlarm >= Settings[S_BLINKINGHZ]) {    // selectable alarm blink freq
    TempBlinkAlarm = 0;
    BlinkAlarm =!BlinkAlarm;     // 10=1Hz, 9=1.1Hz, 8=1.25Hz, 7=1.4Hz, 6=1.6Hz, 5=2Hz, 4=2.5Hz, 3=3.3Hz, 2=5Hz, 1=10Hz
  }


  gps_process();  // Ricezione dati da protocollo GPS-NAZA e protocollo MSP


}  // End of main loop




void ScreenSelSetup(void)
{
  screen_counter=Settings[S_SCREEN_COUNT];  // Read saved screen
  screen_number= 1 << screen_counter;       // Select saved screen
  screen_number_show_timer=8;    // Flashing for n sec the screen number at startup (minus 4sec for boot)
}


void save_screen_n(void)
{
 // Save if screen number != to eeprom and not yet saved and timer expired
 if (EEPROM.read(S_SCREEN_COUNT) != screen_counter && screen_saved==false && screen_number_show_timer <=0){
     EEPROM.write(S_SCREEN_COUNT,screen_counter);
     screen_saved=true;  // saved
     Settings[S_SCREEN_COUNT] = EEPROM.read(S_SCREEN_COUNT);  // Refresh
    }
}



void calculateTrip50ms(void)
{
  if(Fly_Time_Started==true) {   // Se decollo....
    if(Settings[S_UNITSYSTEM]){trip50ms += GPS_speed *0.0016404 * 0.95;} //  50/(100*1000)*3.2808=0.0016404   cm/sec ---> ft/50msec  (*0.95->sottraggo circa 5% del percorso integrato,verificato con prove)    
    else{trip50ms += GPS_speed *0.0005 * 0.95;}        //  50/(100*1000)=0.0005  cm/sec ---> mt/50msec
  }
}



void calculateRssi(void)
{
  float aa=0;  
  aa = rssiADC;  // actual RSSI signal received  (already divided by 4)
  aa = ((aa-Settings[S_RSSIMIN]) *101)/(Settings[S_RSSIMAX]-Settings[S_RSSIMIN]) ;  // Percentage of signal strength
  rssi_Int += ( ( (signed int)((aa*rssiSample) - rssi_Int )) / rssiSample );  // Smoothing the readings with rssiSample
  rssi = rssi_Int / rssiSample ;
  if(rssi<0) rssi=0;
  if(rssi>100) rssi=100;
}

void calculateAmperage(void)
{
  float aa=0;
// calculation of amperage [A*10]
  aa = amperageADC*10;   // *10 We want one decimal digit
  amperage_Int += ( ( (signed int)((aa* 10) - amperage_Int )) / 10 );  // Smoothing the displayed value with average of 10 samples
  amperage = (amperage_Int / 10);
  if (amperage >=999) amperage=999;  // [A*10]
  
// Calculation of amperagesum = amperage integration on 50msec (without reading average)
// 720= *100/72000 --> where:
// *100=mA (instead of *1000 as the value is already multiplied by 10)
// 72000=n. of 50ms in 1 hour
   amperagesum += aa /720; // [mAh]    // NOTE: if want add 3%, change this "amperagesum += aa /690;"
}

void writeEEPROM(void)
{
// For Settings
  for(int en=0;en<EEPROM_SETTINGS;en++){
    if (EEPROM.read(en) != Settings[en]) EEPROM.write(en,Settings[en]);
  }
// For Position of items on screen       
  for(int en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++){
    if (EEPROM.read(en+EEPROM_SETTINGS+1) != Settings[en+EEPROM_SETTINGS+1]) EEPROM.write(en+EEPROM_SETTINGS+1,Settings[en+EEPROM_SETTINGS+1]);
  }  
}

void readEEPROM(void)
{
// For Settings
  for(int en=0;en<EEPROM_SETTINGS;en++)
    Settings[en] = EEPROM.read(en);
     
// NEB Aggiunto refresh Screen N. solo se è cambiato tramite GUI
  if (EEPROM.read(S_SCREEN_COUNT) != screen_counter){
        screen_counter=Settings[S_SCREEN_COUNT];  // Refresh screen_counter
        screen_number= 1 << screen_counter;       // Select saved screen
        screen_number_show_timer=3; // show (flashing) for n sec the screen number
        }
        
// For items position on screen      
  for(int en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++)
     Settings[en+EEPROM_SETTINGS+1] = EEPROM.read(en+EEPROM_SETTINGS+1);
}


// Default for first run to init
void checkEEPROM(void)
{
// For H/W Settings
  uint8_t EEPROM_Loaded = EEPROM.read(0);
//    Serial.print(" start ");    Serial.println(EEPROM_Loaded);
  if (EEPROM_Loaded != EEPROM_VERSION){ // NEB Se byte 0 della eeprom è uguale a eeprom_version in config, non scrivere default, se diverso scrivere default
//    Serial.print(" eeprom 0= ");    Serial.println(EEPROM_Loaded);
//    Serial.print(" EE_VERS= ");    Serial.println(EEPROM_VERSION);
    EEPROM.write(0,EEPROM_VERSION); // NEB Salvo byte 0 EEprom
    for(uint8_t en=1;en<EEPROM_SETTINGS;en++){
      if (EEPROM.read(en) != EEPROM_DEFAULT[en])  EEPROM.write(en,EEPROM_DEFAULT[en]);  // NEB Salvo solo bytes diversi da default
    }
      for(uint16_t en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++) {
        if (EEPROM.read(en+EEPROM_SETTINGS+1) != EEPROM_PAL_DEFAULT[en]) EEPROM.write(en+EEPROM_SETTINGS+1,EEPROM_PAL_DEFAULT[en]);  // NEB Salvo solo bytes diversi da default
    }  
  }
//    Serial.print(" read ");    Serial.println(EEPROM.read(0));
}


//uint8_t safeMode() {
//  return 1;  // XXX
//}

// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void initFontMode() {
  if(fontMode) 
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;

  fontMode = 1;
  setMspRequests();
}

int16_t getNextCharToRequest() {
  if(nextCharToRequest != lastCharToRequest) { // Not at last char
    if(retransmitQueue & 0x02) {                // Missed char at curr-6. Need retransmit!
      return nextCharToRequest-6;
    }

    if((retransmitQueue & 0x11) == 0x10) {      // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest-3;
    }

    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }

  uint8_t temp1 = retransmitQueue & ~0x01; 
  uint8_t temp2 = nextCharToRequest - 6; 

  if(temp1 == 0) {
    fontMode = 0;                            // Exit font mode
  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while(!(temp1 & 0x03)) {
    temp1 >>= 1;
    temp2++;
  }

  return temp2;
}


