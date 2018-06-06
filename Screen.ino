
char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = '.';
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

//  uint8_t bytes = p+8;
  uint8_t bytes = p+6;
  val = val / 100;  //  5 decimals after dot
  
  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = '.';
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes < 3 && val == 0)
       break;
   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint16_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t FindNull(void)
{
  uint8_t xx;
  for(xx=0;screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}


void displayWaitHome(void)
{
  if (!BlinkAlarm || osd_got_home) return;  // Se lampeggio o home OK non mostrare, MANCA non visualizz. quando in STATISTICS!!!
  else
  MAX7456_WriteString_P(configMsgWaitHome, 191);  // Wait... in horizon rect
} 


void displayscreenpage(void)
{
  MAX7456_WriteString_P(configMsgSCREEN, 252);  // screen number in lower horizon rect
  MAX7456_WriteString(itoa(screen_counter+1,screenBuffer,10),257);
}

/*
void displaySensor(void)
{
  if((Settings[L_SENSORPOSITIONDSPL]) & (screen_number)) {
    
    // Sensor Present/Active
    screenBuffer[0] = (MwSensorPresent&GPSSENSOR) ? SYM_GPS_OFF : ' ';
    if((!GPS_fix)&&(BlinkAlarm)) screenBuffer[0] = (MwSensorPresent&GPSSENSOR)? SYM_GPS_ON : ' ';
    screenBuffer[1] =0;
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]);
    
    screenBuffer[0] = (MwSensorPresent&BAROMETER) ? SYM_BAR_OFF : ' ';
    if(MwSensorActive&mode_nav_althold) screenBuffer[0] = SYM_BAR_ON;
    screenBuffer[1] =0;
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]+LINE);
    
    screenBuffer[0] = (MwSensorPresent&MAGNETOMETER) ? SYM_MAG_OFF : ' ';
    if(MwSensorActive&mode_mag) screenBuffer[0] = SYM_MAG_ON; 
    screenBuffer[1] =0;
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]+2*LINE);
    
    screenBuffer[0] = (MwSensorPresent&ACCELEROMETER) ? SYM_ACC_OFF : ' ';
    if(MwSensorActive&mode_angle ||(MwSensorActive&mode_horizon && MwRcData[PITCHSTICK] > 1450 && MwRcData[PITCHSTICK] < 1550)) screenBuffer[0] =  SYM_ACC_ON;
    screenBuffer[1] =0;
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]+3*LINE);
    
    }
}
*/

/*
void display_Nav_State(void)    // Nuovo NEB per iNAV                                                                  
{ 
  if((Settings[L_SENSORPOSITIONDSPL]) & (screen_number))
	if (GPS_fix)
	{
//		screenBuffer[0]=SYM_READY; // GPS ok "G" icon
		screenBuffer[0]=SYM_GPS_ON; // GPS ok "G" icon
		if(NAV_state >2 && NAV_state <5) screenBuffer[0]=SYM_HOLD;// NEB added PosHOLD icon
		else if(NAV_state >0 && NAV_state <3) screenBuffer[0]=SYM_G_HOME; // NEB added RTH icon
                else if(NAV_state >7 && NAV_state <13) screenBuffer[0]=SYM_LAND;    // NEB added LAND icon
		screenBuffer[1]=0;    
		MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]);
	}
}
*/

/*
void displayMode(void)
{
  if((Settings[L_MODEPOSITIONDSPL]) & (screen_number)) {

//  if (!(MwSensorActive&mode_nav_wp))  // NEB Non sono in Mission
//      {
        if(MwSensorActive&mode_failsafe) // NEB added failsafe charset
        {
          if(!BlinkAlarm) screenBuffer[0] = ' ';
          else screenBuffer[0]=SYM_FSAFE;
          if(!BlinkAlarm) screenBuffer[1] = ' ';
          else screenBuffer[1]=SYM_FSAFE_1;
          if(!BlinkAlarm) screenBuffer[2] = ' ';
          else screenBuffer[2]=SYM_FSAFE_2;
          screenBuffer[3] =0;
        }
        
        else if (MwSensorActive&mode_nav_wp)  // NEB Sono in Mission
          {
          screenBuffer[0]=SYM_MISSION;     // NEB mission icon
          screenBuffer[1]=SYM_MISSION_1;
          screenBuffer[2]=SYM_MISSION_2;
          screenBuffer[3]=' ';
          screenBuffer[4]=SYM_WAYPOINT;   // NEB Waypoint icon
          screenBuffer[5]=SYM_WAYPOINT_1;
          itoa(NAV_waypoint,screenBuffer+6,10);  // NEB Waypoint value   
          screenBuffer[8] =0;    
          }
                
         else if(MwSensorActive&mode_nav_rth||MwSensorActive&mode_nav_poshold)
         {
          screenBuffer[0]=SYM_AUTOPILOT;
          screenBuffer[1]=SYM_AUTOPILOT_1;
          screenBuffer[2]=' ';
          screenBuffer[3] =0;
          }
              
         else if(MwSensorActive&mode_horizon)
         {
          screenBuffer[0]=SYM_HORIZON;
          screenBuffer[1]=SYM_HORIZON_1;
          screenBuffer[2]=SYM_HORIZON_2;
          screenBuffer[3] =0;
         }    
         else if(MwSensorActive&mode_angle)
         {
          screenBuffer[0]=SYM_ANGLE;
          screenBuffer[1]=SYM_ANGLE_1;
          screenBuffer[2]=' ';
          screenBuffer[3] =0;
         }

        else
        {
          screenBuffer[0]=SYM_ACRO;
          screenBuffer[1]=SYM_ACRO_1;
          screenBuffer[2]=' ';
          screenBuffer[3] =0;
        }
//         screenBuffer[3] =0;
//      }

      MAX7456_WriteString(screenBuffer,((Settings[L_MODEPOSITIONROW]-1)*30) + Settings[L_MODEPOSITIONCOL]);
   }
}
*/

void displayIntro(void)
{
  MAX7456_WriteString_P(HUD_vers, KVTeamVersionPosition); // Version string
  
  MAX7456_WriteString_P(MultiWiiLogoL1Add, KVTeamVersionPosition+123);
  MAX7456_WriteString_P(MultiWiiLogoL2Add, KVTeamVersionPosition+123+LINE);
}

void displayCallsign(void)
{
  if(Settings[L_CALLSIGNPOSITIONDSPL] & (screen_number)) {
        for(int X=0; X<10; X++) {
            screenBuffer[X] = char(Settings[S_CS0 + X]);
       }   
        screenBuffer[10] = 0;
        MAX7456_WriteString(screenBuffer,((Settings[L_CALLSIGNPOSITIONROW]-1)*30) + Settings[L_CALLSIGNPOSITIONCOL]);
    }
}

/*
float sqrt(float number) {  // NEB non usata
	long i;
	float x, y;
	const float f = 1.5F;

	x = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;			// evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );		// what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( f - ( x * y * y ) );		// 1st iteration
//	y  = y * ( f - ( x * y * y ) );		// 2nd iteration, this can be removed
	return number * y;
}
*/

  // ********************* TEST ****************************
 
void display_Test_Data(void)  // NEB solo per test, da commentare dopo l'uso.
{
/* 
      itoa(test1, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, ((Settings[L_MW_ALTITUDEPOSITIONROW]-3)*30)+10);

      itoa(test2, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, ((Settings[L_MW_ALTITUDEPOSITIONROW]-3)*30)+20);


      itoa(test3, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE04+10);

       itoa(test4, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE04+14);

      itoa(test5, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE05+2);

      itoa(test6, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE05+9);

      itoa(test7, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE05+14);

      itoa(test8, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, LINE05+19);

      itoa(GPS_hdop, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, position+4-30);
      ItoaPadded(GPS_hdop, screenBuffer, 4, 2);
      MAX7456_WriteString(screenBuffer, position+10-30);
      itoa(c1, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, position+3-30);
      itoa(remaining1, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, position+6-30);
      itoa(bit1, screenBuffer, 10);
      MAX7456_WriteString(screenBuffer, position+9-30);
      itoa((MwSensorActive), screenBuffer, 2);  // NEB Rappres. binaria
      MAX7456_WriteString(screenBuffer, position+1);
      itoa(int(trip50ms), screenBuffer, 10);
//      MAX7456_WriteString(screenBuffer, position+10-30);
      MAX7456_WriteString(screenBuffer,LINE05 + 10);
      itoa(tripMinus8Percent, screenBuffer, 10);
//      MAX7456_WriteString(screenBuffer, position+16-30);
      MAX7456_WriteString(screenBuffer,LINE05 + 16);
*/     
}
// ********************* FINE TEST ****************************



void displayHorizon(int rollAngle, int pitchAngle)   // NEB AH vecchio metodo senza patch di Int77
{
    if((Settings[L_HORIZONPOSITIONDSPL])& (screen_number)){
      uint16_t position = ((Settings[L_HORIZONPOSITIONROW]-1)*30) + Settings[L_HORIZONPOSITIONCOL];      
/*
       // NEB Pitch Warning 
       int16_t pitchAngleWarn=pitchAngle/10;
       if(allSec > 7 && (pitchAngleWarn>= Settings[S_PITCH_WARNING]  || pitchAngleWarn <=- Settings[S_PITCH_WARNING])) {
         if(BlinkAlarm){    // Flashing synchro with other alarms   
          screen[position+3] = SYM_WARN;
          screen[position+4] = SYM_WARN_1;
          screen[position+5] = SYM_WARN_2;
          itoa(pitchAngleWarn, screenBuffer, 10);
          uint8_t xx = FindNull();
          screenBuffer[xx++] = SYM_DEGREES;
          screenBuffer[xx] = 0;
          MAX7456_WriteString(screenBuffer, position+7);
          }
      }

      // NEB limitazione per non uscire troppo dal reticolo AH
      if (pitchAngle > 300) pitchAngle=300;
      if (pitchAngle < -300) pitchAngle=-300;
      if (rollAngle > 400) rollAngle=400;
      if (rollAngle < -400) rollAngle=-400;
      
//      pitchAngle=pitchAngle-40;  // NEB corregge l'orizzonte (Abbassa la riga di 4 gradi allineata col rif di centro)
    
      // NEB Disegna linea orizzonte
      for(uint8_t X=0; X<=8; X++) {
//        if (X==4) X=5;    // NEB crea spazio per center ref
        int Y = (rollAngle * (4-X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if(Y >= 0 && Y <= 81) { 
          uint16_t pos = position -1 + LINE*(Y/9) + 3 - 2*LINE + X;
          screen[pos] = SYM_AH_BAR_0+(Y%9);
          if(Y>=9 && (Y%9) == 0)
            screen[pos-LINE] = SYM_AH_BAR_9;
          }
      }
*/
     // NEB Center ref sempre visibile
     screen[position+2*LINE+7] = SYM_AH_CENTER;
   }
}    // NEB AH vecchio metodo senza patch di Int77
   
   
   
void displayVoltage(void)
{
  if((Settings[L_VOLTAGEPOSITIONDSPL]) & (screen_number)) {
    if (Settings[S_MAINVOLTAGE_VBAT]){voltage=MwVBat;} // Se analogica disabilitata prende voltaggio da MWii
//----------- NEB Statistics minimo voltaggio batt
    if(voltage < voltagemin)
        voltagemin = voltage;
    if (voltage ==255) {return;}  // Se=255 (valore non ancora letto via MSP), allora non mostrare    
//-----------  
    if (voltage <=(Settings[S_VOLTAGEMIN]) && !BlinkAlarm){return;}
    ItoaPadded(voltage, screenBuffer, 4, 3);
    screenBuffer[4] = SYM_VOLTS;
    screenBuffer[5] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_VOLTAGEPOSITIONROW]-1)*30) + Settings[L_VOLTAGEPOSITIONCOL]);
  }
} 

void displayVidVoltage(void)
{
  if((Settings[L_VIDVOLTAGEPOSITIONDSPL]) & (screen_number)) {
      if (Settings[S_VIDVOLTAGE_VBAT]){vidvoltage=MwVBat;} // Se analogica disabilitata prende voltaggio da MWii
      else  {;} //  // Se analogica abilitata, dato = voltage
//----------- NEB Statistics copia del voltaggio batt main, questo è aggiunto solo per non visualizzare 25.5V allo startup
      if (vidvoltage ==255) {  // Se=255 (valore non ancora letto via MSP), allora non mostrare
        return;
      }    
//-----------  
      if (vidvoltage <=(Settings[S_VIDVOLTAGEMIN]) && !BlinkAlarm){
      return;
      }
      ItoaPadded(vidvoltage, screenBuffer, 4, 3);
      screenBuffer[4]=SYM_VOLTS;
      screenBuffer[5]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_VIDVOLTAGEPOSITIONROW]-1)*30) + Settings[L_VIDVOLTAGEPOSITIONCOL]);
    }
} 



void displayTime(void)
{
// Fly Time
  if((Settings[L_FLYTIMEPOSITIONDSPL]) & (screen_number)) {
      screenBuffer[0] ='F';
      screenBuffer[1] =0;
      MAX7456_WriteString(screenBuffer,((Settings[L_FLYTIMEPOSITIONROW]-1)*30) + Settings[L_FLYTIMEPOSITIONCOL]);
      formatTime(flyTime, screenBuffer, 0); // flyTime visualizzato sempre
      MAX7456_WriteString(screenBuffer,((Settings[L_FLYTIMEPOSITIONROW]-1)*30) + Settings[L_FLYTIMEPOSITIONCOL]+1);
  } 
// On Time
  if((Settings[L_ONTIMEPOSITIONDSPL]) & (screen_number)) {
      screenBuffer[0] ='T';
      screenBuffer[1] =0;
      MAX7456_WriteString(screenBuffer,((Settings[L_ONTIMEPOSITIONROW]-1)*30) + Settings[L_ONTIMEPOSITIONCOL]-1);
      formatTime(onTime, screenBuffer, 0); // onTime visualizzato sempre
      MAX7456_WriteString(screenBuffer,((Settings[L_ONTIMEPOSITIONROW]-1)*30) + Settings[L_ONTIMEPOSITIONCOL]);
      
// TRIP 4 digit totali da visualizzare "M 9999" (Sopra onTime)
      screenBuffer[0] ='M';
      screenBuffer[1] =0;
      MAX7456_WriteString(screenBuffer,((Settings[L_ONTIMEPOSITIONROW]-2)*30) + Settings[L_ONTIMEPOSITIONCOL]-1);
      ItoaPadded(trip50ms, screenBuffer, 4,0);
      MAX7456_WriteString(screenBuffer,((Settings[L_ONTIMEPOSITIONROW]-2)*30) + Settings[L_ONTIMEPOSITIONCOL]+1);    
    }   
}


void displayAmperage(void)
{
// Real Ampere is ampere / 10
  if((Settings[L_AMPERAGEPOSITIONDSPL]) & (screen_number)) {
    ItoaPadded(amperage, screenBuffer, 4, 3);
    screenBuffer[4] = SYM_AMPS;
    screenBuffer[5] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_AMPERAGEPOSITIONROW]-1)*30) + Settings[L_AMPERAGEPOSITIONCOL]);
  }
}

void displaypMeterSum(void)
{
  if((Settings[L_PMETERSUMPOSITIONDSPL]) & (screen_number)) {
    screenBuffer[0]=0;
    itoa(amperagesum,screenBuffer,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_PMETERSUMPOSITIONROW]-1)*30) + Settings[L_PMETERSUMPOSITIONCOL]);
  }
}

void displayRSSI(void)
{
  if((Settings[L_RSSIPOSITIONDSPL]) & (screen_number)) {
    if (rssi <=(Settings[S_RSSI_ALARM]) && !BlinkAlarm){
      return;
      }
    // Calcul et affichage du Rssi
    ItoaPadded(rssi,screenBuffer,3,0);
    uint8_t xx = FindNull();
    screenBuffer[xx++] = '%';
    screenBuffer[xx] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_RSSIPOSITIONROW]-1)*30) + Settings[L_RSSIPOSITIONCOL]);
    }
}

void displayHeading(void)
{
  if((Settings[L_MW_HEADINGPOSITIONDSPL]) & (screen_number)) {
      int16_t head = MwHeading;
      if (Settings[S_HEADING360]) {
        if(head < 0)
          head += 360;
        ItoaPadded(head,screenBuffer,3,0);
        screenBuffer[3]=SYM_DEGREES;
        screenBuffer[4]=0;
      }
      else {
        if(head < 0)
          head += -0;
        ItoaPadded(head,screenBuffer,4,0);
        screenBuffer[4]=0;
      }
  MAX7456_WriteString(screenBuffer,((Settings[L_MW_HEADINGPOSITIONROW]-1)*30) + Settings[L_MW_HEADINGPOSITIONCOL]);
  }
}

void displayHeadingGraph(void)
{
  if((Settings[L_MW_HEADINGGRAPHPOSDSPL]) & (screen_number)) {
    int xx;
    uint16_t pos;
//    xx = MwHeading * 2; // NEB per grafico originale a 8 punti
//    xx = xx + 440 -36;
//    xx = xx / 90;
    xx = (MwHeading + 11.25)/22.5;  // NEB per grafico a 16 punti
//    uint16_t pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL];  // NEB per grafico a 16 punti 5 digit
//    memcpy_P(screen+pos, headGraph+xx, 5);

//    uint16_t pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]-2;  // NEB per grafico a 16 punti 9 digit
//    memcpy_P(screen+pos, headGraph+xx, 9);

// Heading grafico a 16 punti e 5 digit (tecnica del mezzo carattere) Usa nuova char-map Vers. KVT_HUD_05.mcm (28/12/2017)
    screenBuffer[0] =0XFE; screenBuffer[1] =0;  // Freccia di Rif.
    MAX7456_WriteString(screenBuffer,((Settings[L_MW_HEADINGGRAPHPOSROW]-2)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]+2);  // Freccia di Rif.
    
    pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL];  // NEB per grafico a 16 punti 5 digit Primo carattere
    memcpy_P(screen+pos, headGraph+xx, 1);
    pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]+1;  // NEB per grafico a 16 punti 5 digit Secondo carattere
    memcpy_P(screen+pos, headGraph+xx+2, 1);
    pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]+2;  // NEB per grafico a 16 punti 5 digit Terzo Carattere (centrale)
    memcpy_P(screen+pos, headGraph+xx+4, 1);
    pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]+3;  // NEB per grafico a 16 punti 5 digit Quarto carattere
    memcpy_P(screen+pos, headGraph+xx+6, 1);
    pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL]+4;  // NEB per grafico a 16 punti 5 digit Quinto carattere
    memcpy_P(screen+pos, headGraph+xx+8, 1);
  }
 }
 
//void displayFontScreen(void) {
//  for(uint16_t i = 0; i < 256; i++)
//    screen[90+i] = i;
//}

void displayGPSPosition(void)
{
  if(Settings[S_COORDINATES]){
    if((Settings[L_MW_GPS_LATPOSITIONDSPL]) & (screen_number)) {
        screenBuffer[0] = SYM_LAT;
        FormatGPSCoord(GPS_latitude_Dsply,screenBuffer+1,4,'N','S');
        MAX7456_WriteString(screenBuffer,((Settings[L_MW_GPS_LATPOSITIONROW]-1)*30) + Settings[L_MW_GPS_LATPOSITIONCOL]);
       }
    if((Settings[L_MW_GPS_LONPOSITIONDSPL]) & (screen_number)) {
         screenBuffer[0] = SYM_LON;
         FormatGPSCoord(GPS_longitude_Dsply,screenBuffer+1,4,'E','W');
         MAX7456_WriteString(screenBuffer,((Settings[L_MW_GPS_LONPOSITIONROW]-1)*30) + Settings[L_MW_GPS_LONPOSITIONCOL]);
      }
    }
}
   
void displayNumberOfSat(void)
{
  if((Settings[L_GPS_NUMSATPOSITIONDSPL]) & (screen_number)) {
    itoa(GPS_numSat,screenBuffer,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_NUMSATPOSITIONROW]-1)*30) + Settings[L_GPS_NUMSATPOSITIONCOL]);    // NEB Sats

// NEB HDOP (HDOP è adimensionale, ma siccome EPH e EPV sono in cm, HDOP si assimila a cm, quindi la lettura va divisa per 100).
// NEB 4 digit totali da visualizzare "1.23" e il punto decimale alla posiz. 2 partendo da sinistra (non divido ma metto punto decimale).
    ItoaPadded(GPS_hdop, screenBuffer, 4, 2);
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_NUMSATPOSITIONROW]-1)*30) + Settings[L_GPS_NUMSATPOSITIONCOL]+3);
  }  
}

void displayGPS_speed(void)
{
uint16_t xx;
    if(!Settings[S_UNITSYSTEM])
      xx = GPS_speed * 0.036;      // cm/sec > Km/h
    else
      xx = GPS_speed * 0.02236932; // cm/sec > mph
    if(xx > speedMAX)
      speedMAX = xx;
      
     if((Settings[L_SPEEDPOSITIONDSPL]) & (screen_number)) {
    //screenBuffer[0]=SYM_SPEED;
   // screenBuffer[1]=SYM_SPEED_1;
     itoa(xx,screenBuffer,10);
     MAX7456_WriteString(screenBuffer,((Settings[L_SPEEDPOSITIONROW]-1)*30) + Settings[L_SPEEDPOSITIONCOL]);
    }
//  }
}

void displayAltitude(void)
{
 // NEB altitudine da GPS O via MSP da Simulatore (Configuratore)
int16_t altitude;
        if(Settings[S_UNITSYSTEM]) altitude = MwAltitude * 0.032808; // GPS cm > feet
        else altitude = MwAltitude /100;                             // GPS cm > mt

        if(armed && allSec>5 && altitude > altitudeMAX) altitudeMAX = altitude;
        if((Settings[L_MW_ALTITUDEPOSITIONDSPL]) & (screen_number)) {
          if((altitude >= (Settings[S_VOLUME_ALT_MAX]*10) && !BlinkAlarm) || (flyTime > 60 && altitude < (Settings[S_VOLUME_ALT_MIN]) && !BlinkAlarm))  // NEB 
          return;
  
          ItoaPadded(altitude, screenBuffer,4,0);
          screenBuffer[5] = 0;
          MAX7456_WriteString(screenBuffer,((Settings[L_MW_ALTITUDEPOSITIONROW]-2)*30) + Settings[L_MW_ALTITUDEPOSITIONCOL]);
       }
#ifdef BARO_BMP085
// NEB altitudine da Baro
int16_t baro_alt_dsply;
        if(Settings[S_UNITSYSTEM]) baro_alt_dsply = Baro_Ground_Alt * 0.032808; // Baro cm > feet
        else baro_alt_dsply = Baro_Ground_Alt /100;                             // Baro cm > mt
        
        if(armed && allSec>5 && altitude > altitudeMAX) altitudeMAX = altitude;
        if((Settings[L_MW_ALTITUDEPOSITIONDSPL]) & (screen_number)) {
          if((baro_alt_dsply >= (Settings[S_VOLUME_ALT_MAX]*10) && !BlinkAlarm) || (flyTime > 60 && baro_alt_dsply < (Settings[S_VOLUME_ALT_MIN]) && !BlinkAlarm))  // NEB 
          return;
  
          ItoaPadded(baro_alt_dsply, screenBuffer,4,0);
          screenBuffer[5] = 0;
          MAX7456_WriteString(screenBuffer,((Settings[L_MW_ALTITUDEPOSITIONROW]-1)*30) + Settings[L_MW_ALTITUDEPOSITIONCOL]);
       }
#endif // Baro
}

#ifdef BARO_BMP085
void displayTemperature(void)
{
// Temperatura da baro, posizione a fianco V batt
    itoa(temperature/10,screenBuffer,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_VOLTAGEPOSITIONROW]-1)*30) + Settings[L_VOLTAGEPOSITIONCOL]+6);
}
#endif // Baro

void displayClimbRate(void)
{
	int16_t climbrate = constrain(MwVario,-Settings[S_CLIMB_RATE_ALARM]*150,Settings[S_CLIMB_RATE_ALARM]*150);
	climbrate = map(climbrate, -Settings[S_CLIMB_RATE_ALARM]*150,Settings[S_CLIMB_RATE_ALARM]*150,-18,18);
	//uint16_t position = ((Settings[L_CLIMBRATEPOSITIONROW]-5)*30) + Settings[L_CLIMBRATEPOSITIONCOL]+3;
        uint16_t position = (3*LINE) + 8;
	
	if(MwVario <= -Settings[S_CLIMB_RATE_ALARM]*100 && !BlinkAlarm)	return;

			if((Settings[L_CLIMBRATEPOSITIONDSPL])& (screen_number)){
				if(climbrate >= 0){
					for (int8_t Y = 0;Y<3;Y++){
						screen[position + (4+Y)*LINE] = SYM_CLIMB_EMPTY;
						if(climbrate < Y*6) screen[position + (3-Y)*LINE] = SYM_CLIMB_EMPTY;
							else if ((climbrate >= Y*6)&&(climbrate <= (Y+1)*6)) {
										screen[position + (3-Y)*LINE] = SYM_CLIMB_0P + climbrate - Y*6;
										if ((climbrate == 18)&&(Y==2)) screen[position + (2-Y)*LINE] = SYM_CLIMB_0P_EDGE;
								}
								else if(climbrate > (Y+1)*6) screen[position + (3-Y)*LINE] = SYM_CLIMB_FULL;
							
					}
				}
							
				if(climbrate <= 0){
					for (int8_t Y = 0;Y<3;Y++){
						if (climbrate !=0) screen[position + (3-Y)*LINE] = SYM_CLIMB_EMPTY;
						if(climbrate > -Y*6) screen[position +(4+Y)*LINE] = SYM_CLIMB_EMPTY;
							else if ((climbrate <= -Y*6)&&(climbrate >= -(1+Y)*6)) {
										screen[position + (4+Y)*LINE] = SYM_CLIMB_0N - climbrate - Y*6;
										if ((climbrate == -18)&&(Y==2)) screen[position + (5+Y)*LINE] = SYM_CLIMB_0N_EDGE;
								}
								else if(climbrate < -(1+Y)*6) screen[position + (4+Y)*LINE] = SYM_CLIMB_FULL;
					}
				}
	
			int16_t vario;
			if(Settings[S_UNITSYSTEM])
			  vario = MwVario * 0.032808;       // cm/sec ----> ft/sec
			else
			  vario = MwVario/10;               // cm/sec ----> dm/s
			ItoaPadded(vario, screenBuffer, 3, 2);
			MAX7456_WriteString(screenBuffer,((Settings[L_CLIMBRATEPOSITIONROW])*LINE) + Settings[L_CLIMBRATEPOSITIONCOL]);
		}
}

void displayDistanceToHome(void)
{
    if (GPS_distanceToHome >= (Settings[S_VOLUME_DIST_MAX]*100) && !BlinkAlarm) 
      return;

    int16_t dist;
    if(Settings[S_UNITSYSTEM])
      dist = GPS_distanceToHome * 3.2808;           // mt > feet
    else
      dist = GPS_distanceToHome;                    // mt
    if(dist > distanceMAX)
      distanceMAX = dist;  
      
    if((Settings[L_GPS_DISTANCETOHOMEPOSDSPL]) & (screen_number)) {
    screenBuffer[0] = SYM_LOS;
    itoa(dist, screenBuffer+1, 10);
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_DISTANCETOHOMEPOSROW]-1)*30) + Settings[L_GPS_DISTANCETOHOMEPOSCOL]); 
    }
}


void displayDirectionToHome(void)
{
  if((Settings[L_GPS_DIRECTIONTOHOMEPOSDSPL] & screen_number) && GPS_fix && GPS_distanceToHome >= 2) {  // Display DirToHome if enabled
  int16_t d =  GPS_directionToHome;
  d *= 4;
  d += 45;
  d = 1*(d/90)%16;
  screenBuffer[0] = SYM_ARROW_NORTH + d;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,((Settings[L_GPS_DIRECTIONTOHOMEPOSROW]-1)*30) + Settings[L_GPS_DIRECTIONTOHOMEPOSCOL]);  // NEB readded moveable home arrow
  }
  else return;
}


void displayAngleToHome(void)
{
  if((Settings[L_GPS_ANGLETOHOMEPOSDSPL]) & (screen_number)) {
    if(GPS_distanceToHome <= 2 && !BlinkAlarm)
      return;
    ItoaPadded(GPS_directionToHome,screenBuffer,3,0);
    screenBuffer[3] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_ANGLETOHOMEPOSROW]-1)*30) + Settings[L_GPS_ANGLETOHOMEPOSCOL]);
    }
}


void displayStatistics(void)
  {
    MAX7456_WriteString_P(configMsg90, 38);

    MAX7456_WriteString_P(configMsg91, ROLLT);
    MAX7456_WriteString(itoa(trip50ms,screenBuffer,10),ROLLD-3);

    MAX7456_WriteString_P(configMsg92, PITCHT);
    MAX7456_WriteString(itoa(distanceMAX,screenBuffer,10),PITCHD-3);

    MAX7456_WriteString_P(configMsg93, YAWT);
    MAX7456_WriteString(itoa(altitudeMAX,screenBuffer,10),YAWD-3);

    MAX7456_WriteString_P(configMsg94, ALTT);
    MAX7456_WriteString(itoa(speedMAX,screenBuffer,10),ALTD-3);

    MAX7456_WriteString_P(configMsg95, VELT);
    formatTime(flyingTime, screenBuffer, 1);
    MAX7456_WriteString(screenBuffer,VELD-4);

    MAX7456_WriteString_P(configMsg96, LEVT);
    MAX7456_WriteString(itoa(amperagesum,screenBuffer,10),LEVD-3);

//--------------- aggiunta min battery per statistics
    MAX7456_WriteString_P(configMsg97, MAGT);
    ItoaPadded(voltagemin, screenBuffer, 4, 3);
    screenBuffer[4] = 0;
    MAX7456_WriteString(screenBuffer,MAGD-3);
//---------------    
  }



    

