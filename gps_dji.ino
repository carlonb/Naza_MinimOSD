/**
   ___  _ __           ____  _______ 
  / _ )(_) /____ __ __/ __ \/ __/ _ \
 / _  / / __(_-</ // / /_/ /\ \/ // /
/____/_/\__/___/\_, /\____/___/____/ 
               /___/                 

DJI GPS Parser based on GPS_DJI with some modifications to fit
the minimal approach of this OSD Firmware.

Original work by Joerg-D. Rothfuchs                                           

**/


#include "gps_dji.h"
#include "Arduino.h"


static char gps_rx_buffer[sizeof(struct DJIPacket)];
static struct GPS_RX_STATS gpsRxStats; 


short decodeShort(byte* d, unsigned char mask)
{
    union { short s; unsigned char b[2]; } val;
    for (int i=0; i<2; i++) val.b[i] = *d++ ^ mask;
    return val.s;
}


long decodeLong(byte* d, unsigned char mask)
{
    union { long l; unsigned char b[4]; } val;
    for (int i=0; i<4; i++) val.b[i] = *d++ ^ mask;
    return val.l;
} 

void parse_dji_gps(struct DJI_GPS *gps)
{
	int mask = gps->mask;
	
  if (gps->flags ^ mask & STATUS_FLAGS_GPSFIX_OK) {
      switch (gps->gpsFix ^ mask) {
    		case STATUS_GPSFIX_2DFIX:
    			GPS_fix = GPSPOSITIONSENSOR_STATUS_FIX2D;
                    break;
    		case STATUS_GPSFIX_3DFIX:
    			GPS_fix = GPSPOSITIONSENSOR_STATUS_FIX3D;
                    break;
    		default:
    			GPS_fix = GPSPOSITIONSENSOR_STATUS_NOFIX;
        }
   } else {	// fix is not valid so we make sure to treat it as NOFIX
            GPS_fix = GPSPOSITIONSENSOR_STATUS_NOFIX;
          }

	GPS_numSat	  = gps->numSV;
	GPS_latitude_Dsply	= (float)decodeLong((byte*)&gps->lat,  mask);     //   x10^7 degree decimal
  GPS_latitude = GPS_latitude_Dsply / 10000000;                         // NEB per calcoli Dir to Home deve essere /10M
	GPS_longitude_Dsply	= (float)decodeLong((byte*)&gps->lon,  mask);     //   x10^7 degree decimal
  GPS_longitude = GPS_longitude_Dsply / 10000000;                       // NEB per calcoli Dir to Home deve essere /10M
	GPS_alt	      = (float)decodeLong((byte*)&gps->hMSL, mask)/10;        // mm -> /10=cm
	MwVario       = (float)decodeLong((byte*)&gps->velD, mask);           // cm/sec
  MwVario       = -MwVario;                                             // Cambio segno, GPS restituisce discesa positiva
  
  uint16_t GPS_nDOP      = (float)decodeLong((byte*)&gps->nDOP, mask)/100;    // Northing DOP
  uint16_t GPS_eDOP      = (float)decodeLong((byte*)&gps->eDOP, mask)/100;    // Easting DOP
  GPS_hdop = sqrt(GPS_nDOP * GPS_nDOP + GPS_eDOP * GPS_eDOP);

	float velN		= (float)decodeLong((byte*)&gps->velN, mask);           // cm/sec
	float velE		= (float)decodeLong((byte*)&gps->velE, mask);           // cm/sec
 
 
	// calculate groundspeed
	GPS_speed	= sqrt((velN * velN) + (velE * velE));                      // cm/sec
// gpsSol.groundSpeed = sqrtf(powf(gpsSol.velNED[0], 2)+powf(gpsSol.velNED[1], 2)); //cm/s // NEB copiato da iNAV

 

  // calculate heading; 0=from GPS data (moving)
//  if(DJI_HEADING == 0) {  // NEB Usa heading calcolata con le due velocità relative Nord e Est
   float heading = atan2(velE, velN) * 180.0 / M_PI;
   if (heading < 0.0) heading += 360.0;
//   MwHeading	= heading;  // NEB vers. 08, ora MwHeading viene assegnato in main sopra una certa velocità di spostamento.
   heading_from_gps_calcs = heading; // NEB vers. 08
//  }   
}

void parse_dji_mag(struct DJI_MAG *mag)  // calculate heading; 1=from GPS mag
{
//    if(DJI_HEADING == 1) {  // NEB Usa heading lette da pacchetto dati mag NAZA
/*    	int mask = mag->mask;    // NEB routine decodifica mag originale ma vecchia e incompleta	
    	short x = decodeShort((byte*)&mag->magX, mask) ^ 0x0100;
    	short y = decodeShort((byte*)&mag->magY, mask) ^ 0x0100;       
    	float heading = -atan2(y, x) * 180.0 / M_PI;
    	if (heading < 0.0) heading += 360.0;   	
    	// TODO add magnetic declination
    	// TODO add tilt compensation    	
    	MwHeading	= heading;
*/
// NEB nuova lettura MAG copiata da iNAV "gps_naza.c" (è uguale a quella usata da LibrePilot)
      int mask = mag->magZ & 0xFF;
      mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));
      short x = decodeShort((byte*)&mag->magX, mask);
      short y = decodeShort((byte*)&mag->magY, mask);
      short z = (mag->magZ ^ (mask<<8));    // NEB asse Z per ora non viene usato 
      float heading = -atan2(y, x) * 180.0 / M_PI;

//++++++++
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // NEB Milano +2° 32'E (Positivo) il 15/05/2018
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (2.0 + (32.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
//++++++++

//++++++++++++++++++++++++++++++++++++++
// NEB correzione empirica angolo (...pero' fa sballare Est...)
// Il mag da indicazioni non precise perchè non calibrato e non tilt compensated.
  heading -=15;
//++++++++++++++++++++++++++++++++++++++

      
      if (heading < 0.0) heading += 360.0;    
      // TODO add magnetic declination (NEB fatto sopra)
      // TODO add tilt compensation    (NEB Non si può fare, serve accelerometro o gradi inclinazione) 
//      MwHeading = heading;  // NEB vers. 08, ora MwHeading viene assegnato in main sotto una certa velocità di spostamento.
      heading_from_naza_packet = heading; // NEB vers. 08
//    }  
}


// DJI message parser
void parse_dji_message(struct DJIPacket *dji)
{
    switch (dji->header.id) {
    case DJI_GPS_DATA:
        parse_dji_gps(&dji->payload.dji_gps);
        break;
    case DJI_MAG_DATA:
        parse_dji_mag(&dji->payload.dji_mag);
        break;
    case DJI_XXX_DATA:
        break;
    }
}


// CRC check
bool checksum_dji_message(struct DJIPacket *dji)
{
    int i;
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    ck_a += dji->header.id;
    ck_b += ck_a;

    ck_a += dji->header.len;
    ck_b += ck_a;
    for (i = 0; i < dji->header.len; i++) {
        ck_a += dji->payload.payload[i];
        ck_b += ck_a;
    }
    if (dji->header.ck_a == ck_a && dji->header.ck_b == ck_b) {
        return true;
    } else {
        return false;
    }
}


// parse incoming character stream for messages in DJI format
boolean gps_process(void) { // NEB
//int parse_dji_stream(uint8_t c, char *gps_rx_buffer, struct GPS_RX_STATS *gpsRxStats)
//{
    enum proto_states {
        START,
        DJI_SY2,
        DJI_ID,
        DJI_LEN,
        DJI_PAYLOAD,
        DJI_CHK1,
        DJI_CHK2,
        FINISHED
    };

    static enum proto_states proto_state = START;
    static uint8_t rx_count = 0;
    struct DJIPacket *dji   = (struct DJIPacket *)gps_rx_buffer;

uint8_t c;  // NEB
char *gps_rx_buffer;  // NEB
struct GPS_RX_STATS *gpsRxStats;  // NEB

 while (Serial.available()) { // NEB
        c = Serial.read(); // NEB
        
    if (mspmessage) serialMSPreceive(c); // NEB
    
    switch (proto_state) {
    case START:							// detect protocol
        if (c == DJI_SYNC1) {					// first DJI sync char found
            proto_state = DJI_SY2;
        }
        else if (c=='$') { // NEB se è $, allora è un pacchetto MSP
          serialMSPreceive(c); // NEB
          mspmessage=true; // NEB
        } // NEB
        break;
    case DJI_SY2:
        if (c == DJI_SYNC2) {					// second DJI sync char found
            proto_state = DJI_ID;
        } else {
            proto_state = START;				// reset state
        }
        break;
    case DJI_ID:
        dji->header.id   = c;
        proto_state      = DJI_LEN;
        break;
    case DJI_LEN:
        dji->header.len  = c;
        if (dji->header.len > sizeof(DJIPayload)) {
            gpsRxStats->gpsRxOverflow++;
            proto_state = START;
        } else {
            rx_count    = 0;
            proto_state = DJI_PAYLOAD;
        }
        break;
    case DJI_PAYLOAD:
        if (rx_count < dji->header.len) {
            dji->payload.payload[rx_count] = c;
            if (++rx_count == dji->header.len) {
                proto_state = DJI_CHK1;
            }
        } else {
            gpsRxStats->gpsRxOverflow++;
            proto_state = START;
        }
        break;
    case DJI_CHK1:
        dji->header.ck_a = c;
        proto_state = DJI_CHK2;
        break;
    case DJI_CHK2:
        dji->header.ck_b = c;
        if (checksum_dji_message(dji)) {			// message complete and valid
            parse_dji_message(dji);
            proto_state = FINISHED;
        } else {
            gpsRxStats->gpsRxChkSumError++;
            proto_state = START;
        }
        break;
    default: break;
    }

    if (proto_state == START) {
        return PARSER_ERROR;					// parser couldn't use this byte
    } else if (proto_state == FINISHED) {
        gpsRxStats->gpsRxReceived++;
        proto_state = START;
	      return PARSER_COMPLETE_SET;				// message set complete & processed
    }

    return PARSER_INCOMPLETE;					// message not (yet) complete
  }  // NEB
}

/*boolean gps_process(void) {
uint8_t c;
    while (Serial.available()) {
           c = Serial.read();
           return parse_dji_stream(c, gps_rx_buffer, &gpsRxStats);
          }
}*/

