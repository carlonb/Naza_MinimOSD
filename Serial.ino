#define SERIALBUFFERSIZE 128 //256  NEB
static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;

uint32_t read32() {
  uint32_t t = read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

uint16_t read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
  
}

uint8_t read8()  {
  return serialBuffer[readIndex++];
}

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;

  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();
    if(cmd == OSD_READ_CMD) {
      uint8_t txCheckSum, txSize;
      Serial.write('$');
      Serial.write('M');
      Serial.write('<');
      txCheckSum=0;
      txSize = EEPROM_SETTINGS + 1;
      Serial.write(txSize);
      txCheckSum ^= txSize;
      Serial.write(MSP_OSD);
      txCheckSum ^= MSP_OSD;
      Serial.write(cmd);
      txCheckSum ^= cmd;
      for(uint8_t i=0; i<EEPROM_SETTINGS; i++) {
        Serial.write(Settings[i]);
  	    txCheckSum ^= Settings[i];
        }
      Serial.write(txCheckSum);
   }

    if (cmd == OSD_WRITE_CMD) {
      for(int en=0;en<EEPROM_SETTINGS; en++){
      	uint8_t inSetting = read8();
      	if (inSetting != Settings[en])
      	  EEPROM.write(en,inSetting);
      	Settings[en] = inSetting;
//       if (en==31) test=Settings[en]; // NEB per test Screen Page da eliminare Fino qui è corretto
       }
    readEEPROM();
//       test=Settings[31]; // NEB per test Screen Page da eliminare  Fino qui è corretto
    setMspRequests();
    }

    if(cmd == OSD_GET_FONT) {
      if(dataSize == 5) {
        if(read16() == 7456) {
          nextCharToRequest = read8();
          lastCharToRequest = read8();
          initFontMode();
          }
      }
      else if(dataSize == 56) {
        for(uint8_t i = 0; i < 54; i++)
          fontData[i] = read8();
	      uint8_t c = read8();
        write_NVM(c);
        }
    }
    if(cmd == OSD_RESET) {  // NEB riavvia CPU
      resetFunc();
    }
    if(cmd == OSD_SERIAL_SPEED) {
    ;    
    }                   
  }

  // NEB dati seriali ricevuti solo dal simulatore di OSD config.
  if (cmdMSP==MSP_RAW_GPS)
  {
    GPS_fix=read8();    // NEB non usato
    GPS_numSat=read8();
    GPS_latitude_Dsply = read32();
    GPS_longitude_Dsply = read32();
    GPS_alt = read16();
    GPS_speed = read16();
    read16();  // NEB non usato, gps ground course
    GPS_hdop = read16();
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=read16();
    GPS_directionToHome=read16();
  }

  if (cmdMSP==MSP_ATTITUDE)
  {
    MwAngle[0] = read16();    // NEB non usato
    MwAngle[1] = read16();    // NEB non usato
    MwHeading = read16();
    if (MwHeading <0) MwHeading = MwHeading + 360;
   }
   
  if (cmdMSP==MSP_ALTITUDE)
  {
    MwAltitude =read32();
    MwVario = read16();
  }
; // Qui altre decodifiche da MSP
}

// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

// Dati seriali ricevuti dalla routine gps_dji
void serialMSPreceive(uint8_t c)
{
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;

//  while(Serial.available())
//  {
//    c = Serial.read();

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE) // NEB SERIALBUFFERSIZE=128
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
        mspmessage=false;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
//  }
}

void configExit()
{
;
}

void saveExit()
{
;
}


void fontSerialRequest() {
  int16_t cindex = getNextCharToRequest();
  uint8_t txCheckSum;
  uint8_t txSize;
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  txCheckSum=0;
  txSize=3;
  Serial.write(txSize);
  txCheckSum ^= txSize;
  Serial.write(MSP_OSD);
  txCheckSum ^= MSP_OSD;
  Serial.write(OSD_GET_FONT);
  txCheckSum ^= OSD_GET_FONT;
  Serial.write(cindex);
  txCheckSum ^= cindex;
  Serial.write(cindex>>8);
  txCheckSum ^= cindex>>8;
  Serial.write(txCheckSum);
}
