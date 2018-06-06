
#ifdef BARO_BMP085

//#define INTERNAL_I2C_PULLUPS
#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
//#define I2C_SPEED 100000L   //100kHz normal mode
#define I2C_SPEED 400000L     //400kHz fast mode
//#define BMP085_Addr 0x77    // 7bit address
#define BMP085_Addr 0xEE      // 8bit address


  float temperature = 0;
  float pressure = 0;
  float Baro_alt = 0;
  float Baro_alt_Zero=0;
  float Baro_Ground_Alt=0;
  float Baro_alt_Smooth = 0;

  static int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;
  static uint16_t  ac4,ac5,ac6;
  float ut; //uncompensated T
  float up; //uncompensated P

// *********************
// I2C general functions
// *********************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK  (1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

void i2c_write(uint8_t data ) { 
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
  }
}

void checkStatusI2C() {
  if ( TW_STATUS  == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    TWCR = 0;
  }
}


// **************************
// I2C Barometer BOSCH BMP085
// **************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)

// sensor registers from the BOSCH BMP085 datasheet

void  i2c_Baro_init() {
  delay(10);
  ac1 = i2c_BMP085_readIntRegister(0xAA);
  ac2 = i2c_BMP085_readIntRegister(0xAC);
  ac3 = i2c_BMP085_readIntRegister(0xAE);
  ac4 = i2c_BMP085_readIntRegister(0xB0);
  ac5 = i2c_BMP085_readIntRegister(0xB2);
  ac6 = i2c_BMP085_readIntRegister(0xB4);
  b1  = i2c_BMP085_readIntRegister(0xB6);
  b2  = i2c_BMP085_readIntRegister(0xB8);
  mb  = i2c_BMP085_readIntRegister(0xBA);
  mc  = i2c_BMP085_readIntRegister(0xBC);
  md  = i2c_BMP085_readIntRegister(0xBE);

  i2c_BMP085_calibrate();
}

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(unsigned char r) {
  uint8_t msb, lsb;
  
  i2c_rep_start(BMP085_Addr + 0);
  i2c_write(r);
  i2c_rep_start(BMP085_Addr + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((int16_t)msb<<8) | ((int16_t)lsb));
}

// read uncompensated temperature value: send command first
void i2c_BMP085_readUT_Command() {
  i2c_rep_start(BMP085_Addr + 0);
  i2c_write(0xf4);
  i2c_write(0x2e);
  i2c_rep_start(BMP085_Addr + 0);
  i2c_write(0xF6);
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
uint16_t i2c_BMP085_readUT_Result() {
  uint8_t msb, lsb;
  i2c_rep_start(BMP085_Addr + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((uint16_t)msb<<8) | ((uint16_t)lsb));
}

// read uncompensated pressure value: send command first
void i2c_BMP085_readUP_Command () {
  i2c_rep_start(BMP085_Addr + 0);
  i2c_write(0xf4);
  i2c_write(0xf4); //control register value for oversampling setting 3
  i2c_rep_start(BMP085_Addr + 0); //I2C write direction => 0
  i2c_write(0xf6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
uint32_t i2c_BMP085_readUP_Result () {
  uint8_t msb, lsb, xlsb;
  i2c_rep_start(BMP085_Addr + 1);//I2C read direction => 1
  msb = i2c_readAck();
  lsb = i2c_readAck();
  xlsb = i2c_readNak();
  return (((uint32_t)msb<<16) | ((uint32_t)lsb<<8) | ((uint32_t)xlsb)) >>5;
}

// deduction of true temperature and pressure from sensor, code is described in the BMP085 specs
void i2c_BMP085_CompensatedSensor() {
  int32_t x1, x2, x3, b3, b5, b6, p;
  uint32_t b4, b7;
  uint16_t a,b;

  //calculate true temperature
  x1 = (int32_t)(ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4; // NEB temperatura *10
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ( ( ((int32_t) ac1 * 4 + x3)<<3 ) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = (up - b3) * (50000 >> 3);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

//in a whole cycle: we read temperature one time and pressure 5 times
void i2c_Baro_update() {
  static uint32_t t;
  static uint8_t state1 =0,state2 = 0;

  if ( (micros()-t )  < 30000 ) return; //each read is spaced by 30ms
  t = micros();
  
  if (state1 == 0) { 
    if (state2 == 0) {
      i2c_BMP085_readUT_Command();
      state2=1;
    } else {
      ut = i2c_BMP085_readUT_Result();
      state2=0;
      state1=5;
    }
  } else {
    if (state2 == 0) {
      i2c_BMP085_readUP_Command();
      state2=1;
    } else {
      up = i2c_BMP085_readUP_Result();
      state2=0;
      state1--;
      i2c_BMP085_CompensatedSensor();
      Baro_alt = calcAltitude(pressure); // altitude in centimeter from starting point
      Baro_alt_Smooth = (Baro_alt_Smooth*3+Baro_alt)/4;
      Baro_Ground_Alt = Baro_alt_Smooth - Baro_alt_Zero;
    }
  }
}


float calcAltitude(float pressure){
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
//  C = C /0.0000225577;  // Altitudine in m
    C = C /0.000000225577;  // Altitudine in cm
  return C;
}


void i2c_BMP085_calibrate() {
  Baro_alt_Zero = 0;
  for (uint8_t i=0;i<15;i++) {
    i2c_Baro_update();
    delay(50);
  }
  Baro_alt_Zero = Baro_alt;
}
#endif // Baro







