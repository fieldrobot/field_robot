#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial oSerial(8,9); // RX, TX

typedef struct{
   int16_t steer;
   int16_t speed;
   uint32_t crc;
} Serialcommand;
Serialcommand oCmd;

typedef struct{
   int16_t iSpeedL; // 100* km/h
   int16_t iSpeedR; // 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;  // °C
   uint16_t iVolt;  // 100* V
   int16_t iAmpL;  // 100* A
   int16_t iAmpR;  // 100* A
   uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Hoverhack Test v1.0");
    
  oSerial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t crc32_for_byte(uint32_t r) 
{
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}


void Send(int16_t iSpeed,int16_t iSteer)
{
  oCmd.steer = iSteer;
  oCmd.speed = iSpeed;

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand)-4,   &crc);
  oCmd.crc = crc;
  
  oSerial.write((uint8_t *) &oCmd, sizeof(oCmd)); 
}

int iFailedRec = 0;
boolean Receive()
{
  //while (oSerial.available()) {Serial.print(" ");Serial.print(oSerial.read(),HEX);}return false;

  if (oSerial.available() <  sizeof(SerialFeedback))
    return false;

  SerialFeedback oNew;
  byte* p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
    *p++ = oSerial.read();;

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback)-4,   &crc);

#ifdef DEBUG_RX
  char sBuff[10];
  p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
  {
    sprintf(sBuff," %02x",p[i]);
    Serial.print(sBuff);
  }
  Serial.print(" ?= ");Serial.println(crc,HEX);
#endif

  if (oNew.crc == crc)
  {
    memcpy(&oFeedback,&oNew,sizeof(SerialFeedback));
    return true;    
  }

#ifdef DEBUG_RX
  while (oSerial.available()) {Serial.print(" ");Serial.print(oSerial.read(),HEX);}Serial.println("\t:-(");
#else
  while (oSerial.available()) oSerial.read();   // empty garbage
  Serial.print("X");
  iFailedRec++;
#endif
  return false;
}

#define TIME_SEND 200
int iTest = 1000;
unsigned long iTimeSend = 0;

int iSpeed = 0;
int iSpeedMax = 1000;   // for SPEED_IS_KMH in config.h use 60 for max speed of 6.0 km/h
int iSpeedDelta = 20;   // for SPEED_IS_KMH in config.h use 2 for speed steps of 0.2 km/h 

void loop(void)
{ 
  unsigned long iNow = millis();
  if (Receive())
  {
    if (iFailedRec)
      Serial.println();
    iFailedRec = 0;
    Serial.print("speedL: ");Serial.print(-0.01*(float)oFeedback.iSpeedL);
    Serial.print("\tspeedR: ");Serial.print(-0.01*(float)oFeedback.iSpeedR);
    Serial.print("\tskippedL: ");Serial.print(oFeedback.iHallSkippedL);
    Serial.print("\tskippedR: ");Serial.print(oFeedback.iHallSkippedR);
    Serial.print("\t°C: ");Serial.print(oFeedback.iTemp);
    Serial.print("\tU: ");Serial.print(0.01 * (float)oFeedback.iVolt);
    Serial.print("\tlA: ");Serial.print(0.01 * (float)oFeedback.iAmpL);
    Serial.print("\trA: ");Serial.println(0.01 * (float)oFeedback.iAmpR);
  }

  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;

  iSpeed += iSpeedDelta;
  if ( abs(iSpeed) >= iSpeedMax ) iSpeedDelta *= -1;
  Send(iSpeed,0);


  digitalWrite(LED_BUILTIN, (iNow%2000)<1000);
}
