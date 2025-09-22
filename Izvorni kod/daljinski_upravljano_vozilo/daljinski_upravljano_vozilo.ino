// ==========================
// RX (CAR SIDE) — FULL BIDIRECTIONAL
// Near-field profile: ch=100, 2Mbps, PA=MIN, CRC8, DPL OFF, PL=12
// L298N: ENA=D3, IN1=D2, IN2=D4, ENB=D5, IN3=D6, IN4=D7
// Samotest: pritisnuti džojstik SW dok je štap u centru (kratko naprijed/nazad)
// ==========================

#include <SPI.h> 
#include <Wire.h> 
#include <nRF24L01.h> 
#include <RF24.h> 
#include <MPU6050.h>

RF24 radio(8, 9); // CE=D8, CSN=D9
const uint8_t ADDR_CMD[5] = {'1','N','o','d','e'};// prima komande od TX-a 
const uint8_t ADDR_TELE[5] = {'2','N','o','d','e'};// šalje telemetriju TX-u 
const uint8_t PL = 12; // fiksno 12 B (AX,AY,AZ,GX,GY,GZ)

const int ENA=3, IN1=2, IN2=4;	// L298N lijevi 
const int ENB=5, IN3=6, IN4=7;	// L298N desni

MPU6050 mpu;
unsigned long lastCmdMs = 0;

static inline int shapeSpeed(int v){
  if (abs(v) < 30) return 0;	// dead-zone 
  int m = map(abs(v), 30, 255, 90, 255);    // kick-start 
  return (v >= 0) ? m : -m;
}

static inline void rotateMotor(int s1, int s2){ 
  s1 = shapeSpeed(s1); 
  s2 = shapeSpeed(s2);
  digitalWrite(IN1,	s1>=0);	
  digitalWrite(IN2,	s1<0);
  digitalWrite(IN3,	s2>=0);	
  digitalWrite(IN4,	s2<0);
  analogWrite(ENA, abs(s1)); 
  analogWrite(ENB, abs(s2));
}

void setup() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH); // SS high 
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENA,OUTPUT);	pinMode(IN1,OUTPUT);	pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT);	pinMode(IN3,OUTPUT);	pinMode(IN4,OUTPUT);
  rotateMotor(0,0);		

  Wire.begin(); 
  mpu.initialize();
  radio.begin(); radio.flush_rx(); radio.flush_tx(); 
  radio.setAddressWidth(5);
  radio.setChannel(100); 
  radio.setDataRate(RF24_2MBPS); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.setCRCLength(RF24_CRC_8); 
  radio.setAutoAck(true); 
  radio.disableDynamicPayloads();
  radio.setPayloadSize(PL); 
  radio.setRetries(5,15); 
  radio.openReadingPipe(0, ADDR_CMD); 
  radio.openWritingPipe(ADDR_TELE); 
  radio.startListening();

  Serial.begin(115200);
  Serial.println(F("[RX] ch=100 2Mbps PA=MIN PL=12 (near-field)"));

  /*
  // --- ZA VEĆU UDALJENOST ---
  // radio.setChannel(40);
  // radio.setDataRate(RF24_1MBPS);
  // radio.setPALevel(RF24_PA_LOW); // ili RF24_PA_HIGH ako je daleko
  */
}

void loop() {
if (millis() - lastCmdMs > 600) rotateMotor(0,0); // failsafe

  while (radio.available()) { 
    uint8_t c[PL]; radio.read(c, PL); 
    lastCmdMs = millis();

    int x = c[0], y = c[1]; uint8_t sw = c[2];

    // Daljinski samotest
    if (sw==1 && abs(x-127)<10 && abs(y-127)<10) { 
      rotateMotor(150,150); delay(350); 
      rotateMotor(-150,-150); delay(350); 
      rotateMotor(0,0);
    } else {
      // Tank-steer mixing
      int th = map(y, 0,254, -255, 255);
      int st = map(x, 0,254, -255, 255);
      rotateMotor(constrain(th-st,-255,255), constrain(th+st,-255,255));
    }

    // Telemetrija 12B (AX,AY,AZ,GX,GY,GZ) — int16 LE 
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getAcceleration(&ax,&ay,&az); mpu.getRotation(&gx,&gy,&gz); uint8_t t[PL] = {
    (uint8_t)(ax&0xFF),(uint8_t)((ax>>8)&0xFF), (uint8_t)(ay&0xFF),(uint8_t)((ay>>8)&0xFF),
    
    (uint8_t)(az&0xFF),(uint8_t)((az>>8)&0xFF), (uint8_t)(gx&0xFF),(uint8_t)((gx>>8)&0xFF), (uint8_t)(gy&0xFF),(uint8_t)((gy>>8)&0xFF), (uint8_t)(gz&0xFF),(uint8_t)((gz>>8)&0xFF)
    };

    radio.stopListening(); delayMicroseconds(150);
    (void)radio.write(t, PL); // reply (ACK+retries ON) delayMicroseconds(150);
    radio.startListening();
  }
}
