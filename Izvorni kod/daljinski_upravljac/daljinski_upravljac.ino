#include <SPI.h> 
#include <Wire.h> 
#include <nRF24L01.h> 
#include <RF24.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

RF24 radio(8, 9);
const uint8_t ADDR_CMD[5] = {'1','N','o','d','e'};
const uint8_t ADDR_TELE[5] = {'2','N','o','d','e'}; 
const uint8_t PL = 12;

#define JS_SW 2
static inline uint8_t map8u(int v){ v = map(v,0,1023,0,254); return 
(uint8_t)constrain(v,0,254); }

#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
 
unsigned long lastRxMs = 0;

void setup() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH); 
  pinMode(JS_SW, INPUT_PULLUP);

  // OLED 
  Wire.begin();
  Wire.setClock(100000); 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.clearDisplay(); 
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0,0); 
  display.println("RC + Telemetry"); 
  display.display();

  // Radio 
  radio.begin(); 
  radio.flush_rx(); 
  radio.flush_tx();
  radio.setAddressWidth(5); 
  radio.setChannel(100); 
  radio.setDataRate(RF24_2MBPS); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.setCRCLength(RF24_CRC_8); 
  radio.setAutoAck(true); 
  radio.disableDynamicPayloads(); 
  radio.setPayloadSize(PL);
  radio.setRetries(3,3); // kratko; komanda je NO-ACK

  radio.openWritingPipe(ADDR_CMD); // komande 
  radio.openReadingPipe(0, ADDR_TELE); // telemetrija

  Serial.begin(115200);
  Serial.println(F("[TX] ch=100 2Mbps PA=MIN PL=12 (near-field)"));

  /*
  // --- ZA VEĆU UDALJENOST ---
  // radio.setChannel(40);
  // radio.setDataRate(RF24_1MBPS);
  // radio.setPALevel(RF24_PA_LOW); // ili RF24_PA_HIGH
  */
 
}

void loop() {
  // 1) Slanje 12B NO-ACK komande: [X,Y,SW, 9×0] uint8_t pkt[PL] = {0};
  pkt[0] = map8u(analogRead(A0)); pkt[1] = map8u(analogRead(A1));
  pkt[2] = (digitalRead(JS_SW)==LOW)?1:0;

  radio.stopListening();
  (void)radio.write(pkt, PL, true); // true = NO-ACK (brz preklop) 
  radio.startListening();

  // 2) Čekanje telemetrije do ~20 ms (12B)
  bool got=false; int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0; uint32_t t0 = micros();
  while (micros()-t0 < 20000UL){ 
    if (radio.available()){
    uint8_t t[PL]; radio.read(t, PL); ax = (int16_t)(t[0] | (t[1]<<8));
    ay = (int16_t)(t[2] | (t[3]<<8));
    az = (int16_t)(t[4] | (t[5]<<8));
    gx = (int16_t)(t[6] | (t[7]<<8));
    gy = (int16_t)(t[8] | (t[9]<<8));
    gz = (int16_t)(t[10] | (t[11]<<8)); 
    got = true; lastRxMs = millis(); 
    break;
    }
  }

  // 3) OLED UI
  display.clearDisplay(); 
  display.setCursor(0,0); 
  display.println("RC + Telemetry"); 
  display.print("X:"); 
  display.print(pkt[0]);
  display.print(" Y:"); 
  display.print(pkt[1]); 
  display.print(" SW:"); 
  display.println((int)pkt[2]); 
  display.print("TELE: "); 
  display.println(got ? "GOT" :
  (millis()-lastRxMs>1000?"NO (1s+)":"NO")); 
  if (got){
    display.print("AX:"); 
    display.print(ax); 
    display.print(" AY:"); 
    display.print(ay); 
    display.print(" AZ:"); 
    display.println(az); 
    display.print("GX:"); 
    display.print(gx); 
    display.print(" GY:"); 
    display.print(gy); 
    display.print(" GZ:"); 
    display.println(gz);
  }

  display.display();

  delay(110);
}

