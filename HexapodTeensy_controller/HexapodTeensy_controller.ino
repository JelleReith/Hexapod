#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <Encoder.h>

#define X_POT A15
#define Y_POT A14
#define Z_POT A16

#define PITCH_POT A2
#define ROLL_POT A1
#define YAW_POT A3

#define ACCEL_POT A17
#define SPEED_POT A0

#define BUTTON_TOP_L 26
#define BUTTON_TOP_M 25
#define BUTTON_TOP_R 24

#define BUTTON_STICK_LEFT 2
#define BUTTON_STICK_RIGHT 1

#define ROTARY_A 3
#define ROTARY_B 4
#define BUTTON_ROTARY 5

Encoder myEnc(ROTARY_A, ROTARY_B);
long encoderValue;


#include <SPI.h>
#include <RH_NRF24.h>
#include <RHHardwareSPI.h>
RH_NRF24 nrf24(8, 9, hardware_spi);

typedef struct
{
  uint16_t xRadio;
  uint16_t yRadio;
  uint16_t zRadio;
  uint16_t pitchRadio;
  uint16_t rollRadio;
  uint16_t yawRadio;
} MyDataStruct;

MyDataStruct data;

int x, y, z, pitch, roll, yaw;

void setup() {
  Serial.begin(9600);
  Serial.println("hi");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();

  pinMode(BUTTON_TOP_L , INPUT_PULLUP);
  pinMode(BUTTON_TOP_M , INPUT_PULLUP);
  pinMode(BUTTON_TOP_R , INPUT_PULLUP);
  pinMode(BUTTON_STICK_LEFT , INPUT_PULLUP);
  pinMode(BUTTON_STICK_RIGHT , INPUT_PULLUP);
  pinMode(BUTTON_ROTARY , INPUT_PULLUP);

  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(100))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps , RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  delay(1);
}


int niceRead(int pin) {
  int value = 0;
  int returnValue = 0;
  for (int i = 0; i < 100; i++) {
    value = value + analogRead(pin);
  }
  returnValue = int(value / 100);

  return returnValue;

}
void loop() {
  encoderValue = myEnc.read();
  x = niceRead(X_POT);
  y = niceRead(Y_POT);
  z = niceRead(Z_POT);
  pitch = niceRead(PITCH_POT);
  roll = niceRead(ROLL_POT);
  yaw = niceRead(YAW_POT);

  x = map(x, 0, 1024, 1024, 0);
  y = map(y, 0, 1024, 1024, 0);
  pitch = map(pitch, 0, 1024, 1024, 0);
  roll = map(roll, 0, 1024, 1024, 0);


  int accel = niceRead(ACCEL_POT);
  int speed = niceRead(SPEED_POT);

  int button_l = digitalRead(BUTTON_TOP_L);
  int button_m = digitalRead(BUTTON_TOP_M);
  int button_r = digitalRead(BUTTON_TOP_R);
  int button_stick_l = digitalRead(BUTTON_STICK_LEFT);
  int button_stick_r = digitalRead(BUTTON_STICK_RIGHT);
  int button_rotary = digitalRead(BUTTON_ROTARY);

//    display.clearDisplay();
//    display.setTextSize(1);             // Normal 1:1 pixel scale
//    display.setTextColor(SSD1306_WHITE);        // Draw white text
//    display.setCursor(0, 0);            // Start at top-left corner
//
//    display.print("X      :");
//    display.println(x);
//
//    display.print("Y      :");
//    display.println(y);
//
//    display.print("Z      :");
//    display.println(z);
//
//    display.print("Pitch  :");
//    display.println(pitch);
//
//    display.print("Roll   :");
//    display.println(roll);
//
//    display.print("Yaw    :");
//    display.println(yaw);
//
//    display.print("Accel  :");
//    display.println(accel);
//
//    display.print("Speed  :");
//    display.println(speed);
//
//    display.setCursor(80, 0);            // Start at top-left corner
//    display.print("Knob:");
//    display.print(encoderValue);
//
//    display.setCursor(80, 8);            // Start at top-left corner
//    display.print("Knob b:");
//    display.print(button_rotary);
//
//    display.setCursor(80, 16);            // Start at top-left corner
//    display.print("Sw L  :");
//    display.print(button_l);
//
//    display.setCursor(80, 24);            // Start at top-left corner
//    display.print("Sw M  :");
//    display.print(button_m);
//
//    display.setCursor(80, 32);            // Start at top-left corner
//    display.print("Sw R  :");
//    display.print(button_r);
//
//    display.setCursor(80, 40);            // Start at top-left corner
//    display.print("B St L:");
//    display.print(button_stick_l);
//
//    display.setCursor(80, 48);            // Start at top-left corner
//    display.print("B St R:");
//    display.print(button_stick_r);
//
//
//    display.display();
  data.xRadio = x;
  data.yRadio = y;
  data.zRadio = z;
  data.pitchRadio = pitch;
  data.rollRadio = roll;
  data.yawRadio = yaw;

  //  data.xRadio = 512;
  //  data.yRadio = 512;
  //  data.zRadio = 512;
  //  data.pitchRadio = 512;
  //  data.rollRadio = 512;
  //  data.yawRadio = 512;
  nrf24.send((uint8_t*)&data, sizeof(data));
  delayMicroseconds(10);
}
