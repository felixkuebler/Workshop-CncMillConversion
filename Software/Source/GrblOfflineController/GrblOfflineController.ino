#include <SoftwareSerial.h>

SoftwareSerial mySerial(8,9); //RX, TX

const uint8_t encoderPinA = 2;
const uint8_t encoderPinB = 3;

const uint8_t axisSelectPinX = 7;
const uint8_t axisSelectPinY = 6;
const uint8_t axisSelectPinZ = 5;
const uint8_t axisSelectPinA = 4;
const uint8_t axisSelectPinB = A3;
const uint8_t axisSelectPinC = A2;

const uint8_t multiplierSelectPinX1 = A0;
const uint8_t multiplierSelectPinX10 = 13;
const uint8_t multiplierSelectPinX100 = A1;

bool jogEnable = false;
bool previousJogEnable = false;

volatile int encoderCount = 0;
int previousCount = 0;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  pinMode(axisSelectPinX, INPUT);
  pinMode(axisSelectPinY, INPUT);
  pinMode(axisSelectPinZ, INPUT);
  pinMode(axisSelectPinA, INPUT);
  pinMode(axisSelectPinB, INPUT);
  pinMode(axisSelectPinC, INPUT);

  pinMode(multiplierSelectPinX1, INPUT);
  pinMode(multiplierSelectPinX10, INPUT);
  pinMode(multiplierSelectPinX100, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), isr, RISING);
}

void loop() {

  bool selectedX = digitalRead(axisSelectPinX);
  bool selectedY = digitalRead(axisSelectPinY);
  bool selectedZ = digitalRead(axisSelectPinZ);

  jogEnable = selectedX || selectedY || selectedZ;

  // check if any axis was selected
  if (jogEnable) {
    // reset the encoder counter when just enabled
    if (previousJogEnable != jogEnable) {
      noInterrupts();
      encoderCount = 0;
      previousCount = 0;
      interrupts();
    }

    // get which multiplier was selected
    uint8_t multipier = 0;
    if (digitalRead(multiplierSelectPinX1)) {
      multipier = 1;
    }
    else if(digitalRead(multiplierSelectPinX10)) {
      multipier = 10;
    }
    else if (digitalRead(multiplierSelectPinX100)) {
      multipier = 100;
    }

    noInterrupts();
    int protectedCount = encoderCount;
    interrupts();

    if(protectedCount != previousCount) {

      double distance = (double)((previousCount-protectedCount) * multipier) / 100;

              Serial.println((previousCount-protectedCount) * multipier);


      double distanceX = selectedX ? distance : 0;
      double distanceY = selectedY ? distance : 0;
      double distanceZ = selectedZ ? distance : 0;

      String cmd = generateJogCode("G21", "G91", distanceX, distanceY, distanceZ, 100);
      SendCommand(cmd, "Encoder");
    }
    previousCount = protectedCount;

    if(Serial.available() > 0) { 
      String msg = Serial.readString();
      if(msg[0] == '<') {
        String cmd = msg.substring(1);
        SendCommand(cmd,"Manuell");
      } else {
        Serial.println("< fehlt");
      }
    }
  }

  previousJogEnable = jogEnable;

  delay(50);
}

String generateJogCode(String unit, String distance, double x, double y, double z, int feedRate) {
  return "$J="+unit+distance+"X"+String(x)+"Y"+String(y)+"Z"+String(z)+"F"+String(feedRate); // G21G91X10F100"
}

void SendCommand(String cmd, String src) {
  mySerial.println(cmd);
  Serial.println(src+": "+cmd);
}

void isr() {
  if (!jogEnable) {
    return;
  }

  if (digitalRead(encoderPinB) == digitalRead(encoderPinA)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}