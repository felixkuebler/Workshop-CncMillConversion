#include <U8g2lib.h>
#include <SPI.h>

// reboot function
void(* reset) (void) = 0;

// GPIO to reset GRBL controller after a reset of this controller
const uint8_t grblResetPin = 17;

// GPIOs for pendant hand wheel encoder
const uint8_t encoderPinA = 2;
const uint8_t encoderPinB = 3;

// GPIOs for pendant axis selection
const uint8_t axisSelectPinX = A9;
const uint8_t axisSelectPinY = A8;
const uint8_t axisSelectPinZ = A7;
const uint8_t axisSelectPinA = A6;
const uint8_t axisSelectPinB = A5;
const uint8_t axisSelectPinC = A4;

// GPIOs for pendant input multiplier selection
const uint8_t multiplierSelectPinX1 = A3;
const uint8_t multiplierSelectPinX10 = A2;
const uint8_t multiplierSelectPinX100 = A1;

// GPIO for pendant indicator LED
const uint8_t ledPin = A0;

// GPIOs for Display SPI
const uint8_t displaySpiCs = 53;

bool jogEnable = false;
bool previousJogEnable = false;

// encodcer counters
volatile int encoderCount = 0;
int previousCount = 0;

// timer too keep track of the unlock pulse
unsigned long previousUnlockTime = 0;

// flag to indicate if a received GRBL report shall be tropped
bool forwardGrblReport = true;

// create instance for SPI LCD display
U8G2_ST7920_128X64_F_HW_SPI display(U8G2_R0, displaySpiCs);

// message buffer for serial communication to GRBL controller
char grblMsgBuffer[255] = {0};
uint8_t grblMsgBufferLength = 0;

//
const uint16_t grblMsgPollInterval = 250;
unsigned long grblMsgPollLastTime = 0;

void setup() {
  // reboot the GRBL controller
  pinMode(grblResetPin, OUTPUT);
  digitalWrite(grblResetPin, LOW);
  delay(100);
  digitalWrite(grblResetPin, HIGH);

  // setup serial connection to an external PC
  Serial.begin(115200);
  while (!Serial) {}

  // setup serial connection to GRBL controller
  Serial1.begin(115200);
  while (!Serial1) {}

  // configure pendant hand wheel endoder GPIOs
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  // configure pendant axis selection GPIOs
  pinMode(axisSelectPinX, INPUT_PULLUP);
  pinMode(axisSelectPinY, INPUT_PULLUP);
  pinMode(axisSelectPinZ, INPUT_PULLUP);
  pinMode(axisSelectPinA, INPUT_PULLUP);
  pinMode(axisSelectPinB, INPUT_PULLUP);
  pinMode(axisSelectPinC, INPUT_PULLUP);

  // configure pendant input multiplier selection GPIOs
  pinMode(multiplierSelectPinX1, INPUT_PULLUP);
  pinMode(multiplierSelectPinX10, INPUT_PULLUP);
  pinMode(multiplierSelectPinX100, INPUT_PULLUP);

  // configure pendant indicator LED GPIO
  pinMode(ledPin, OUTPUT);

  // Attach interrupt to pendant hand wheel encoder GPIO
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isr, RISING);

  // activate display
  display.begin();
}

void loop() {
  // read pendant axis selection GPIOs
  bool selectedX = digitalRead(axisSelectPinX) == LOW;
  bool selectedY = digitalRead(axisSelectPinY) == LOW;
  bool selectedZ = digitalRead(axisSelectPinZ) == LOW;
  bool selectedA = digitalRead(axisSelectPinA) == LOW;

  // jogging is enabled if any of the available axes were selected
  jogEnable = selectedX || selectedY || selectedZ || selectedA;

  // check if any axis was selected
  if (jogEnable) {
    // enable indicator ligh
    digitalWrite(ledPin, HIGH);

    // reset the encoder counter when just enabled
    if (previousJogEnable != jogEnable) {
      noInterrupts();
      encoderCount = 0;
      previousCount = 0;
      interrupts();
    }
    
    // get which multiplier was selected
    uint8_t multipier = 0;
    if (digitalRead(multiplierSelectPinX1) == LOW) {
      multipier = 1;
    }
    else if(digitalRead(multiplierSelectPinX10) == LOW) {
      multipier = 10;
    }
    else if (digitalRead(multiplierSelectPinX100) == LOW) {
      multipier = 100;
    }

    noInterrupts();
    int protectedCount = encoderCount;
    interrupts();

    // check if the current encoder position is different from the previous
    if(protectedCount != previousCount) {
      // calculate distances for the jog command
      double distance = (double)((previousCount-protectedCount) * multipier) / 100;
      double distanceX = selectedX ? distance : 0;
      double distanceY = selectedY ? distance : 0;
      double distanceZ = selectedZ ? distance : 0;
      double distanceA = selectedA ? distance : 0;

      // disable report forwarding for the responds to the jog command
      forwardGrblReport = false;

      // generate jog command and send it to GRBL controller
      String cmd = generateJogCode("G21", "G91", 100, distanceX, distanceY, distanceZ, distanceA);
      Serial1.println("~");
      Serial1.println(cmd);
      Serial1.println("?");
      Serial.println("Panent: " + cmd);
    }
    // update encoder value
    previousCount = protectedCount;

    // check if the GRBL info shall be polled
    if (millis() - grblMsgPollLastTime > grblMsgPollInterval) {
      //Serial1.println("?");
      grblMsgPollLastTime = millis();
    }
  }
  // use axis-c selsct as locking command
  else if (digitalRead(axisSelectPinC) == LOW) {
    // check if the timing since last btn press is greater then 1 senond
    if (millis()-previousUnlockTime > 1000) {
      // disable report forwarding for the responds to the unlock command
      forwardGrblReport = false;

      // enable indicator ligh
      digitalWrite(ledPin, HIGH);
      delay(10);

      // send unlock command
      Serial1.println("$X");

      previousUnlockTime = millis();
    }
    else {
      // disbale indicator light
      digitalWrite(ledPin, LOW);
    }
  }
  else {
    // check if joggin was just aborted
    if (previousJogEnable == true && jogEnable == false) {
      // send feed hold command
      Serial1.println("!");
    }

    // enable report forwarding again
    forwardGrblReport = true;
    // clear previouse lock time
    previousUnlockTime=0;
    // disbale indicator light
    digitalWrite(ledPin, LOW);
  }

  // forward serial data to grbl controller
  while(Serial.available()) { 
    Serial1.write(Serial.read());
  }

  bool grblMsgComplete = false;

  // forward grbl controller message to serial
  while(Serial1.available()) {
    char byte = Serial1.read();

    // check if the current symbol is a message complete token
    if (byte == '>') {
      grblMsgComplete = true;
    }

    if (forwardGrblReport) {
      Serial.write(byte);
    }
    grblMsgBuffer[grblMsgBufferLength++] = byte;
  }

  if (grblMsgComplete) {
    parseStatusReport(grblMsgBuffer, grblMsgBufferLength);
    grblMsgBufferLength = 0;
  }

  previousJogEnable = jogEnable;

  //delay(50);
}

String generateJogCode(String unit, String distance, int feedRate, double x, double y, double z, double a) {
  return "$J="+unit+distance+"X"+String(x)+"Y"+String(y)+"Z"+String(z)+"A"+String(a)+"F"+String(feedRate); // G21G91X10F100"
}

void parseStatusReport(char* report, uint8_t length) {

  int statusReportStartIdx = -1;
  int statusReportEndIdx = -1;

  // get start and end index of status report
  for (uint8_t i = 0; i < length; i++) {
    if (report[i] == '<') {
      statusReportStartIdx = i+1;
    }
    else if (report[i] == '>') {
      statusReportEndIdx = i-1;
    }
  }

  if (statusReportStartIdx >= 0 && statusReportEndIdx >= 0) {
    // find the state of the mashine
    // the state is located at the beginning of a report
    // we only need to find its length

    // Idle, Run, Hold, Jog, Alarm, Door, Check, Home, Sleep
    // max state length is 5, a separator thus has to be at least at idx 6
    uint8_t stateIdx = statusReportStartIdx;
    const uint8_t maxStateLength = 5;
    uint8_t stateLength = 0;

    // find end of state string by searching for '|' separator
    while (stateLength <= maxStateLength && report[stateIdx+stateLength] != '|') {
      stateLength++;
    }

    //find the mashine position
    uint8_t mposTokenIdx = stateIdx + stateLength + 1;
    const uint8_t mposTokenLength = 4;
    const char mposToken[mposTokenLength] = "MPos";
    while (mposTokenIdx <= maxStateLength && memcmp(mposToken, &report[mposTokenIdx], mposTokenLength) != 0) {
      mposTokenIdx++;
    }

    // the mpos data begins after the ':'
    uint8_t mposDataIdx = mposTokenIdx + mposTokenLength + 1;
    uint8_t maxMposDataLength = 100;
    uint8_t mposDataLength = 0;
    // find end of mpos data by searching for '|' separator
    while (mposDataLength <= maxMposDataLength && report[mposDataIdx+mposDataLength] != '|') {
      mposDataLength++;
    }

    uint8_t mposDataIdxX = mposDataIdx;
    uint8_t mposDataIdxY = 0;
    uint8_t mposDataIdxZ = 0;
    uint8_t mposDataIdxA = 0;
    // separate x,y,z position data
    for (uint8_t i=mposDataIdx; i<mposDataIdx+mposDataLength; i++) {
      if (mposDataIdxY == 0 && report[i]==',') {
        mposDataIdxY = i+1;
      }
      else if (mposDataIdxZ == 0 && report[i]==',') {
        mposDataIdxZ = i+1;
      }
      else if (mposDataIdxA == 0 && report[i]==',') {
        mposDataIdxA = i+1;
      }
    }

    display.clearBuffer();				
    display.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font

    // print state onto screen
    char stateStr[stateLength];
    memcpy(stateStr, &report[stateIdx], stateLength);
    stateStr[stateLength] = '\0';

    display.drawStr(0,10,"State: ");
    display.drawStr(8*5,10,stateStr);

    // print x position onto screen
    uint8_t xPosStrLength = mposDataIdxY-mposDataIdxX-1;
    char xPosStr[xPosStrLength];
    memcpy(xPosStr, &report[mposDataIdxX], xPosStrLength);
    xPosStr[xPosStrLength] = '\0';

    display.drawStr(0,30,"X: ");
    display.drawStr(3*8,30,xPosStr);

    // print y position onto screen
    uint8_t yPosStrLength = mposDataIdxZ-mposDataIdxY-1;
    char yPosStr[yPosStrLength];
    memcpy(yPosStr, &report[mposDataIdxY], yPosStrLength);
    yPosStr[yPosStrLength] = '\0';

    display.drawStr(0,40,"Y: ");
    display.drawStr(3*8,40,yPosStr);

    // print z position onto screen
    uint8_t zPosStrLength = mposDataIdxA-mposDataIdxZ-1;
    char zPosStr[zPosStrLength];
    memcpy(zPosStr, &report[mposDataIdxZ], zPosStrLength);
    zPosStr[zPosStrLength] = '\0';

    display.drawStr(0,50,"Z: ");
    display.drawStr(3*8,50,zPosStr);

    // print a position onto screen
    uint8_t aPosStrLength = (mposDataIdx+mposDataLength)-mposDataIdxA;
    char aPosStr[aPosStrLength];
    memcpy(aPosStr, &report[mposDataIdxA], aPosStrLength);
    aPosStr[aPosStrLength] = '\0';

    display.drawStr(0,60,"A: ");
    display.drawStr(3*8,60,aPosStr);

    display.sendBuffer();		
  }
}

void isr() {
  noInterrupts();

  if (!jogEnable) {
    interrupts();
    return;
  }

  if (digitalRead(encoderPinB) == digitalRead(encoderPinA)) {
    encoderCount--;
  } else {
    encoderCount++;
  }

  interrupts();
}