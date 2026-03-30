#include "Sensor.h"

//..................................................................................................................Button Logic variables and pins
#include "Button.h"

//..................................................................................................................Sensor definations and declarations
#include <algorithm>



//..................................................................................................................PWM Channel and Frequency Configuration
#define PWM_FREQ 5000
#define PWM_RES 10
#define PWM_CHANNEL_E1 7
#define PWM_CHANNEL_E2 0
// --- Motor Pin Definitions ---
#define AIN1 45
#define AIN2 48
#define PWMA 47

#define BIN1 38
#define BIN2 39
#define PWMB 40


//............................................................................................................Oled Display Variables and fonts
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// source   https://github.com/robjen/GFX_fonts/tree/master/GFX_fonts
#include "Fonts/Font5x5Fixed.h"
#include "Fonts/Font4x5Fixed.h"
#include "Fonts/Font5x7Fixed.h"
#include "Fonts/Font5x7FixedMono.h"
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define WHITE SSD1306_WHITE
#define BLACK SSD1306_BLACK

// I2C Pins for ESP32
#define OLED_SDA 20
#define OLED_SCL 21



//.................................................................................................................Lawaris
// const uint8_t sensorCount = 16;
int PositionMultiplyer[sensorCount - 1] = { -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7 };
float PositionMultiplyer14[sensorCount - 2] = { -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5 };

int DryRun[400];
bool oreantation = true;
bool raceCompleted = false;
int netAccumulator = 0;
bool overFlowDirection = 0;
bool overFlow = 0;
unsigned long lastBreaksOn = 0;
unsigned long leftLastOn = 0;
unsigned long rightLastOn = 0;
unsigned long lastEdgeUpdateTime = 0;
bool edgingCooldown = true;
bool stopCondition = false;
unsigned long stopConditionTimer = 0;
bool stopConditionTimerStarted = false;
int theCollector = 0;
bool stillOnWhite = false;
int LowerThreshold = 300;




struct DirectionLibrary {
  unsigned long leftEdge[3] = { 0 };
  unsigned long rightEdge[3] = { 0 };

  bool leftHigh = false;
  bool rightHigh = false;
  bool timerStarted = false;

  unsigned long leftTriggerTime = 0;
  unsigned long rightTriggerTime = 0;

  unsigned long caseThreshold = 40;       // ms
  unsigned long decisionThreshold = 80;  // ms
  uint8_t edgingCooldown = 750;
  unsigned long lastEdged = 0;
  unsigned long finalTimer = 0;
  int directionFlag = 0;  // 0=none, 1=left, 2=right
  uint8_t currentPrediction = 0;
};
DirectionLibrary DL;  // single instance




struct modeProfile {
  int maxSpeed;
  int baseSpeed;

  int8_t modeNo;
  int8_t activeSelect;
  int8_t noOfElements;
  int8_t currentSlide;
  int8_t currentSelect;
  const char* elementNames[5];  // Store up to 5 element names
};




modeProfile emberIITB = {
  550, 300,                                  // PID and Stats
  0,                                         // modeNo
  0,                                         // ActiveSelect
  4,                                         // number of UI elements
  0,                                         // currentSlide
  0,                                         // currentSelect
  { "START", "SETTING", "IITB", "C-VALUE" }  // names of UI parameters
};

modeProfile onyxSpeed = {
  500, 400,                                  // PID and Stats
  1,                                          // modeNo
  0,                                          // Active Select
  4,                                          // number of UI elements
  0,                                          // current slide
  0,                                          // currentSelect;
  { "START", "SETTING", "SPEED", "C-VALUE" }  // names of UI parameters
};



struct PID {
  int P;
  int D;
  int I;
  int error;
  int previousError;
  float Kp;
  float Kd;
  float Ki;
  float retention;
  bool retentionAllowed;
  bool isOnWhite;
  int edgeCase;
  int totalWeightThreshold;
  unsigned long previousTime;
};
PID pid = {
  0,
  0,
  0,
  0,
  0,
  0.10,//0.115
  0.005,
  0.0,
  0.55,
  false,
  false,
  2500,
  2000,
  0
};



// Modes and Ui
int8_t currentMode = -1;


//dryrun parameters
bool isDryRunDone = false;



//.............................................................................................................................SETUP............................
void setup() {
  //Sensor initialisation
  initSensors();

  // Motor direction pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttachChannel(PWMA, PWM_FREQ, PWM_RES, PWM_CHANNEL_E1);
  ledcAttachChannel(PWMB, PWM_FREQ, PWM_RES, PWM_CHANNEL_E2);


  Serial.begin(115200);

  // --- Setup buttons ---
  initButtons();

  // --- Start I2C and OLED ---
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  oled.clearDisplay();
  oled.display();


  // add your lock screen image or text here
  printLockScreen();

  while (buttonCheck() == -1) {
    delay(50);
  }

  modeSelectPage();


  delay(200);
}



//.........................................................................................................................................................LOOP
void loop() {
  // int result = buttonCheck();
  // printButton(result);
  // printCalibratedSensorValues();
  if (currentMode == 0)
    pressControl(&emberIITB);
  else if (currentMode == 1)
    pressControl(&onyxSpeed);

  delay(50);
}



void pressControl(modeProfile* profile) {
  // Serial.println("came to the mode select");
  oled.clearDisplay();
  // oled.drawRect(0, 0, 128, 64, WHITE);
  oled.fillRect(0, 0, 128, 64, WHITE);
  // oled.drawRoundRect(3, 3, 100, 58, 2, BLACK);
  oled.fillRoundRect(2, 2, 84, 60, 3, BLACK);


  oled.setTextColor(BLACK);
  oled.setFont(&Font5x5Fixed);
  char buffer[10];  // Buffer to store formatted float values

  oled.setCursor(89, 7);
  oled.setTextColor(BLACK);
  oled.setTextSize(1);

  dtostrf(pid.Kp, 0, 4, buffer);  // Format Kp with 3 decimal places
  oled.print("KP=");
  oled.print(buffer);

  oled.setCursor(89, 15);
  dtostrf(pid.Kd, 0, 4, buffer);  // Format Kd with 3 decimal places
  oled.print("KD=");
  oled.print(buffer);

  oled.setCursor(89, 23);
  dtostrf(pid.Ki, 0, 4, buffer);  // Format Ki with 3 decimal places
  oled.print("KI=");
  oled.print(buffer);

  oled.setCursor(89, 31);
  oled.print("BS=");
  oled.print(profile->baseSpeed);

  oled.setCursor(89, 39);
  oled.print("MS=");
  oled.print(profile->maxSpeed);


  oled.setTextColor(WHITE);
  oled.setFont(&FreeMonoBold9pt7b);
  if (profile->currentSlide == 0) {
    oled.setCursor(5, 17);
    oled.print(profile->elementNames[0]);
    oled.setCursor(5, 36);
    oled.print(profile->elementNames[1]);
    oled.setCursor(5, 55);
    oled.print(profile->elementNames[2]);
  } else {
    oled.setCursor(5, 17);
    oled.print(profile->elementNames[1]);
    oled.setCursor(5, 36);
    oled.print(profile->elementNames[2]);
    oled.setCursor(5, 55);
    oled.print(profile->elementNames[3]);
  }

  if (profile->currentSelect == 0)
    oled.drawRoundRect(3, 4, 81, 18, 3, WHITE);
  else if (profile->currentSelect == 1)
    oled.drawRoundRect(3, 23, 81, 18, 3, WHITE);
  else
    oled.drawRoundRect(3, 42, 81, 18, 3, WHITE);

  oled.display();


  int8_t buttonStatus = buttonCheck();
  if (buttonStatus == -1) return;
  else if (buttonStatus == 0) {
    profile->activeSelect--;
    profile->currentSelect--;
  } else if (buttonStatus == 2) {
    profile->activeSelect++;
    profile->currentSelect++;
  } else if (buttonStatus == 1) {
    if (profile->activeSelect == 0) {
      if (profile->modeNo == 0)
        emberIITBmodeStartPage();
      else if (profile->modeNo == 1){
        stopConditionTimerStarted = false;
        speedModeStartPage();
      }
    } else if (profile->activeSelect == 2)
      modeSelectPage();

    profile->currentSelect = 0;
    profile->activeSelect = 0;
    profile->currentSlide = 0;
    return;
  } else {
    calibrationPage();
  }

  if (profile->currentSelect < 0 && profile->activeSelect < 0) {
    profile->currentSelect = 2;
    profile->activeSelect = profile->noOfElements - 1;
    profile->currentSlide = profile->noOfElements - 3;
  }
  if (profile->currentSelect > 2 && profile->activeSelect > (profile->noOfElements - 1)) {
    profile->currentSelect = 0;
    profile->activeSelect = 0;
    profile->currentSlide = 0;
  }
  if (profile->currentSelect < 0) {
    profile->currentSelect = 0;
    // profile->activeSelect--;
    profile->currentSlide--;
  }
  if (profile->currentSelect > 2) {
    profile->currentSelect = 2;
    // profile->activeSelect++;
    profile->currentSlide++;
  }
  if (profile->currentSlide < 0)
    profile->currentSlide = 0;
  else if (profile->currentSlide > (profile->noOfElements - 3))
    profile->currentSlide = max(0, profile->noOfElements - 3);
}





//..................................................................................................................Lock Screen Display Function
void printLockScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(2);
  oled.setCursor(10, 5);
  oled.println("HAWKEYE!");

  oled.setTextSize(1);
  oled.setCursor(15, 28);
  oled.println("Robotics Team");
  // oled.setCursor(15, 40);
  // oled.println("JUNGLE :)");

  oled.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  oled.display();
}




void modeSelectPage() {
  // Mode select layout design
  auto displayLayout = []() {
    oled.clearDisplay();  // Clear screen before drawing

    // Outline
    oled.drawRect(0, 0, 128, 64, WHITE);
    oled.drawRoundRect(0, 0, 128, 64, 4, WHITE);

    oled.setTextColor(WHITE);  // Default text color
    oled.setFont(&Font5x7FixedMono);
    oled.setCursor(25, 10);
    oled.print("Select Mode :");

    // Decorative line
    oled.drawLine(24, 12, 100, 12, WHITE);

    // Mode boxes (outer white)
    oled.fillRoundRect(3, 16, 39, 45, 3, WHITE);
    oled.fillRoundRect(44, 16, 40, 45, 3, WHITE);
    oled.fillRoundRect(86, 16, 39, 45, 3, WHITE);

    // Inner black boxes
    oled.fillRoundRect(5, 18, 35, 28, 2, BLACK);
    oled.fillRoundRect(46, 18, 36, 28, 2, BLACK);
    oled.fillRoundRect(88, 18, 35, 28, 2, BLACK);

    // Text inside boxes (black)
    oled.setTextColor(BLACK);
    oled.setCursor(10, 56);
    oled.print("iitb");
    oled.setCursor(49, 56);
    oled.print("Speed");
    oled.setCursor(95, 56);
    oled.print("TBM");

    oled.display();  // Update screen
  };

  // Setting up the initial mode
  while (true) {
    displayLayout();  // Call the lambda

    uint8_t temp = buttonCheck();
    if (temp != -1) {
      if (temp == 0 || temp == 3) {
        currentMode = 0;
        break;
      } else if (temp == 1 || temp == 4) {
        currentMode = 1;
        break;
      } else continue;
    }

    delay(50);
  }
}




void calibrationPage() {
  // Reinitialize min and max collectors to defaults for a fresh calibration
  for (int i = 0; i < sensorCount; i++) {
    sensorCal.maxCollector[i] = 4095;  // Bright baseline starts at 0
    sensorCal.minCollector[i] = 0;     // Dark baseline starts at max ADC
  }
  auto PrintCurrentCalibrationStatus = [](float multiplyer) {
    oled.fillRoundRect(15, 10, 98, 44, 3, BLACK);
    oled.drawRoundRect(16, 11, 96, 42, 3, WHITE);
    oled.setTextColor(WHITE);  // Default text color
    oled.setFont(&Font5x7FixedMono);
    oled.setCursor(25, 23);
    oled.print("Calibrating!!");
    oled.drawLine(24, 25, 100, 25, WHITE);
    oled.fillRoundRect(18, 39, 92, 12, 2, BLACK);
    oled.drawRoundRect(18, 39, 92, 12, 2, WHITE);

    //VISUALISING HOW MUCH CALIBRATION IS DONE
    oled.fillRoundRect(20, 41, (int)(88 * multiplyer), 8, 1, WHITE);
  };
  unsigned long startTime = millis();
  unsigned long lastPrintedTime = millis();
  while ((millis() - startTime) < sensorCal.calibrationDuration) {
    sensorCalibrate();
    if (millis() - lastPrintedTime > 50) {
      lastPrintedTime = millis();
      PrintCurrentCalibrationStatus((float)(millis() - startTime) / sensorCal.calibrationDuration);
      oled.display();
    }
  }
  // --- Post-calibration correction for untouched sensors ---
  float avgMax = 0, avgMin = 0;
  int validMaxCount = 0, validMinCount = 0;

  // Step 1: Calculate averages for valid entries
  for (int i = 0; i < sensorCount; i++) {
    if (sensorCal.maxCollector[i] < 4094) {  // skip untouched (still 4095)
      avgMax += sensorCal.maxCollector[i];
      validMaxCount++;
    }
    if (sensorCal.minCollector[i] > 1) {  // skip untouched (still 0)
      avgMin += sensorCal.minCollector[i];
      validMinCount++;
    }
  }

  if (validMaxCount > 0) avgMax /= validMaxCount;
  if (validMinCount > 0) avgMin /= validMinCount;

  // Step 2: Replace untouched sensors with average
  for (int i = 0; i < sensorCount; i++) {
    if (sensorCal.maxCollector[i] >= 4094) {  // still default
      sensorCal.maxCollector[i] = avgMax;
    }
    if (sensorCal.minCollector[i] <= 1) {  // still default
      sensorCal.minCollector[i] = avgMin;
    }
  }



  // Debug print
  Serial.println("Calibration Done");
  Serial.println("Min Values:");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print((int)sensorCal.minCollector[i]);
    Serial.print("\t");
  }
  Serial.println("\nMax Values:");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print((int)sensorCal.maxCollector[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(200);
}




void speedModeStartPage() {
  int8_t buttonStatus = -1;
  while ((buttonStatus != 1) && (buttonStatus != 4)) {
    buttonStatus = buttonCheck();
    oled.clearDisplay();
    oled.drawRect(0, 0, 128, 64, WHITE);
    oled.drawRoundRect(42, 8, 44, 15, 3, WHITE);
    oled.drawRoundRect(41, 7, 46, 17, 3, WHITE);
    oled.setTextColor(WHITE);
    oled.setCursor(50, 18);
    oled.setFont(&Font5x7Fixed);
    oled.print("START");
    printGraph();
    oled.display();
    delay(50);
  }
  if (buttonStatus == 1) {
    while (buttonStatus != 4) {
      updateIRValues();
      findError01();
      if(pid.isOnWhite){
        if((millis()-lastBreaksOn>650)&&!stillOnWhite){
          stillOnWhite = true;
          lastBreaksOn = millis();
          theUltimateBreak();
          delay(175);
        }
        runForword(0, onyxSpeed.maxSpeed);
      }
      else{
        stillOnWhite = false;
        runForword(onyxSpeed.baseSpeed, onyxSpeed.maxSpeed);
      }
      if (buttonStatus == 0)
      {
        //overFlow = !overFlow;
        overFlowDirection = 0;
      }
      if (buttonStatus == 2)
      {
        //overFlow = !overFlow;
        overFlowDirection = 1;
      }
      if (buttonStatus == 3)
      {
        //overFlow = !overFlow;
        overFlow = !overFlow;
      }
      if(buttonStatus == 5){
        edgingCooldown = !edgingCooldown;
      }
      // if(!stopConditionTimerStarted && stopCondition){
      //   stopConditionTimerStarted = !stopConditionTimerStarted;
      //   stopConditionTimer = millis();
      //   stopCondition = false;
      // }
      // if(stopConditionTimerStarted){
      //   if(millis()- stopConditionTimer > 100){
      //     theCollector = 0;
      //     for(int i = 0; i<14; i++){
      //       if(i<7)
      //         theCollector += sensorCal.calibratedValues[i];
      //       else
      //         theCollector += sensorCal.calibratedValues[i+2];
      //     }
      //     if(theCollector >= 12500){
      //       buttonStatus = buttonCheck();
      //       delay(100);
      //       theUltimateBreak();
      //       delay(150);
      //       stopMotion();
      //       while((buttonStatus!=1)&&(buttonStatus!=4)){
      //         buttonStatus = buttonCheck();
      //         stopMotion();
      //         // delay(20);
      //       }
      //       if(buttonStatus == 4){
      //         stopConditionTimerStarted = false;
      //         break;
      //       }
      //       else{
      //         stopConditionTimerStarted = false;
      //       }
      //     }
      //   }
      // }
      buttonStatus = buttonCheck();
    }
    stopMotion();
    delay(100);
  }

}




void theUltimateBreak(){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // --- Apply to motors ---
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  // delay(50);

  ledcWrite(PWMA, 280);
  ledcWrite(PWMB, 280);
}




void runForword(int minSpeed, int maxSpeed) {
  pid.P = pid.Kp*pid.error;
  unsigned long currentTime = micros();
  float dt = (currentTime - pid.previousTime) * 1e-6;

  // clamp dt (critical)
  if (dt < 0.001) dt = 0.001;
  if (dt > 0.02) dt = 0.02;

  float rawD = (pid.error - pid.previousError) / dt;

  // smooth derivative (low-pass filter)
  pid.D = 0.7 * pid.D + 0.3 * (pid.Kd * rawD);

  // optional safety clamp (prevents crazy spikes)
  if (pid.D > 300) pid.D = 300;
  if (pid.D < -300) pid.D = -300;

  pid.previousError = pid.error;
  pid.previousTime = currentTime;

  int eLeft = minSpeed - pid.P - pid.D;
  int eRight = minSpeed + pid.P + pid.D;

  bool a = true, b = true;
  if (eLeft < 0) {
    a = false;
    eLeft = -eLeft;
  }
  if (eLeft > maxSpeed)
    eLeft = maxSpeed;

  if (eRight < 0) {
    b = false;
    eRight = -eRight;
  }
  if (eRight > maxSpeed)
    eRight = maxSpeed;

  // --- Apply to motors ---
  digitalWrite(AIN1, !(a));
  digitalWrite(AIN2, a);
  // --- Apply to motors ---
  digitalWrite(BIN1, !(b));
  digitalWrite(BIN2, b);

  ledcWrite(PWMA, eLeft);
  ledcWrite(PWMB, eRight);
}



void findError01() {
  int totalWeight = 0;
  int collectiveWeight = 0;
  pid.isOnWhite = false;
  pid.error = 0;


  int filtered[16];

for(int i = 0; i < sensorCount; i++){
    if(sensorCal.calibratedValues[i] > LowerThreshold)
        filtered[i] = sensorCal.calibratedValues[i];
    else
        filtered[i] = 0;
}


if(!overFlow){
    for (int i = 0; i < 7; i++) {
        collectiveWeight += (PositionMultiplyer14[6 - i] * filtered[6 - i]) +
                            (PositionMultiplyer14[7 + i] * filtered[9 + i]);

        totalWeight += filtered[6 - i] + filtered[9 + i];

        if (totalWeight > pid.totalWeightThreshold)
            break;
    }
}

else if(!overFlowDirection){
    for(int i = 0; i < sensorCount - 2; i++){
        if(i < 7){
            collectiveWeight += PositionMultiplyer14[i] * filtered[i];
            totalWeight += filtered[i];
        }
        if(i > 6){
            collectiveWeight += PositionMultiplyer14[i] * filtered[i + 2];
            totalWeight += filtered[i + 2];
        }
        if (totalWeight > pid.totalWeightThreshold)
            break;
    }
}

else{
    for(int i = 13; i >= 0; i--){
        if(i < 7){
            collectiveWeight += PositionMultiplyer14[i] * filtered[i];
            totalWeight += filtered[i];
        }
        if(i > 6){
            collectiveWeight += PositionMultiplyer14[i] * filtered[i + 2];
            totalWeight += filtered[i + 2];
        }
        if (totalWeight > pid.totalWeightThreshold)
            break;
    }
}



  //stopConditionFinder;
  // if(!stopConditionTimerStarted){
  //   theCollector = 0;
  //   for(int i = 0; i<14; i++){
  //     if(i<7)
  //       theCollector += sensorCal.calibratedValues[i];
  //     else
  //       theCollector += sensorCal.calibratedValues[i+2];

  //     if(theCollector >= 13500)
  //       stopCondition = true;
  //     else
  //       stopCondition = false;
  //   }
  // }

  if(edgingCooldown){
    if (sensorCal.calibratedValues[0] > 850){
      if(millis()-lastEdgeUpdateTime>150){
        pid.edgeCase = -3200;
        lastEdgeUpdateTime = millis();
      }
    }
    else if (sensorCal.calibratedValues[sensorCount-1] > 850){
      if(millis()-lastEdgeUpdateTime>150){
        pid.edgeCase = 3200;
        lastEdgeUpdateTime = millis();
      }
    }
  }
  else{
    if (sensorCal.calibratedValues[0] > 850)
      pid.edgeCase = -1800;
    else if (sensorCal.calibratedValues[sensorCount-1] > 850)
      pid.edgeCase = 1800;
  }

  if (totalWeight < 900) {
    pid.error = pid.edgeCase;
    pid.isOnWhite = true; 
  }
  else
    pid.error = (collectiveWeight*1000)/totalWeight;
  // Serial.println(pid.error);
  // Serial.println(pid.isOnWhite);
}



void emberIITBmodeStartPage() {
  // Serial.println("emberIITBmodeStartPage");
  int8_t buttonStatus = -1;
  while ((buttonStatus != 0) && (buttonStatus != 4) && (buttonStatus != 2)) {
    buttonStatus = buttonCheck();
    oled.clearDisplay();
    oled.drawRect(0, 0, 128, 64, WHITE);
    oled.drawRoundRect(12, 10, 44, 15, 3, WHITE);
    oled.drawRoundRect(11, 9, 46, 17, 3, WHITE);
    oled.drawRoundRect(72, 10, 44, 15, 3, WHITE);
    oled.drawRoundRect(71, 9, 46, 17, 3, WHITE);
    oled.setTextColor(WHITE);
    oled.setCursor(19, 20);
    oled.setFont(&Font5x7Fixed);
    oled.print("START");
    oled.setCursor(79, 20);
    oled.print("TRACE");
    printGraph();
    oled.display();
    delay(50);
  }
  if (buttonStatus == 2) {
    // dryRunPage();
    Serial.println("Celibrated");
  }
}



void stopMotion(){
  // Serial.print("Motor Stopped");
  // --- Apply to motors ---
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  // --- Apply to motors ---
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  // delay(50);

  // ledcWrite(PWMA, 0);
  // ledcWrite(PWMB, 0);
  // delay(5000);
}



void printGraph() {
  updateIRValues();
  uint8_t individualHeight = 10;
  uint8_t startYposition = 38;
  uint8_t startXposition = 5;
  // auto uint8_t Xposition = [](uint8_t input, uint8_t height){

  // }
  for (uint8_t i = 0; i < sensorCount; i++) {
    // uint8_t Xposition = getXposition(sensorCal.calibratedValues[i], individualHeight)
    int8_t Yposition = individualHeight * (1 - (sensorCal.calibratedValues[i] / 1000.0f));
    int8_t Xposition = (i > 7) ? (i - 1) * 8 : i * 8;
    int8_t finalHeight = individualHeight - Yposition;
    if (i == 7)
      Yposition -= (individualHeight / 2) + 2;
    if (i == 8)
      Yposition += (individualHeight / 2) + 3;
    oled.fillRoundRect(startXposition + Xposition, startYposition + Yposition, 6, finalHeight + 3, 2, WHITE);
  }
}



void printCalibratedSensorValues() {
  for (int i = 0; i < sensorCount; i++) {
    Serial.print((1000-readCalibrated(i)));
    Serial.print("\t");
  }
  Serial.println();
}



int edging() {
  unsigned long currentTime = millis();

  // --- 1. Update timestamps when edge sensors go above 800 ---
  for (int i = 0; i < 3; i++) {
    if (sensorCal.calibratedValues[i] > 800)
      DL.leftEdge[i] = currentTime;

    if (sensorCal.calibratedValues[13 + i] > 800)
      DL.rightEdge[i] = currentTime;
  }

  // --- 2. Calculate max time difference for each side ---
  auto getMin = [](unsigned long arr[3]) {
    unsigned long minT = arr[0];
    for (int i = 1; i < 3; i++) {
      if (arr[i] < minT) minT = arr[i];
    }
    return minT;
  };

  unsigned long leftMin = getMin(DL.leftEdge);
  unsigned long rightMin = getMin(DL.rightEdge);

  if (!DL.timerStarted) {
    // --- 3. Determine if edges are "High" ---
    if (currentTime - leftMin < DL.caseThreshold) {
      // if (!DL.leftHigh) DL.leftTriggerTime = currentTime;
      DL.leftHigh = true;
      DL.currentPrediction = 1;
      DL.finalTimer = currentTime;
      DL.timerStarted = true;
      // Serial.println("LEFT High");
    }

    if (currentTime - rightMin < DL.caseThreshold) {
      // if (!DL.rightHigh) DL.rightTriggerTime = currentTime;
      DL.rightHigh = true;
      DL.currentPrediction = 3;
      DL.finalTimer = currentTime;
      DL.timerStarted = true;
      // Serial.println("RIGHT High");
    }
  }

  // --- 4. Make decision (non-blocking) ---
  else {
    DL.directionFlag = 0;  // default: no decision yet
    if (currentTime - DL.finalTimer < DL.decisionThreshold) {

      if (DL.currentPrediction == 1) {
        if (currentTime - rightMin < DL.caseThreshold) {
          DL.currentPrediction = 0;
          DL.timerStarted = false;
          DL.rightHigh = false;
          DL.leftHigh = false;
          DL.directionFlag = 2;
          // Serial.println(DL.directionFlag);
          DL.lastEdged = currentTime;
          return DL.directionFlag;
        }
      }
      if (DL.currentPrediction == 3) {
        if (currentTime - leftMin < DL.caseThreshold) {
          DL.currentPrediction = 0;
          DL.timerStarted = false;
          DL.rightHigh = false;
          DL.leftHigh = false;
          DL.directionFlag = 2;
          // Serial.println(DL.directionFlag);
          DL.lastEdged = currentTime;
          return DL.directionFlag;
        }
      }
    } else {
      DL.directionFlag = DL.currentPrediction;
      DL.currentPrediction = 0;
      DL.timerStarted = false;
      DL.rightHigh = false;
      DL.leftHigh = false;
      DL.lastEdged = currentTime;
      // Serial.println(DL.directionFlag);
      return DL.directionFlag;
    }
  }
  return 0;
}


