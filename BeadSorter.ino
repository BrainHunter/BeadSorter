#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

#define dirPin 2 //Stepper
#define stepPin 3 //Stepper
#define motorInterfaceType 1 //Stepper

#define stepperMaxSpeed 6000
#define stepperAccel 9000

//#define stepperMulti 100 //Stepper
#define stepperStepsPerRot 200
#define stepperMicroStepping 16
#define numContainerSlots 16    // in reality there are only 12, but 
#define stepperMulti (stepperStepsPerRot*stepperMicroStepping/numContainerSlots)

#define motorSpeed 255 //Container Motor
#define GSM2 5 // Container Motor
#define in3 7 //Container Motor
#define in4 6 //Container Motor
#define hopperMotorReverseTime  9000 //in ms - time of ~ revolution

#define setupPin 11 //Setup

#define photoSensorPin A0 //Photo Sensor
#define photoSensorThreshold 600


#define nullScanOffset 150 

#define servoAngleIn 28 //Servo
#define servoAngleOut 51
#define servoAngleWiggle 2
#define servoPin 8


// #define DEBUG_PROG 1 // Stepper test
// #define DEBUG_PROG 2 // Color Sensor test
// #define DEBUG_PRINT_PHOTOSENS


// Parameters: https://learn.adafruit.com/adafruit-color-sensors/program-it
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

//Max allowed color difference = thresholdFactor ∗ colorValue + offset (0,08/40)
//dark colors 0.02 + 10
float thresholdFactor = 0.04;
int offset = 40;

Servo servo;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

boolean autoSort = true;
unsigned int resultColor[4] = {0, 0, 0, 0};
unsigned int medianColors[4][4];
unsigned int storedColors[16][4];


boolean calibrateNullScan = true;
int nullScanValues[4] = {6618, 1860, 2282, 2116}; //adjust these if no calibration

int autoColorCounter = 0;
int beadCounter = 0;

const int dynamicContainerArraySize = 16;
int dynamicContainerArray[dynamicContainerArraySize] = { -1, 666, -1, -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666 , -1 , -1};

void setup() {
  // Setup Hopper Motor Pins 
  pinMode(GSM2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(setupPin, INPUT);
  //  pinMode(stepperTunerPin, INPUT);

  Serial.begin(9600);
  Serial.println("");
  Serial.println("BeadSorter start");

  //Serial.println("Analyzer start");
  servo.attach(servoPin);
  servo.write(servoAngleIn);

  //stepper.setMaxSpeed(4000);
  //stepper.setAcceleration(5000);
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAccel);
  stepper.setCurrentPosition(0);

#if defined DEBUG_PROG && DEBUG_PROG == 1
  while (1)
  {
      Serial.println("goto 1000");
      stepper.moveTo(1000);
      stepper.runToPosition();

    Serial.println("goto 0");
      delay(400);
      stepper.moveTo(0);
      stepper.runToPosition();

      delay(400);
  }
#endif



  if (tcs.begin()) {
    Serial.println("Sensor found");
  } else {
    Serial.println("TCS34725 not found ... exiting!");
    while (1); // Stop program
  }

#if defined DEBUG_PROG && DEBUG_PROG == 2
  servoFeedOut();
  servoFeedIn();
  while (1)
  {
    readColorSensor();
    Serial.print("Clear:"); Serial.print(resultColor[0]);
    Serial.print("\tRed:"); Serial.print(resultColor[1]);
    Serial.print("\tGreen:"); Serial.print(resultColor[2]);
    Serial.print("\tBlue:"); Serial.print(resultColor[3]);
    Serial.println("");
    delay(400);
  }
#endif

  if (!autoSort) {
    Serial.println("Import default Colors");
    importDefaultColorSet();
  }

  if (calibrateNullScan) {
    Serial.print("Calibrating: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(i);
      servoFeedIn();
      servoFeedOut();
    }
    Serial.println("");
    servoFeedIn();
    readColorSensor();
    setNullScanValues();
  }

  Serial.println("Starting continous color scan.");
  Serial.println("");
}

void loop() {
  static bool successfullBead = false;

  while(Serial.available())
  {
    int c;
    c = Serial.read();
    switch(c)
    {
      case 't':
      case 'T': printTables();
                break;
      default: break;
    }
  }




  handleHopperMotor(successfullBead);


  if (digitalRead(setupPin) == HIGH) {
    while (digitalRead(setupPin) == HIGH) {
    }
    // for manual colors, there are more restricted thresholds
    autoSort = false;
    thresholdFactor = 0.03;
    offset = 20;
    addColor();
  }

  //  if (digitalRead(stepperTunerPin) == HIGH) {
  //    while (digitalRead(stepperTunerPin) == HIGH) {
  //    }
  //    moveStepperForward();
  //  }

  servoFeedIn();
  readColorSensor();

  if (!nullScan()) {
    Serial.println("");
    Serial.print("Beads analyzed: "); Serial.println(beadCounter++);
    Serial.print("\tClear:"); Serial.print(resultColor[0]);
    Serial.print("\tRed:"); Serial.print(resultColor[1]);
    Serial.print("\tGreen:"); Serial.print(resultColor[2]);
    Serial.print("\tBlue:"); Serial.print(resultColor[3]);
    Serial.println("");
    sortBeadToDynamicArray();
    successfullBead = true;
  } else {
    Serial.print(".");
    //Serial.print("\tClear:"); Serial.print(resultColor[0]);
    //Serial.print("\tRed:"); Serial.print(resultColor[1]);
    //Serial.print("\tGreen:"); Serial.print(resultColor[2]);
    //Serial.print("\tBlue:"); Serial.print(resultColor[3]);
    //Serial.println("");
    successfullBead = false;
  }

  servoFeedOut();


}

/*
*   print all color tables to serial for debugging.
*/
void printTables(){
  char line[128];

  Serial.println("Nullscan:");
  snprintf(line, 128,"%2d c=%5u r=%5u g=%5u b=%5u", 0, nullScanValues[0],nullScanValues[1],nullScanValues[2],nullScanValues[3]);
  Serial.println(line);

  Serial.println("Stored colors:");
  for(int i = 0; i< 16; i++)
  {
    snprintf(line, 128,"%2d c=%5u r=%5u g=%5u b=%5u", i, storedColors[i][0],storedColors[i][1],storedColors[i][2],storedColors[i][3]);
    Serial.println(line);
  }

  Serial.println("Stored colors:");
  for( int i=0; i< dynamicContainerArraySize; i++)
  {
    Serial.print(dynamicContainerArray[i]);
    Serial.print(" ");
  }
  Serial.println(" ");

}


/* Logic for the hopper motor
*
*  param successfullBead
*
*/
void handleHopperMotor(bool successfullBead)
{
  static unsigned long timestamp = 0;
  static bool direction = false;  
  int photoSensor = 0;

  // reverse motor if no successfull beads after timeout
  if(!successfullBead )
  {
    if(timediff(timestamp, millis()) > hopperMotorReverseTime)
    {
      direction = !direction;
      timestamp = millis();
    }
  }
  else
  {
    timestamp = millis();
  }

  photoSensor = analogRead(photoSensorPin);
#ifdef DEBUG_PRINT_PHOTOSENS
  Serial.println("");Serial.println(photoSensor);
#endif
  if (photoSensor > photoSensorThreshold) {   //PhotoSensor detected no beads in the feeding tube
    //if (!isHopperMotorRunning()) {
      startHopperMotor(direction);
    //}
    //Serial.println("Photosensor OK");
  } else {                                    //PhotoSensor detected beads in the feeding tube --> stop the motor
    stopHopperMotor();
    Serial.print("Photosensor detected Beads ");
    Serial.println(photoSensor);
  }

}

unsigned long timediff(unsigned long t1, unsigned long t2)
{
    signed long d = (signed long)t1 - (signed long)t2;
    if(d < 0) d = -d;
    return (unsigned long) d;
}


void addColor() {
  int colorIndex;

  clearMedianColors();
  stopHopperMotor();

  Serial.print("Insert Color to register. Press button when done...");

  while (digitalRead(setupPin) == LOW) {
  }

  startHopperMotor(false);
  delay(10000);
  stopHopperMotor();
  for (int i = 0; i < 4; i++) {
    Serial.print(i + 1); Serial.print("/4: ");
    servoFeedIn();
    readColorSensor();

    addColorToMedianColors(i);

    servoFeedOut();
  }

  calcMedianAndStore();
  startHopperMotor(false);
}

void servoFeedIn() {
  int low=servoAngleIn-servoAngleWiggle;
  int high=servoAngleIn+servoAngleWiggle;

  for(int i = low ; i<high ;i++)
  {
    servo.write(i);
    delay(100);
  } 
}

void servoWiggleIn() {
  int low=servoAngleIn-servoAngleWiggle;
  int high=servoAngleIn+servoAngleWiggle;

  for(int c=0; c< 3; c++)
  {
    for(int i = high ; i>low ;i--)
    {
      servo.write(i);
      delay(100);
    } 
    for(int i = low ; i<high ;i++)
    {
      servo.write(i);
      delay(100);
    } 
  }
}

void servoFeedOut() {

  int low=servoAngleOut-servoAngleWiggle;
  int high=servoAngleOut+servoAngleWiggle;

  for(int count = 0 ; count < 3 ; count ++)
  {
    for(int i = low ; i<high ;i++)
    {
      servo.write(i);
      delay(100);
    } 
    for(int i = high ; i<low ;i--)
    {
      servo.write(i);
      delay(50);
    } 
  }
  delay(200);
}

void readColorSensor() {
  
  // Sensor returns R G B and Clear value
  uint16_t clearcol, red, green, blue;
  delay(200); // Farbmessung dauert c. 50ms
  tcs.getRawData(&red, &green, &blue, &clearcol);

  resultColor[0] = (unsigned int)clearcol;
  resultColor[1] = (unsigned int)red;
  resultColor[2] = (unsigned int)green;
  resultColor[3] = (unsigned int)blue;
}

int addColorToMedianColors(int noOfTest) {
  medianColors[ noOfTest ][ 0 ] += resultColor[0];
  medianColors[ noOfTest ][ 1 ] += resultColor[1];
  medianColors[ noOfTest ][ 2 ] += resultColor[2];
  medianColors[ noOfTest ][ 3 ] += resultColor[3];
}

int getNextFreeArrayPlace() {
  int arrayCounter = 0;
  while (
    (storedColors[ arrayCounter ][ 0 ] > 0) and
    (storedColors[ arrayCounter ][ 1 ] > 0) and
    (storedColors[ arrayCounter ][ 2 ] > 0) and
    (storedColors[ arrayCounter ][ 3 ] > 0)) {
    arrayCounter++;
  }
  return arrayCounter;
}

void clearMedianColors() {
  memset(medianColors, 0, sizeof(medianColors));
}

void calcMedianAndStore() {
  memset(resultColor, 0, sizeof(resultColor));
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      resultColor[i] += medianColors[ j ][ i ];
    }
    resultColor[i] = resultColor[i] / 4;
  }

  int nextColorNo = getNextFreeArrayPlace();

  for (int i = 0; i < 4; i++) {
    storedColors[ nextColorNo ][ i ] = resultColor[i];
  }

  Serial.print("I stored color to bank #"); Serial.print(nextColorNo);
  Serial.println("");
}

void importDefaultColorSet() {
  storeColor(0, -1, 1032, 854, 766); //Orange
  storeColor(1, -1, 1331, 1321, 918); //Bright Yellow
  storeColor(2, -1, 532, 914, 797); //Green
  storeColor(3, -1, 512, 845, 967); //Dark Blue
  storeColor(4, -1, 1257, 1151, 1129); //Rose
  storeColor(5, -1, 1265, 1215, 1100); //Skin
  storeColor(6, -1, 756, 757, 721); // Red
  storeColor(7, -1, 1525, 1858, 1735); //White
  storeColor(8, -1, 512, 742, 700); //Black
  storeColor(9, -1, 939, 1462, 1101); //Lime Green
  storeColor(10, -1, 712, 1298, 1121); //Mint Green
  storeColor(11, -1, 1213, 975, 914); //Coral
  storeColor(12, -1, 1185, 1115, 876); //Dark Yellow
}

String getColorNameFromNo(int colorNo) {
  switch (colorNo) {
    case 0: return "Orange";
    case 1: return "Bright Yellow";
    case 2: return "Green";
    case 3: return "Dark Blue";
    case 4: return "Rose";
    case 5: return "Skin";
    case 6: return "Red";
    case 7: return "White";
    case 8: return "Black";
    case 9: return "Lime Green";
    case 10: return "Mint Green";
    case 11: return "Coral";
    case 12: return "Dark Yellow";
    default: return "Unknown";
  }
}

void storeColor(int index, int Clear, int red, int green, int blue) {
  storedColors[ index ][0] = Clear; 
  storedColors[ index ][1] = red;
  storedColors[ index ][2] = green;
  storedColors[ index ][3] = blue;
  //  storedColors[ index ][4] = 1;
}

boolean nullScan() {
  if ((resultColor[0] > (nullScanValues[0] - nullScanOffset) and resultColor[0] < (nullScanValues[0] + nullScanOffset)) and
      (resultColor[1] > (nullScanValues[1] - nullScanOffset) and resultColor[1] < (nullScanValues[1] + nullScanOffset)) and
      (resultColor[2] > (nullScanValues[2] - nullScanOffset) and resultColor[2] < (nullScanValues[2] + nullScanOffset)) and
      (resultColor[3] > (nullScanValues[3] - nullScanOffset) and resultColor[3] < (nullScanValues[3] + nullScanOffset))) {
    return true;
  }
  return false;
}

void setNullScanValues() {
  nullScanValues[0] = resultColor[0];
  nullScanValues[1] = resultColor[1];
  nullScanValues[2] = resultColor[2];
  nullScanValues[3] = resultColor[3];

  Serial.print("Calibration results: "); Serial.print(nullScanValues[0]); Serial.print(" "); Serial.print(nullScanValues[1]); Serial.print(" "); Serial.print(nullScanValues[2]); Serial.print(" "); Serial.print(nullScanValues[3]); Serial.println("");
}

int findColorInStorage()
{
  int ret=-1;

  for (int i = 0; i < 16; i++) {
    threshold = thresholdFactor * resultColor[0] + offset;
    upperLimit = resultColor[0] + threshold;
    lowerLimit = resultColor[0] - threshold;
    if (storedColors[ i ][0] >= lowerLimit and storedColors[ i ][0] <= upperLimit) {
      threshold = thresholdFactor * resultColor[1] + offset;
      upperLimit = resultColor[1] + threshold;
      lowerLimit = resultColor[1] - threshold;
      //        Serial.print(i);Serial.print(":R:"); Serial.print(threshold);Serial.println("");
      //        Serial.print(i);Serial.print(":R:"); Serial.print(upperLimit);Serial.println("");
      //        Serial.print(i);Serial.print(":R:"); Serial.print(lowerLimit);Serial.println("");
      //        Serial.println(storedColors[ i][1]);
      if (storedColors[ i ][1] >= lowerLimit and storedColors[ i ][1] <= upperLimit) {
        threshold = thresholdFactor * resultColor[2] + offset;
        upperLimit = resultColor[2] + threshold;
        lowerLimit = resultColor[2] - threshold;
        //
        //            Serial.print(i);Serial.print(":G:"); Serial.print(threshold);Serial.println("");
        //            Serial.print(i);Serial.print(":G:"); Serial.print(upperLimit);Serial.println("");
        //            Serial.print(i);Serial.print(":G:"); Serial.print(lowerLimit);Serial.println("");

        if (storedColors[ i ][2] >= lowerLimit and storedColors[ i ][2] <= upperLimit) {
          threshold = thresholdFactor * resultColor[3] + offset;
          upperLimit = resultColor[3] + threshold;
          lowerLimit = resultColor[3] - threshold;
          //
          //                Serial.print(i);Serial.print(":B:"); Serial.print(threshold);Serial.println("");
          //                Serial.print(i);Serial.print(":B:"); Serial.print(upperLimit);Serial.println("");
          //                Serial.print(i);Serial.print(":B:"); Serial.print(lowerLimit);Serial.println("");

          if (storedColors[ i ][3] >= lowerLimit and storedColors[ i ][3] <= upperLimit) {
            //          Serial.print("Color is #"); Serial.print(storedColors[ i ][0]); Serial.println("");
            return i;
          }
        }
      }
    }
  }
  return ret;
}

void sortBeadToDynamicArray() {
  int threshold;
  int upperLimit;
  int lowerLimit;

  Serial.println("Analyzing Results:");

  int index=findColorInStorage();

  if(index != -1)
  {
    Serial.print("Color is #"); Serial.print(storedColors[ index ][0]); Serial.println("");
    if (autoSort) {
      Serial.print("Color is R:"); Serial.print(storedColors[ index ][1]); Serial.print(" G:"); Serial.print(storedColors[ index ][2]); Serial.print(" B:"); Serial.print(storedColors[ index ][3]);
      //            updateStoredColorCount(i);
      //            updateDetectedColorFromTempStoredColor(i);
    } else {
      Serial.print("Color is #"); Serial.print(getColorNameFromNo(index)); Serial.print(". ");
    }
    //Serial.println(storedColors[ index ][0]);

    int containerNo = getContainerNo(index);
    Serial.print("move stepper to container No:"); Serial.println(containerNo);
    moveSorterToPosition(containerNo);

  }
  else
  {
    Serial.println("not found");
    if (autoSort) {
      Serial.println("autosort!");
      if (!allContainerFull()) {
        Serial.print("not allContainerFull. StoreColor ");
        Serial.println(autoColorCounter);
        storeColor(autoColorCounter, resultColor[0] , resultColor[1], resultColor[2], resultColor[3]);
        int containerNo = getContainerNo(autoColorCounter);
        autoColorCounter++;
        moveSorterToPosition(containerNo);
      } else {
        moveSorterToPosition(dynamicContainerArraySize - 1);
      }
    } else {
      moveSorterToPosition(dynamicContainerArraySize - 1);
    }
  }
}

int getContainerNo(int colorIndex) {
  int containerNo = -1;
  int i = 0;

  for (i; i < dynamicContainerArraySize; i++) {
    //Serial.print("looking for ");Serial.print(colorIndex);Serial.print(" at position ");Serial.println(i);
    if (colorIndex == dynamicContainerArray[i]) {
      Serial.print("Color found in container array: "); Serial.println(dynamicContainerArray[i]);
      containerNo = i;
      break;
    }
  }

  if (containerNo == -1) {
    if (allContainerFull()) {
      containerNo = 15;
    } else {
      containerNo = getNextContainerNo(colorIndex);
    }
  }

  return containerNo;
}

int getNextContainerNo(int colorIndex) {
  Serial.println("finding container for new color");
  int arrayCounter = 0;

  while ((dynamicContainerArray[ arrayCounter ] > -1)) {
    arrayCounter++;
  }
  dynamicContainerArray[arrayCounter] = colorIndex;

  Serial.print("Return Color Index: "); Serial.print(colorIndex); Serial.println("");
  return arrayCounter;
}

bool allContainerFull() {
  int arrayCounter = 0;
  //Serial.println("testing all container full");

  while ((dynamicContainerArray[ arrayCounter ] > -1)) {
    arrayCounter++;
  }

  return arrayCounter >= 15; // 15 is reserved for non sortable
}

void moveSorterToPosition(int position) {
  int currentPos = stepper.currentPosition() / stepperMulti;
  int diffToPosition = position - currentPos;

  // TODO: Es wird nur sequentiell angesteuert. 
  if (diffToPosition > 8){  // motor can go in negative direction over the zero
     position = currentPos + diffToPosition - 16;
     //int diff = 16 - diffToPosition ;
     //position = -diff;
  }else if(diffToPosition < -8){  // motor can go in positive direction over the 15)
    position = currentPos + diffToPosition + 16;
  }
  position *= stepperMulti;

  stepper.moveTo(position);
  stepper.runToPosition();

  // correct stepper position into ususal range:
  stepper.setCurrentPosition( stepper.currentPosition() % (16*stepperMulti) );
}

void stopHopperMotor() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(GSM2, motorSpeed);
}

void startHopperMotor(bool dir) {
  if(dir)
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(GSM2, motorSpeed);
}

// return true if 
bool isHopperMotorRunning()
{
  return digitalRead(in3) || digitalRead(in4);
}


//void updateDetectedColorFromTempStoredColor(int i) {
//  Serial.println(""); Serial.println("Updating color.");
//  Serial.println(""); Serial.print("Red before: "); Serial.print(storedColors[ i ][1]); Serial.print(" Color Counter: "); Serial.print(storedColors[ i ][4]);
//  storedColors[ i ][1] = ((storedColors[ i ][1] * (storedColors[ i ][4] - 1)) + (resultColor[1])) / storedColors[ i ][4];
//  storedColors[ i ][2] = ((storedColors[ i ][2] * (storedColors[ i ][4] - 1)) + (resultColor[2])) / storedColors[ i ][4];
//  storedColors[ i ][3] = ((storedColors[ i ][3] * (storedColors[ i ][4] - 1)) + (resultColor[3])) / storedColors[ i ][4];
//  Serial.println(""); Serial.print("Red after: "); Serial.print(storedColors[ i ][1]);
//  Serial.println("");
//
//}
//
//void updateStoredColorCount(int i){
//  storedColors[ i ][4] += 1;
//}
