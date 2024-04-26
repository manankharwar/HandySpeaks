#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define MAX_DATA_POINTS 400  
float gx[MAX_DATA_POINTS];
float gy[MAX_DATA_POINTS];
float gz[MAX_DATA_POINTS];
int dataCount = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

#define INTERRUPT_PIN 2
#define BUTTON_PIN 4
#define LED_PIN 13 

int buttonState = 0;     // variable to store the current state of the button
int lastButtonState = 0;  // variable to store the previous button state
int ledState = LOW;
bool dataCollectionActive = false;  // flag to indicate whether data collection is active
int matches = 0;
int thresholdMatches = 5;

float meangx, variancegx, stdDevgx, mediangx, modegx, skewnessgx, energygx;
int rangegx, iqrgx;

float meangy, variancegy, stdDevgy, mediangy, modegy, skewnessgy, energygy;
int rangegy, iqrgy;

float meangz, variancegz, stdDevgz, mediangz, modegz, skewnessgz, energygz;
int rangegz, iqrgz;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const float alpha = 0.9; // Adjust this value between 0 and 1 (higher values increase smoothing but introduce more delay)
// Previous filtered values
float filteredRoll = 0.0;
float filteredPitch = 0.0;
float filteredYaw = 0.0;

// exponential moving avergae - applying low pass filter
void applyLowPassFilter(float& filteredValue, float rawValue) {
    filteredValue = alpha * filteredValue + (1.0 - alpha) * rawValue;
}

struct Thresholds {
  float Mean[2];
  float Variance[2];
  float StdDev[2];
  float Median[2];
  float Mode[2];
  float Skewness[2];
  float Energy[2];
  float Range[2];
  float IQR[2];
  float ZCR[2];
  float TAT[2];
  float AutoCorrection[2];
};

Thresholds sayHello = {
  {30.0, 60.0},       // Mean
  {87, 171},      // Variance
  {9, 13},        // StdDev
  {-36, -62},     // Median
  {-41, -58},     // Mode
  {0, 4},         // Skewness
  {2e6, 8e6},     // Energy
  {41, 62},       // Range
  {7, 18},        // IQR
  {0, 0},         // ZCR
  {1, 1},         // TAT
  {0.2e-3, 0.5e-3} // AutoCorrection
};

// Function to calculate mean
float calculateMean(float* data, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

// Function to calculate variance
float calculateVariance(float* data, int size, float mean) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += pow(data[i] - mean, 2);
  }
  return sum / size;
}

float calculateStdDev(float variance) {
  return sqrt(variance);
}

// Function to perform the quicksort algorithm
void quicksort(float* arr, int low, int high) {
  int i = low, j = high;
  float pivot = arr[(low + high) / 2];

  // Partition
  while (i <= j) {
    while (arr[i] < pivot) {
      i++;
    }
    while (arr[j] > pivot) {
      j--;
    }
    if (i <= j) {
      // Swap
      float temp = arr[i];
      arr[i] = arr[j];
      arr[j] = temp;
      i++;
      j--;
    }
  }

  // Recursion
  if (low < j) {
    quicksort(arr, low, j);
  }
  if (i < high) {
    quicksort(arr, i, high);
  }
}

// Function to find the median of an array
float findMedian(float* arr, int size) {
  if (size % 2 == 0) {
    // If the size is even, return the average of the two middle elements
    return (arr[size / 2 - 1] + arr[size / 2]) / 2.0;
  } else {
    // If the size is odd, return the middle element
    return arr[size / 2];
  }
}

// Function to calculate interquartile range
float calculateIQR(float* data, int size) {
  // Copy the data to avoid modifying the original array
  float sortedData[size];
  for (int i = 0; i < size; i++) {
    sortedData[i] = data[i];
  }

  // Sort the data using quicksort
  quicksort(sortedData, 0, size - 1);

  // Find the median of the lower and upper halves of the sorted data
  int lowerSize = size / 2;
  int upperSize = size - lowerSize;
  float lowerMedian = findMedian(sortedData, lowerSize);
  float upperMedian = findMedian(sortedData + lowerSize, upperSize);

  // Return the difference between these two medians
  return upperMedian - lowerMedian;
}

int calculateMode(float* data, int size) {
  // Create an array to store the count of each unique value
  int counts[size];
  for (int i = 0; i < size; i++) {
    counts[i] = 0;
  }

  // Count occurrences of each unique value
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      if (data[j] == data[i]) {
        counts[i]++;
      }
    }
  }

  // Find the maximum count
  int maxCount = 0;
  for (int i = 0; i < size; i++) {
    if (counts[i] > maxCount) {
      maxCount = counts[i];
    }
  }

  // Check for multiple modes
  int modeCount = 0;
  int modeIndex;
  for (int i = 0; i < size; i++) {
    if (counts[i] == maxCount) {
      modeCount++;
      modeIndex = i;
    }
  }

  // If there is a unique mode, return it; otherwise, return -1
  if (modeCount == 1) {
    return data[modeIndex];
  } else {
    return -1;  // Indicate that there is no unique mode
  }
}

// Function to calculate skewness
float calculateSkewness(float* data, int size, float mean, float stdDev) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += pow((data[i] - mean) / stdDev, 3);
  }
  return sum / size;
}

// Function to calculate energy
float calculateEnergy(float* data, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += pow(data[i], 2);
  }
  return sum;
}

// Function to calculate range
float calculateRange(float* data, int size) {
  // Initialize max and min with the first element of the array
  float maxVal = data[0];
  float minVal = data[0];

  // Find the maximum and minimum values in the array
  for (int i = 1; i < size; i++) {
    if (data[i] > maxVal) {
      maxVal = data[i];
    }
    if (data[i] < minVal) {
      minVal = data[i];
    }
  }

  // Return the difference between the maximum and minimum values
  return maxVal - minVal;
}

// Function to check if a value falls within a specified range
bool checkThreshold(float value, float threshold[2]) {
  return value >= threshold[0] && value <= threshold[1];
}

bool checkAllThresholds() {
  int matches = 0;

  matches += checkThreshold(meangx, sayHello.Mean);
  matches += checkThreshold(variancegx, sayHello.Variance);
  matches += checkThreshold(stdDevgx, sayHello.StdDev);
  matches += checkThreshold(mediangx, sayHello.Median);
  matches += checkThreshold(modegx, sayHello.Mode);
  matches += checkThreshold(skewnessgx, sayHello.Skewness);
  matches += checkThreshold(energygx, sayHello.Energy);
  matches += checkThreshold(rangegx, sayHello.Range);
  matches += checkThreshold(iqrgx, sayHello.IQR);
  //matches += checkThreshold(zeroCrossRateX, sayHello.ZCR);
  //matches += checkThreshold(timeAboveThresholdX, sayHello.TAT);
  //matches += checkThreshold(autocorrelationX, sayHello.AutoCorrection);

  Serial.print("Matches: ");
  Serial.println(matches);

  return matches >= thresholdMatches;
}


void printDataArray() {
  Serial.println("Data Array:");
  for (int i = 0; i < dataCount; i++) {
    Serial.println(gx[i]);
    Serial.println(gy[i]);
    Serial.println(gz[i]);
  }
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // configure LED for output
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 

  if (devStatus == 0) {

      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void loop() {
  
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    buttonState = digitalRead(BUTTON_PIN);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == HIGH && lastButtonState == LOW) {
      // turn LED on:
      ledState = (ledState == LOW) ? HIGH : LOW;
      digitalWrite(LED_PIN, ledState);

      dataCollectionActive = (ledState == HIGH);

      if (ledState == HIGH) {
        Serial.println("Data collection started.");
      } else {
        Serial.println("Data collection stopped.");
      }
    }
    else if(buttonState == LOW && lastButtonState == HIGH && !dataCollectionActive){
      int sizegx = sizeof(gx) / sizeof(gx[0]);
      int sizegy = sizeof(gy) / sizeof(gy[0]);
      int sizegz = sizeof(gz) / sizeof(gz[0]);
      
      float meangx = calculateMean(gx, sizegx);
      float variancegx = calculateVariance(gx, sizegx, meangx);
      float stdDevgx = calculateStdDev(variancegx);
      float mediangx = findMedian(gx, sizegx);
      int modegx = calculateMode(gx, sizegx);
      float skewnessgx = calculateSkewness(gx, sizegx, meangx, stdDevgx);
      float energygx = calculateEnergy(gx, sizegx);
      int rangegx = calculateRange(gx, sizegx);
      int iqrgx = calculateIQR(gx, sizegx);

      float meangy = calculateMean(gy, sizegy);
      float variancegy = calculateVariance(gy, sizegy, meangy);
      float stdDevgy = calculateStdDev(variancegy);
      float mediangy = findMedian(gy, sizegy);
      int modegy = calculateMode(gy, sizegy);
      float skewnessgy = calculateSkewness(gy, sizegy, meangy, stdDevgy);
      float energygy = calculateEnergy(gy, sizegy);
      int rangegy = calculateRange(gy, sizegy);
      int iqrgy = calculateIQR(gy, sizegy);

      float meangz = calculateMean(gz, sizegz);
      float variancegz = calculateVariance(gz, sizegz, meangz);
      float stdDevgz = calculateStdDev(variancegz);
      float mediangz = findMedian(gz, sizegz);
      int modegz = calculateMode(gz, sizegz);
      float skewnessgz = calculateSkewness(gz, sizegz, meangz, stdDevgz);
      float energygz = calculateEnergy(gz, sizegz);
      int rangegz = calculateRange(gz, sizegz);
      int iqrgz = calculateIQR(gz, sizegz);


      Serial.print("Mean: ");
      Serial.println(meangx);
      Serial.print("Variance: ");
      Serial.println(variancegx);
      Serial.print("StdDev: ");
      Serial.println(stdDevgx);
      Serial.print("Median: ");
      Serial.println(mediangx);
      Serial.print("Mode: ");
      Serial.println(modegx);
      Serial.print("Skewness: ");
      Serial.println(skewnessgx);
      Serial.print("Energy: ");
      Serial.println(energygx);
      Serial.print("Range: ");
      Serial.println(rangegx);
      Serial.print("IQR: ");
      Serial.println(iqrgx);

      if(checkAllThresholds()){
        Serial.println("HELLO");
      }
    }
    lastButtonState = buttonState;

    if (dataCollectionActive) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      float roll = ypr[2] * 180/M_PI;
      float pitch = ypr[1] * 180/M_PI;
      float yaw = ypr[0] * 180/M_PI;

      //apply low-pass filter to Euler angles
      applyLowPassFilter(filteredRoll, roll);
      applyLowPassFilter(filteredPitch, pitch);
      applyLowPassFilter(filteredYaw, yaw);
    }
  
    if (dataCollectionActive && dataCount < MAX_DATA_POINTS) {
      gx[dataCount] = filteredRoll;
      gy[dataCount] = filteredPitch;
      gz[dataCount] = filteredYaw;
      dataCount++;
    }
    
    int thresholdMin = -150;
    int thresholdMax = 150;
    // Calculate the number of elements in the arrays
    int arraySize = sizeof(gx) / sizeof(gx[0]);

    // Create a boolean array to store valid indices
    bool validIndices[arraySize];

    // Filter out invalid data points
    for (int i = 0; i < arraySize; i++) {
      validIndices[i] = (gx[i] >= thresholdMin && gx[i] <= thresholdMax) &&
                        (gy[i] >= thresholdMin && gy[i] <= thresholdMax) &&
                        (gz[i] >= thresholdMin && gz[i] <= thresholdMax);
    }

    // Create filtered arrays based on valid indices
    int validCount = 0;
    for (int i = 0; i < arraySize; i++) {
      if (validIndices[i]) {
        gx[validCount] = gx[i];
        gy[validCount] = gy[i];
        gz[validCount] = gz[i];
        validCount++;
      }
    }
    arraySize = validCount;

    // if (dataCount >= 100) {
    //   printDataArray();
    //   dataCount = 0;
    // }
    delay(300);
  }
}
