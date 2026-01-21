/*
 * Environmental Monitoring System
 * 
 * Sensors:
 *   - HC-SR04 Ultrasonic Distance Sensor
 *   - SW-420 Vibration Sensor
 *   - DHT11 Temperature & Humidity Sensor
 * 
 * Board: Arduino UNO R3 (or compatible)
 * 
 * Output: Serial monitor at 9600 baud, updates every 1 second
 */

#include <DHT.h>

// ==================== PIN DEFINITIONS ====================

// HC-SR04 Ultrasonic Sensor
#define ULTRASONIC_TRIG_PIN  9
#define ULTRASONIC_ECHO_PIN  10

// SW-420 Vibration Sensor
#define VIBRATION_PIN        2

// DHT11 Temperature & Humidity Sensor
#define DHT_PIN              7
#define DHT_TYPE             DHT11

// ==================== CONSTANTS ====================

#define SOUND_SPEED_CM_US    0.0343  // Speed of sound in cm/microsecond
#define MAX_DISTANCE_CM      400     // Maximum measurable distance
#define MEASUREMENT_INTERVAL 1000    // 1 second between measurements

// ==================== GLOBAL OBJECTS ====================

DHT dht(DHT_PIN, DHT_TYPE);

// ==================== GLOBAL VARIABLES ====================

unsigned long lastMeasurementTime = 0;
volatile bool vibrationDetected = false;
volatile unsigned long vibrationCount = 0;

// ==================== INTERRUPT SERVICE ROUTINE ====================

void vibrationISR() {
  vibrationDetected = true;
  vibrationCount++;
}

// ==================== SETUP ====================

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  // Initialize HC-SR04 pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  // Initialize SW-420 vibration sensor with interrupt
  pinMode(VIBRATION_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), vibrationISR, RISING);
  
  // Initialize DHT11 sensor
  dht.begin();
  
  // Print header
  Serial.println(F("====================================="));
  Serial.println(F("  Environmental Monitoring System"));
  Serial.println(F("====================================="));
  Serial.println(F("Sensors: HC-SR04, SW-420, DHT11"));
  Serial.println(F("Update interval: 1 second"));
  Serial.println(F("=====================================\n"));
  
  // Allow sensors to stabilize
  delay(2000);
  Serial.println(F("System ready. Starting measurements...\n"));
}

// ==================== SENSOR FUNCTIONS ====================

/**
 * Measure distance using HC-SR04 ultrasonic sensor
 * Returns distance in centimeters, or -1 if out of range
 */
float measureDistance() {
  long duration, distance;
  
  // Clear the trigger pin (same as working code)
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse (same as working code)
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  // Read the echo pin - no timeout like working code
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  
  // Calculate distance using same formula as working code
  distance = (duration / 2) / 29.1;
  
  // Validate range
  if (distance >= MAX_DISTANCE_CM || distance <= 0) {
    return -1;
  }
  
  return distance;
}

/**
 * Read vibration status and count
 * Returns true if vibration was detected since last check
 */
bool checkVibration(unsigned long &count) {
  noInterrupts();
  bool detected = vibrationDetected;
  count = vibrationCount;
  vibrationDetected = false;  // Reset flag
  vibrationCount = 0;         // Reset count for next interval
  interrupts();
  
  return detected;
}

/**
 * Read temperature from DHT11
 * Returns temperature in Celsius, or NAN if error
 */
float readTemperature() {
  return dht.readTemperature();
}

/**
 * Read humidity from DHT11
 * Returns humidity percentage, or NAN if error
 */
float readHumidity() {
  return dht.readHumidity();
}

// ==================== OUTPUT FUNCTIONS ====================

void printMeasurements(float distance, bool vibration, unsigned long vibCount, 
                       float temperature, float humidity) {
  Serial.println(F("--- Sensor Readings ---"));
  
  // Distance
  Serial.print(F("Distance:    "));
  if (distance < 0) {
    Serial.println(F("Out of range"));
  } else {
    Serial.print(distance, 1);
    Serial.println(F(" cm"));
  }
  
  // Vibration
  Serial.print(F("Vibration:   "));
  if (vibration) {
    Serial.print(F("DETECTED ("));
    Serial.print(vibCount);
    Serial.println(F(" events)"));
  } else {
    Serial.println(F("None"));
  }
  
  // Temperature
  Serial.print(F("Temperature: "));
  if (isnan(temperature)) {
    Serial.println(F("Error reading"));
  } else {
    Serial.print(temperature, 1);
    Serial.print(F(" °C ("));
    Serial.print(temperature * 9.0 / 5.0 + 32.0, 1);
    Serial.println(F(" °F)"));
  }
  
  // Humidity
  Serial.print(F("Humidity:    "));
  if (isnan(humidity)) {
    Serial.println(F("Error reading"));
  } else {
    Serial.print(humidity, 1);
    Serial.println(F(" %"));
  }
  
  // Heat index (if valid readings)
  if (!isnan(temperature) && !isnan(humidity)) {
    float heatIndex = dht.computeHeatIndex(temperature, humidity, false);
    Serial.print(F("Heat Index:  "));
    Serial.print(heatIndex, 1);
    Serial.println(F(" °C"));
  }
  
  Serial.println();
}

// JSON output for data logging/parsing
void printJSON(float distance, bool vibration, unsigned long vibCount,
               float temperature, float humidity) {
  Serial.print(F("{\"distance\":"));
  if (distance < 0) {
    Serial.print(F("null"));
  } else {
    Serial.print(distance, 1);
  }
  
  Serial.print(F(",\"vibration\":"));
  Serial.print(vibration ? F("true") : F("false"));
  
  Serial.print(F(",\"vib_count\":"));
  Serial.print(vibCount);
  
  Serial.print(F(",\"temp\":"));
  if (isnan(temperature)) {
    Serial.print(F("null"));
  } else {
    Serial.print(temperature, 1);
  }
  
  Serial.print(F(",\"humidity\":"));
  if (isnan(humidity)) {
    Serial.print(F("null"));
  } else {
    Serial.print(humidity, 1);
  }
  
  Serial.print(F(",\"timestamp\":"));
  Serial.print(millis());
  
  Serial.println(F("}"));
}

// ==================== MAIN LOOP ====================

void loop() {
  unsigned long currentTime = millis();
  
  // Check if it's time for a measurement (every 1 second)
  if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
    lastMeasurementTime = currentTime;
    
    // Read all sensors
    float distance = measureDistance();
    unsigned long vibCount;
    bool vibration = checkVibration(vibCount);
    float temperature = readTemperature();
    float humidity = readHumidity();
    
    // Output human-readable format
    printMeasurements(distance, vibration, vibCount, temperature, humidity);
    
    // Uncomment line below for JSON output instead (useful for data logging)
    // printJSON(distance, vibration, vibCount, temperature, humidity);
  }
}
