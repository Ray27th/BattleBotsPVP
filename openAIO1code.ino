/**************************************************************
 * ESP32 WebSocket Client + Autonomous Sensor Logic
 * 
 * Hardware/Pin Definitions remain as you specified:
 *   - Color Sensor: S0=14, S1=27, S2=26, S3=25, OUT=33, LED=32
 *   - Ultrasonic: TRIG=15, ECHO=34
 *   - NeoPixel Strip on pin 16 (1 pixel)
 *   - NeoMatrix 8x8 on pin 21
 *   - Single Limit Switch on pin 13
 *   - Motors: IN1=23, IN2=22, ENA=4, IN3=19, IN4=18, ENB=5
 *   - Servo on pin 2
 **************************************************************/

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <Servo.h>
#include <ezButton.h>

// ------------------ Wi-Fi + WebSocket Config ------------------
const char* WIFI_SSID     = "Hacker";
const char* WIFI_PASSWORD = "mo123456";

// Replace this with the IP address (or hostname) of the machine
// running your Python WebSocket server. If your Python server is at 
// 192.168.1.100 on port 8765, you do:
const char* WEBSOCKET_HOST = "192.168.36.113";
uint16_t    WEBSOCKET_PORT = 8765;

// -------------- Pin Definitions (as specified) ---------------
#define S0 14
#define S1 27
#define S2 26
#define S3 25
#define OUT 33
#define LED_PIN_SENSOR 32

#define ULTRASONIC_TRIGGER 15
#define ULTRASONIC_ECHO    34

#define LED_PIN_NEOPIXEL_STRIP 16   // 1 Pixel
#define LED_PIN_NEOMATRIX       21  // 8x8 Matrix
#define SWITCH_PIN              13
#define NUM_LEDS_STRIP          1

#define SERVO_PIN 2

#define IN1 23
#define IN2 22
#define ENA 4
#define IN3 19
#define IN4 18
#define ENB 5

// ------------------- Color Sensor Calibration ---------------
int R_Min = 45, R_Max = 950;
int G_Min = 45, G_Max = 1300;
int B_Min = 40, B_Max = 1150;

int redValue = 0, greenValue = 0, blueValue = 0;

// ------------------ Objects & Global Variables --------------
WebSocketsClient webSocket;
Servo servoMotor;

// NeoPixel Strip
Adafruit_NeoPixel strip(NUM_LEDS_STRIP, LED_PIN_NEOPIXEL_STRIP, NEO_GRB + NEO_KHZ800);

// NeoMatrix
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(
  8, 8, LED_PIN_NEOMATRIX,
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB + NEO_KHZ800
);

// Limit Switch
ezButton limitSwitch(SWITCH_PIN);

// Tracking ignoring commands if BLUE is seen
bool ignoringCommands = false;
unsigned long ignoringStartTime = 0;
const unsigned long IGNORE_DURATION_MS = 5000; // 5 seconds

// Freeze / Halt logic
bool isFrozen = false;       // for limit switch triggers
unsigned long freezeStartTime = 0;
const unsigned long FREEZE_DURATION_MS = 30000; // 30 seconds

// Basic servo logic
bool servoMovingForward = true;
unsigned long lastServoMoveTime = 0;
const unsigned long servoInterval = 500; // 0.5s

// Movement states
bool isMovingForward = false;

// Matrix Rainbow
unsigned long lastMatrixUpdate = 0;
const unsigned long matrixInterval = 50; // 50ms
uint16_t hue = 0;

// -------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Serial communication started");

  // Initialize limit switch
  limitSwitch.setDebounceTime(50);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // Initialize color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(LED_PIN_SENSOR, OUTPUT);
  digitalWrite(LED_PIN_SENSOR, HIGH); // Turn on the color sensor LED
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW); // Frequency scaling to 20%

  // Initialize ultrasonic
  pinMode(ULTRASONIC_TRIGGER, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Initialize motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopAll(); // Start with everything stopped

  // Initialize servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90); // Neutral

  // Initialize NeoPixel Strip
  strip.begin();
  strip.show(); // Off

  // Initialize NeoMatrix
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(40);
  matrix.fillScreen(0);
  matrix.show();

  // Connect to WiFi
  connectToWiFi();

  // Setup WebSocket
  webSocket.onEvent(webSocketEvent);
  // Connect to your Python server
  // (the path "/" can be changed if your Python server expects a different URL)
  webSocket.begin(WEBSOCKET_HOST, WEBSOCKET_PORT, "/");
  webSocket.setReconnectInterval(5000);

  Serial.println("Setup complete.");
}

// -------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------
void loop() {
  limitSwitch.loop();
  webSocket.loop(); // Continuously process WebSocket events

  // Check if ignoring commands due to BLUE color
  if (ignoringCommands && (millis() - ignoringStartTime >= IGNORE_DURATION_MS)) {
    ignoringCommands = false;
    Serial.println("Resuming command acceptance after BLUE detection timeout.");
  }

  // Check if we're frozen by limit switch
  if (isFrozen) {
    // If the 30s freeze is up, unfreeze
    if (millis() - freezeStartTime >= FREEZE_DURATION_MS) {
      isFrozen = false;
      Serial.println("Freeze period ended. Resuming operations.");
      turnNeoPixelOff();
      matrix.fillScreen(0); // Turn off matrix or resume rainbow
      matrix.show();
    } else {
      // While frozen, ensure motors are stopped, do nothing else
      stopAll();
      return;
    }
  }

  // Check limit switch
  if (limitSwitch.isPressed()) {
    Serial.println("Limit switch triggered -> FREEZE!");
    freezeOperations();
  }

  // Check color sensor
  checkColorSensor(); // If BLUE is found, ignoringCommands is set for 5s

  // Check ultrasonic sensor for < 30cm => Attack
  float distance = readUltrasonicDistance();
  if (distance > 0 && distance < 30.0) {
    Serial.println("Ultrasonic < 30cm -> auto Attack!");
    attackServo();
  }

  // NeoMatrix rainbow
  updateNeoMatrix();

  delay(10); // small delay to avoid flooding
}

// -------------------------------------------------------------------------
// WIFI + WEBSOCKET
// -------------------------------------------------------------------------
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected. IP address: " + WiFi.localIP().toString());
}

// WebSocket event callback
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WebSocket] Connected to server\n");
      break;
    case WStype_TEXT: {
      // We received a text message from the Python server
      if (length == 0) return;
      String cmd = String((char*)payload).substring(0, length);
      Serial.print("Received WebSocket cmd: ");
      Serial.println(cmd);

      // If we are ignoring commands (BLUE color detected), skip.
      if (ignoringCommands) {
        Serial.println("Ignoring incoming command due to BLUE detection.");
        return;
      }
      // If frozen (limit switch triggered), skip.
      if (isFrozen) {
        Serial.println("Commands ignored - robot is FROZEN by limit switch.");
        return;
      }

      // Otherwise parse the command
      handleWebSocketCommand(cmd);
    } break;
    default:
      break;
  }
}

// Parse the incoming command from the WebSocket server
void handleWebSocketCommand(const String& cmd) {
  if (cmd == "W") {
    moveForward(255); // Full speed
  }
  else if (cmd == "A") {
    turnLeft(100);
  }
  else if (cmd == "S") {
    moveBack(100);
  }
  else if (cmd == "D") {
    turnRight(100);
  }
  else if (cmd == "Space") {
    attackServo();
  }
}

// -------------------------------------------------------------------------
// MOTOR CONTROL
// -------------------------------------------------------------------------

// Move forward at a given speed
void moveForward(int speedVal) {
  Serial.printf("Moving FORWARD at speed %d\n", speedVal);
  isMovingForward = true;

  // Flip logic: originally set to IN1=HIGH, IN2=LOW, etc.
  // Invert them here:
  digitalWrite(IN1, LOW);   // was HIGH
  digitalWrite(IN2, HIGH);  // was LOW
  digitalWrite(IN3, LOW);   // was HIGH
  digitalWrite(IN4, HIGH);  // was LOW

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void moveBack(int speedVal) {
  Serial.printf("Moving BACKWARD at speed %d\n", speedVal);
  isMovingForward = false;

  // Flip logic
  digitalWrite(IN1, HIGH);  // was LOW
  digitalWrite(IN2, LOW);   // was HIGH
  digitalWrite(IN3, HIGH);  // was LOW
  digitalWrite(IN4, LOW);   // was HIGH

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnLeft(int speedVal) {
  Serial.printf("Turning LEFT at speed %d\n", speedVal);
  isMovingForward = false;

  // Flip logic
  digitalWrite(IN1, HIGH);  // was LOW
  digitalWrite(IN2, LOW);   // was HIGH
  digitalWrite(IN3, LOW);   // was HIGH
  digitalWrite(IN4, HIGH);  // was LOW

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnRight(int speedVal) {
  Serial.printf("Turning RIGHT at speed %d\n", speedVal);
  isMovingForward = false;

  // Flip logic
  digitalWrite(IN1, LOW);   // was HIGH
  digitalWrite(IN2, HIGH);  // was LOW
  digitalWrite(IN3, HIGH);  // was LOW
  digitalWrite(IN4, LOW);   // was HIGH

  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void stopAll() {
  if (isMovingForward) {
    Serial.println("Stopping all motors.");
  }
  isMovingForward = false;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// -------------------------------------------------------------------------
// SERVO ATTACK
// -------------------------------------------------------------------------
void attackServo() {
  Serial.println("Attack servo triggered!");
  // Example: do a quick 0 -> 180 -> 90 strike
  servoMotor.write(0);
  delay(200);
  servoMotor.write(180);
  delay(200);
  servoMotor.write(90);
}

// -------------------------------------------------------------------------
// COLOR SENSOR
// -------------------------------------------------------------------------
void checkColorSensor() {
  // Read red
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redValue = pulseIn(OUT, HIGH, 100000); // 100ms timeout

  // Read green
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenValue = pulseIn(OUT, HIGH, 100000);

  // Read blue
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueValue = pulseIn(OUT, HIGH, 100000);

  // Debug
  Serial.printf("ColorSensor -> R:%d G:%d B:%d\n", redValue, greenValue, blueValue);

  // If we detect BLUE, ignore commands for 5s
  if (!ignoringCommands) {
    bool isBlue = (blueValue >= B_Min && blueValue <= B_Max &&
                   redValue < R_Min && greenValue < G_Min);
    if (isBlue) {
      ignoringCommands = true;
      ignoringStartTime = millis();
      Serial.println("BLUE detected! Ignoring WS commands for 5s...");
    }
  }
}

// -------------------------------------------------------------------------
// ULTRASONIC
// -------------------------------------------------------------------------
float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  if (duration == 0) {
    //Serial.println("Ultrasonic timeout");
    return -1;
  }
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// -------------------------------------------------------------------------
// FREEZE (LIMIT SWITCH TRIGGER)
// -------------------------------------------------------------------------
void freezeOperations() {
  stopAll();
  isFrozen = true;
  freezeStartTime = millis();

  // Turn entire 8x8 matrix RED, plus the single NeoPixel if desired
  turnNeoPixelRed();
  matrix.fillScreen(matrix.Color(255, 0, 0)); // Full red
  matrix.show();
}

// -------------------------------------------------------------------------
// NEO PIXEL (1 LED) CONTROL
// -------------------------------------------------------------------------
void turnNeoPixelRed() {
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
}

void turnNeoPixelOff() {
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
}

// -------------------------------------------------------------------------
// NEO MATRIX RAINBOW
// -------------------------------------------------------------------------
void updateNeoMatrix() {
  // If we're currently frozen, we skip the rainbow
  if (isFrozen) return;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMatrixUpdate >= matrixInterval) {
    lastMatrixUpdate = currentMillis;

    matrix.fillScreen(0);
    // Draw a rainbow across the matrix
    for(int x=0; x<8; x++) {
      for(int y=0; y<8; y++) {
        uint16_t pixelHue = (hue + (x * 32) + (y * 32)) % 65536;
        matrix.drawPixel(x, y, matrix.ColorHSV(pixelHue));
      }
    }
    matrix.show();
    hue += 256;
  }
}
