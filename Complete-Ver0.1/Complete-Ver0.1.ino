// --- CÓDIGO FINAL PARA TORRETA NERF (VERSIÓN CORREGIDA Y MEJORADA) ---

#include <Stepper.h>
#include <AccelStepper.h>

//======================================================================
// --- CONFIGURACIÓN PRINCIPAL ---
//======================================================================

// --- PINES DE CONEXIÓN ---
#define SERIAL_COMMS_RX 16
#define PAN_IN1_PIN 32
#define PAN_IN2_PIN 33
#define PAN_IN3_PIN 25
#define PAN_IN4_PIN 26
#define PUSHER_STEP_PIN 18
#define PUSHER_DIR_PIN  19
#define PUSHER_ENABLE_PIN 14
#define RELAY_PIN 23

// --- AJUSTES DE LA CÁMARA Y SEGUIMIENTO ---
const int FRAME_WIDTH  = 96;
const float CONFIDENCE_THRESHOLD = 0.70;
const int CENTER_DEAD_ZONE = 18;

// --- AJUSTES DE LOS MOTORES ---
// Motor de Paneo (ULN2003)
const int PAN_STEPS_PER_REV = 2048;
const int PAN_MOTOR_SPEED = 20;
const int PAN_STEPS_TO_MOVE = 1500;
const long MAX_PAN_STEPS_FROM_CENTER = 4265;

// Motor Empujador (A4988)
const int PUSHER_MAX_SPEED = 1500;
const int PUSHER_ACCELERATION = 1000;
const int PUSHER_TRAVEL_STEPS = 400;

// --- AJUSTES DE DISPARO ---
const int MIN_REV_TIME = 1000;
const int FIRE_COOLDOWN = 1500;

//======================================================================
// --- FIN DE LA CONFIGURACIÓN ---
//======================================================================

// ***** CAMBIO: Corregido el orden de los pines (IN2 y IN3) para movimiento bidireccional *****
Stepper panStepper(PAN_STEPS_PER_REV, PAN_IN1_PIN, PAN_IN3_PIN, PAN_IN2_PIN, PAN_IN4_PIN);
AccelStepper pusherStepper(AccelStepper::DRIVER, PUSHER_STEP_PIN, PUSHER_DIR_PIN);

long currentPanPosition = 0;

String serialBuffer = "";
unsigned long lastDetectionTime = 0;
const int DETECTION_TIMEOUT = 1000;
unsigned long revStartTime = 0;
unsigned long lastFireTime = 0;
enum FiringState { IDLE, REVING, PUSHING, RETRACTING };
FiringState firingState = IDLE;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL_COMMS_RX, -1);
  panStepper.setSpeed(PAN_MOTOR_SPEED);
  pusherStepper.setMaxSpeed(PUSHER_MAX_SPEED);
  pusherStepper.setAcceleration(PUSHER_ACCELERATION);
  pinMode(PUSHER_ENABLE_PIN, OUTPUT);
  digitalWrite(PUSHER_ENABLE_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  currentPanPosition = 0;
  Serial.println("Torreta V2.5 (Parseo Corregido) - Lista.");
}

void loop() {
  if (digitalRead(PUSHER_ENABLE_PIN) == LOW) {
    pusherStepper.run();
  }
  handleSerial();
  handleFiringSequence();
  handleTimeout();
}

void handleSerial() {
  while (Serial1.available() > 0) {
    char incomingChar = Serial1.read();
    if (incomingChar == '\n') {
      if (serialBuffer.indexOf("x: ") != -1) {
        parseAndTrack(serialBuffer);
      }
      serialBuffer = "";
    } else {
      serialBuffer += incomingChar;
    }
  }
}

void parseAndTrack(String data) {
  int openParenIndex = data.indexOf('(');
  int closeParenIndex = data.indexOf(')');
  if (openParenIndex == -1 || closeParenIndex == -1) return;
  float confidence = data.substring(openParenIndex + 1, closeParenIndex).toFloat();

  if (confidence < CONFIDENCE_THRESHOLD || confidence >= 0.99) {
    return;
  }
  
  lastDetectionTime = millis();
  
  // ***** CAMBIO: Lógica de parseo mejorada para leer números de más de un dígito *****
  int x_start_index = data.indexOf("x: ") + 3;
  if (x_start_index == 2) return; // Si no encuentra "x: ", sale
  int x_end_index = data.indexOf(',', x_start_index); // Busca la coma después de la x
  int x = data.substring(x_start_index, x_end_index).toInt();
  
  int width_start_index = data.indexOf("width: ") + 7;
  if (width_start_index == 6) return; // Si no encuentra "width: ", sale
  int width_end_index = data.indexOf(']', width_start_index); // Busca el corchete después del width
  int width = data.substring(width_start_index, width_end_index).toInt();

  int object_center_x = x + (width / 2);
  
  int error = object_center_x - (FRAME_WIDTH / 2);

  if (abs(error) > CENTER_DEAD_ZONE) {
    if (error < 0) { // Mover a la izquierda
      if (currentPanPosition - PAN_STEPS_TO_MOVE > -MAX_PAN_STEPS_FROM_CENTER) {
        panStepper.step(-PAN_STEPS_TO_MOVE);
        currentPanPosition -= PAN_STEPS_TO_MOVE;
      }
    } else { // Mover a la derecha
      if (currentPanPosition + PAN_STEPS_TO_MOVE < MAX_PAN_STEPS_FROM_CENTER) {
        panStepper.step(PAN_STEPS_TO_MOVE);
        currentPanPosition += PAN_STEPS_TO_MOVE;
      }
    }
  } else {
    if (firingState == IDLE && (millis() - lastFireTime > FIRE_COOLDOWN)) {
      firingState = REVING;
      revStartTime = millis();
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Objetivo centrado. Revolucionando motores...");
    }
  }
}

void handleFiringSequence() {
  if (firingState == REVING && (millis() - revStartTime > MIN_REV_TIME)) {
    Serial.println("Empujando dardo...");
    firingState = PUSHING;
    digitalWrite(PUSHER_ENABLE_PIN, LOW); 
    pusherStepper.moveTo(PUSHER_TRAVEL_STEPS);
  }

  if (firingState == PUSHING && pusherStepper.distanceToGo() == 0) {
    Serial.println("Retrayendo empujador...");
    firingState = RETRACTING;
    pusherStepper.moveTo(0);
  }

  if (firingState == RETRACTING && pusherStepper.distanceToGo() == 0) {
    Serial.println("Secuencia de disparo completa. Apagando motores.");
    firingState = IDLE;
    lastFireTime = millis();
    digitalWrite(PUSHER_ENABLE_PIN, HIGH);
    digitalWrite(RELAY_PIN, LOW); 
  }
}

void handleTimeout() {
  if (millis() - lastDetectionTime > DETECTION_TIMEOUT && lastDetectionTime != 0) {
    Serial.println("Objetivo perdido. Apagando sistemas.");
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(PUSHER_ENABLE_PIN, HIGH); 
    firingState = IDLE;
    lastDetectionTime = 0;
  }
}