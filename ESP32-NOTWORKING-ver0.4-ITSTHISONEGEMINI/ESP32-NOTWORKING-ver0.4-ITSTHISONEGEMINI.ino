// --- CÓDIGO FINAL PARA TORRETA NERF (V2.23 - DISPARO ATÓMICO CORREGIDO) ---

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
#define PUSHER_ENABLE_PIN 22
#define RELAY_PIN 23

// --- AJUSTES DE LA CÁMARA Y SEGUIMIENTO ---
const int FRAME_WIDTH  = 96;
const float CONFIDENCE_THRESHOLD = 0.70;
const int CENTER_DEAD_ZONE = 5;
const int SETTLE_DELAY_MS = 1000;

// --- AJUSTES DE LOS MOTORES ---
// Motor de Paneo (ULN2003)
const int PAN_STEPS_PER_MOTOR_REV = 2048;
const int PAN_MOTOR_SPEED = 1190;
const int PAN_MOTOR_ACCEL = 400;
const int PAN_STEPS_TO_MOVE = 1190;
const long PAN_STEPS_FOR_FULL_PLATFORM_REVOLUTION = 11900;
const long MAX_PAN_STEPS_FROM_CENTER = PAN_STEPS_FOR_FULL_PLATFORM_REVOLUTION / 4; 

// Motor Empujador (A4988)
const int PUSHER_MAX_SPEED = 1500;
const int PUSHER_ACCELERATION = 1000;
const int PUSHER_TRAVEL_STEPS = 400;

// --- AJUSTES DE DISPARO ---
// Mencionaste "medio segundo", así que puedes cambiar 1000 a 500 si lo deseas.
const int MIN_REV_TIME = 1000; 
const int FIRE_COOLDOWN = 1000;

// --- CONFIGURACIÓN DE DEPURACIÓN ---
const int DEBUG_DELAY_MS = 0;
//======================================================================
// --- FIN DE LA CONFIGURACIÓN ---
//======================================================================

AccelStepper panStepper(AccelStepper::FULL4WIRE, PAN_IN1_PIN, PAN_IN3_PIN, PAN_IN2_PIN, PAN_IN4_PIN);
AccelStepper pusherStepper(AccelStepper::DRIVER, PUSHER_STEP_PIN, PUSHER_DIR_PIN);

// Declaraciones anticipadas
void handleSerial();
void parseAndTrack(String data);
void handleFiringSequence();
void handleTimeout();

// Variables Globales
long currentPanPosition = 0;
String serialBuffer = "";
unsigned long lastDetectionTime = 0;
const int DETECTION_TIMEOUT = 1000;
unsigned long revStartTime = 0;
unsigned long lastFireTime = 0;
enum FiringState { IDLE, REVING, PUSHING, RETRACTING };
FiringState firingState = IDLE;

bool isPanMotorMoving = false;
unsigned long motorSettleTime = 0;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL_COMMS_RX, -1);
  
  panStepper.setMaxSpeed(PAN_MOTOR_SPEED);
  panStepper.setAcceleration(PAN_MOTOR_ACCEL);
  
  pusherStepper.setMaxSpeed(PUSHER_MAX_SPEED);
  pusherStepper.setAcceleration(PUSHER_ACCELERATION);
  
  pinMode(PUSHER_ENABLE_PIN, OUTPUT);
  digitalWrite(PUSHER_ENABLE_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  Serial.println("--- INICIANDO TORRETA V2.23 (Disparo Corregido) ---");
  Serial.println("--- CALIBRANDO... ---");
  
  panStepper.runToNewPosition(1190);
  Serial.println("[DEBUG] Calibración: Derecha...");
  panStepper.runToNewPosition(-1190);
  Serial.println("[DEBUG] Calibración: Izquierda...");
  panStepper.runToNewPosition(0);
  Serial.println("[DEBUG] Calibración: Centro...");
  
  currentPanPosition = 0;
  Serial.println("--- CALIBRACIÓN COMPLETA. LISTA. ---");
}

void loop() {
  // 1. Comprueba si el motor se está moviendo
  if (panStepper.distanceToGo() != 0) {
    if (!isPanMotorMoving) {
        Serial.println("\n[DEBUG] ESTADO: Moviendo motor de paneo...");
        isPanMotorMoving = true; 
    }
  } else if (isPanMotorMoving) {
    // El motor acaba de detenerse
    isPanMotorMoving = false;
    motorSettleTime = millis(); 
    Serial.println("[DEBUG] ESTADO: Motor detenido. Iniciando pausa de estabilización...");
    
    Serial.println("[DEBUG] Limpiando buffer de la cámara de datos viejos...");
    while (Serial1.available() > 0) {
      Serial1.read();
    }
    serialBuffer = "";
  }
  
  // 2. Ejecuta los motores
  panStepper.run();
  if (digitalRead(PUSHER_ENABLE_PIN) == LOW) {
    pusherStepper.run();
  }
  
  // 3. Lógica principal de cámara y timeout
  bool isSettled = millis() - motorSettleTime > SETTLE_DELAY_MS;
  
  // 4. La secuencia de disparo se ejecuta siempre
  handleFiringSequence();
  
  // 5. Solo leemos la cámara y comprobamos el timeout si el motor está quieto y asentado
  if (!isPanMotorMoving && isSettled) {
    handleSerial();
    handleTimeout(); // Esta función ahora se auto-desactiva si está disparando
  }
}

void handleSerial() {
  while (Serial1.available() > 0) {
    char incomingChar = Serial1.read();
    Serial.print(incomingChar); // PASSTHROUGH
    if (incomingChar == '\n') {
      if (serialBuffer.indexOf("x: ") != -1) {
        Serial.println("\n[DEBUG] Línea válida recibida. Parseando...");
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
  if (confidence < CONFIDENCE_THRESHOLD) {
    Serial.printf("[DEBUG] RECHAZADO: Confianza (%.2f) < Umbral (%.2f)\n", confidence, CONFIDENCE_THRESHOLD);
    return;
  }
  
  Serial.printf("[DEBUG] ACEPTADO: Confianza (%.2f)\n", confidence);
  lastDetectionTime = millis();
  
  int x_start_index = data.indexOf("x: ") + 3;
  if (x_start_index == 2) return;
  int x_end_index = data.indexOf(',', x_start_index);
  int x = data.substring(x_start_index, x_end_index).toInt();
  int width_start_index = data.indexOf("width: ") + 7;
  if (width_start_index == 6) return;
  int width_end_index = data.indexOf(',', width_start_index);
  if (width_end_index == -1) { 
    width_end_index = data.indexOf(']', width_start_index);
  }
  int width = data.substring(width_start_index, width_end_index).toInt();
  Serial.printf("[DEBUG] Parseo: x=%d, width=%d\n", x, width);

  int object_center_x = x + (width / 2);
  int error = object_center_x - (FRAME_WIDTH / 2);

  Serial.printf("[DEBUG] Lógica: CentroObj=%d, CentroFrame=%d, Error=%d, ZonaMuerta=%d\n", object_center_x, (FRAME_WIDTH / 2), error, CENTER_DEAD_ZONE);
  if (abs(error) > CENTER_DEAD_ZONE) {
    Serial.println("[DEBUG] DECISIÓN: Mover.");
    long newPos = currentPanPosition;
    // ***** LÓGICA INVERTIDA (DEJAR COMO ESTÁ) *****
    if (error < 0) { // Mover a la izquierda (Error negativo)
      Serial.println("[DEBUG] Dirección: IZQUIERDA (Físicamente)");
      // El motor ahora se mueve a una POSICIÓN POSITIVA
      newPos += PAN_STEPS_TO_MOVE;
    } else { // Mover a la derecha (Error positivo)
      Serial.println("[DEBUG] Dirección: DERECHA (Físicamente)");
      // El motor ahora se mueve a una POSICIÓN NEGATIVA
      newPos -= PAN_STEPS_TO_MOVE;
    }

    // Límites (invertidos también)
    if (newPos > MAX_PAN_STEPS_FROM_CENTER) {
      newPos = MAX_PAN_STEPS_FROM_CENTER;
      Serial.println("[DEBUG] INFO: Límite izquierdo físico alcanzado.");
    }
    if (newPos < -MAX_PAN_STEPS_FROM_CENTER) {
      newPos = -MAX_PAN_STEPS_FROM_CENTER;
      Serial.println("[DEBUG] INFO: Límite derecho físico alcanzado.");
    }

    Serial.printf("[DEBUG] ACCIÓN: Moviendo a nueva posición: %ld\n", newPos);
    panStepper.moveTo(newPos);
    currentPanPosition = newPos;
    if (DEBUG_DELAY_MS > 0) delay(DEBUG_DELAY_MS);

  } else {
    Serial.println("[DEBUG] DECISIÓN: Centrado. Comprobando lógica de disparo...");
    if (firingState == IDLE && (millis() - lastFireTime > FIRE_COOLDOWN)) {
      firingState = REVING;
      revStartTime = millis();
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("[DEBUG] ACCIÓN: Objetivo centrado. Revolucionando motores...");
    } else if (firingState != IDLE) {
      Serial.println("[DEBUG] INFO: Objetivo centrado, pero ya está en proceso de disparo.");
    } else {
      Serial.println("[DEBUG] INFO: Objetivo centrado, pero en cooldown.");
    }
  }
}

void handleFiringSequence() {
  if (firingState == REVING && (millis() - revStartTime > MIN_REV_TIME)) {
    Serial.println("[DEBUG] Secuencia: Empujando dardo...");
    firingState = PUSHING;
    digitalWrite(PUSHER_ENABLE_PIN, LOW); 
    pusherStepper.moveTo(PUSHER_TRAVEL_STEPS);
  }

  if (firingState == PUSHING && pusherStepper.distanceToGo() == 0) {
    Serial.println("[DEBUG] Secuencia: Retrayendo empujador...");
    firingState = RETRACTING;
    pusherStepper.moveTo(0);
  }

  if (firingState == RETRACTING && pusherStepper.distanceToGo() == 0) {
    Serial.println("[DEBUG] Secuencia: Disparo completo. Apagando motores.");
    firingState = IDLE;
    lastFireTime = millis();
    digitalWrite(PUSHER_ENABLE_PIN, HIGH);
    digitalWrite(RELAY_PIN, LOW);
  }
}

// =====================================================================
// --- FUNCIÓN CORREGIDA ---
// =====================================================================
void handleTimeout() {
  // --- INICIO DE LA MODIFICACIÓN ---
  // Si estamos en cualquier estado de disparo (REVING, PUSHING, RETRACTING),
  // no debemos ejecutar la lógica de timeout. El disparo debe completarse.
  if (firingState != IDLE) {
    return;
  }
  // --- FIN DE LA MODIFICACIÓN ---

  if (millis() - lastDetectionTime > DETECTION_TIMEOUT && lastDetectionTime != 0) {
    Serial.println("\n[DEBUG] TIMEOUT: Objetivo perdido. Volviendo al centro.");
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(PUSHER_ENABLE_PIN, HIGH); 
    firingState = IDLE;
    lastDetectionTime = 0;
    
    panStepper.moveTo(0);
    currentPanPosition = 0;
  }
}