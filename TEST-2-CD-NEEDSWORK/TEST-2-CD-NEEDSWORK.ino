// --- SCRIPT DE PRUEBA PARA EL MOTOR EMPUJADOR (A4988) ---

#include <AccelStepper.h>

// --- Pines del motor empujador (A4988) ---
#define PUSHER_STEP_PIN 18
#define PUSHER_DIR_PIN  19
#define PUSHER_ENABLE_PIN 22 // Pin para activar/desactivar el driver

// --- Ajustes del motor ---
// (Valores tomados de nuestro código principal)
#define PUSHER_MAX_SPEED 500
#define PUSHER_ACCELERATION 1000
#define PUSHER_TRAVEL_STEPS 250 // Pasos para empujar el dardo (ajusta si es necesario)

// Inicializa el motor
AccelStepper pusherStepper(AccelStepper::DRIVER, PUSHER_STEP_PIN, PUSHER_DIR_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("--- Prueba del Motor Empujador ---");

  // Configura el pin ENABLE
  pinMode(PUSHER_ENABLE_PIN, OUTPUT);
  // Pone el pin ENABLE en BAJO para ACTIVAR el driver
  // (Recuerda que en el código principal lo ponemos en ALTO para desactivarlo)
  digitalWrite(PUSHER_ENABLE_PIN, LOW);

  // Configura la velocidad y aceleración
  pusherStepper.setMaxSpeed(PUSHER_MAX_SPEED);
  pusherStepper.setAcceleration(PUSHER_ACCELERATION);

  Serial.println("Motor activado. Iniciando movimiento en 2 segundos...");
  delay(5000);
}

void loop() {
  Serial.println("Moviendo para EMPUJAR...");
  // Mueve el motor a la posición de "empuje" y espera a que termine
  pusherStepper.runToNewPosition(PUSHER_TRAVEL_STEPS);
  
  delay(5000); // Espera 1 segundo

  Serial.println("Moviendo para RETRAER...");
  // Mueve el motor de vuelta a la posición "cero" y espera a que termine
  pusherStepper.runToNewPosition(0);
  
  delay(000); // Espera 1 segundo
}