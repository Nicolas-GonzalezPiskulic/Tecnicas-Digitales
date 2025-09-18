// --- CÓDIGO PARA EL ESP32 DE CONTROL (PRUEBA CON LED) ---

#include <Stepper.h>

// --- CONFIGURACIÓN ---
#define SERIAL_COMMS_RX 16
// CAMBIO: El pin 23 ahora controlará un LED en vez de un relé.
#define LED_PIN 23

const int FRAME_WIDTH = 96;
const float CONFIDENCE_THRESHOLD = 0.75; 

// --- MOTOR A PASOS ---
const int stepsPerRevolution = 2048; 
const int motorSpeed = 15;           
const int stepsToMove = 100;

// Pines del driver ULN2003
Stepper myStepper(stepsPerRevolution, 26, 33, 25, 32);

// --- LÓGICA DE SEGUIMIENTO Y DISPARO ---
const int FRAME_CENTER = FRAME_WIDTH / 2;
const int CENTER_DEAD_ZONE = 20; 

// --- VARIABLES GLOBALES ---
String serialBuffer = "";
unsigned long lastDetectionTime = 0;
const int DETECTION_TIMEOUT = 1000; 

unsigned long revStartTime = 0;
bool isRevving = false; // "isRevving" ahora significa "isLedOn"
const int MIN_REV_TIME = 2000; 

unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 1000; 

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL_COMMS_RX, -1);
  myStepper.setSpeed(motorSpeed);
  
  // Configura el pin del LED como salida y asegúrate de que esté apagado
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Controlador (Prueba LED) listo. Esperando datos...");
}

void loop()
{
  while (Serial1.available() > 0)
  {
    char incomingChar = Serial1.read();
    Serial.print(incomingChar); 
    
    if (incomingChar == '\n')
    {
      if (serialBuffer.indexOf("x: ") != -1 && serialBuffer.indexOf("width: ") != -1)
      {
        parseAndMoveMotor(serialBuffer);
      }
      serialBuffer = "";
    }
    else
    {
      serialBuffer += incomingChar;
    }
  }

  if (millis() - lastDetectionTime > DETECTION_TIMEOUT) {
    if (lastDetectionTime != 0) {
      if (millis() - lastPrintTime > PRINT_INTERVAL) {
        Serial.println("\nESTADO: Objetivo perdido (Timeout). Apagando LED y buscando...");
        lastPrintTime = millis();
      }
      digitalWrite(LED_PIN, LOW);
      isRevving = false;
      myStepper.step(300);
      delay(50);
      myStepper.step(-600);
      delay(50);
      myStepper.step(300);
      lastDetectionTime = 0;
    }
  }
}

void parseAndMoveMotor(String data)
{
  int openParenIndex = data.indexOf('(');
  int closeParenIndex = data.indexOf(')');
  if (openParenIndex == -1 || closeParenIndex == -1) return;
  float confidence = data.substring(openParenIndex + 1, closeParenIndex).toFloat();

  if (confidence >= 0.99) {
    if (millis() - lastPrintTime > PRINT_INTERVAL) {
      Serial.println("\nESTADO: Detección ignorada (Confianza perfecta sospechosa).");
      lastPrintTime = millis();
    }
    return;
  }
  
  if (confidence < CONFIDENCE_THRESHOLD) {
    return; 
  }
  
  lastDetectionTime = millis();
  
  int x_index = data.indexOf("x: ");
  int width_index = data.indexOf("width: ");
  if (x_index == -1 || width_index == -1) return;
  int x = data.substring(x_index + 3).toInt();
  int width = data.substring(width_index + 7).toInt();
  int object_center = x + (width / 2);

  bool shouldPrint = millis() - lastPrintTime > PRINT_INTERVAL;

  if (object_center < FRAME_CENTER - CENTER_DEAD_ZONE)
  {
    if(shouldPrint) Serial.printf("\nESTADO: Moviendo Izquierda (Conf: %.2f | Centro: %d)\n", confidence, object_center);
    myStepper.step(-stepsToMove);
    if (isRevving && (millis() - revStartTime > MIN_REV_TIME)) {
      digitalWrite(LED_PIN, LOW);
      isRevving = false;
    }
  }
  else if (object_center > FRAME_CENTER + CENTER_DEAD_ZONE)
  {
    if(shouldPrint) Serial.printf("\nESTADO: Moviendo Derecha (Conf: %.2f | Centro: %d)\n", confidence, object_center);
    myStepper.step(stepsToMove);
    if (isRevving && (millis() - revStartTime > MIN_REV_TIME)) {
      digitalWrite(LED_PIN, LOW);
      isRevving = false;
    }
  }
  else
  {
    if(shouldPrint) Serial.println("\nESTADO: ¡Objetivo centrado! Encendiendo LED...");
    if (!isRevving) {
      digitalWrite(LED_PIN, HIGH);
      revStartTime = millis();
      isRevving = true;
    }
  }

  if(shouldPrint) {
    lastPrintTime = millis();
  }
}