/*

Ver imagen de conexion !!!!

probado con:
MKS8825   fuente a 12v y corriente a 0.50 aprox
A4988                  y corriente a 0.12 aprox

Tomado de internet. Lo que importa son las conexiones dado que el 
motor puede solo vibrar sin girar si son incorrectas.

*/


// Define pin connections & motor's steps per revolution
const int dirPin = 32;
const int stepPin = 33;
const int stepsPerRevolution = 200;
const int velocity=750; // A menor numero mayor velocidad (750 es lo maximo)

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);

	// Spin motor slowly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(velocity);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(velocity);
	}
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);

	// Spin motor quickly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(velocity);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(velocity);
	}
	delay(1000); // Wait a second
}