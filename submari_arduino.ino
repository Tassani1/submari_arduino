// Programa que segons la inclinació mesurada amb un giroscop/acceleròmetre activi una bomba o una altra.

// Libreries per controlar el mpu6050, IMU (Inertial Measurment Units).
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

// Valors RAW (sense processar) de l'aceleròmetre en els eixos x,y,z
int ax, ay, az;
// Valor inicial del Joystick
int joystickYRepos;

// Control dels motors
const int motorA = 12; // Groc-Verd
const int motorB = 13; // Blau-Vermell

// Pin de Y del joystick
const int pinJoystickY = A1;

void setup() {
  joystickYRepos = analogRead(pinJoystickY);
  setupAccelerometre();
  setupBombes();
}

void loop() {
  // Llegeix acceleracions
  sensor.getAcceleration(&ax, &ay, &az);
  //Calcula angles d'inclinació:
  float accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);

  // Llegeix la posició del Joystick
  int joystickY = analogRead(pinJoystickY);

  //Mostra dades
  Serial.print("X: ");
  Serial.print(accel_ang_x);
  Serial.print("\tY:");
  Serial.print(accel_ang_y);
  Serial.print("\tjoystickYRepos:");
  Serial.print(joystickYRepos);
  Serial.print("\tJoystick:");
  Serial.println(joystickY);

  if (joystickY < joystickYRepos - 200) {
    encendreMotorA();
  } else if (joystickY > joystickYRepos + 200) {
    encendreMotorB();
  }
  else {
    if (accel_ang_y>25) {
      apagarMotors();
      encendreMotorA();
    }
    else if (accel_ang_y<-25){
      apagarMotors();
      encendreMotorB();
    }
    else {
      apagarMotors();
    }
  }
  delay(10);
}

void setupAccelerometre() {
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciat correctament");
  else Serial.println("Error en iniciar el sensor");
}

void setupBombes() {
  // Configura els pins de control als outputs
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
}

void encendreMotorA() {
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, HIGH);
}

void encendreMotorB() {
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, LOW);
}

void apagarMotors() {
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, HIGH);
}
