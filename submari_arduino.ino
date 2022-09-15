// Programa que segons la inclinació mesurada amb un giroscop/acceleròmetre activi una bomba o una altra.

// Libreries per controlar el mpu6050, IMU (Inertial Measurment Units).
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Llibreria per sprintf
#include "stdio.h"

// Llibreria per controlar el display LCD.
#include <LiquidCrystal.h>

MPU6050 sensor;

// Control dels motors
const int motorA = 12; // Groc-Verd
const int motorB = 13; // Blau-Vermell

// Pin de Y del joystick
const int pinJoystickY = A1;

// Valors RAW (sense processar) de l'aceleròmetre en els eixos x,y,z
int ax, ay, az;
// Valors de l'acceleració
float accel_ang_x, accel_ang_y;
// Valor inicial del Joystick
int joystickYRepos;
// Objecte per controlar el LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup() {
  joystickYRepos =llegeixPosicioJoystick();
  setupAccelerometre();
  setupBombes();
  lcd.begin(16, 2);
}

void loop() {
  llegeixAcceleracions();
  int joystickY = llegeixPosicioJoystick();

  if (joystickY < joystickYRepos - 200) {
    encenMotorA();
  } else if (joystickY > joystickYRepos + 200) {
    encenMotorB();
  }
  else {
    if (accel_ang_y>25) {
      apagaMotors();
      encenMotorA();
    }
    else if (accel_ang_y<-25){
      apagaMotors();
      encenMotorB();
    }
    else {
      apagaMotors();
    }
  }
  mostraDades(accel_ang_x, accel_ang_y, joystickY, joystickYRepos);
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

void encenMotorA() {
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, HIGH);
}

void encenMotorB() {
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, LOW);
}

void apagaMotors() {
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, HIGH);
}

void mostraDades(int accel_ang_x, int accel_ang_y, int joystickY, int joystickYRepos) {
  char xy[16];
  sprintf(xy, "X: %3d Y: %3d", accel_ang_x, accel_ang_y);
  lcd.setCursor(0,0);
  lcd.print(xy);
  lcd.setCursor(0, 1);
  lcd.print("Joystick:");
  lcd.print(joystickY);

  Serial.print("X: ");
  Serial.print(accel_ang_x);
  Serial.print("\tY:");
  Serial.print(accel_ang_y);
  Serial.print("\tjoystickYRepos:");
  Serial.print(joystickYRepos);
  Serial.print("\tJoystick:");
  Serial.println(joystickY);
}

void llegeixAcceleracions() {
  // Llegeix acceleracions
  sensor.getAcceleration(&ax, &ay, &az);
  //Calcula angles d'inclinació:
  accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
}

int llegeixPosicioJoystick() {
  return analogRead(pinJoystickY);
}
