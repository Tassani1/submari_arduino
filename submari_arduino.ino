// Programa que segons la inclinació mesurada amb un giroscop/acceleròmetre activi una bomba o una altra.

// Libreries per controlar el mpu6050, IMU (Inertial Measurment Units).
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

// Valors RAW (sense processar) de l'aceleròmetre en els eixos x,y,z
int ax, ay, az;

// Connexions del Motor A 
const int enA = 9;
const int inA1 = 8;
const int inA2 = 7;

// Connexions del Motor B
const int enB = 3;
const int inB1 = 5;
const int inB2 = 4;

// Pin de Y del joystick
const int pinJoystickY = A1;

void setup() {
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
  Serial.print("\tJoystick:");
  Serial.println(joystickY);

  
   
   if (joystickY < 400) {
    encendreMotorA();
  } else if (joystickY > 600) {
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
  
  /*
  encendreMotorA();
  delay(1000);
  apagarMotors();
  delay(1500);
  encendreMotorB();
  delay(1000);
  apagarMotors();
  delay(1500);
  */
  delay(10);
  
}

void setupAccelerometre() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void setupBombes() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);
}

void encendreMotorA() {
 analogWrite(enA, 255);
 digitalWrite(inA1, HIGH);
 digitalWrite(inA2, LOW);
}

void encendreMotorB() {
 analogWrite(enB, 255);
 digitalWrite(inB1, HIGH);
 digitalWrite(inB2, LOW);
}

void apagarMotors() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);
}
