#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

MPU6050 mpu(Wire);
Servo escDer;
Servo escIzq;  

Matrix<2, 2> A = {0.0,  1.0, 
                  0.0, -0.625};
Matrix<2, 1> B = {0.0, 
                  0.0327};
Matrix<1, 2> C = {1.0, 0.0};

//Matriz del observador
Matrix<2, 1> L = {10.49, 
                  44.97}; 
Matrix<1, 3> K = {63.03, 48.86, -27.39}; //Ganancias del servosistema LQR

Matrix<2, 1> x_hat = {0.0, 0.0}; 
Matrix<1, 1> y_medido = {0.0};   
Matrix<1, 1> u_matriz = {0.0};

float setpoint = 0.0;      
float error_integral = 0.0;
unsigned long T_anterior = 0;

int pwm_min = 1000;
int pwm_max = 2000;
int pwm_base = 1100;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  escDer.attach(18, pwm_min, pwm_max); 
  escIzq.attach(19, pwm_min, pwm_max);
  
  escDer.writeMicroseconds(1000);
  escIzq.writeMicroseconds(1000);

  if (mpu.begin() != 0) {
    Serial.println(F("Error MPU6050"));
    while(1){} 
  }
  mpu.setAccOffsets(-0.04, -0.02, 0.00);
  mpu.setGyroOffsets(-2.71, 0.38, -1.14);

  Serial.println(F("Iniciando ESC"));
  delay(3000); 
  Serial.println(F("Sistema Iniciado"));
  T_anterior = micros();
}

void loop() {
  if (Serial.available() > 0) {
    String dato = Serial.readStringUntil('\n');
    setpoint = dato.toFloat();
  }
  mpu.update();
  unsigned long T_actual = micros();
  float dt = (T_actual - T_anterior) / 1000000.0;
  T_anterior = T_actual;
  
  if (dt <= 0.0 || dt > 0.1) return; 

  float angulo_grados = -1.0 * mpu.getAngleX(); 
  float velocidad_grados_s = -1.0 * mpu.getGyroX();   
  
  float theta = angulo_grados * (PI / 180.0);
  float theta_dot = velocidad_grados_s * (PI / 180.0);
  float referencia = setpoint * (PI / 180.0);

  y_medido(0,0) = theta;   
  Matrix<2, 1> x_dot_hat = A * x_hat + B * u_matriz + L * (y_medido - C * x_hat);
  x_hat += x_dot_hat * dt;

  float error = referencia - theta;
  if (abs(error) < 0.008) error = 0; 

  error_integral += error * dt;
  error_integral = constrain(error_integral, -5.0, 5.0); 

  Matrix<3, 1> z;
  z(0,0) = theta - referencia; 
  z(1,0) = theta_dot;          
  z(2,0) = error_integral;     

  u_matriz = -K * z;
  float u = u_matriz(0,0);

  int pwm_der = pwm_base + u;
  int pwm_izq = pwm_base - u;

  escDer.writeMicroseconds(constrain(pwm_der, pwm_min, pwm_max));
  escIzq.writeMicroseconds(constrain(pwm_izq, pwm_min, pwm_max));
  
  Serial.print(setpoint); Serial.print(",");
  Serial.print(angulo_grados); Serial.print(",");
  Serial.println(x_hat(0,0) * (180.0 / PI));

  delay(4); 
}
  