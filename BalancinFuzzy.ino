#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>
#include <Fuzzy.h>

MPU6050 mpu(Wire);
Servo escDer;
Servo escIzq;

Fuzzy *difuso = new Fuzzy();

float setpoint = -10.0;      
unsigned long T_anterior = 0;

int pwm_min = 1000;
int pwm_max = 2000;
int pwm_base_der = 1080;
int pwm_base_izq = 1080;
float vel_filtrada = 0.0; 

// GANANCIAS VALIDADAS EN SIMULINK
float Ke = 0.2; 
float Ki = 0.1;  
float error_integral = 0.0;
void setupFuzzy();

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

  setupFuzzy();

  Serial.println(F("Iniciando ESC y Cargando Reglas Difusas..."));
  delay(3000); 
  Serial.println(F("¡Sistema Fuzzy-PID Iniciado!"));
  T_anterior = micros();
}

void loop() {
  // --- LECTURA DE COMANDOS ---
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

  float error = setpoint - angulo_grados;
  if (abs(error) < 1.0) error = 0; 
  
  float derivada_error = -velocidad_grados_s; 

  error_integral += error * dt;
  error_integral = constrain(error_integral, -20.0, 20.0);
  float accion_integral = -(error_integral * Ki);

  float entrada_1 = constrain(error * Ke, -15.0, 15.0);
  float entrada_2 = constrain(derivada_error, -80.0, 80.0);

  difuso->setInput(1, entrada_1);
  difuso->setInput(2, entrada_2);
  difuso->fuzzify();
  float u_fuzzy = difuso->defuzzify(1);

  float u_total = (u_fuzzy + accion_integral) * -1.0;
  u_total = constrain(u_total, -250.0, 250.0);
  int pwm_der = pwm_base_der + u_total;
  int pwm_izq = pwm_base_izq - u_total;

  int limite_inferior = 1100; 
  escDer.writeMicroseconds(constrain(pwm_der, limite_inferior, pwm_max));
  escIzq.writeMicroseconds(constrain(pwm_izq, limite_inferior, pwm_max));
  
  Serial.print(setpoint); Serial.print(",");
  Serial.print(angulo_grados); Serial.print(",");
  Serial.println(u_total);
  
  delay(4); 
}

void setupFuzzy() {
  // ENTRADA 1: ERROR Rango: [-15, 15]
  FuzzyInput *error = new FuzzyInput(1);
  
  FuzzySet *err_NG = new FuzzySet(-15, -15, -6, -2); 
  FuzzySet *err_NP = new FuzzySet(-6, -2, -2, 0);    
  FuzzySet *err_Z  = new FuzzySet(-2, 0, 0, 2);      
  FuzzySet *err_PP = new FuzzySet(0, 2, 2, 6);       
  FuzzySet *err_PG = new FuzzySet(2, 6, 15, 15);     
  
  error->addFuzzySet(err_NG); error->addFuzzySet(err_NP); error->addFuzzySet(err_Z);
  error->addFuzzySet(err_PP); error->addFuzzySet(err_PG);
  difuso->addFuzzyInput(error);
  
  // ENTRADA 2: VELOCIDAD ANGULAR Rango: [-80, 80]
  FuzzyInput *vel = new FuzzyInput(2);
  
  FuzzySet *vel_NG = new FuzzySet(-80, -80, -40, -10); 
  FuzzySet *vel_NP = new FuzzySet(-40, -10, -10, 0);   
  FuzzySet *vel_Z  = new FuzzySet(-10, 0, 0, 10);      
  FuzzySet *vel_PP = new FuzzySet(0, 10, 10, 40);      
  FuzzySet *vel_PG = new FuzzySet(10, 40, 80, 80);    
  
  vel->addFuzzySet(vel_NG); vel->addFuzzySet(vel_NP); vel->addFuzzySet(vel_Z);
  vel->addFuzzySet(vel_PP); vel->addFuzzySet(vel_PG);
  difuso->addFuzzyInput(vel);

  // SALIDA 1: ESFUERZO DE CONTROL Rango: [-250, 250]
  FuzzyOutput *esfuerzo = new FuzzyOutput(1);
  
  FuzzySet *esf_NG = new FuzzySet(-250, -250, -100, -20); 
  FuzzySet *esf_NP = new FuzzySet(-100, -20, -20, 0);     
  FuzzySet *esf_Z  = new FuzzySet(-20, 0, 0, 20);        
  FuzzySet *esf_PP = new FuzzySet(0, 20, 20, 100);        
  FuzzySet *esf_PG = new FuzzySet(20, 100, 250, 250);     
  
  esfuerzo->addFuzzySet(esf_NG); esfuerzo->addFuzzySet(esf_NP); esfuerzo->addFuzzySet(esf_Z);
  esfuerzo->addFuzzySet(esf_PP); esfuerzo->addFuzzySet(esf_PG);
  difuso->addFuzzyOutput(esfuerzo);

  // LAS 25 REGLAS (FAM MATRIX)
  FuzzyRuleConsequent *then_NG = new FuzzyRuleConsequent(); then_NG->addOutput(esf_NG);
  FuzzyRuleConsequent *then_NP = new FuzzyRuleConsequent(); then_NP->addOutput(esf_NP);
  FuzzyRuleConsequent *then_Z  = new FuzzyRuleConsequent(); then_Z->addOutput(esf_Z);
  FuzzyRuleConsequent *then_PP = new FuzzyRuleConsequent(); then_PP->addOutput(esf_PP);
  FuzzyRuleConsequent *then_PG = new FuzzyRuleConsequent(); then_PG->addOutput(esf_PG);

  // REGLAS PARA ERROR NG (Reglas 1 al 5)
  FuzzyRuleAntecedent *if_errNG_velNG = new FuzzyRuleAntecedent(); if_errNG_velNG->joinWithAND(err_NG, vel_NG);
  difuso->addFuzzyRule(new FuzzyRule(1, if_errNG_velNG, then_PG));
  FuzzyRuleAntecedent *if_errNG_velNP = new FuzzyRuleAntecedent(); if_errNG_velNP->joinWithAND(err_NG, vel_NP);
  difuso->addFuzzyRule(new FuzzyRule(2, if_errNG_velNP, then_PG));
  FuzzyRuleAntecedent *if_errNG_velZ = new FuzzyRuleAntecedent(); if_errNG_velZ->joinWithAND(err_NG, vel_Z);
  difuso->addFuzzyRule(new FuzzyRule(3, if_errNG_velZ, then_PG));
  FuzzyRuleAntecedent *if_errNG_velPP = new FuzzyRuleAntecedent(); if_errNG_velPP->joinWithAND(err_NG, vel_PP);
  difuso->addFuzzyRule(new FuzzyRule(4, if_errNG_velPP, then_PP));
  FuzzyRuleAntecedent *if_errNG_velPG = new FuzzyRuleAntecedent(); if_errNG_velPG->joinWithAND(err_NG, vel_PG);
  difuso->addFuzzyRule(new FuzzyRule(5, if_errNG_velPG, then_Z));

  // REGLAS PARA ERROR NP (Reglas 6 al 10)
  FuzzyRuleAntecedent *if_errNP_velNG = new FuzzyRuleAntecedent(); if_errNP_velNG->joinWithAND(err_NP, vel_NG);
  difuso->addFuzzyRule(new FuzzyRule(6, if_errNP_velNG, then_PG));
  FuzzyRuleAntecedent *if_errNP_velNP = new FuzzyRuleAntecedent(); if_errNP_velNP->joinWithAND(err_NP, vel_NP);
  difuso->addFuzzyRule(new FuzzyRule(7, if_errNP_velNP, then_PP));
  FuzzyRuleAntecedent *if_errNP_velZ = new FuzzyRuleAntecedent(); if_errNP_velZ->joinWithAND(err_NP, vel_Z);
  difuso->addFuzzyRule(new FuzzyRule(8, if_errNP_velZ, then_PP));
  FuzzyRuleAntecedent *if_errNP_velPP = new FuzzyRuleAntecedent(); if_errNP_velPP->joinWithAND(err_NP, vel_PP);
  difuso->addFuzzyRule(new FuzzyRule(9, if_errNP_velPP, then_Z));
  FuzzyRuleAntecedent *if_errNP_velPG = new FuzzyRuleAntecedent(); if_errNP_velPG->joinWithAND(err_NP, vel_PG);
  difuso->addFuzzyRule(new FuzzyRule(10, if_errNP_velPG, then_NP));

  // REGLAS PARA ERROR Z (Reglas 11 al 15)
  FuzzyRuleAntecedent *if_errZ_velNG = new FuzzyRuleAntecedent(); if_errZ_velNG->joinWithAND(err_Z, vel_NG);
  difuso->addFuzzyRule(new FuzzyRule(11, if_errZ_velNG, then_PP));
  FuzzyRuleAntecedent *if_errZ_velNP = new FuzzyRuleAntecedent(); if_errZ_velNP->joinWithAND(err_Z, vel_NP);
  difuso->addFuzzyRule(new FuzzyRule(12, if_errZ_velNP, then_PP));
  FuzzyRuleAntecedent *if_errZ_velZ = new FuzzyRuleAntecedent(); if_errZ_velZ->joinWithAND(err_Z, vel_Z);
  difuso->addFuzzyRule(new FuzzyRule(13, if_errZ_velZ, then_Z));
  FuzzyRuleAntecedent *if_errZ_velPP = new FuzzyRuleAntecedent(); if_errZ_velPP->joinWithAND(err_Z, vel_PP);
  difuso->addFuzzyRule(new FuzzyRule(14, if_errZ_velPP, then_NP));
  FuzzyRuleAntecedent *if_errZ_velPG = new FuzzyRuleAntecedent(); if_errZ_velPG->joinWithAND(err_Z, vel_PG);
  difuso->addFuzzyRule(new FuzzyRule(15, if_errZ_velPG, then_NP));

  // REGLAS PARA ERROR PP (Reglas 16 al 20)
  FuzzyRuleAntecedent *if_errPP_velNG = new FuzzyRuleAntecedent(); if_errPP_velNG->joinWithAND(err_PP, vel_NG);
  difuso->addFuzzyRule(new FuzzyRule(16, if_errPP_velNG, then_PP));
  FuzzyRuleAntecedent *if_errPP_velNP = new FuzzyRuleAntecedent(); if_errPP_velNP->joinWithAND(err_PP, vel_NP);
  difuso->addFuzzyRule(new FuzzyRule(17, if_errPP_velNP, then_Z));
  FuzzyRuleAntecedent *if_errPP_velZ = new FuzzyRuleAntecedent(); if_errPP_velZ->joinWithAND(err_PP, vel_Z);
  difuso->addFuzzyRule(new FuzzyRule(18, if_errPP_velZ, then_NP));
  FuzzyRuleAntecedent *if_errPP_velPP = new FuzzyRuleAntecedent(); if_errPP_velPP->joinWithAND(err_PP, vel_PP);
  difuso->addFuzzyRule(new FuzzyRule(19, if_errPP_velPP, then_NP));
  FuzzyRuleAntecedent *if_errPP_velPG = new FuzzyRuleAntecedent(); if_errPP_velPG->joinWithAND(err_PP, vel_PG);
  difuso->addFuzzyRule(new FuzzyRule(20, if_errPP_velPG, then_NG));

  // REGLAS PARA ERROR PG (Reglas 21 al 25)
  FuzzyRuleAntecedent *if_errPG_velNG = new FuzzyRuleAntecedent(); if_errPG_velNG->joinWithAND(err_PG, vel_NG);
  difuso->addFuzzyRule(new FuzzyRule(21, if_errPG_velNG, then_Z));
  FuzzyRuleAntecedent *if_errPG_velNP = new FuzzyRuleAntecedent(); if_errPG_velNP->joinWithAND(err_PG, vel_NP);
  difuso->addFuzzyRule(new FuzzyRule(22, if_errPG_velNP, then_NP));
  FuzzyRuleAntecedent *if_errPG_velZ = new FuzzyRuleAntecedent(); if_errPG_velZ->joinWithAND(err_PG, vel_Z);
  difuso->addFuzzyRule(new FuzzyRule(23, if_errPG_velZ, then_NG));
  FuzzyRuleAntecedent *if_errPG_velPP = new FuzzyRuleAntecedent(); if_errPG_velPP->joinWithAND(err_PG, vel_PP);
  difuso->addFuzzyRule(new FuzzyRule(24, if_errPG_velPP, then_NG));
  FuzzyRuleAntecedent *if_errPG_velPG = new FuzzyRuleAntecedent(); if_errPG_velPG->joinWithAND(err_PG, vel_PG);
  difuso->addFuzzyRule(new FuzzyRule(25, if_errPG_velPG, then_NG));
}
