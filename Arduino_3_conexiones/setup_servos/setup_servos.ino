#include <esp_now.h>
#include <WiFi.h>
#include "src/RoboticsUB.h"
#include <ESP32Servo.h>

//variables de identificacion para los servos
Servo servo_roll;
Servo servo_pitch;
Servo servo_yaw;
Servo servo_gripper;

//variables de identificacion de los pines

/*ROLL*/
const int PIN_ANALOG_ROLL = 34;
const int PIN_SIGNAL_ROLL = 33;
/*PITCH*/
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 25;
/*YAW*/
const int PIN_ANALOG_YAW = 35;
const int PIN_SIGNAL_YAW = 32;
/*GRIPPER*/
const int PIN_ANALOG_GRIPPER = 36;
const int PIN_SIGNAL_GRIPPER = 27;
/*SHUNT*/
float Rshunt = 1.6;

/*DECLARACION DE LAS FUNCIONES A UTILIZAR*/
float getCurrent(uint32_t integrationTimeMs, int servo) {
  /*
   * leemos el ADC donde esta conectada la shunt para obtener el corriente
   * una vez tenemos la lectura, convertimos el dato del ADC para poder aplicar V/R
   */
  uint32_t startTime = millis();
  float integratedCurrent = 0;

  // Vamos sumando la medicion de corriente durante el tiempo fijado -> integrationTimeMs (en milisegundos)
  while(millis()< startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(servo);
    integratedCurrent = integratedCurrent + ((float)adcValue/4095.0 * 3.3)/Rshunt;
  }  
  return integratedCurrent;
}


/*SETUP DEL PROGRAMA*/
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  //comprobamos que se inicializa bien nuestro slave
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW (slave)");
    return;
  }

  //configuramos los PWM que alimentan los servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //designamos un frecuencia a los servos
  servo_roll.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_yaw.setPeriodHertz(50);
  servo_gripper.setPeriodHertz(50);

  //asignamos los pines a cada servo
  servo_roll.attach(PIN_SIGNAL_ROLL);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_gripper.attach(PIN_SIGNAL_GRIPPER);

  //iniciamos los servos 
  servo_roll.write(0);
  delay(500);
  servo_pitch.write(0);
  delay(500);
  servo_yaw.write(0);
  delay(500);
  servo_gripper.write(0);
  delay(1000);
  
for(int r=0;r<181;r++){
    servo_roll.write(r);
    float current = getCurrent(20, PIN_ANALOG_ROLL);
    Serial.println(current);
    if(current > 10.0){
      r = 181;
      Serial.print("finalizado engranaje de roll\n");
    }
  }
  delay(1000);
  
for(int p=0;p<160;p++){
    servo_pitch.write(p);
    float current = getCurrent(20, PIN_ANALOG_PITCH);
    Serial.println(current);

    if(current > 20.0){
      p = 161;
      Serial.print("finalizado engranaje de pitch\n");
    }
  }
  delay(1000);
    
  /*for(int y=0;y<181;y++){
    servo_yaw.write(y);
    float current = getCurrent(20, PIN_ANALOG_YAW);
    Serial.println(current);
          Serial.print("finalizado engranaje de yaw\n");
    if(current > 1000.0){
      y = 181;
      Serial.print("finalizado engranaje de yaw\n");
    }
  }
  delay(3000);*/
    
for(int g=0;g<90;g++){
    servo_gripper.write(g);
    float current = getCurrent(20, PIN_ANALOG_GRIPPER);
    Serial.println(current);
    if(current > 15.0){
      g = 91;
      Serial.print("finalizado engranaje de gripper\n");
    }
  }
  delay(1000);
}

/*BUCLE DE REPRODUCCION CONTINUA*/
void loop(){

}
