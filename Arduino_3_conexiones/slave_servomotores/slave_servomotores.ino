#include <esp_now.h>
#include <WiFi.h>
#include "src/RoboticsUB.h"
#include <ESP32Servo.h>

// Direccion MAC del master
uint8_t masterMacAddress[] = {0x94, 0xb9, 0x7e, 0xc4, 0x90, 0x30};

//variables de identificacion para los servos
Servo servo_roll;
Servo servo_pitch;
Servo servo_yaw1;
Servo servo_yaw2;

float NewValueRoll = 0;         // variable to store the read value
float OldValueRoll = 0; 
float roll=0;
float valRoll=0;

float NewValuePitch=0;
float OldValuePitch=0;
float pitch=0;
float valPitch=0;

float NewValueYaw=0;
float OldValueYaw=0;
float yaw=0;
float valYaw=0;

float NewValueYaw2=0;
float OldValueYaw2=0;
float yaw2=0;
float valYaw2=0;

//variables de identificacion de los pines

/*ROLL*/
const int PIN_ANALOG_ROLL = 34;
const int PIN_SIGNAL_ROLL = 33;
/*PITCH*/
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 25;
/*YAW*/
const int PIN_ANALOG_YAW1 = 35;
const int PIN_SIGNAL_YAW1 = 32;
/*GRIPPER*/
const int PIN_ANALOG_YAW2 = 36;
const int PIN_SIGNAL_YAW2 = 27;

//constantes para las resistencias shunt (son todas iguales)
const float Rshunt = 1.6;

//variables para almacenar el dato leido correspondiente al torque de los servos
float current_roll = 0;
float current_pitch = 0;
float current_yaw1 = 0;
float current_yaw2 = 0;

//variables de estado de los botones
int s1_status;
int s2_status;

//estructura de datos que enviara este slave
typedef struct {
  float current_roll;
  float current_pitch;
  float current_yaw1;
  float current_yaw2;
} TxMessage;

//variable del tipo estructura para enviar datos
TxMessage dataServos;

//estructura de datos que recibiremos del master
typedef struct {
  float roll;
  float pitch;
  float yaw;
  int s1_pinzas;
  int s2_pinzas;
} RxMessage;

//variable del tipo estructura para recibir datos
RxMessage dataPencil;

/*DECLARACION DE LAS FUNCIONES A UTILIZAR*/
float getCurrent(uint32_t integrationTimeMs, int servo) {
  /*
     leemos el ADC donde esta conectada la shunt para obtener el corriente
     una vez tenemos la lectura, convertimos el dato del ADC para poder aplicar V/R
  */
  uint32_t startTime = millis();
  float integratedCurrent = 0;

  // Vamos sumando la medicion de corriente durante el tiempo fijado -> integrationTimeMs (en milisegundos)
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(servo);
    integratedCurrent = integratedCurrent + ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  /*
     nos llegan los datos del master y los copiamos en el slave
  */
  memcpy(&dataPencil, incomingData, sizeof(dataPencil));
  //memcpy(destination, source, size)
}

/*SETUP DEL PROGRAMA*/
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  //comprobamos que se inicializa bien nuestro slave
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW (slave)");
    return;
  }

  // Registramos el master
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // comprobamos si se ha podido añadir el master
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add master");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  //configuramos los PWM que alimentan los servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //designamos un frecuencia a los servos
  servo_roll.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_yaw1.setPeriodHertz(50);
  servo_yaw2.setPeriodHertz(50);

  //asignamos los pines a cada servo
  servo_roll.attach(PIN_SIGNAL_ROLL);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_yaw1.attach(PIN_SIGNAL_YAW1);
  servo_yaw2.attach(PIN_SIGNAL_YAW2);

  //configuramos los pines analogicos de los servos como entradas
  pinMode(PIN_ANALOG_ROLL, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_YAW1, INPUT);
  pinMode(PIN_ANALOG_YAW2, INPUT);

  //iniciamos los servos en la posicion central (0º)
  servo_roll.write(90);
  servo_pitch.write(90);
  servo_yaw1.write(90);
  servo_yaw2.write(90);
  delay(2000);
  
}

/*BUCLE DE REPRODUCCION CONTINUA*/
void loop() {
  //empezamos enviando los datos de la corriente de los servos
  esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *) &dataServos, sizeof(dataServos));

  //comprobamos que se han enviado bien los datos
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  NewValueRoll=dataPencil.roll;
  valRoll=NewValueRoll-OldValueRoll;
  if (valRoll>10) {
    roll=OldValueRoll;
  } 
  if (valRoll<10) {
    roll=NewValueRoll;
  }
  OldValueRoll=roll;

  NewValuePitch=dataPencil.pitch;
  valPitch=NewValuePitch-OldValuePitch;
  if (valPitch>10) {
    pitch=OldValuePitch;
  } 
  if (valPitch<10) {
    pitch=NewValuePitch;
  }
  OldValuePitch=pitch;
  

  NewValueYaw=dataPencil.yaw;
  valYaw=NewValueYaw-OldValueYaw;
  if (valYaw>10) {
    yaw=OldValueYaw;
  } 
  if (valYaw<10) {
    yaw=NewValueYaw;
  }
  OldValueYaw=yaw;

  
  
  //escribimos los datos recibidos del master en los servos
  servo_roll.write(roll);
  servo_pitch.write(pitch);
  servo_yaw1.write(yaw);
  servo_yaw2.write(180 - yaw);

  //definimos los límites para que no se mueva fuera de su rango de movimiento
  if (dataPencil.roll > 180 && dataPencil.roll < 270) {
    servo_roll.write(180);
  } else if (dataPencil.roll > 270 && dataPencil.roll < 360) {
    servo_roll.write(0);
  }

  if (dataPencil.yaw > 180 && dataPencil.yaw < 270) {
    servo_yaw1.write(180);
  } else if (dataPencil.yaw > 270 && dataPencil.yaw < 360) {
    servo_yaw1.write(0);
  }

  if (dataPencil.yaw > 180 && dataPencil.yaw < 270) {
    servo_yaw2.write(0);
  } else if (dataPencil.yaw > 270 && dataPencil.yaw < 360) {
    servo_yaw2.write(180);
  }

  if (dataPencil.pitch > 0 && dataPencil.pitch < 90) {
    servo_pitch.write(dataPencil.pitch + 90);
  } else if (dataPencil.pitch > 270 && dataPencil.pitch < 360) {
    servo_pitch.write(dataPencil.pitch - 270);
  }

  //abrimos y cerramos pinzas
  if (dataPencil.s1_pinzas == LOW || dataPencil.s2_pinzas == LOW) {
    if(dataPencil.yaw < 90){
      servo_yaw2.write(180 - dataPencil.yaw-30);
      delay(500);
      servo_yaw2.write(180 - dataPencil.yaw);
    }
  }
  if (dataPencil.s1_pinzas == LOW || dataPencil.s2_pinzas == LOW) {
    if(dataPencil.yaw > 90){
      servo_yaw1.write(dataPencil.yaw-30);
      delay(500);
      servo_yaw1.write(dataPencil.yaw);
   }
  }


  
  //llamamos a la funcion de lectura de la corriente para guardarla
  //en la estructura de datos de salida
  dataServos.current_roll = getCurrent(20, PIN_ANALOG_ROLL);
  dataServos.current_pitch = getCurrent(20, PIN_ANALOG_PITCH);
  dataServos.current_yaw1 = getCurrent(20, PIN_ANALOG_YAW1);
  dataServos.current_yaw2 = getCurrent(20, PIN_ANALOG_YAW2);
  Serial.println(dataServos.current_yaw2,DEC);
  Serial.println(dataServos.current_yaw1 ,DEC);
  delay(10);
}
