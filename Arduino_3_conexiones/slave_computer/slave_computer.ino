#include <esp_now.h>
#include <WiFi.h>
#include "src/RoboticsUB.h"
#include <ESP32Servo.h>

enum class Command : byte
{
  GET_RPW = 1
};

Command command = Command::GET_RPW;

// Direccion MAC del master
uint8_t masterMacAddress[] = {0x94, 0xb9, 0x7e, 0xc4, 0x90, 0x30};

//estructura de datos que se recibe del master
typedef struct {
  float roll;
  float pitch;
  float yaw;
  float current_roll;
  float current_pitch;
  float current_yaw1;
  float current_yaw2;
  int s1_robot;
  int s2_robot;
  int s1_pinzas;
  int s2_pinzas;
} RxMessage;

//variable de tipo estructura para guardar lo que llega del master
RxMessage dataFromMaster;

/*DECLARACION DE FUNCIONES QUE VAMOS A UTILIZAR*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  /*
   * nos llegan los datos del master y los copiamos en el slave
   */
   memcpy(&dataFromMaster, incomingData, sizeof(dataFromMaster));
   //memcpy(destination, source, size)
}

/*SETUP DEL PROGRAMA*/
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW (computer slave)");
    return;
  }
  
  //Registramos el master
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  //comprobamos que se ha registrado bien el master     
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add master");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

/*BUCLE DE REPRODUCCION CONTINUA*/
void loop() {
  if (Serial.available() > 0)
  {
    command = (Command)Serial.read();

    switch (command)
    {
    case Command::GET_RPW:
      Serial.println(dataFromMaster.roll,DEC);
      Serial.println(dataFromMaster.pitch,DEC);
      Serial.println(dataFromMaster.yaw,DEC);
      Serial.println(dataFromMaster.current_roll,DEC);
      Serial.println(dataFromMaster.current_pitch,DEC);
      Serial.println(dataFromMaster.current_yaw1,DEC);
      Serial.println(dataFromMaster.current_yaw2,DEC);
      Serial.println(dataFromMaster.s1_robot,DEC);
      Serial.println(dataFromMaster.s2_robot,DEC);
      Serial.println(dataFromMaster.s1_pinzas,DEC);
      Serial.println(dataFromMaster.s2_pinzas,DEC);
      break;
    
    }
  }
  

  delay(10);
}
