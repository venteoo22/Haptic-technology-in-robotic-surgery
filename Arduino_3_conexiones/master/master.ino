#include "src/RoboticsUB.h"
#include <esp_now.h>
#include <WiFi.h>

// Direccion MAC del slave que contiene los servos
uint8_t servosMacAddress[] = {0x7c, 0x9e, 0xbd, 0x61, 0xa1, 0x34};

// Direccion MAC del slave que se comunica con el ordenador
uint8_t computerMacAddress[] = {0x7c, 0x9e, 0xbd, 0x66, 0xe2, 0x58};

//variables de declaracion de los pines de la IMU y los botones
const int PIN_IMU_INT = 18;
const int PIN_S1 = 14;
const int PIN_S2 = 27;

IMU imu;

//variables de estado de los botones
int s1Status = HIGH;
int s2Status = HIGH;
int pinzas = false;

//variable para guardar los datos de roll, pitch y yaw
float *rpw;

//estructura de datos que enviara el master a los servos
typedef struct {
  float roll;
  float pitch;
  float yaw;
  int s1_pinzas;
  int s2_pinzas;
} TxMessage;

//variable tipo estructura para los datos de salida hacia los servos
TxMessage dataToServos;

//estructura de datos que enviara el master al ordenador
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
} Tx2Message;

//variable tipo estructura para los datos de salida hacia el ordenador
Tx2Message dataToComputer;
//estructura de datos que recibira el master de los servos
typedef struct {
  float current_roll;
  float current_pitch;
  float current_yaw1;
  float current_yaw2;
} RxMessage;

//variable tipo estructura para los datos de entrada de las corrientes
RxMessage dataCurrent;

/*DECLARACION DE FUNCIONES QUE UTILIZAREMOS*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  /*
     nos llegan los datos del slave de los servos y los copiamos en el master
  */
  memcpy(&dataCurrent, incomingData, sizeof(dataCurrent));
  //memcpy(destination, source, size)
}

/*SETUP DEL PROGRAMA*/
void setup() {
  Serial.begin(115200);
  //configuramos los pines de los botones y la imu como salidas
  pinMode(PIN_IMU_INT, INPUT_PULLUP);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  imu.Install();

  // Habilitamos el WiFi en modo estacion
  WiFi.mode(WIFI_STA);

  // comprobamos que el master se inicializa correctamente
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW (master)");
    return;
  }

  // Registramos el slave de los servos
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, servosMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //comprobamos que el slave de los servos se ha añadido bien
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add servo slave");
    return;
  }

  //Configuramos la funcion a utilizar cada vez que se reciba un mensaje
  esp_now_register_recv_cb(OnDataRecv);

  // Registramos el slave computer
  esp_now_peer_info_t peerInfo_computer = {};
  memcpy(peerInfo_computer.peer_addr, computerMacAddress, 6);
  peerInfo_computer.channel = 1;
  peerInfo_computer.encrypt = false;

  //comprobamos que el slave del ordenador se ha añadido bien
  if (esp_now_add_peer(&peerInfo_computer) != ESP_OK) {
    //Serial.println("Failed to add computer slave");
    return;
  }
}

/*BUCLE DE REPRODUCCION CONTINUA*/
void loop() {
  //para cada interrupcion de la imu leemos los datos que nos proporciona
  //guardamos esos datos en nuestra variable
  if (digitalRead(PIN_IMU_INT) == HIGH)
  {
    imu.ReadSensor();
    rpw = imu.GetRPW();
  }

  //leemos el estados de los botones (que inicialmente hemos forzado a HIGH)
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
  
  //si los dos botones estan pulsados a la vez hacemos una conmutacion de pinzas
  //esta variable controla si queremos mover el gripper o el brazo
  //inicialmente esta configurada para mover el brazo (FALSE)
  if (s1Status == LOW && s2Status == LOW) {
    pinzas ^= true;
  }

  //enviaremos los datos de la imu y los botones al slave de los servos
  esp_err_t data_servos = esp_now_send(servosMacAddress, (uint8_t *) &dataToServos, sizeof(dataToServos));
  delay(10);

  //comprobamos que se ha enviado bien el dato a los servos
  if (data_servos == ESP_OK) {
    //Serial.println("Sent with success to servos");
  } else {
    //Serial.println("Error sending the data to servos");
  }

  //enviamos tambien los datos al ordenador
  esp_err_t data_computer = esp_now_send(computerMacAddress, (uint8_t *) &dataToComputer, sizeof(dataToComputer));
  delay(10);

  //comprobamos que se ha enviado bien el dato al ordenador
  if (data_computer == ESP_OK) {
    //Serial.println("Sent with success to computer");
  } else {
    //Serial.println("Error sending the data to computer");
  }

  //guardamos todos los datos que tenemos que enviar a los distintos slaves

  //datos de la imu a los servos
  dataToServos.roll = rpw[0];
  dataToServos.pitch = rpw[1];
  dataToServos.yaw = rpw[2];


  delay(10);

  //datos de la imu y de corrientes al ordenador
  dataToComputer.roll = rpw[0];
  dataToComputer.pitch = rpw[1];
  dataToComputer.yaw = rpw[2];
  dataToComputer.current_roll = dataCurrent.current_roll;
  dataToComputer.current_pitch = dataCurrent.current_pitch;
  dataToComputer.current_yaw1 = dataCurrent.current_yaw1;
  dataToComputer.current_yaw2 = dataCurrent.current_yaw2;
  //algoritmo de decision para enviar los datos de los botones
  //si la variable pinzas es FALSE entonces se tiene que mover el brazo
  //se envia al ordenador los datos de los botones y a los servos que se queden quietos
  if (pinzas == false) {
    dataToComputer.s1_robot = s1Status;
    dataToComputer.s2_robot = s2Status;
    dataToComputer.s1_pinzas = true;
    dataToComputer.s2_pinzas = true;
    dataToServos.s1_pinzas = true;
    dataToServos.s2_pinzas = true;
  } else {
    //por el contrario si pinzas es TRUE a los servos se envia el status de los botones
    //y al robot que se quede quieto y que abra el gripper
    dataToServos.s1_pinzas = s1Status;
    dataToServos.s2_pinzas = s2Status;
    dataToComputer.s1_pinzas = s1Status;
    dataToComputer.s2_pinzas = s1Status;
    dataToComputer.s1_robot = true;
    dataToComputer.s2_robot = true;
  }

  delay (10);
  
}
