/*
 * @Descripttion: 
 * @version: 
 * @Author: Elegoo
 * @Date: 2020-06-04 11:42:27
 * @LastEditors: Changhua
 * @LastEditTime: 2020-09-07 09:40:03
 */
//#include <EEPROM.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <esp_now.h>

#define RXD2 33
#define TXD2 4
#define TELEMETRY      1
#define IMAGE_PART     2

bool WA_en = false;
#define my_mac_address ="EC:62:60:87:18:84"
#define controller_mac_address ="C0:49:EF:65:55:60";
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0x65, 0x55, 0x60};
String arduino_response_Buff;
String last_status="Init";

int loop_counter =0;

float r_x =0;
float r_y =0;
float l_x =0;
float l_y =0;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float r_x;
    float r_y;
    float l_x;
    float l_y;
} struct_message;


typedef struct image_part_message {
    uint8_t type;
    uint8_t part_number;
    char *part_buf[64];
    
} image_part_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingControl_instructions;

typedef struct tank_struct_message {
    int message_type;
    float ultrasound_distance;
    char state[30];
    
} tank_struct_message;
tank_struct_message outgoing_tank_data;

esp_now_peer_info_t peerInfo; 
// Variable to store if sending data was successful
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingControl_instructions, incomingData, sizeof(incomingControl_instructions));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  r_x = incomingControl_instructions.r_x;
  r_y = incomingControl_instructions.r_y;
  l_x = incomingControl_instructions.l_x;
  l_y = incomingControl_instructions.l_y;
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
   // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  
  //http://192.168.4.1/control?var=framesize&val=3
  //http://192.168.4.1/Test?var=
  //CameraWebServerAP.CameraWebServer_AP_Init();
  //server.begin();
  //delay(100);
  // while (Serial.read() >= 0)
  // {
  //   /*清空串口缓存...*/
  // }
  // while (Serial2.read() >= 0)
  // {
  //   /*清空串口缓存...*/
  // }
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.println("Elegoo-2020...");
  //Serial2.print("{Factory}");
  outgoing_tank_data.ultrasound_distance = 1;
  outgoing_tank_data.message_type=TELEMETRY;
}
void loop()
{
  
  //Serial.print(r_x);
  //Serial.print(",");
  //Serial.print(r_y);
  //Serial.print(",");
  //Serial.print(l_x);
  //Serial.print(",");
  //Serial.print(l_y);
  //Serial.println("");
  loop_counter=loop_counter+1;
  
  int d1 = map(l_x,0,4095,-255,255)- map(l_y,0,4095,-255,255);
  int d2 = map(l_x,0,4095,-255,255)+ map(l_y,0,4095,-255,255);
  d1 = constrain(d1,-255,255);
  d2 = constrain(d2,-255,255);
  String control_message = "{N:4,D1:"+String(d1)+",D2:"+String(d2)+"}";
  
  if (loop_counter % 2){ 
    //Serial2.print("{N:3,D1:3,D2:50}");
    Serial2.print(control_message); //set motor speed
    
    //Serial2.print("{N:5,D1:1,D2:1,D3:100}");
  }else{
    Serial2.print("{N:21,D1:2}"); //request ultrasonic value
  }
  if (loop_counter>1000){
    loop_counter=0;
  }
  if (Serial2.available())
  {
    char c = Serial2.read();
    arduino_response_Buff += c;
    Serial.println(c);
    while (c != '}'){
      Serial.println("read char from serial2");
      c = Serial2.read();
      Serial.print(c);
      
      if (c == '}')
      {
        Serial.println("command terminator reached: " +arduino_response_Buff); 
        if (arduino_response_Buff.startsWith("ultrasounddist:")){
            String dist = arduino_response_Buff.substring(15);
            //Serial.println("ultrasound_distance_report");
            //Serial.println(dist);
            //Serial.println("end_ultrasound_distance_report");
            outgoing_tank_data.ultrasound_distance=dist.toInt();
        }else{
          last_status=arduino_response_Buff;
        }
        arduino_response_Buff = "";
        
      }else{
        arduino_response_Buff += c;
      }
    }
    
  }else{
    Serial.println("serial2 not available");
    //last_status="Arduino unavailable";
  }
  
  
  char status_char_array[30];
  last_status.toCharArray(status_char_array,30);
  
  strcpy(outgoing_tank_data.state,status_char_array);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing_tank_data, sizeof(outgoing_tank_data));
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    //Serial.println("Error sending the data");
  }
}
