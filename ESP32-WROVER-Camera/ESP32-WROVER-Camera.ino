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
#define CAMERA_MODEL_M5STACK_WIDE
#include "esp_camera.h"
#include "camera_pins.h"
#include <esp_now.h>
#include "img_converters.h"
#include "fb_gfx.h"

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

bool sending_image=false;
int loop_counter =0;

float r_x =0;
float r_y =0;
float l_x =0;
float l_y =0;
bool send_image=false;
bool sending=false;
//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float r_x;
    float r_y;
    float l_x;
    float l_y;
    bool send_image;
} struct_message;


typedef struct image_part_message {
    uint8_t type;
    int part_number;
    int total_parts;
    int total_size;
    uint8_t part_buf[220];
    
} image_part_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingControl_instructions;

typedef struct tank_struct_message {
    uint8_t type;
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
  if (sending!=true){
    send_image = incomingControl_instructions.send_image;
    if (send_image){
  
      Serial.println("send image request");
    }
  }
}

/* ***************************************************************** */
/* INIT CAMERA                                                       */
/* ***************************************************************** */
void initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;//20000000
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 30;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_HVGA); // 480x320
  //s->set_framesize(s, FRAMESIZE_QVGA); // 320x240

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
#endif
  s->set_vflip(s, 0);   
  s->set_hmirror(s, 0); 
  Serial.println("Camera initialised");
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
  //esp_now_register_send_cb(OnDataSent);
  
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
  initCamera();
  Serial.println("Elegoo-2020...");
  //Serial2.print("{Factory}");
  outgoing_tank_data.ultrasound_distance = 1;
  outgoing_tank_data.type=TELEMETRY;
}
void loop()
{
  
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
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
  if (loop_counter > 2000){
    loop_counter=0;
  }
  if (loop_counter % 2){ 
    //Serial2.print("{N:3,D1:3,D2:50}");
    Serial2.print(control_message); //set motor speed
    
    //Serial2.print("{N:5,D1:1,D2:1,D3:100}");
  }else{
    Serial2.print("{N:21,D1:2}"); //request ultrasonic value
  }
  if (loop_counter % 10 && send_image){ //only send images on request.
    sending=true;
    
    camera_fb_t * fb = NULL;

    // Take Picture with Camera
    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
    }
    else
    {
        Serial.println("Camera capture obtained");
        size_t out_len, out_width, out_height;
        uint8_t *out_buf;
        bool s;
        int width=480;
        int height=320;
        size_t fb_len = 0;
        
        if (fb->format == PIXFORMAT_JPEG)
        {
            image_part_message image_part;
            image_part.type=IMAGE_PART;
            
            //Serial.println("pixelformat is jpeg");
            fb_len = fb->len;
            image_part.total_size=fb_len;
            Serial.println(fb_len);
            //for (int i = 0; i < fb_len; i++){
            //  Serial.print(fb->buf[i] < 16 ? "0" : "");
            //  Serial.print(fb->buf[i],HEX);
            //  Serial.print(" ");
            //}
            //divide fb into chunks we can send over ESPNow 250 byte limited messages
            
            //res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
            int chunk_size = 220;
            int num_chunks = fb_len / chunk_size;
            image_part.total_parts=num_chunks;
            Serial.println("Sending "+ String(num_chunks)+ " chunks of image");
            for (int i = 0; i < num_chunks; i++) {
              // Calculate the start and end positions of the current chunk
              int start = i * chunk_size;
              int end = start + chunk_size;
              if (end > width * height) {
                end = width * height;
              }
              
              image_part.part_number=i;
              // Allocate a buffer for the current chunk
              int chunk_size_bytes = (end - start) * sizeof(uint8_t);
              //uint8_t* chunk = (uint8_t*)malloc(chunk_size_bytes);
            
              // Copy the current chunk from the framebuffer to the buffer
              memcpy(image_part.part_buf, fb->buf + start, chunk_size_bytes);
              
              //Serial.println(sizeof(image_part));
              // Send the current chunk as an ESPNow message
              esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &image_part, sizeof(image_part));
           //   if (result == ESP_OK) {
              delay(15);
           //     //Serial.println("Sent with success");
           //   }
           //   else {
           //     while (result != ESP_OK){
           //       delay(10);
           //       //Serial.println("Error sending image data");
           //       result = esp_now_send(broadcastAddress, (uint8_t *) &image_part, sizeof(image_part));
           //       if (result == ESP_OK) {
           //         //Serial.println("Sent with success");
           //         
           //       }
           //       else {
           //         Serial.println("still failed "+String(result)
           //         +"send on retry of " + String(image_part.part_number));
           //       }
           //     }
            //  }
              
              
            }
        }

        esp_camera_fb_return(fb);
        fb = NULL;
        _jpg_buf = NULL;
        sending=false;
    }
  }
  if (Serial2.available())
  {
    char c = Serial2.read();
    arduino_response_Buff += c;
    //Serial.println(c);
    while (c != '}'){
      //Serial.println("read char from serial2");
      c = Serial2.read();
      //Serial.print(c);
      
      if (c == '}')
      {
        //Serial.println("command terminator reached: " +arduino_response_Buff); 
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
