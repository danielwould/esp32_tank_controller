// All the mcufriend.com UNO shields have the same pinout.
// i.e. control pins A0-A4.  Data D2-D9.  microSD D10-D13.
// Touchscreens are normally A1, A2, D7, D6 but the order varies
//
// This demo should work with most Adafruit TFT libraries
// If you are not using a shield,  use a full Adafruit constructor()
// e.g. Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define DEBUG 0
// *** enable following is for ESP32 ***
#define LCD_CS 33 // Chip Select ESP32 GPIO33
#define LCD_RS 15 // LCD_RS = Register Select or LCD_CD = Command/Data
#define LCD_WR 4 // LCD Write goes to ESP32 GPIO4
#define LCD_RD 2 // LCD Read goes to ESP32 GPIO2
#define LCD_RESET 32 // ESP32 GPIO32

#define left_switch 5 //left joy push button
#define left_y 34 //left joy y axis
#define left_x 35 //left joy x axis

#define right_switch 18 //left joy push button
#define right_y 39 //left joy y axis
#define right_x 36 //left joy x axis
#define TELEMETRY      1
#define IMAGE_PART     2

// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "WiFi.h"
#include <esp_now.h>
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

// #include <TJpg_Decoder.h>
#include <JPEGDecoder.h>

//#include <Adafruit_TFTLCD.h>
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define my_mac_address ="C0:49:EF:65:55:60";
#define tank_mac_address ="EC:62:60:87:18:84"
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x87, 0x18, 0x84};


#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

void setup(void);
void loop(void);
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();

int r_x=0;
int r_y=0;
int l_x=0;
int l_y=0;
int r_b=0;
int l_b=0;

int l_x_offset=0;
int l_y_offset=0;

int r_x_offset=0;
int r_y_offset=0;


String prev_text_l="";
String prev_text_r="";
String new_text_l="";
String new_text_r="";
void progmemPrint(const char *str);
void progmemPrintln(const char *str);

int loop_counter=0;
void runtests(void);
bool connected=false;
uint16_t g_identifier;

float ultrasound_distance=0;
String tank_status ="";
float last_ultrasound_distance=0;
String last_tank_status ="";

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float r_x;
    float r_y;
    float l_x;
    float l_y;
    bool send_image;
} struct_message;
// Create a struct_message to hold incoming sensor readings
struct_message outgoungControl_instructions;

float last_r_x_sent=0;
float last_r_y_sent=0;
float last_l_x_sent=0;
float last_l_y_sent=0;


typedef struct {
  uint8_t type;
} common_t;

typedef struct tank_struct_message {
    uint8_t type;
    float ultrasound_distance;
    char state[30];
    
} tank_struct_message;

typedef struct image_part_message {
    uint8_t type;
    int part_number;
    int total_parts;
    int total_size;
    uint8_t part_buf[220];
    
} image_part_message;

tank_struct_message incoming_tank_data;

int width=320;
int height=480;

uint8_t *fb_ptr;
uint8_t *last_image;
bool image_available=false;
int image_buffer_len=0;
bool memallocated=false;

esp_now_peer_info_t peerInfo; 
// Variable to store if sending data was successful
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
    connected=true;
    
  }
  else{
    success = "Delivery Fail :(";
    
    connected=false;
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incommingData, int len) {
  common_t *common = (common_t*)incommingData;
  
  if (common->type == TELEMETRY) {
    //Serial.println("recevied telemtry packet");
    //tank_struct_message *incoming_tank_data = (tank_struct_message*)data;
    // Handle messages that have telemetry data
    memcpy(&incoming_tank_data, incommingData, sizeof(incoming_tank_data));
    //Serial.print(incoming_tank_data.ultrasound_distance);
    ultrasound_distance = incoming_tank_data.ultrasound_distance;
    tank_status = String(incoming_tank_data.state);
  } else if (common->type == IMAGE_PART) {
    //Serial.println("recevied image fragment");
    
    image_part_message incoming_image_part; 
    
    memcpy(&incoming_image_part, incommingData, sizeof(incoming_image_part));
    
    // Handle message that contain image fragments
    //Serial.println("fragment " +String(incoming_image_part.part_number)+ " of " + String(incoming_image_part.total_parts));
    int start=incoming_image_part.part_number*220;
    if (incoming_image_part.part_number==0){
       //Serial.println("allocating memory for incoming image of size " + String(incoming_image_part.total_size));
       free(fb_ptr);
       fb_ptr = (uint8_t*)malloc(incoming_image_part.total_size);
       outgoungControl_instructions.send_image=false;
       //Serial.println("Memory allocated");
       
       
       memallocated=true;
    }
    if (memallocated){
      //Serial.println("fragement startpoint "+String(start) + " for length " + String(sizeof(incoming_image_part.part_buf)));
      //uint8_t* buffer = (uint8_t*)heap_caps_malloc(50 * sizeof(uint8_t), MALLOC_CAP_32BIT);
      memcpy(fb_ptr + start, incoming_image_part.part_buf, sizeof(incoming_image_part.part_buf));
      //Serial.println("framebuffer now "+ *fb_ptr);
      if (incoming_image_part.part_number==incoming_image_part.total_parts-1){
         //Serial.println("Finished receinving image");
         //for (int i = 0; i < incoming_image_part.total_size; i++){
         // Serial.print(fb_ptr[i] < 16 ? "0" : "");
         // Serial.print(fb_ptr[i],HEX);
         // Serial.print(" ");
         //}
         
         while (image_available){
           delay(1);
         }
         free(last_image);
         image_buffer_len=incoming_image_part.total_size;
         last_image = (uint8_t*)malloc(image_buffer_len);
         memcpy(last_image, fb_ptr, image_buffer_len);
         image_available=true;
         outgoungControl_instructions.send_image=true;
         // uint16_t w = 0, h = 0;
         //uint16_t w = 0, h = 0;
         //TJpgDec.getJpgSize(&w, &h, last_image, incoming_image_part.total_size);
         //Serial.print("Width = "); Serial.print(w); Serial.print(", height = "); Serial.println(h);
  
          
      }
    }
  }
      
}


void setup(void) {
    Serial.begin(115200);
    uint32_t when = millis();
    //    while (!Serial) ;   //hangs a Leonardo until you connect a Serial
    if (!Serial) delay(5000);           //allow some time for Leonardo
    Serial.println("Serial took " + String((millis() - when)) + "ms to start");
    //    tft.reset();                 //hardware reset
    uint16_t ID = tft.readID(); //
    Serial.print("ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
//    ID = 0x9329;                             // force ID
    tft.begin(ID);

    tft.fillScreen(BLACK);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.setRotation( 1 );
    WiFi.mode(WIFI_MODE_STA);
    Serial.println(WiFi.macAddress());
    tft.setCursor(0, 80);
    tft.setTextColor(GREEN);
    //static labels
    tft.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
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

    //calibrate joysticks
    r_x = analogRead(right_x);
    r_y = analogRead(right_y);
  
    l_x = analogRead(left_x);
    l_y = analogRead(left_y);

    float midpoint = 4095/2;
    l_x_offset=l_x-midpoint;
    l_y_offset=l_y-midpoint;

    r_x_offset=r_x-midpoint;
    r_y_offset=r_y-midpoint;
}

#if defined(MCUFRIEND_KBV_H_)
uint16_t scrollbuf[320];    // my biggest screen is 320x480
#define READGRAM(x, y, buf, w, h)  tft.readGRAM(x, y, buf, w, h)
#else
uint16_t scrollbuf[320];    // Adafruit only does 240x320
// Adafruit can read a block by one pixel at a time
int16_t  READGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h)
{
    uint16_t *p;
    for (int row = 0; row < h; row++) {
        p = block + row * w;
        for (int col = 0; col < w; col++) {
            *p++ = tft.readPixel(x + col, y + row);
        }
    }
}
#endif

void windowScroll(int16_t x, int16_t y, int16_t wid, int16_t ht, int16_t dx, int16_t dy, uint16_t *buf)
{
    if (dx) for (int16_t row = 0; row < ht; row++) {
            READGRAM(x, y + row, buf, wid, 1);
            tft.setAddrWindow(x, y + row, x + wid - 1, y + row);
            tft.pushColors(buf + dx, wid - dx, 1);
            tft.pushColors(buf + 0, dx, 0);
        }
    if (dy) for (int16_t col = 0; col < wid; col++) {
            READGRAM(x + col, y, buf, 1, ht);
            tft.setAddrWindow(x + col, y, x + col, y + ht - 1);
            tft.pushColors(buf + dy, ht - dy, 1);
            tft.pushColors(buf + 0, dy, 0);
        }
}

void printmsg(int row, const char *msg)
{
    tft.setTextColor(YELLOW, BLACK);
    tft.setCursor(0, row);
    tft.println(msg);
}

void loop(void) {
    tft.fillScreen(BLACK);
    outgoungControl_instructions.send_image=true;
    image_buffer_len = 0;
    image_available=false;
    loop_counter=0;
    tft.setCursor(0, 0);
    tft.setTextColor(GREEN);
    //static labels
    tft.println("right x,y :");
    //static labels
    tft.println("left x,y  :");
    
    
    while (loop_counter <1000){
      loop_counter=loop_counter+1;

      
      if (image_available){
        Serial.println("Drawing image to screen");
      // Get the width and height in pixels of the jpeg if you wish
        //TJpgDec.drawJpg(0, 0, last_image, image_buffer_len); // Runs until complete image decoded
        drawArrayJpeg(last_image, image_buffer_len, 0, 0);
        image_available=false;
        outgoungControl_instructions.send_image=true;
      }
      
      //read joysticks
      r_x = analogRead(right_x);
      r_y = analogRead(right_y);
  
      l_x = analogRead(left_x);
      l_y = analogRead(left_y);

      outgoungControl_instructions.r_x = r_x-r_x_offset;
      outgoungControl_instructions.r_y = r_x-r_y_offset;
      outgoungControl_instructions.l_x = l_x-l_x_offset;
      outgoungControl_instructions.l_y = l_y-l_y_offset;
      
      new_text_r = String(outgoungControl_instructions.r_x)+ ", " + String(outgoungControl_instructions.r_y);
      new_text_l = String(outgoungControl_instructions.l_x)+ ", " + String(outgoungControl_instructions.l_y);
      tft.setTextColor(BLACK); //blank previous values
      
      tft.setCursor(160, 0);
      tft.print(prev_text_r);
      tft.setCursor(160, 25);
      tft.print(prev_text_l);

      tft.setCursor(10, 150);
      tft.print(last_tank_status);
      tft.setCursor(10, 175);
      tft.print(last_ultrasound_distance);
      
      tft.setTextColor(GREEN); //write new value
      tft.setCursor(160, 0);
      tft.print(new_text_r);
      tft.setCursor(160, 25);
      tft.print(new_text_l);
      tft.setCursor(10, 150);
      last_tank_status=tank_status;
      tft.print(last_tank_status);
      tft.setCursor(10, 175);
      last_ultrasound_distance=ultrasound_distance;
      tft.print(last_ultrasound_distance);
      
      prev_text_l=new_text_l;
      prev_text_r=new_text_r;
      
      
      if (outgoungControl_instructions.send_image || outgoungControl_instructions.r_x != last_r_x_sent || outgoungControl_instructions.r_y != last_r_y_sent || outgoungControl_instructions.l_x != last_l_x_sent || outgoungControl_instructions.l_y != last_l_y_sent){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoungControl_instructions, sizeof(outgoungControl_instructions));
        last_r_x_sent=outgoungControl_instructions.r_x;
        last_r_y_sent=outgoungControl_instructions.r_y;
        last_l_x_sent=outgoungControl_instructions.l_x;
        last_l_y_sent=outgoungControl_instructions.l_y;
        if (result == ESP_OK) {
          //Serial.println("Sent with success");
        }
        else {
          //Serial.println("Error sending the data");
          tank_status="offline";
        }
        
      }
      
      
      if (connected){
        tft.drawCircle(400, 300, 10, GREEN);
      }else{
        tft.drawCircle(400, 300, 10, RED);
        tank_status="offline";
      }
    }
}


bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;
   tft.drawRGBBitmap(x, y, bitmap, w, h);
   // Return 1 to decode next block
   return 1;
}
