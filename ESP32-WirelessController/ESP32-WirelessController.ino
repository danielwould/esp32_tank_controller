// All the mcufriend.com UNO shields have the same pinout.
// i.e. control pins A0-A4.  Data D2-D9.  microSD D10-D13.
// Touchscreens are normally A1, A2, D7, D6 but the order varies
//
// This demo should work with most Adafruit TFT libraries
// If you are not using a shield,  use a full Adafruit constructor()
// e.g. Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


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

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "WiFi.h"
#include <esp_now.h>
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
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
} struct_message;
// Create a struct_message to hold incoming sensor readings
struct_message outgoungControl_instructions;

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
    uint8_t part_number;
    char *part_buf[64];
    
} image_part_message;

tank_struct_message incoming_tank_data;

esp_now_peer_info_t peerInfo; 
// Variable to store if sending data was successful
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  common_t *common = (common_t*)data;
  if (common->type == TELEMETRY) {
    tank_struct_message *incoming_tank_data = (tank_struct_message*)data;
    // Handle message of type A
    ultrasound_distance = incoming_tank_data.ultrasound_distance;
    tank_status = String(incoming_tank_data.state);
  } else if (common->type == IMAGE_PART) {
    image_part_message *incoming_image_part = (image_part_message*)data;
    // Handle message of type B
    
  }
  
  memcpy(&incoming_tank_data, incomingData, sizeof(incoming_tank_data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  if (incoming_tank_data.message_type=TELEMETRY){
    ultrasound_distance = incoming_tank_data.ultrasound_distance;
    tank_status = String(incoming_tank_data.state);
  }
  
}

extern const uint8_t hanzi[];
void showhanzi(unsigned int x, unsigned int y, unsigned char index)
{
    uint8_t i, j, c, first = 1;
    uint8_t *temp = (uint8_t*)hanzi;
    uint16_t color;
    tft.setAddrWindow(x, y, x + 31, y + 31); //设置区域
    temp += index * 128;
    for (j = 0; j < 128; j++)
    {
        c = pgm_read_byte(temp);
        for (i = 0; i < 8; i++)
        {
            if ((c & (1 << i)) != 0)
            {
                color = RED;
            }
            else
            {
                color = BLACK;
            }
            tft.pushColors(&color, 1, first);
            first = 0;
        }
        temp++;
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
    loop_counter=0;
    tft.setCursor(0, 0);
    tft.setTextColor(GREEN);
    //static labels
    tft.println("right x,y :");
    //static labels
    tft.println("left x,y  :");

    while (loop_counter <1000){
      loop_counter=loop_counter+1;
     
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
      

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoungControl_instructions, sizeof(outgoungControl_instructions));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
        tank_status="offline";
      }
      if (connected){
        tft.drawCircle(400, 300, 10, GREEN);
      }else{
        tft.drawCircle(400, 300, 10, RED);
        tank_status="offline";
      }
    }
}

typedef struct {
    PGM_P msg;
    uint32_t ms;
} TEST;
TEST result[12];

#define RUNTEST(n, str, test) { result[n].msg = PSTR(str); result[n].ms = test; delay(500); }

void runtests(void)
{
    uint8_t i, len = 24, cnt;
    uint32_t total;
    RUNTEST(0, "FillScreen               ", testFillScreen());
    RUNTEST(1, "Text                     ", testText());
    RUNTEST(2, "Lines                    ", testLines(CYAN));
    RUNTEST(3, "Horiz/Vert Lines         ", testFastLines(RED, BLUE));
    RUNTEST(4, "Rectangles (outline)     ", testRects(GREEN));
    RUNTEST(5, "Rectangles (filled)      ", testFilledRects(YELLOW, MAGENTA));
    RUNTEST(6, "Circles (filled)         ", testFilledCircles(10, MAGENTA));
    RUNTEST(7, "Circles (outline)        ", testCircles(10, WHITE));
    RUNTEST(8, "Triangles (outline)      ", testTriangles());
    RUNTEST(9, "Triangles (filled)       ", testFilledTriangles());
    RUNTEST(10, "Rounded rects (outline)  ", testRoundRects());
    RUNTEST(11, "Rounded rects (filled)   ", testFilledRoundRects());

    tft.fillScreen(BLACK);
    tft.setTextColor(GREEN);
    tft.setCursor(0, 0);
    uint16_t wid = tft.width();
    if (wid > 176) {
        tft.setTextSize(2);
#if defined(MCUFRIEND_KBV_H_)
        tft.print("MCUFRIEND ");
#if MCUFRIEND_KBV_H_ != 0
        tft.print(0.01 * MCUFRIEND_KBV_H_, 2);
#else
        tft.print("for");
#endif
        tft.println(" UNO");
#else
        tft.println("Adafruit-Style Tests");
#endif
    } else len = wid / 6 - 8;
    tft.setTextSize(1);
    total = 0;
    for (i = 0; i < 12; i++) {
        PGM_P str = result[i].msg;
        char c;
        if (len > 24) {
            if (i < 10) tft.print(" ");
            tft.print(i);
            tft.print(": ");
        }
        uint8_t cnt = len;
        while ((c = pgm_read_byte(str++)) && cnt--) tft.print(c);
        tft.print(" ");
        tft.println(result[i].ms);
        total += result[i].ms;
    }
    tft.setTextSize(2);
    tft.print("Total:");
    tft.print(0.000001 * total);
    tft.println("sec");
    g_identifier = tft.readID();
    tft.print("ID: 0x");
    tft.println(tft.readID(), HEX);
//    tft.print("Reg(00):0x");
//    tft.println(tft.readReg(0x00), HEX);
    tft.print("F_CPU:");
    tft.print(0.000001 * F_CPU);
#if defined(__OPTIMIZE_SIZE__)
    tft.println("MHz -Os");
#else
    tft.println("MHz");
#endif

    delay(10000);
}

// Standard Adafruit tests.  will adjust to screen size

unsigned long testFillScreen() {
    unsigned long start = micros();
    tft.fillScreen(BLACK);
    tft.fillScreen(RED);
    tft.fillScreen(GREEN);
    tft.fillScreen(BLUE);
    tft.fillScreen(BLACK);
    return micros() - start;
}

unsigned long testText() {
    unsigned long start;
    tft.fillScreen(BLACK);
    start = micros();
    tft.setCursor(0, 0);
    tft.setTextColor(WHITE);  tft.setTextSize(1);
    tft.println("Hello World!");
    tft.setTextColor(YELLOW); tft.setTextSize(2);
    tft.println(123.45);
    tft.setTextColor(RED);    tft.setTextSize(3);
    tft.println(0xDEADBEEF, HEX);
    tft.println();
    tft.setTextColor(GREEN);
    tft.setTextSize(5);
    tft.println("Groop");
    tft.setTextSize(2);
    tft.println("I implore thee,");
    tft.setTextSize(1);
    tft.println("my foonting turlingdromes.");
    tft.println("And hooptiously drangle me");
    tft.println("with crinkly bindlewurdles,");
    tft.println("Or I will rend thee");
    tft.println("in the gobberwarts");
    tft.println("with my blurglecruncheon,");
    tft.println("see if I don't!");
    return micros() - start;
}

unsigned long testLines(uint16_t color) {
    unsigned long start, t;
    int           x1, y1, x2, y2,
                  w = tft.width(),
                  h = tft.height();

    tft.fillScreen(BLACK);

    x1 = y1 = 0;
    y2    = h - 1;
    start = micros();
    for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    t     = micros() - start; // fillScreen doesn't count against timing

    tft.fillScreen(BLACK);

    x1    = w - 1;
    y1    = 0;
    y2    = h - 1;
    start = micros();
    for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    t    += micros() - start;

    tft.fillScreen(BLACK);

    x1    = 0;
    y1    = h - 1;
    y2    = 0;
    start = micros();
    for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    t    += micros() - start;

    tft.fillScreen(BLACK);

    x1    = w - 1;
    y1    = h - 1;
    y2    = 0;
    start = micros();
    for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);

    return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
    unsigned long start;
    int           x, y, w = tft.width(), h = tft.height();

    tft.fillScreen(BLACK);
    start = micros();
    for (y = 0; y < h; y += 5) tft.drawFastHLine(0, y, w, color1);
    for (x = 0; x < w; x += 5) tft.drawFastVLine(x, 0, h, color2);

    return micros() - start;
}

unsigned long testRects(uint16_t color) {
    unsigned long start;
    int           n, i, i2,
                  cx = tft.width()  / 2,
                  cy = tft.height() / 2;

    tft.fillScreen(BLACK);
    n     = min(tft.width(), tft.height());
    start = micros();
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        tft.drawRect(cx - i2, cy - i2, i, i, color);
    }

    return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
    unsigned long start, t = 0;
    int           n, i, i2,
                  cx = tft.width()  / 2 - 1,
                  cy = tft.height() / 2 - 1;

    tft.fillScreen(BLACK);
    n = min(tft.width(), tft.height());
    for (i = n; i > 0; i -= 6) {
        i2    = i / 2;
        start = micros();
        tft.fillRect(cx - i2, cy - i2, i, i, color1);
        t    += micros() - start;
        // Outlines are not included in timing results
        tft.drawRect(cx - i2, cy - i2, i, i, color2);
    }

    return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
    unsigned long start;
    int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

    tft.fillScreen(BLACK);
    start = micros();
    for (x = radius; x < w; x += r2) {
        for (y = radius; y < h; y += r2) {
            tft.fillCircle(x, y, radius, color);
        }
    }

    return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
    unsigned long start;
    int           x, y, r2 = radius * 2,
                        w = tft.width()  + radius,
                        h = tft.height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    start = micros();
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            tft.drawCircle(x, y, radius, color);
        }
    }

    return micros() - start;
}

unsigned long testTriangles() {
    unsigned long start;
    int           n, i, cx = tft.width()  / 2 - 1,
                        cy = tft.height() / 2 - 1;

    tft.fillScreen(BLACK);
    n     = min(cx, cy);
    start = micros();
    for (i = 0; i < n; i += 5) {
        tft.drawTriangle(
            cx    , cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            tft.color565(0, 0, i));
    }

    return micros() - start;
}

unsigned long testFilledTriangles() {
    unsigned long start, t = 0;
    int           i, cx = tft.width()  / 2 - 1,
                     cy = tft.height() / 2 - 1;

    tft.fillScreen(BLACK);
    start = micros();
    for (i = min(cx, cy); i > 10; i -= 5) {
        start = micros();
        tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         tft.color565(0, i, i));
        t += micros() - start;
        tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         tft.color565(i, i, 0));
    }

    return t;
}

unsigned long testRoundRects() {
    unsigned long start;
    int           w, i, i2, red, step,
                  cx = tft.width()  / 2 - 1,
                  cy = tft.height() / 2 - 1;

    tft.fillScreen(BLACK);
    w     = min(tft.width(), tft.height());
    start = micros();
    red = 0;
    step = (256 * 6) / w;
    for (i = 0; i < w; i += 6) {
        i2 = i / 2;
        red += step;
        tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(red, 0, 0));
    }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    return micros() - start;
}

unsigned long testFilledRoundRects() {
    unsigned long start;
    int           i, i2, green, step,
                  cx = tft.width()  / 2 - 1,
                  cy = tft.height() / 2 - 1;

    tft.fillScreen(BLACK);
    start = micros();
    green = 256;
    step = (256 * 6) / min(tft.width(), tft.height());
    for (i = min(tft.width(), tft.height()); i > 20; i -= 6) {
        i2 = i / 2;
        green -= step;
        tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, green, 0));
    }

    return micros() - start;
}
