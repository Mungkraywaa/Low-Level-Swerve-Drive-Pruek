#pragma once
#include <Arduino.h>

// ---- change these to move the joy link to another port (see README) ----
// Teensy 4.1 hardware serial pins (RX / TX):
//   Serial1 0/1   Serial2 7/8   Serial3 15/14  Serial4 16/17
//   Serial5 21/20 Serial6 25/24 Serial7 28/29  Serial8 34/35
#define JOY_PORT Serial8
#define JOY_BAUD_RATE 115200 // must match COM_BAUD_RATE in ps4_esp32/include/serial.h
// -------------------------------------------------------------------------

#define JOY_RX_SIZE 21
#define JOY_TIMEOUT 200 // ms without a valid frame = disconnected
#define JOY_DEADZONE 20 // of 127

// frame : '#' 's' Lx Ly L2 Rx Ry R2 touch[8] btnLeft btnRight btnCenter '\r' '\n'
//   btnLeft   : bit0 up, bit1 right, bit2 down, bit3 left, bit4 L1, bit5 share
//   btnRight  : bit0 triangle, bit1 circle, bit2 cross, bit3 square, bit4 R1, bit5 options
//   btnCenter : bit0 joy connected
typedef struct
{
    // raw data, stick center = 127, up = 255
    uint8_t Lx = 127;
    uint8_t Ly = 127;
    uint8_t Rx = 127;
    uint8_t Ry = 127;

    // map data -127 -> 127, deadzone applied
    int8_t _Lx = 0;
    int8_t _Ly = 0;
    int8_t _Rx = 0;
    int8_t _Ry = 0;

    uint8_t L2 = 0;
    uint8_t R2 = 0;
    bool L1 = 0;
    bool R1 = 0;

    bool triangle = 0;
    bool circle = 0;
    bool cross = 0;
    bool square = 0;
    bool option = 0;

    bool up = 0;
    bool right = 0;
    bool down = 0;
    bool left = 0;
    bool share = 0;

    bool isConnected = 0; // frames arriving and joy paired on the ESP32

    // link statistics, for checking the wiring
    uint32_t bytes = 0;       // every byte received
    uint32_t frame_count = 0; // frames with a good '\r' '\n' tail
    uint32_t bad_frames = 0;  // frames with a wrong tail
    uint32_t count = 0;       // millis() of the last good frame

    uint8_t rxBuffer[JOY_RX_SIZE];
    uint8_t counter = 0;

    void init(void);
    void update(void);
    void decode(void);
    void clear(void);
    void debug(void);
    void debugRaw(void);

} JOY;

extern JOY joy;
