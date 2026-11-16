#include "joy.h"

JOY joy;

void JOY ::init(void)
{
    JOY_PORT.begin(JOY_BAUD_RATE);
    clear();
}

// read every byte waiting in the UART, call often
void JOY ::update(void)
{
    while (JOY_PORT.available() > 0)
    {
        uint8_t data = JOY_PORT.read();
        bytes++;

        if (counter == 0 && data != '#')
        {
            continue;
        }
        if (counter == 1 && data != 's')
        {
            counter = (data == '#') ? 1 : 0;
            continue;
        }

        rxBuffer[counter++] = data;

        if (counter >= JOY_RX_SIZE)
        {
            counter = 0;
            if (rxBuffer[JOY_RX_SIZE - 2] == '\r' && rxBuffer[JOY_RX_SIZE - 1] == '\n')
            {
                decode();
            }
            else
            {
                bad_frames++;
            }
        }
    }

    if (isConnected && millis() - count > JOY_TIMEOUT)
    {
        clear();
    }
}

static int8_t stick(uint8_t raw)
{
    int8_t v = map(raw, 0, 255, -127, 127);
    if (abs(v) < JOY_DEADZONE)
    {
        v = 0;
    }
    return v;
}

void JOY ::decode(void)
{
    count = millis();
    frame_count++;

    // joy not paired on the ESP32
    if (!(rxBuffer[18] & 0x01))
    {
        clear();
        return;
    }

    Lx = rxBuffer[2];
    Ly = rxBuffer[3];
    L2 = rxBuffer[4];
    Rx = rxBuffer[5];
    Ry = rxBuffer[6];
    R2 = rxBuffer[7];

    _Lx = stick(Lx);
    _Ly = stick(Ly);
    _Rx = stick(Rx);
    _Ry = stick(Ry);

    uint8_t btnLeft = rxBuffer[16];
    up = btnLeft & 0x01;
    right = (btnLeft >> 1) & 0x01;
    down = (btnLeft >> 2) & 0x01;
    left = (btnLeft >> 3) & 0x01;
    L1 = (btnLeft >> 4) & 0x01;
    share = (btnLeft >> 5) & 0x01;

    uint8_t btnRight = rxBuffer[17];
    triangle = btnRight & 0x01;
    circle = (btnRight >> 1) & 0x01;
    cross = (btnRight >> 2) & 0x01;
    square = (btnRight >> 3) & 0x01;
    R1 = (btnRight >> 4) & 0x01;
    option = (btnRight >> 5) & 0x01;

    isConnected = 1;
}

// neutral sticks, no buttons
void JOY ::clear(void)
{
    isConnected = 0;

    Lx = Ly = Rx = Ry = 127;
    L2 = R2 = 0;
    _Lx = _Ly = _Rx = _Ry = 0;

    up = right = down = left = L1 = share = 0;
    triangle = circle = cross = square = R1 = option = 0;
}

void JOY ::debug(void)
{
    if (!isConnected)
    {
        // bytes = 0 -> nothing on the wire, frames rising -> ESP32 alive but no joy paired
        Serial.printf("joy off | bytes %lu frames %lu bad %lu\n", bytes, frame_count, bad_frames);
        return;
    }

    Serial.printf("joy on  | Lx %4d Ly %4d Rx %4d Ry %4d L2 %3d R2 %3d | ", _Lx, _Ly, _Rx, _Ry, L2, R2);
    Serial.printf("L1 %d R1 %d | up %d right %d down %d left %d | ", L1, R1, up, right, down, left);
    Serial.printf("tri %d cir %d cro %d squ %d | share %d opt %d | frames %lu bad %lu\n",
                  triangle, circle, cross, square, share, option, frame_count, bad_frames);
}

// last good frame as hex
void JOY ::debugRaw(void)
{
    Serial.print("raw :");
    for (uint8_t i = 0; i < JOY_RX_SIZE; i++)
    {
        Serial.printf(" %02X", rxBuffer[i]);
    }
    Serial.println();
}
