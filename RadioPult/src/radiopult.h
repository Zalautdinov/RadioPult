#pragma once
#include <Arduino.h>
#include "nRF24L01.h"
#include "RF24.h"

#define RP_WIFI 0
#define RP_NRF24 1

class Radio_Pult
{
public:
    void begin(uint8_t netmod = RP_WIFI);
    void servo(byte port, int t = 50);

private:
    byte ch = 255; // Канал для подключения NRF модуля
    byte net_mod = RP_WIFI;
    //uint8_t pip[6];            // труба для связи присылает пульт
    //uint8_t pip0[6] = "setup"; // труба для настройки
};
extern Radio_Pult RadioPult;
