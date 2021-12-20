#pragma once
#include <Arduino.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <WiFiUdp.h>

#if defined(ESP8266) || defined(ARDUINO_ESP8266_NODEMCU)
#include <ESP8266WiFi.h>
#endif
#if defined(ESP32)
#include <WiFi.h>
#endif

#define RP_WIFI 0
#define RP_NRF24 1

class Radio_Pult
{
public:
    void begin(uint8_t netmod = RP_WIFI, byte ce = 0, byte cs = 0);
    bool Priem();
    void init_nrf();
    void PowerControl(byte *h, byte min_value = 0);
    String Name = "";
    byte *p_mode = b_read;      // код описания протокола 0 - прием данных управления с пульта
    byte *x1 = b_read + 1;      // левый джойстик ось *X1 УКАЗАТЕЛЬ
    byte *y1 = b_read + 2;      // левый джойстик ось *Y1 УКАЗАТЕЛЬ
    byte *x2 = b_read + 3;      // првый джойстик ось *X2 УКАЗАТЕЛЬ
    byte *y2 = b_read + 4;      // правый джойстик ось *Y2 УКАЗАТЕЛЬ
    byte *key_xy1 = b_read + 5; // кнопка левого джойстика
    byte *key_xy2 = b_read + 6; // кнопка правого джойстика
    byte *key1 = b_read + 7;    // левая кнопка на панели
    byte *key2 = b_read + 8;    // средняя кнопка на понели
    byte *key3 = b_read + 9;    // правая кнопка на панели
    byte *L_fire = b_read + 10; // кнопка левая "ОГОНЬ"
    byte *R_fire = b_read + 11; // кнопка правая "ОГОНЬ"
    int udp_port = 4220;        // Порт подключения по UDP
    byte power_control = 0;

private:
    //char name[19] = "Кораблик"; // Имя устройства выводится в меню пульта управления
    byte set_ch = 255; // Канал для подключения NRF модуля
    byte net_mod = RP_WIFI;
    bool _connect = false; // состояние связи
    uint8_t pip[6];        // труба для связи присылает пульт
    char s[20] = "#";
    RF24 radio;
    WiFiUDP udp;
    int err = 0;
    unsigned long t1 = 100000;
    uint8_t b_read[32]; // буфер для приема данных

    /* структура пакета данных от пульта управления
        msg[0]  - назначение протокола данных 0 - данные с пульта
        msg[1]  - левый джойстик X1
        msg[2]  - левый джойстик Y1
        msg[3]  - правый джойстик X1
        msg[4]  - правый джойстик Y1
        msg[5]  - кнопка левого джойстика
        msg[6]  - кнопка правого джойстика
        msg[7]  - кнопка слева
        msg[8]  - кнопка по центру 
        msg[9]  - кнопка справа
        msg[10] - левый огонь
        msg[11] - правый огонь
    */
    char ssid[20] = "-";               //Ship Kater";  //Имя точки доступа WIFI
    const char *password = "85371571"; //пароль точки доступа WIFI
};
extern Radio_Pult RadioPult;
