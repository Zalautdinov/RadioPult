#include <radiopult.h>

void Radio_Pult::begin(uint8_t netmod)
{
    net_mod = netmod;
}
void Radio_Pult::servo(byte port, int t)
{ 
    pinMode(port, OUTPUT);
    t = map(t, 0, 100, 540, 2400);
    digitalWrite(port, 1);
    delayMicroseconds(t);
    digitalWrite(port, 0);
    delayMicroseconds(20000 - t);
}

void Radio_Pult::ReadData()
{
    if (net_mod == 1) // чтенире данных с радиомодуля NRF24
    {
    }
}

Radio_Pult RadioPult = Radio_Pult();
