#include <radiopult.h>

void Radio_Pult::begin(uint8_t netmod, byte ce, byte cs)
{
    net_mod = netmod;
    if (net_mod == 0) // чтенире данных с радиомодуля Wifi
    {
        IPAddress local_IP(192, 168, 4, 1); // не определнн IP адрес
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        IPAddress dns(8, 8, 8, 8);

        for (byte n = 0; n < 19; n++)
        {
            ssid[n + 1] = Name[n];
        }
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, password);
        udp.begin(udp_port);
        t1 = millis();
    }

    if (net_mod == 1) // чтенире данных с радиомодуля NRF24
    {
        if (ce == 0 && cs == 0)
        {

#if defined(ESP8266) || defined(ARDUINO_ESP8266_NODEMCU)
            ce = D4;
            cs = D8;
#endif
#if defined(ESP32)
            ce = 4;
            cs = 5;
#endif
        }
        radio.begin(ce, cs);
        init_nrf();
        t1 = millis();
    }
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

bool Radio_Pult::Priem()
{
    if (millis() - t1 > 2000)
        for (byte n = 0; n < sizeof(b_read); n++)
        {
            //b_read[n] = 0;
        }
    switch (net_mod)
    {
    case RP_WIFI:
    {
        int packetSize = udp.parsePacket();
        if (packetSize)
        {
            udp.read(b_read, 32);
            t1 = millis();
            return true;
        }
        return false;
        break;
    }
    case RP_NRF24:
    {
        if (_connect) // если связь установлена
        {
            // установить буфер ответа если он включен
            if (radio.available()) // Слушаем канал связи
            {
                t1 = millis();
                radio.read(&b_read, sizeof(b_read));
                // загрузить буфер ответа если установлен флаг
                return true;
            }
        }
        return false;
        break;
    }

    default:
        return false;
    }
}
void Radio_Pult::init_nrf() // поиск каналоа и ожидание подключения
{
    uint8_t pip0[6] = "setup"; // труба для настройки

    //radio.begin(); // Инициируем работу модуля nRF24L01+.
    radio.setAutoAck(true);
    radio.setDataRate(RF24_1MBPS);  // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
    radio.setPALevel(RF24_PA_HIGH); // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm).
    radio.openReadingPipe(1, pip0);
    radio.powerUp();

    byte ch = 1;
    while (ch <= 125)
    {
        int scanal = 0;
        int i = 100;

        while (i--)
        {
            radio.setChannel(ch);
            radio.startListening();
            delayMicroseconds(200);
            radio.stopListening();
            if (radio.testCarrier())
                scanal++;
        }
        //Serial.println(String(ch) + " - " + String(scanal));

        if (scanal < 2) // нашел чистый канал
        {
            break;
        }
        ch++;
    }
    //  формирует название машинки для пульта
    for (byte n = 0; n < 19; n++)
    {
        s[n + 1] = Name[n];
    }

    //Serial.println(ch);
    radio.setAutoAck(true);
    //radio.setRetries(1, 15);
    radio.enableAckPayload();
    radio.openReadingPipe(1, pip0);
    radio.startListening(); // включаем прослушку
    //radio.openWritingPipe(pip0);
    //radio.setChannel(ch); // устанавливаем очередной канал работы модуля
    radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.
    //Serial.println("Поиск пульта");

    for (;;)
    {

        if (radio.available())
        {
            // Если в буфере приёма имеются принятые данные от передатчика, то ...
            char msg[32] = "";
            radio.read(&msg, sizeof(msg));           // Читаем данные из буфера приёма в массив myData указывая сколько всего байт может поместиться в массив.
            radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.

            //Serial.println(String(radio.getChannel()) + " - " + msg);

            if (String(msg).startsWith("#pip#"))
            {
                pip[0] = msg[5];
                pip[1] = msg[6];
                pip[2] = msg[7];
                pip[3] = msg[8];
                pip[4] = msg[9];

                set_ch = radio.getChannel();
                radio.openReadingPipe(1, pip);
                _connect = true;
                //Сохранить данные в памяти
                //write_eprom();
                return;
            }
        }
    }
}
void Radio_Pult::PowerControl(byte *h, byte min_value)
{
    if (*h == min_value)
        power_control = 1;

    if (power_control == 0)
    {
        *h = 0;
    }
}
Radio_Pult RadioPult = Radio_Pult();
