#include <radiopult.h>

void Radio_Pult::begin(uint8_t netmod, byte ce, byte cs)
{
    pinMode(35, INPUT);

    net_mod = netmod;
    if (net_mod == 0) // чтенире данных с радиомодуля Wifi
    {

        uint8_t encryptionType;
        int32_t RSSI;
        uint8_t *BSSID;
        int32_t channel;
        bool isHidden;
        int chs[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        IPAddress local_IP(192, 168, 4, 100); // не определнн IP адрес
        IPAddress gateway(192, 168, 4, 100);
        IPAddress subnet(255, 255, 255, 0);
        //        IPAddress dns(8, 8, 8, 8);

        for (byte n = 0; n < 19; n++)
        {
            ssid[n + 1] = Name[n];
        }
        String sss = ssid;
        int n = WiFi.scanNetworks();

        for (int i = 0; i < n; i++)
        {
#if defined(ESP8266) || defined(ARDUINO_ESP8266_NODEMCU)
            WiFi.getNetworkInfo(i, sss, encryptionType, RSSI, BSSID, channel, isHidden);
#endif
#if defined(ESP32)
            WiFi.getNetworkInfo(i, sss, encryptionType, RSSI, BSSID, channel);
#endif
            chs[channel] = chs[channel] + 1;
            //            Serial.printf("%d: %s, Ch:%d (%ddBm) %s %s\n", i + 1, sss.c_str(), channel, RSSI, encryptionType == ENC_TYPE_NONE ? "open" : "", isHidden ? "hidden" : "");
        }

        for (int i = 1; i < 15; i++)
        {
            // Serial.print(String(chs[i]) + "  ");
            if (chs[i] == 0)
            {
                channel = i;
                break;
            }
        }

        //        WiFi.setOutputPower(0);
        // WiFi.setAutoConnect(true);
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(local_IP, gateway, subnet);
        WiFi.softAP(ssid, password, channel);
        delay(1000);

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

bool Radio_Pult::Priem()
{
    if (millis() - t1 > 2000)
        for (byte n = 0; n < sizeof(b_read); n++)
        {
            b_read[n] = 0; // занчения по умолчанию
            power_control = 0;
        }

    // b_read[3]= 50;

    // Формирование буфера для отправки на  пульт
    s[1] = AkbRead(35); // напряжение на модели

    switch (net_mod)
    {
    case RP_WIFI:
    {

        // Отправка ответ на пульт
        udp.beginPacket(udp.remoteIP(), 4220);
        // udp.print(send_p);
        for (byte j = 0; j < 32; j++)
        {
            udp.write(s[j]);
        }
        udp.endPacket();

        int packetSize = udp.parsePacket();
        if (packetSize)
        {
            err = 0;
            udp.read(b_read, 32);
            t1 = millis();

            return true;
        }
        else
        {
            err++;
        }
        // return false;
        break;
    }
    case RP_NRF24:
    {
        if (_connect) // если связь установлена
        {
            // установить буфер ответа если он включен
            if (radio.available()) // Слушаем канал связи
            {
                err = 0;
                t1 = millis();
                // загрузить буфер ответа если установлен флаг
                s[1] = round(AkbRead(35) * 10);          // напряжение на модели
                radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.
                radio.read(&b_read, sizeof(b_read));
                radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.

                return true;
            }
            else
            {
                err++;
                if (err > 500)
                {
                    ESP.restart();
                    delay(10);
                }
            }
            // Serial.println(err);
        }
        // return false;
        break;
    }

    default:
        return false;
    }
    if (err > 10)
    {
        b_read[1] = 127;
        b_read[2] = 127;
        b_read[3] = 127;
        b_read[4] = 127;
    }
    //  Анализ принятого протокала данных

    if (b_read[0] == 6) // получен запрос на отправку данных о модели
    {
        // b_send[]
    }

    return false;
}
void Radio_Pult::init_nrf() // поиск каналоа и ожидание подключения
{
    uint8_t pip0[6] = "setup"; // труба для настройки

    radio.powerUp();
    delay(150);
    radio.begin(); // Инициируем работу модуля nRF24L01+.
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setPayloadSize(32);
    radio.setDataRate(RF24_1MBPS); // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
    radio.setPALevel(RF24_PA_LOW); // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm).
    radio.openReadingPipe(1, pip0);
    // radio.setCRCLength(RF24_CRC_16);
    radio.printDetails();

    byte ch = 2;
    // radio.setAutoAck(false);

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
            if (radio.testCarrier() || radio.testRPD())
                scanal++;
        }
        //   Serial.println(String(radio.getChannel()) + " - " + String(scanal));

        if (scanal < 1) // нашел чистый канал
        {
            break;
        }
        ch = ch + 1;
    }

    // ch = 2;
    // radio.setChannel(ch);

    Serial.println(String(radio.getChannel()) + " - Канал");
    //  формирует название машинки для пульта
    for (byte n = 0; n < 19; n++)
    {
        s[n + 1] = Name[n];
    }

    Serial.println(s);
    Serial.println(AkbRead(35));
    radio.flush_rx();
    radio.flush_tx();
    radio.setAutoAck(true);
    radio.enableAckPayload();
    // radio.setRetries(1, 15);
    radio.openReadingPipe(1, pip0);
    radio.startListening(); // включаем прослушку
    // radio.openWritingPipe(pip0);
    // radio.setChannel(ch); // устанавливаем очередной канал работы модуля
    radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.
    // Serial.println("Поиск пульта");

    for (;;)
    {

        if (radio.available())
        {
            // Если в буфере приёма имеются принятые данные от передатчика, то ...
            char msg[32] = "";
            // s[0] = 0;
            // s[1] = AkbRead(A0);
            // radio.writeAckPayload(1, &s, sizeof(s)); // Помещаем данные всего массива ackData в буфер FIFO для их отправки на следующее получение данных от передатчика на 1 трубе.
            radio.read(&msg, sizeof(msg)); // Читаем данные из буфера приёма в массив myData указывая сколько всего байт может поместиться в массив.

            //  Serial.println(String(radio.getChannel()) + " - " + String(msg));

            if (String(msg).startsWith("#pip#"))
            {
                pip[0] = msg[5];
                pip[1] = msg[6];
                pip[2] = msg[7];
                pip[3] = msg[8];
                pip[4] = msg[9];

                set_ch = radio.getChannel();
                radio.flush_rx();
                radio.flush_tx();
                radio.openReadingPipe(1, pip);
                // radio.startListening(); // включаем прослушку
                _connect = true;
                //Сохранить данные в памяти
                // write_eprom();
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

float Radio_Pult::AkbRead(byte port)
{

#if defined(ESP8266) || defined(ARDUINO_ESP8266_NODEMCU)
    ak = analogRead(A0); //  *3.3 / 4094 * 2; // расчет напряжения пульта
    ak = round((ak * 3.3 * 1.4 / 1024 * 2.5) * 10) / 10;
#endif
#if defined(ESP32)
    ak = analogRead(port); //  *3.3 / 4094 * 2; // расчет напряжения пульта
    ak = round((ak * 3.3 * 1.6 / 4094 * 10) * 10) / 10;

#endif
    return ak;
}

Radio_Pult RadioPult = Radio_Pult();
