#include <Arduino.h>
#include <Wire.h>

static constexpr auto TEMPERATURE_NO_HOLD_CMD = 0xF3;
static constexpr auto HUMIDITY_NO_HOLD_CMD = 0xF5;

static constexpr auto SENSOR_ADDR = 0x40;

void measureTemperature();
void measureHumidity();

// https://stackoverflow.com/questions/51752284/how-to-calculate-crc8-in-c
uint8_t gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    size_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
}

void loop()
{
    measureTemperature();
    measureHumidity();

    delay(5000);
}

void measureTemperature()
{
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(TEMPERATURE_NO_HOLD_CMD);
    Wire.endTransmission();

    delay(50);

    int len = 0;

    do
    {
        len = Wire.requestFrom(SENSOR_ADDR, 3);
        if (len == 3)
        {
            break;
        }

        delay(50);
    } while (len < 3);

    uint8_t data[3] = {0};
    Wire.readBytes(data, 3);

    uint8_t msb = data[0];
    uint8_t lsb = data[1];
    uint8_t crc8 = data[2];

    uint8_t crc_res = gencrc(data, 2);
    if (crc8 != crc_res)
    {
        Serial.printf("CRC8 is invalid! 0x%02x (calculated) != 0x%02x (received)\r\n", crc_res, crc8);

        return;
    }

    uint8_t cleared_lsb = lsb & ~(0x03);
    uint16_t s_temp = (msb << 8) | cleared_lsb;
    float temp = -46.85 + (175.72 * (s_temp / 65536.0));

    Serial.printf("Temperature: %f *C\r\n", temp);
}

void measureHumidity()
{
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(HUMIDITY_NO_HOLD_CMD);
    Wire.endTransmission();

    delay(50);

    int len = 0;

    do
    {
        len = Wire.requestFrom(SENSOR_ADDR, 3);
        if (len == 3)
        {
            break;
        }

        delay(50);
    } while (len < 3);

    uint8_t data[3] = {0};
    Wire.readBytes(data, 3);

    uint8_t msb = data[0];
    uint8_t lsb = data[1];
    uint8_t crc8 = data[2];

    uint8_t crc_res = gencrc(data, 2);
    if (crc8 != crc_res)
    {
        Serial.printf("CRC8 is invalid! 0x%02x (calculated) != 0x%02x (received)\r\n", crc_res, crc8);

        return;
    }

    uint8_t cleared_lsb = lsb & ~(0b00001111);
    uint16_t s_hum = msb << 8 | cleared_lsb;

    float humidity = -6 + (125 * (s_hum / 65536.0));

    Serial.printf("Humidity: %f %%\r\n", humidity);
}
