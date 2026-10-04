// Тест платы: частота кварца и напряжение питания ATmega.
// Собирается только в окружении clocktest (см. platformio.ini).
//
// Прошивка считает, что кварц 16 МГц, и каждую "секунду" переключает
// PB5 (D13, светодиод на большинстве плат), а в UART печатает
// счётчик и измеренное напряжение питания.
//
// Частота: засечь секундомером 30 переключений.
//   ~30 с -> 16 МГц,  ~40 с -> 12 МГц,  ~60 с -> 8 МГц,  ~80 с -> 6 МГц ...
//   F_кварца = 16 МГц * 30 / измеренные_секунды.
// UART: если на 9600 читается текст - кварц 16 МГц; если мусор - открыть
//   монитор на 4800 (кварц 8 МГц) или 7200 (12 МГц).
// Питание: Vcc в мВ, точность около 10 % (разброс опорного 1,1 В) -
//   отличить 3,3 В от 5 В хватает.

#include <Arduino.h>

static uint16_t readVccMv()
{
    // опорное = AVCC, вход = внутренний источник 1,1 В
    ADMUX = _BV(REFS0) | 0x0E;
    delay(2);                       // время установления опорного
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC))
        ;
    uint16_t adc = ADC;
    return adc ? (uint16_t)(1125300UL / adc) : 0;   // 1,1 В * 1023 * 1000
}

void setup()
{
    pinMode(13, OUTPUT);
    Serial.begin(9600);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    readVccMv();                    // первое измерение после включения АЦП неточное
    Serial.println(F("clocktest: F_CPU assumed 16 MHz"));
}

void loop()
{
    static uint16_t n;

    digitalWrite(13, n & 1);

    Serial.print(F("t="));
    Serial.print(n);
    Serial.print(F(" Vcc="));
    Serial.print(readVccMv());
    Serial.println(F(" mV"));

    n++;
    delay(1000);
}
