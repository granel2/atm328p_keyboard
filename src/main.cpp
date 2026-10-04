// Клавиатура 4x4 на ATmega328P. По I2C сама (ведущий) передаёт пакеты в STM32F407.
// Протокол - include/protocol.h, распиновка и логика ввода - include/config.h.
//
// Раскладка: цифры и A..D копятся в буфере, '#' отправляет буфер, '*' стирает его.

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <string.h>
#include "config.h"
#include "protocol.h"

// KBD_UART_TEST (окружение uarttest): тест модуля без STM32 через UART PD1/PD0.
// Печатает нажатия и каждый пакет так, как он уйдёт в STM32 (с CRC и хвостом),
// и считает его доставленным - I2C не используется.
#ifdef KBD_UART_TEST
#define KBD_DEBUG
#endif

#ifdef KBD_DEBUG
#define DBG(...) Serial.print(__VA_ARGS__)
#define DBGLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG(...)
#define DBGLN(...)
#endif

// ---------------------------------------------------------------------------
// Очередь пакетов (только основной цикл)

struct Packet {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t data[KBD_MAX_DATA];
};

static Packet queue[QUEUE_LEN];
static uint8_t qHead;
static uint8_t qCount;
static uint8_t nextSeq = 1;

static void queuePush(uint8_t type, const uint8_t *data, uint8_t len)
{
    // номер расходуется и при переполнении: STM32 увидит пропуск seq
    uint8_t seq = nextSeq;
    if (++nextSeq == 0)
        nextSeq = 1;

    if (qCount >= QUEUE_LEN) {
        DBG(F("queue full, lost seq ")); DBGLN(seq);
        return;
    }
    Packet &p = queue[(qHead + qCount) % QUEUE_LEN];
    p.type = type;
    p.seq = seq;
    p.len = len;
    memcpy(p.data, data, len);
    qCount++;
}

static void queueDrop()
{
    memset(&queue[qHead], 0, sizeof(queue[0]));   // не оставляем пароль в памяти
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
}

// Пакет из головы очереди в формате протокола: с CRC и хвостом
static uint8_t buildFrame(uint8_t *buf)
{
    const Packet &p = queue[qHead];
    buf[0] = p.type;
    buf[1] = p.seq;
    buf[2] = p.len;
    memcpy(&buf[3], p.data, p.len);
    uint8_t n = 3 + p.len;
    buf[n] = kbd_crc8(kbd_crc8_update(0, KBD_HOST_ADDR << 1), buf, n);
    buf[n + 1] = KBD_TAIL;
    return n + 2;
}

// ---------------------------------------------------------------------------
// Передача в STM32

#ifndef KBD_UART_TEST
static uint32_t lastSendMs;
static uint8_t sendFails;

static void sendPoll()
{
    if (!qCount)
        return;
    if (sendFails && millis() - lastSendMs < SEND_RETRY_MS)
        return;

    uint8_t buf[KBD_MAX_FRAME];
    uint8_t n = buildFrame(buf);
    Wire.beginTransmission(KBD_HOST_ADDR);
    Wire.write(buf, n);
    uint8_t res = Wire.endTransmission();   // 0 - хвост подтверждён = пакет принят
    memset(buf, 0, sizeof(buf));
    lastSendMs = millis();

    DBG(F("TX seq ")); DBG(queue[qHead].seq); DBG(F(" res ")); DBGLN(res);

    if (res == 0) {
        queueDrop();
        sendFails = 0;
    } else if (res == 2) {
        sendFails = 1;                      // STM32 не готов: ждём сколько угодно
    } else if (++sendFails > SEND_MAX_FAILS) {
        DBG(F("dropped seq ")); DBGLN(queue[qHead].seq);
        queueDrop();                        // 3 - CRC не сошлась, 4/5 - ошибка шины
        sendFails = 0;
    }
}
#endif

// ---------------------------------------------------------------------------
// Ввод

static char inBuf[KBD_INPUT_MAX];
static uint8_t inLen;
static uint32_t lastKeyMs;

static void clearInput()
{
    memset(inBuf, 0, sizeof(inBuf));   // не оставляем набранный пароль в памяти
    inLen = 0;
}

static void sendGroup(uint8_t reason)
{
    uint8_t d[KBD_MAX_DATA];
    d[0] = reason;
    memcpy(&d[1], inBuf, inLen);
    queuePush(KBD_PKT_GROUP, d, 1 + inLen);
    memset(d, 0, sizeof(d));
    DBG(F("GROUP r=")); DBG(reason); DBG(F(" len=")); DBGLN(inLen);
    clearInput();
}

static void sendCancel(uint8_t reason)
{
    queuePush(KBD_PKT_CANCEL, &reason, 1);
    DBG(F("CANCEL r=")); DBGLN(reason);
    clearInput();
}

static void onKey(char k, bool pressed)
{
    DBG(pressed ? F("down ") : F("up   ")); DBGLN(k);

    if (k == END_KEY) {
        if (pressed)
            sendGroup(KBD_END_ENTER);
    } else if (k == CLEAR_KEY) {
        if (pressed)
            sendCancel(KBD_END_CLEAR);
    } else if ((k >= '0' && k <= '9') || (k >= 'A' && k <= 'D' && !LETTERS_KEY)) {
        if (pressed) {
            inBuf[inLen++] = k;
            lastKeyMs = millis();
            if (inLen >= INPUT_MAX_LEN)
                sendGroup(KBD_END_MAXLEN);
        }
    } else if (pressed || KEY_RELEASE) {
        uint8_t code = (uint8_t)k | (pressed ? 0x80 : 0);
        queuePush(KBD_PKT_KEY, &code, 1);
    }
}

static void checkTimeout()
{
    if (inLen == 0 || INPUT_TIMEOUT_S == 0)
        return;
    if (millis() - lastKeyMs < (uint32_t)INPUT_TIMEOUT_S * 1000UL)
        return;
    if (TIMEOUT_SEND)
        sendGroup(KBD_END_TIMEOUT);
    else
        sendCancel(KBD_END_TIMEOUT);
}

// ---------------------------------------------------------------------------
// Опрос матрицы с антидребезгом

static uint16_t stableKeys;
static uint8_t debounceCnt[16];

static uint16_t readMatrix()
{
    uint16_t m = 0;
    for (uint8_t r = 0; r < 4; r++) {
        pinMode(ROW_PINS[r], OUTPUT);
        digitalWrite(ROW_PINS[r], LOW);
        delayMicroseconds(10);
        for (uint8_t c = 0; c < 4; c++)
            if (digitalRead(COL_PINS[c]) == LOW)
                m |= 1u << (r * 4 + c);
        pinMode(ROW_PINS[r], INPUT);   // неактивная строка в Z: нет КЗ при двух нажатых
    }
    return m;
}

static void scanKeys()
{
    uint16_t raw = readMatrix();
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t bit = 1u << i;
        if ((raw ^ stableKeys) & bit) {
            if (++debounceCnt[i] >= DEBOUNCE_MS / SCAN_PERIOD_MS) {
                debounceCnt[i] = 0;
                stableKeys ^= bit;
                onKey(KEYMAP[i / 4][i % 4], stableKeys & bit);
            }
        } else {
            debounceCnt[i] = 0;
        }
    }
}

// ---------------------------------------------------------------------------

#ifdef KBD_UART_TEST
static uint16_t readVccMv()
{
    ADMUX = _BV(REFS0) | 0x0E;      // опорное AVCC, вход - внутренний 1,1 В
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC))
        ;
    uint16_t adc = ADC;
    return adc ? (uint16_t)(1125300UL / adc) : 0;
}

static void printHex(uint8_t b)
{
    if (b < 0x10)
        Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
}

// Выводит пакет из головы очереди так, как он уйдёт в STM32, и считает его доставленным
static void uartTestPoll()
{
    if (!qCount)
        return;

    const Packet &p = queue[qHead];
    uint8_t buf[KBD_MAX_FRAME];
    uint8_t n = buildFrame(buf);

    Serial.print(F("PKT "));
    for (uint8_t i = 0; i < n; i++)
        printHex(buf[i]);

    switch (p.type) {
    case KBD_PKT_KEY:
        Serial.print(F("| KEY "));
        Serial.print((char)(p.data[0] & 0x7F));
        Serial.print(p.data[0] & 0x80 ? F(" down") : F(" up"));
        break;
    case KBD_PKT_GROUP:
        Serial.print(F("| GROUP reason="));
        Serial.print(p.data[0]);
        Serial.print(F(" \""));
        for (uint8_t i = 1; i < p.len; i++)
            Serial.print((char)p.data[i]);
        Serial.print('"');
        break;
    case KBD_PKT_CANCEL:
        Serial.print(F("| CANCEL reason="));
        Serial.print(p.data[0]);
        break;
    case KBD_PKT_START:
        Serial.print(F("| START fw="));
        Serial.print(p.data[0]);
        Serial.print(F(" MCUSR=0x"));
        Serial.print(p.data[1], HEX);
        break;
    }
    Serial.println();

    memset(buf, 0, sizeof(buf));
    queueDrop();
}
#endif

// ---------------------------------------------------------------------------

void setup()
{
    uint8_t resetCause = MCUSR;     // 1 питание, 2 внешний сброс, 4 BOD, 8 watchdog
    MCUSR = 0;
    wdt_disable();

#ifdef KBD_DEBUG
    Serial.begin(KBD_DEBUG_BAUD);
    DBGLN(F("kbd start"));
#endif
#ifdef KBD_UART_TEST
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    readVccMv();
    Serial.print(F("UART test, F_CPU="));
    Serial.print(F_CPU / 1000000UL);
    Serial.print(F(" MHz, Vcc="));
    Serial.print(readVccMv());
    Serial.println(F(" mV. Packets are not sent over I2C."));
#endif

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(ROW_PINS[i], INPUT);
        pinMode(COL_PINS[i], INPUT_PULLUP);
    }

#ifndef KBD_UART_TEST
    Wire.begin();                   // ведущий
    // Wire включает внутренние подтяжки к 5 В - отключаем, шина подтянута снаружи
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.setClock(KBD_I2C_HZ);
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
#endif

    // первым пакетом - сообщение о старте: STM32 сбрасывает проверку дублей seq
    uint8_t start[2] = {FW_VERSION, resetCause};
    queuePush(KBD_PKT_START, start, 2);

    wdt_enable(WDTO_500MS);
}

void loop()
{
    static uint32_t lastScan;

    wdt_reset();

    uint32_t now = millis();
    if (now - lastScan >= SCAN_PERIOD_MS) {
        lastScan = now;
        scanKeys();
        checkTimeout();
    }

#ifdef KBD_UART_TEST
    uartTestPoll();
#else
    sendPoll();
#endif
}
