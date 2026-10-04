// Клавиатура 4x4 на ATmega328P, I2C-ведомый для STM32F407.
// Протокол - include/protocol.h, распиновка - include/config.h.
//
// Раскладка: цифры копятся в буфере, '#' отправляет буфер, '*' стирает его,
// A..D уходят сразу одиночными пакетами.

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <string.h>
#include "config.h"
#include "protocol.h"

// KBD_UART_TEST (окружение uarttest): тест модуля без STM32 через UART PD1/PD0.
// Печатает нажатия и каждый пакет так, как его прочитает STM32 (с CRC),
// и сам подтверждает пакеты вместо STM32.
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
// Очередь пакетов. Добавляет основной цикл, читает и удаляет обработчик I2C
// (прерывание), поэтому добавление идёт с запретом прерываний.

struct Packet {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t data[KBD_MAX_DATA];
};

static Packet queue[QUEUE_LEN];
static volatile uint8_t qHead;
static volatile uint8_t qCount;
static uint8_t nextSeq = 1;

static volatile uint8_t statusFlags;
static volatile bool statusRequested;

struct Config {
    uint8_t maxLen;
    uint8_t endKey;
    uint8_t clearKey;
    uint8_t timeoutS;
    uint8_t flags;
};

static Config cfg = {DEF_MAX_LEN, DEF_END_KEY, DEF_CLEAR_KEY, DEF_TIMEOUT_S, DEF_CFG_FLAGS};
static Config cfgNew;
static volatile bool cfgPending;
static volatile bool resetPending;

// Буфер ввода группы (только основной цикл)
static char inBuf[KBD_INPUT_MAX];
static uint8_t inLen;
static uint32_t lastKeyMs;

static void updateInt()
{
    // открытый сток: либо тянем к нулю, либо отпускаем линию
    if (qCount)
        INT_DDR |= _BV(INT_BIT);
    else
        INT_DDR &= ~_BV(INT_BIT);
}

static void queuePush(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t sreg = SREG;
    cli();
    if (qCount >= QUEUE_LEN) {
        statusFlags |= KBD_FLAG_OVERFLOW;
    } else {
        Packet &p = queue[(qHead + qCount) % QUEUE_LEN];
        p.type = type;
        p.seq = nextSeq;
        p.len = len;
        memcpy(p.data, data, len);
        if (++nextSeq == 0)
            nextSeq = 1;
        qCount++;
        updateInt();
    }
    SREG = sreg;
}

// ---------------------------------------------------------------------------
// I2C-ведомый (вызывается из прерывания TWI)

static void onRequest()
{
    uint8_t buf[KBD_MAX_PACKET];
    uint8_t n;

    if (statusRequested) {
        buf[0] = KBD_PKT_STATUS;
        buf[1] = 0;
        buf[2] = 3;
        buf[3] = statusFlags;
        buf[4] = qCount;
        buf[5] = FW_VERSION;
        n = 6;
    } else if (qCount == 0) {
        buf[0] = KBD_PKT_NONE;
        buf[1] = 0;
        buf[2] = 0;
        n = 3;
    } else {
        const Packet &p = queue[qHead];
        buf[0] = p.type;
        buf[1] = p.seq;
        buf[2] = p.len;
        memcpy(&buf[3], p.data, p.len);
        n = 3 + p.len;
    }

    uint8_t crc = kbd_crc8_update(0, (KBD_I2C_ADDR << 1) | 1);
    buf[n] = kbd_crc8(crc, buf, n);
    Wire.write(buf, n + 1);
}

static void onReceive(int count)
{
    uint8_t buf[8];
    uint8_t n = 0;

    while (Wire.available()) {
        uint8_t b = Wire.read();
        if (n < sizeof(buf))
            buf[n] = b;
        n++;
    }
    if (n < 2 || n > sizeof(buf)) {
        statusFlags |= KBD_FLAG_BAD_CMD;
        return;
    }

    uint8_t crc = kbd_crc8_update(0, KBD_I2C_ADDR << 1);
    if (kbd_crc8(crc, buf, n - 1) != buf[n - 1]) {
        statusFlags |= KBD_FLAG_CRC_ERR;
        return;
    }

    uint8_t argc = n - 2;   // без кода команды и CRC
    const uint8_t *arg = &buf[1];

    switch (buf[0]) {
    case KBD_CMD_ACK:
        if (argc != 1)
            break;
        // если номер не совпал (повторный ACK), просто ничего не удаляем
        if (qCount && queue[qHead].seq == arg[0]) {
            qHead = (qHead + 1) % QUEUE_LEN;
            qCount--;
            updateInt();
        }
        statusRequested = false;
        return;

    case KBD_CMD_CONFIG:
        if (argc != 5 || arg[0] < 1 || arg[0] > KBD_INPUT_MAX)
            break;
        cfgNew.maxLen = arg[0];
        cfgNew.endKey = arg[1];
        cfgNew.clearKey = arg[2];
        cfgNew.timeoutS = arg[3];
        cfgNew.flags = arg[4];
        cfgPending = true;
        statusRequested = false;
        return;

    case KBD_CMD_STATUS:
        if (argc != 0)
            break;
        statusRequested = true;
        return;

    case KBD_CMD_CLEAR_FLAGS:
        if (argc != 0)
            break;
        statusFlags = 0;
        statusRequested = false;
        return;

    case KBD_CMD_RESET:
        if (argc != 0)
            break;
        qHead = 0;
        qCount = 0;
        updateInt();
        resetPending = true;
        statusRequested = false;
        return;
    }
    statusFlags |= KBD_FLAG_BAD_CMD;
}

// ---------------------------------------------------------------------------
// Ввод

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

    if ((uint8_t)k == cfg.endKey) {
        if (pressed)
            sendGroup(KBD_END_ENTER);
    } else if ((uint8_t)k == cfg.clearKey) {
        if (pressed)
            sendCancel(KBD_END_CLEAR);
    } else if (k >= '0' && k <= '9') {
        if (pressed) {
            inBuf[inLen++] = k;
            lastKeyMs = millis();
            if (inLen >= cfg.maxLen)
                sendGroup(KBD_END_MAXLEN);
        }
    } else if (pressed || (cfg.flags & KBD_CFG_KEY_RELEASE)) {
        uint8_t code = (uint8_t)k | (pressed ? 0x80 : 0);
        queuePush(KBD_PKT_KEY, &code, 1);
    }
}

static void checkTimeout()
{
    if (inLen == 0 || cfg.timeoutS == 0)
        return;
    if (millis() - lastKeyMs < (uint32_t)cfg.timeoutS * 1000UL)
        return;
    if (cfg.flags & KBD_CFG_TIMEOUT_SEND)
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

// Выводит пакет из головы очереди с CRC, как его прочитает STM32, и подтверждает
static void uartTestPoll()
{
    if (!qCount)
        return;

    const Packet &p = queue[qHead];
    uint8_t buf[KBD_MAX_PACKET];
    buf[0] = p.type;
    buf[1] = p.seq;
    buf[2] = p.len;
    memcpy(&buf[3], p.data, p.len);
    uint8_t n = 3 + p.len;
    buf[n] = kbd_crc8(kbd_crc8_update(0, (KBD_I2C_ADDR << 1) | 1), buf, n);

    Serial.print(F("PKT "));
    for (uint8_t i = 0; i <= n; i++)
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
    }
    Serial.println();

    cli();
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
    updateInt();
    sei();
}
#endif

// ---------------------------------------------------------------------------

void setup()
{
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
    Serial.println(F(" mV. Packets are auto-ACKed."));
#endif

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(ROW_PINS[i], INPUT);
        pinMode(COL_PINS[i], INPUT_PULLUP);
    }
    INT_PORT &= ~_BV(INT_BIT);   // при включении на выход будет 0
    INT_DDR &= ~_BV(INT_BIT);    // линия отпущена

    Wire.begin(KBD_I2C_ADDR);
    // Wire включает внутренние подтяжки к 5 В - отключаем, шина подтянута снаружи
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    wdt_enable(WDTO_500MS);
}

void loop()
{
    static uint32_t lastScan;

    wdt_reset();

    if (cfgPending) {
        cli();
        cfg = cfgNew;
        cfgPending = false;
        sei();
        if (inLen >= cfg.maxLen)
            clearInput();
    }
    if (resetPending) {
        resetPending = false;
        clearInput();
    }

    uint32_t now = millis();
    if (now - lastScan >= SCAN_PERIOD_MS) {
        lastScan = now;
        scanKeys();
        checkTimeout();
    }

#ifdef KBD_UART_TEST
    uartTestPoll();
#endif
}
