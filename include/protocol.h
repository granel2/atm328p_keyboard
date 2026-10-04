// Протокол обмена клавиатуры ATmega328P с основным модулем STM32F407 по I2C.
// Файл без зависимостей от Arduino: его можно копировать в проект STM32 как есть.
//
// Направление одно: ATmega - ведущий-передатчик, STM32 - ведомый-приёмник
// с адресом KBD_HOST_ADDR. Пакет пишется одной транзакцией:
//   [0] тип  [1] номер (seq)  [2] длина данных N  [3..3+N-1] данные  [3+N] CRC-8  [4+N] KBD_TAIL
//
// CRC-8: полином 0x07, начальное значение 0 (SMBus PEC). Считается по байту
// адреса (KBD_HOST_ADDR << 1, запись) и по байтам пакета до CRC. Хвост в CRC не входит.
//
// Подтверждение - бит ACK самого I2C на байт-хвост:
//   STM32 принимает побайтно (прерывание RXNE). Получив байт CRC, проверяет пакет
//   и до прихода хвоста выставляет ответ на хвост: ACK - пакет принят, NACK - CRC
//   или формат неверны. (ACK байта CRC поставлен аппаратно ещё до проверки - поэтому
//   и нужен хвост.) Срок на решение - пока идут 8 бит хвоста (~80 мкс при 100 кГц):
//   удержание SCL у F4 наступает уже после импульса ACK следующего байта и не
//   помогает, поэтому прерывание I2C на STM32 - с высоким приоритетом, а CRC
//   считается по ходу приёма (по байту CRC - только сравнение).
//   ATmega: хвост подтверждён - пакет удаляется; NACK адреса (STM32 не готов) или
//   NACK данных/хвоста - пакет остаётся в очереди и повторяется.
//
// Повтор с тем же seq (ACK хвоста исказился и ATmega не узнала об успехе)
// STM32 подтверждает и отбрасывает как дубль. Пропуск seq = потерянный пакет
// (очередь ATmega переполнилась или пакет отброшен после KBD_SEND_MAX_FAILS неудач).
// seq = 1..255 по кругу; после сброса ATmega первым идёт KBD_PKT_START с seq = 1 -
// по нему STM32 сбрасывает проверку дублей.

#ifndef KBD_PROTOCOL_H
#define KBD_PROTOCOL_H

#include <stdint.h>

#define KBD_HOST_ADDR       0x30    // 7-битный адрес STM32 (ведомый) на шине клавиатуры

#define KBD_TAIL            '$'     // байт после CRC: ответ STM32 на него = результат проверки

#define KBD_INPUT_MAX       16      // максимум символов в группе
#define KBD_MAX_DATA        (1 + KBD_INPUT_MAX)        // причина + символы
#define KBD_MAX_PACKET      (3 + KBD_MAX_DATA + 1)     // = 21, без хвоста
#define KBD_MAX_FRAME       (KBD_MAX_PACKET + 1)       // = 22, с хвостом

// --- Типы пакетов ---
#define KBD_PKT_KEY         0x01    // одиночная клавиша A-D (только если ATmega собрана с KBD_LETTERS_KEY):
                                    //   N = 1, бит 7 = нажата, биты 0..6 = ASCII
#define KBD_PKT_GROUP       0x02    // завершённый ввод: [причина][символы ASCII 0-9, A-D...]
#define KBD_PKT_CANCEL      0x03    // ввод отменён: [причина], символов нет
#define KBD_PKT_START       0x04    // ATmega включилась/сбросилась: [версия ПО][MCUSR - причина сброса]

// --- Причина завершения ввода (первый байт данных GROUP / CANCEL) ---
#define KBD_END_ENTER       1       // нажата клавиша завершения (#)
#define KBD_END_MAXLEN      2       // набрана предельная длина
#define KBD_END_TIMEOUT     3       // истекла пауза
#define KBD_END_CLEAR       4       // сброс клавишей (*)

static inline uint8_t kbd_crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

static inline uint8_t kbd_crc8(uint8_t crc, const uint8_t *buf, uint8_t len)
{
    while (len--)
        crc = kbd_crc8_update(crc, *buf++);
    return crc;
}

#endif // KBD_PROTOCOL_H
