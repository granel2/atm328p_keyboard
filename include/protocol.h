// Протокол обмена клавиатуры ATmega328P (I2C-ведомый) с основным модулем STM32F407.
// Файл без зависимостей от Arduino: его можно копировать в проект STM32 как есть.
//
// ЧТЕНИЕ (STM32 <- ATmega): любая операция чтения возвращает пакет
//   [0] тип  [1] номер (seq)  [2] длина данных N  [3..3+N-1] данные  [3+N] CRC-8
// STM32 читает KBD_MAX_PACKET байт за одну транзакцию и разбирает по полю N
// (байты после CRC не нужны, ведомый отдаёт там 0xFF).
// Чтение пакет НЕ удаляет: при неверной CRC STM32 просто читает ещё раз.
// После успешной проверки STM32 шлёт KBD_CMD_ACK с номером пакета.
// seq = 1..255, 0 не используется (у пакета KBD_PKT_NONE seq = 0).
//
// ЗАПИСЬ (STM32 -> ATmega): [команда] [аргументы...] [CRC-8]
// Команду с неверной CRC ATmega игнорирует и ставит KBD_FLAG_CRC_ERR.
//
// CRC-8: полином 0x07, начальное значение 0 (SMBus PEC). Считается по байту
// адреса (addr<<1 | R/W) и далее по всем байтам пакета до CRC.
//
// Линия INT: активный низкий, открытый сток. Низкий уровень = очередь не пуста.
// На стороне STM32 включить внутреннюю подтяжку к 3,3 В.

#ifndef KBD_PROTOCOL_H
#define KBD_PROTOCOL_H

#include <stdint.h>

#define KBD_I2C_ADDR        0x20    // 7-битный адрес

#define KBD_INPUT_MAX       16      // максимум символов в группе
#define KBD_MAX_DATA        (1 + KBD_INPUT_MAX)        // причина + символы
#define KBD_MAX_PACKET      (3 + KBD_MAX_DATA + 1)     // = 21

// --- Типы пакетов ---
#define KBD_PKT_NONE        0x00    // событий нет, N = 0
#define KBD_PKT_KEY         0x01    // одиночная клавиша: N = 1, бит 7 = нажата, биты 0..6 = ASCII
#define KBD_PKT_GROUP       0x02    // завершённый ввод: [причина][символы ASCII...]
#define KBD_PKT_CANCEL      0x03    // ввод отменён: [причина], символов нет
#define KBD_PKT_STATUS      0x10    // ответ на KBD_CMD_STATUS: [флаги][число пакетов в очереди][версия ПО]

// --- Причина завершения ввода (первый байт данных GROUP / CANCEL) ---
#define KBD_END_ENTER       1       // нажата клавиша завершения (#)
#define KBD_END_MAXLEN      2       // набрана предельная длина
#define KBD_END_TIMEOUT     3       // истекла пауза
#define KBD_END_CLEAR       4       // сброс клавишей (*)

// --- Команды STM32 -> ATmega ---
#define KBD_CMD_ACK         0x01    // [seq]  пакет принят, убрать из очереди
#define KBD_CMD_CONFIG      0x02    // [maxLen][endKey][clearKey][timeout, с][cfgFlags]
#define KBD_CMD_STATUS      0x03    // следующие чтения возвращают KBD_PKT_STATUS (до любой другой команды)
#define KBD_CMD_CLEAR_FLAGS 0x04    // сбросить флаги ошибок
#define KBD_CMD_RESET       0x05    // очистить очередь и буфер ввода

// --- cfgFlags в KBD_CMD_CONFIG ---
#define KBD_CFG_TIMEOUT_SEND 0x01   // по паузе отправлять набранное (GROUP), иначе стирать (CANCEL)
#define KBD_CFG_KEY_RELEASE  0x02   // для функциональных клавиш слать и отпускание

// --- Флаги состояния ---
#define KBD_FLAG_CRC_ERR    0x01    // пришла команда с неверной CRC
#define KBD_FLAG_OVERFLOW   0x02    // очередь переполнена, событие потеряно
#define KBD_FLAG_BAD_CMD    0x04    // неизвестная команда или неверная длина/аргументы

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
