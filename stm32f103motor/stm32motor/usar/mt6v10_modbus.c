#include "mt6v10_modbus.h"
#include "rs485_port.h"

#define MT6V10_SLAVE_ADDRESS          1U
#define MT6V10_FUNCTION_READ_HOLDING  0x03U
#define MT6V10_REGISTER_ANGLE         0x0001U
#define MT6V10_REGISTER_COUNT         2U
#define MT6V10_RESPONSE_LENGTH        9U
#define MT6V10_EXCEPTION_LENGTH       5U
#define MT6V10_POLL_PERIOD_MS         20U
#define MT6V10_RESPONSE_TIMEOUT_MS    20U
#define MT6V10_INTERBYTE_TIMEOUT_MS   5U
#define MT6V10_ONLINE_TIMEOUT_MS      100U
#define MT6V10_TX_TIMEOUT_MS          5U
#define MT6V10_MRAD_PER_TURN          6283L

typedef enum {
    MT6V10_RX_IDLE = 0,
    MT6V10_RX_FRAME
} Mt6v10RxState;

static Mt6v10Status encoder_status;
static Mt6v10RxState rx_state;
static uint8_t rx_frame[MT6V10_RESPONSE_LENGTH];
static uint8_t rx_length;
static uint8_t expected_length;
static uint8_t update_pending;
static uint32_t last_request_ms;
static uint32_t request_started_ms;
static uint32_t last_byte_ms;

static uint16_t mt6v10_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static void mt6v10_reset_receiver(void)
{
    rx_state = MT6V10_RX_IDLE;
    rx_length = 0U;
    expected_length = 0U;
}

static int16_t mt6v10_calculate_speed(int32_t position_mrad, uint32_t now_ms)
{
    int64_t delta_mrad;
    int64_t numerator;
    int64_t denominator;
    int64_t speed_rpm_x10;
    uint32_t elapsed_ms;

    if (encoder_status.has_sample == 0U) {
        return 0;
    }
    elapsed_ms = now_ms - encoder_status.sample_ms;
    if (elapsed_ms == 0U) {
        return encoder_status.speed_rpm_x10;
    }

    delta_mrad = (int64_t)position_mrad - encoder_status.angle_mrad;
    numerator = delta_mrad * 600000LL;
    denominator = (int64_t)MT6V10_MRAD_PER_TURN * elapsed_ms;
    speed_rpm_x10 = numerator / denominator;
    if (speed_rpm_x10 > 32767LL) {
        speed_rpm_x10 = 32767LL;
    } else if (speed_rpm_x10 < -32768LL) {
        speed_rpm_x10 = -32768LL;
    }
    return (int16_t)speed_rpm_x10;
}

static void mt6v10_accept_sample(uint32_t now_ms)
{
    uint16_t angle_raw = (uint16_t)(((uint16_t)rx_frame[3] << 8U) |
                                    rx_frame[4]);
    int32_t turns = (int32_t)(((uint16_t)rx_frame[5] << 8U) | rx_frame[6]);
    int32_t fractional_mrad;
    int32_t position_mrad;

    if (turns >= 32768L) {
        turns -= 65536L;
    }
    fractional_mrad = (int32_t)(((uint32_t)angle_raw *
        (uint32_t)MT6V10_MRAD_PER_TURN + 32768U) / 65536U);
    position_mrad = turns * MT6V10_MRAD_PER_TURN + fractional_mrad;

    encoder_status.speed_rpm_x10 = mt6v10_calculate_speed(position_mrad, now_ms);
    encoder_status.angle_raw = angle_raw;
    encoder_status.turn_count = (int16_t)turns;
    encoder_status.angle_mrad = position_mrad;
    encoder_status.sample_ms = now_ms;
    encoder_status.online = 1U;
    encoder_status.has_sample = 1U;
    update_pending = 1U;
}

static void mt6v10_process_frame(uint32_t now_ms)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    received_crc = (uint16_t)(rx_frame[expected_length - 2U] |
        ((uint16_t)rx_frame[expected_length - 1U] << 8U));
    calculated_crc = mt6v10_crc16(rx_frame, (uint16_t)(expected_length - 2U));
    if (received_crc != calculated_crc) {
        encoder_status.crc_errors++;
        return;
    }

    if (rx_frame[1] == (uint8_t)(MT6V10_FUNCTION_READ_HOLDING | 0x80U)) {
        encoder_status.protocol_errors++;
        return;
    }
    if (rx_frame[1] != MT6V10_FUNCTION_READ_HOLDING || rx_frame[2] != 4U) {
        encoder_status.protocol_errors++;
        return;
    }
    mt6v10_accept_sample(now_ms);
}

static void mt6v10_process_byte(uint8_t byte, uint32_t now_ms)
{
    if (rx_length == 0U) {
        if (byte != MT6V10_SLAVE_ADDRESS) {
            return;
        }
        rx_state = MT6V10_RX_FRAME;
        rx_frame[rx_length++] = byte;
        last_byte_ms = now_ms;
        return;
    }

    if (rx_length >= sizeof(rx_frame)) {
        encoder_status.protocol_errors++;
        mt6v10_reset_receiver();
        return;
    }
    rx_frame[rx_length++] = byte;
    last_byte_ms = now_ms;

    if (rx_length == 2U) {
        if (rx_frame[1] == MT6V10_FUNCTION_READ_HOLDING) {
            expected_length = MT6V10_RESPONSE_LENGTH;
        } else if (rx_frame[1] ==
                   (uint8_t)(MT6V10_FUNCTION_READ_HOLDING | 0x80U)) {
            expected_length = MT6V10_EXCEPTION_LENGTH;
        } else {
            encoder_status.protocol_errors++;
            mt6v10_reset_receiver();
            return;
        }
    } else if (rx_length == 3U && rx_frame[1] == MT6V10_FUNCTION_READ_HOLDING &&
               rx_frame[2] != 4U) {
        encoder_status.protocol_errors++;
        mt6v10_reset_receiver();
        return;
    }

    if (expected_length != 0U && rx_length == expected_length) {
        mt6v10_process_frame(now_ms);
        mt6v10_reset_receiver();
    }
}

static HAL_StatusTypeDef mt6v10_start_read(uint32_t now_ms)
{
    uint8_t request[8] = {
        MT6V10_SLAVE_ADDRESS,
        MT6V10_FUNCTION_READ_HOLDING,
        (uint8_t)(MT6V10_REGISTER_ANGLE >> 8U),
        (uint8_t)MT6V10_REGISTER_ANGLE,
        0U,
        MT6V10_REGISTER_COUNT,
        0U,
        0U
    };
    uint16_t crc = mt6v10_crc16(request, 6U);

    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8U);
    last_request_ms = now_ms;
    if (RS485_Transmit(request, sizeof(request), MT6V10_TX_TIMEOUT_MS) != HAL_OK) {
        encoder_status.tx_errors++;
        return HAL_ERROR;
    }
    request_started_ms = now_ms;
    rx_state = MT6V10_RX_FRAME;
    rx_length = 0U;
    expected_length = 0U;
    return HAL_OK;
}

void Mt6v10_Init(uint32_t now_ms)
{
    encoder_status.online = 0U;
    encoder_status.has_sample = 0U;
    encoder_status.angle_raw = 0U;
    encoder_status.turn_count = 0;
    encoder_status.angle_mrad = 0;
    encoder_status.speed_rpm_x10 = 0;
    encoder_status.sample_ms = 0U;
    encoder_status.crc_errors = 0U;
    encoder_status.response_timeouts = 0U;
    encoder_status.protocol_errors = 0U;
    encoder_status.tx_errors = 0U;
    update_pending = 0U;
    last_request_ms = now_ms - MT6V10_POLL_PERIOD_MS;
    request_started_ms = now_ms;
    last_byte_ms = now_ms;
    mt6v10_reset_receiver();
}

void Mt6v10_Task(uint32_t now_ms)
{
    uint8_t byte;
    HAL_StatusTypeDef status;

    if (encoder_status.online != 0U &&
        (uint32_t)(now_ms - encoder_status.sample_ms) >= MT6V10_ONLINE_TIMEOUT_MS) {
        encoder_status.online = 0U;
        encoder_status.speed_rpm_x10 = 0;
        update_pending = 1U;
    }

    if (rx_state == MT6V10_RX_FRAME) {
        status = RS485_Receive(&byte, 1U, 0U);
        if (status == HAL_OK) {
            mt6v10_process_byte(byte, now_ms);
        } else if (status == HAL_ERROR) {
            encoder_status.protocol_errors++;
            mt6v10_reset_receiver();
        }
        if (rx_state == MT6V10_RX_FRAME) {
            if ((rx_length == 0U &&
                 (uint32_t)(now_ms - request_started_ms) >= MT6V10_RESPONSE_TIMEOUT_MS) ||
                (rx_length != 0U &&
                 (uint32_t)(now_ms - last_byte_ms) >= MT6V10_INTERBYTE_TIMEOUT_MS)) {
                encoder_status.response_timeouts++;
                mt6v10_reset_receiver();
            } else {
                return;
            }
        }
    }

    if ((uint32_t)(now_ms - last_request_ms) < MT6V10_POLL_PERIOD_MS) {
        return;
    }

    status = RS485_Receive(&byte, 1U, 0U);
    if (status == HAL_OK) {
        return;
    }
    if (status == HAL_BUSY) {
        return;
    }
    if (status == HAL_ERROR) {
        encoder_status.protocol_errors++;
    }
    (void)mt6v10_start_read(now_ms);
}

void Mt6v10_GetStatus(Mt6v10Status *status)
{
    if (status != 0) {
        *status = encoder_status;
    }
}

uint8_t Mt6v10_TakeUpdate(Mt6v10Status *status)
{
    if (status == 0 || update_pending == 0U) {
        return 0U;
    }
    *status = encoder_status;
    update_pending = 0U;
    return 1U;
}
