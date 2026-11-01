#include "mt6v10_modbus.h"
#include "rs485_port.h"

#define TEST_RX_CAPACITY 16U

uint32_t test_checks;
uint32_t test_failed_line;
uint32_t test_done;

static uint8_t mock_rx[TEST_RX_CAPACITY];
static uint8_t mock_rx_length;
static uint8_t mock_rx_position;
static uint8_t mock_tx[8];
static uint8_t mock_tx_length;
static uint32_t mock_tx_count;
static HAL_StatusTypeDef mock_tx_status;

#define CHECK(condition) do { \
    test_checks++; \
    if (!(condition) && test_failed_line == 0U) { \
        test_failed_line = __LINE__; \
    } \
} while (0)

HAL_StatusTypeDef RS485_Transmit(const uint8_t *data, uint16_t length,
                                 uint32_t timeout_ms)
{
    uint16_t index;
    (void)timeout_ms;
    mock_tx_count++;
    mock_tx_length = (uint8_t)length;
    if (mock_tx_status != HAL_OK || length > sizeof(mock_tx)) {
        return mock_tx_status == HAL_OK ? HAL_ERROR : mock_tx_status;
    }
    for (index = 0U; index < length; ++index) {
        mock_tx[index] = data[index];
    }
    return HAL_OK;
}

HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t length,
                                uint32_t timeout_ms)
{
    uint16_t index;
    (void)timeout_ms;
    if (length == 0U) {
        return HAL_OK;
    }
    if (data == 0) {
        return HAL_ERROR;
    }
    if ((uint16_t)mock_rx_position + length > mock_rx_length) {
        return HAL_TIMEOUT;
    }
    for (index = 0U; index < length; ++index) {
        data[index] = mock_rx[mock_rx_position++];
    }
    return HAL_OK;
}

uint32_t RS485_GetRxOverrunCount(void)
{
    return 0U;
}

static uint16_t test_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    for (index = 0U; index < length; ++index) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static void queue_response(uint16_t angle_raw, int16_t turns, uint8_t corrupt_crc)
{
    uint16_t crc;
    mock_rx[0] = 1U;
    mock_rx[1] = 3U;
    mock_rx[2] = 4U;
    mock_rx[3] = (uint8_t)(angle_raw >> 8U);
    mock_rx[4] = (uint8_t)angle_raw;
    mock_rx[5] = (uint8_t)((uint16_t)turns >> 8U);
    mock_rx[6] = (uint8_t)turns;
    crc = test_crc16(mock_rx, 7U);
    mock_rx[7] = (uint8_t)crc;
    mock_rx[8] = (uint8_t)(crc >> 8U);
    if (corrupt_crc != 0U) {
        mock_rx[7] ^= 1U;
    }
    mock_rx_length = 9U;
    mock_rx_position = 0U;
}

static void inject_response(uint16_t angle_raw, int16_t turns,
                            uint32_t now_ms, uint8_t corrupt_crc)
{
    uint8_t index;
    queue_response(angle_raw, turns, corrupt_crc);
    for (index = 0U; index < 9U; ++index) {
        Mt6v10_Task(now_ms);
    }
}

static void test_reset(void)
{
    uint8_t index;
    for (index = 0U; index < TEST_RX_CAPACITY; ++index) {
        mock_rx[index] = 0U;
    }
    for (index = 0U; index < sizeof(mock_tx); ++index) {
        mock_tx[index] = 0U;
    }
    mock_rx_length = 0U;
    mock_rx_position = 0U;
    mock_tx_length = 0U;
    mock_tx_count = 0U;
    mock_tx_status = HAL_OK;
    test_checks = 0U;
    test_failed_line = 0U;
    test_done = 0U;
    Mt6v10_Init(0U);
}

uint32_t Test_ReferenceCrc(void)
{
    static const uint8_t sample[] = {0x01U, 0x03U, 0x02U, 0x12U, 0x34U};
    uint16_t crc = test_crc16(sample, sizeof(sample));
    return (crc == 0x33B5U) ? 1U : 0U;
}

uint32_t Test_Run(void)
{
    static const uint8_t request[] = {
        0x01U, 0x03U, 0x00U, 0x01U, 0x00U, 0x02U, 0x95U, 0xCBU
    };
    uint8_t index;
    Mt6v10Status status;

    test_reset();
    CHECK(Test_ReferenceCrc() == 1U);
    Mt6v10_Task(0U);
    CHECK(mock_tx_count == 1U);
    CHECK(mock_tx_length == sizeof(request));
    for (index = 0U; index < sizeof(request); ++index) {
        CHECK(mock_tx[index] == request[index]);
    }

    inject_response(0x1234U, 2, 2U, 0U);
    Mt6v10_GetStatus(&status);
    CHECK(status.online == 1U);
    CHECK(status.has_sample == 1U);
    CHECK(status.angle_raw == 0x1234U);
    CHECK(status.turn_count == 2);
    CHECK(status.angle_mrad == 13013L);
    CHECK(status.speed_rpm_x10 == 0);
    CHECK(Mt6v10_TakeUpdate(&status) == 1U);
    CHECK(Mt6v10_TakeUpdate(&status) == 0U);

    Mt6v10_Task(20U);
    CHECK(mock_tx_count == 2U);
    inject_response(0x12EAU, 2, 22U, 0U);
    Mt6v10_GetStatus(&status);
    CHECK(status.online == 1U);
    CHECK(status.angle_raw == 0x12EAU);
    CHECK(status.speed_rpm_x10 > 0);
    CHECK(status.speed_rpm_x10 < 32767);

    Mt6v10_Task(40U);
    CHECK(mock_tx_count == 3U);
    queue_response(0x1234U, 2, 0U);
    for (index = 0U; index < 9U; ++index) {
        Mt6v10_Task(61U);
    }
    Mt6v10_GetStatus(&status);
    CHECK(status.speed_rpm_x10 < 0);
    CHECK(status.sample_ms == 61U);

    inject_response(0x1234U, 2, 63U, 1U);
    Mt6v10_GetStatus(&status);
    CHECK(status.crc_errors == 1U);
    CHECK(status.sample_ms == 61U);

    Mt6v10_Task(81U);
    CHECK(mock_tx_count == 5U);
    Mt6v10_Task(100U);
    CHECK(mock_tx_count == 5U);
    Mt6v10_Task(101U);
    CHECK(mock_tx_count == 6U);
    Mt6v10_Task(120U);
    CHECK(mock_tx_count == 6U);
    Mt6v10_Task(121U);
    CHECK(mock_tx_count == 7U);
    Mt6v10_GetStatus(&status);
    CHECK(status.response_timeouts == 2U);

    Mt6v10_Task(161U);
    Mt6v10_GetStatus(&status);
    CHECK(status.online == 0U);
    CHECK(status.speed_rpm_x10 == 0);
    CHECK(status.has_sample == 1U);
    CHECK(status.response_timeouts == 3U);
    CHECK(mock_tx_count == 8U);
    CHECK(Mt6v10_TakeUpdate(&status) == 1U);

    test_done = 1U;
    return test_failed_line;
}
