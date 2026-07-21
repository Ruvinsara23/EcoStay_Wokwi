#include "wokwi-api.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PZEM_ADDRESS       0x01
#define CMD_READ_HOLDING   0x03
#define CMD_READ_INPUT     0x04
#define CMD_WRITE_SINGLE   0x06
#define CMD_RESET_ENERGY   0x42

#define VOLTAGE_V          235.0
#define FAN_POWER_W        70.0
#define LAMP_POWER_W       12.0

// The EcoStay four-channel relay board is active-low: LOW means load ON.
#define LOAD_ON_LEVEL      LOW

// One real simulation second represents 60 virtual seconds.
#define TIME_SCALE         60.0

typedef struct {
  uart_dev_t uart;

  pin_t fan_pin;
  pin_t lamp_pin;

  bool fan_on;
  bool lamp_on;

  uint8_t rx_buffer[8];
  uint8_t rx_length;
  uint8_t address;

  double energy_wh;
  uint64_t last_update_ns;

  uint8_t tx_buffer[25];
} chip_state_t;

static uint16_t calculate_crc(
  const uint8_t *data,
  uint32_t length
) {
  uint16_t crc = 0xFFFF;

  for (uint32_t position = 0; position < length; position++) {
    crc ^= data[position];

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static bool valid_crc(
  const uint8_t *data,
  uint8_t length
) {
  if (length < 4) {
    return false;
  }

  uint16_t expected = calculate_crc(data, length - 2);

  uint16_t received =
    (uint16_t)data[length - 2] |
    ((uint16_t)data[length - 1] << 8);

  return expected == received;
}

static void add_crc(
  uint8_t *data,
  uint8_t length_without_crc
) {
  uint16_t crc = calculate_crc(data, length_without_crc);

  data[length_without_crc] = crc & 0xFF;
  data[length_without_crc + 1] = crc >> 8;
}

static void put_u16_be(
  uint8_t *destination,
  uint16_t value
) {
  destination[0] = value >> 8;
  destination[1] = value & 0xFF;
}

// The PZEM protocol places the low 16-bit register first.
static void put_u32_pzem(
  uint8_t *destination,
  uint32_t value
) {
  put_u16_be(destination, (uint16_t)(value & 0xFFFF));
  put_u16_be(destination + 2, (uint16_t)(value >> 16));
}

static double get_power_w(const chip_state_t *chip) {
  double power = 0.0;

  if (chip->fan_on) {
    power += FAN_POWER_W;
  }

  if (chip->lamp_on) {
    power += LAMP_POWER_W;
  }

  return power;
}

static void update_energy(chip_state_t *chip) {
  uint64_t now_ns = get_sim_nanos();

  if (now_ns < chip->last_update_ns) {
    chip->last_update_ns = now_ns;
    return;
  }

  uint64_t elapsed_ns = now_ns - chip->last_update_ns;
  chip->last_update_ns = now_ns;

  double elapsed_seconds = (double)elapsed_ns / 1000000000.0;
  double virtual_hours = (elapsed_seconds * TIME_SCALE) / 3600.0;

  chip->energy_wh += get_power_w(chip) * virtual_hours;
}

static void on_load_change(
  void *user_data,
  pin_t pin,
  uint32_t value
) {
  chip_state_t *chip = (chip_state_t *)user_data;

  // Account for energy using the previous load state before changing it.
  update_energy(chip);

  if (pin == chip->fan_pin) {
    chip->fan_on = value == LOAD_ON_LEVEL;
  }

  if (pin == chip->lamp_pin) {
    chip->lamp_on = value == LOAD_ON_LEVEL;
  }

  printf(
    "EcoStay load change: fan=%s lamp=%s power=%.1f W\n",
    chip->fan_on ? "ON" : "OFF",
    chip->lamp_on ? "ON" : "OFF",
    get_power_w(chip)
  );
}

static void send_measurements(
  chip_state_t *chip,
  uint8_t reply_address
) {
  update_energy(chip);

  double power_w = get_power_w(chip);

  // Deliberate simulation assumption: power factor is 1.00.
  double current_a = power_w > 0.0 ? power_w / VOLTAGE_V : 0.0;

  uint16_t voltage_raw = (uint16_t)(VOLTAGE_V * 10.0 + 0.5);
  uint32_t current_raw = (uint32_t)(current_a * 1000.0 + 0.5);
  uint32_t power_raw = (uint32_t)(power_w * 10.0 + 0.5);

  // PZEM-004T reports accumulated energy in whole Wh.
  uint32_t energy_raw = (uint32_t)chip->energy_wh;

  uint16_t frequency_raw = 500; // 50.0 Hz
  uint16_t pf_raw = power_w > 0.0 ? 100 : 0;
  uint16_t alarm_raw = 0;

  uint8_t *data = chip->tx_buffer;

  data[0] = reply_address;
  data[1] = CMD_READ_INPUT;
  data[2] = 0x14;

  put_u16_be(&data[3], voltage_raw);
  put_u32_pzem(&data[5], current_raw);
  put_u32_pzem(&data[9], power_raw);
  put_u32_pzem(&data[13], energy_raw);
  put_u16_be(&data[17], frequency_raw);
  put_u16_be(&data[19], pf_raw);
  put_u16_be(&data[21], alarm_raw);

  add_crc(data, 23);
  uart_write(chip->uart, data, 25);
}

static void send_address(
  chip_state_t *chip,
  uint8_t reply_address
) {
  uint8_t *data = chip->tx_buffer;

  data[0] = reply_address;
  data[1] = CMD_READ_HOLDING;
  data[2] = 0x02;
  data[3] = 0x00;
  data[4] = chip->address;

  add_crc(data, 5);
  uart_write(chip->uart, data, 7);
}

static void process_request(chip_state_t *chip) {
  uint8_t *request = chip->rx_buffer;
  uint8_t length = chip->rx_length;

  if (!valid_crc(request, length)) {
    printf("Invalid Modbus CRC\n");
    return;
  }

  uint8_t request_address = request[0];
  uint8_t function = request[1];

  if (request_address != chip->address && request_address != 0xF8) {
    return;
  }

  if (
    length == 8 &&
    function == CMD_READ_INPUT &&
    request[2] == 0x00 &&
    request[3] == 0x00 &&
    request[4] == 0x00 &&
    request[5] == 0x0A
  ) {
    send_measurements(chip, request_address);
    return;
  }

  if (
    length == 8 &&
    function == CMD_READ_HOLDING &&
    request[2] == 0x00 &&
    request[3] == 0x02 &&
    request[4] == 0x00 &&
    request[5] == 0x01
  ) {
    send_address(chip, request_address);
    return;
  }

  if (length == 8 && function == CMD_WRITE_SINGLE) {
    memcpy(chip->tx_buffer, request, 8);
    uart_write(chip->uart, chip->tx_buffer, 8);

    if (
      request[2] == 0x00 &&
      request[3] == 0x02 &&
      request[4] == 0x00 &&
      request[5] >= 0x01 &&
      request[5] <= 0xF7
    ) {
      chip->address = request[5];
    }

    return;
  }

  if (length == 4 && function == CMD_RESET_ENERGY) {
    update_energy(chip);
    chip->energy_wh = 0.0;

    memcpy(chip->tx_buffer, request, 4);
    uart_write(chip->uart, chip->tx_buffer, 4);
  }
}

static void on_uart_rx_data(
  void *user_data,
  uint8_t byte
) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->rx_length >= sizeof(chip->rx_buffer)) {
    chip->rx_length = 0;
  }

  chip->rx_buffer[chip->rx_length++] = byte;

  if (chip->rx_length < 2) {
    return;
  }

  uint8_t expected_length =
    chip->rx_buffer[1] == CMD_RESET_ENERGY ? 4 : 8;

  if (chip->rx_length == expected_length) {
    process_request(chip);
    chip->rx_length = 0;
  }
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));

  chip->address = PZEM_ADDRESS;
  chip->energy_wh = 0.0;
  chip->rx_length = 0;

  // Pull-ups match the firmware's active-low OFF level and avoid a startup
  // phantom load before the ESP32 configures its relay output pins.
  chip->fan_pin = pin_init("FAN", INPUT_PULLUP);
  chip->lamp_pin = pin_init("LAMP", INPUT_PULLUP);

  chip->fan_on = pin_read(chip->fan_pin) == LOAD_ON_LEVEL;
  chip->lamp_on = pin_read(chip->lamp_pin) == LOAD_ON_LEVEL;
  chip->last_update_ns = get_sim_nanos();

  const pin_watch_config_t load_watch = {
    .edge = BOTH,
    .pin_change = on_load_change,
    .user_data = chip,
  };

  pin_watch(chip->fan_pin, &load_watch);
  pin_watch(chip->lamp_pin, &load_watch);

  const uart_config_t uart_config = {
    .tx = pin_init("TX", INPUT_PULLUP),
    .rx = pin_init("RX", INPUT),
    .baud_rate = 9600,
    .rx_data = on_uart_rx_data,
    .write_done = NULL,
    .user_data = chip,
  };

  chip->uart = uart_init(&uart_config);

  printf(
    "EcoStay PZEM ready: address=0x01 active-low loads, %.0f W fan + %.0f W lamp\n",
    FAN_POWER_W,
    LAMP_POWER_W
  );
}

