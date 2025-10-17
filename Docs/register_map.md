# Modbus Register Map - DAL Module

## Tổng quan
Module DAL (Data Acquisition and Logging) sử dụng giao thức Modbus RTU qua UART2 với các tính năng:
- **Slave Address**: 4 (mặc định, có thể thay đổi)
- **Baudrate**: 115200 bps (mặc định, có thể thay đổi 9600-115200)
- **Parity**: None (mặc định, có thể thay đổi)
- **Stop Bits**: 1 (mặc định, có thể thay đổi)
- **Function Codes**: 0x03, 0x04, 0x06, 0x10
- **Tổng số thanh ghi**: 256 (0x0000 - 0x00FF)

---

## 1. IMU Registers (0x0000 - 0x000A)
Dữ liệu từ cảm biến BNO055 IMU

| Địa chỉ (Dec) | Địa chỉ (Hex) | Tên | Mô tả | Đơn vị | R/W | Kiểu dữ liệu |
|---------------|---------------|-----|-------|--------|-----|--------------|
| 0 | 0x0000 | REG_ACCEL_X | Gia tốc trục X | LSB | R | uint16 |
| 1 | 0x0001 | REG_ACCEL_Y | Gia tốc trục Y | LSB | R | uint16 |
| 2 | 0x0002 | REG_ACCEL_Z | Gia tốc trục Z | LSB | R | uint16 |
| 3 | 0x0003 | REG_GYRO_X | Gyroscope trục X | LSB | R | uint16 |
| 4 | 0x0004 | REG_GYRO_Y | Gyroscope trục Y | LSB | R | uint16 |
| 5 | 0x0005 | REG_GYRO_Z | Gyroscope trục Z | LSB | R | uint16 |
| 6 | 0x0006 | REG_VELOCITY | Vận tốc thiết bị | 0.01 m/s | R | int16 |
| 7 | 0x0007 | REG_HEADING | Hướng thiết bị | 0.1° | R | uint16 |
| 9 | 0x0009 | REG_IMU_STATUS | Trạng thái IMU | Bit field | R | uint16 |
| 10 | 0x000A | REG_IMU_ERROR | Lỗi IMU | Bit field | R | uint16 |

### Chi tiết:

#### REG_VELOCITY (0x0006)
- **Format**: Signed integer, scale = 100
- **Giá trị**: -5000 đến +5000 (-50.00 m/s đến +50.00 m/s)
- **Ví dụ**:
  - Giá trị = 123 → Vận tốc = 1.23 m/s
  - Giá trị = -50 → Vận tốc = -0.50 m/s (lùi)
  - Giá trị = 0 → Đứng yên

#### REG_HEADING (0x0007)
- **Format**: Unsigned integer, scale = 10
- **Giá trị**: 0 đến 3599 (0.0° đến 359.9°)
- **Ví dụ**:
  - Giá trị = 0 → Hướng = 0.0° (Bắc)
  - Giá trị = 900 → Hướng = 90.0° (Đông)
  - Giá trị = 1800 → Hướng = 180.0° (Nam)
  - Giá trị = 2700 → Hướng = 270.0° (Tây)
  - Giá trị = 1234 → Hướng = 123.4°

#### REG_IMU_STATUS (0x0009)
- **Bit 0**: 1 = IMU khởi tạo thành công, 0 = IMU chưa sẵn sàng

#### REG_IMU_ERROR (0x000A)
- **Bit 0**: 1 = IMU có lỗi, 0 = IMU hoạt động bình thường

---

## 2. Digital Input Registers (0x000B - 0x0010)
Trạng thái các digital inputs

| Địa chỉ (Dec) | Địa chỉ (Hex) | Tên | Mô tả | Giá trị | R/W | Kiểu dữ liệu |
|---------------|---------------|-----|-------|---------|-----|--------------|
| 11 | 0x000B | REG_DI_1 | Digital Input 1 (PA7) | 0 hoặc 1 | R | uint16 |
| 12 | 0x000C | REG_DI_2 | Digital Input 2 (PA6) | 0 hoặc 1 | R | uint16 |
| 13 | 0x000D | REG_DI_3 | Digital Input 3 (PA5) | 0 hoặc 1 | R | uint16 |
| 14 | 0x000E | REG_DI_4 | Digital Input 4 (PA4) | 0 hoặc 1 | R | uint16 |
| 15 | 0x000F | REG_DI_STATUS | Trạng thái tất cả DI | Bit field | R | uint16 |
| 16 | 0x0010 | REG_DI_ERROR | Lỗi Digital Input | Bit field | R | uint16 |

### Chi tiết:

#### REG_DI_STATUS (0x000F)
- **Bit 0**: DI_1 state (0 = LOW, 1 = HIGH)
- **Bit 1**: DI_2 state
- **Bit 2**: DI_3 state
- **Bit 3**: DI_4 state
- **Bit 4-15**: Reserved

**Ví dụ**:
- Giá trị = 0b0000000000001111 (15) → Tất cả inputs HIGH
- Giá trị = 0b0000000000000101 (5) → DI_1 và DI_3 HIGH, DI_2 và DI_4 LOW

---

## 3. PN532 NFC/RFID Registers (0x0020 - 0x0026)
Dữ liệu từ module PN532 NFC reader

| Địa chỉ (Dec) | Địa chỉ (Hex) | Tên | Mô tả | R/W | Kiểu dữ liệu |
|---------------|---------------|-----|-------|-----|--------------|
| 32 | 0x0020 | REG_PN532_DATA_LOW | Reserved - không dùng | R | uint16 |
| 33 | 0x0021 | REG_PN532_DATA_HIGH | Reserved - không dùng | R | uint16 |
| 34 | 0x0022 | REG_PN532_STATUS | Trạng thái PN532 | R | uint16 |
| 35 | 0x0023 | REG_PN532_ERROR | Lỗi PN532 | R | uint16 |
| 36 | 0x0024 | REG_PN532_CARD_TYPE | Loại thẻ NFC | R | uint16 |
| 37 | 0x0025 | REG_PN532_CARD_UID_HIGH | UID cao (Byte 0-1) | R | uint16 |
| 38 | 0x0026 | REG_PN532_CARD_UID_LOW | UID thấp (Byte 2-3) | R | uint16 |

### Chi tiết:

#### REG_PN532_STATUS (0x0022)
- **0**: PN532 không sẵn sàng
- **1**: PN532 sẵn sàng và hoạt động

#### REG_PN532_ERROR (0x0023)
- **0**: Không có lỗi
- **1**: PN532 có lỗi

#### REG_PN532_CARD_TYPE (0x0024)
- **0**: Không có thẻ
- **1**: Mifare Classic (4-byte UID)
- **2**: Mifare Ultralight (7-byte UID)

#### REG_PN532_CARD_UID_HIGH & LOW (0x0025-0x0026)
**Format**: Big-Endian, 32-bit UID

- **REG_PN532_CARD_UID_HIGH**: `(Byte0 << 8) | Byte1`
- **REG_PN532_CARD_UID_LOW**: `(Byte2 << 8) | Byte3`

**Ví dụ**: Thẻ có UID = `0x12 0x34 0x56 0x78`
- REG_PN532_CARD_UID_HIGH = 0x1234 (4660)
- REG_PN532_CARD_UID_LOW = 0x5678 (22136)

**Cách đọc trong Python**:
```python
uid_high = read_register(0x0025)
uid_low = read_register(0x0026)

# Reconstruct 32-bit UID
uid = (uid_high << 16) | uid_low
print(f"Card UID: 0x{uid:08X}")

# Hoặc tách từng byte
byte0 = (uid_high >> 8) & 0xFF
byte1 = uid_high & 0xFF
byte2 = (uid_low >> 8) & 0xFF
byte3 = uid_low & 0xFF
print(f"UID: {byte0:02X} {byte1:02X} {byte2:02X} {byte3:02X}")
```

---

## 4. Configuration Registers (0x0030 - 0x0034)
Các thanh ghi cấu hình hệ thống

| Địa chỉ (Dec) | Địa chỉ (Hex) | Tên | Mô tả | R/W | Kiểu dữ liệu |
|---------------|---------------|-----|-------|-----|--------------|
| 48 | 0x0030 | REG_IMU_SAMPLE_RATE_CONFIG | Tốc độ lấy mẫu IMU | R/W | uint16 |
| 49 | 0x0031 | REG_DI_DEBOUNCE_TIME_CONFIG | Thời gian debounce DI | R/W | uint16 |
| 50 | 0x0032 | REG_NFC_READ_TIMEOUT_CONFIG | Timeout đọc NFC | R/W | uint16 |
| 51 | 0x0033 | REG_DATA_VALID_STATE | Trạng thái data valid | R/W | uint16 |
| 52 | 0x0034 | REG_FAULT_REPORTING | Báo cáo lỗi | R/W | uint16 |

**Lưu ý**: Các thanh ghi này hiện chưa được implement đầy đủ, reserved cho tương lai.

---

## 5. System Registers (0x0100 - 0x0109)
Thông tin hệ thống và cấu hình Modbus

| Địa chỉ (Dec) | Địa chỉ (Hex) | Tên | Mô tả | Giá trị | R/W | Kiểu dữ liệu |
|---------------|---------------|-----|-------|---------|-----|--------------|
| 256 | 0x0100 | REG_DEVICE_ID | Modbus Slave Address | 1-247 | R/W | uint16 |
| 257 | 0x0101 | REG_CONFIG_BAUDRATE | Cấu hình baudrate | 1-5 | R/W | uint16 |
| 258 | 0x0102 | REG_CONFIG_PARITY | Cấu hình parity | 0-2 | R/W | uint16 |
| 259 | 0x0103 | REG_CONFIG_STOP_BITS | Cấu hình stop bits | 1-2 | R/W | uint16 |
| 260 | 0x0104 | REG_MODULE_TYPE | Loại module | 0x0002 | R | uint16 |
| 261 | 0x0105 | REG_FIRMWARE_VERSION | Phiên bản firmware | 0x0101 | R | uint16 |
| 262 | 0x0106 | REG_HARDWARE_VERSION | Phiên bản hardware | 0x0101 | R | uint16 |
| 263 | 0x0107 | REG_SYSTEM_STATUS | Trạng thái hệ thống | Bit field | R | uint16 |
| 264 | 0x0108 | REG_SYSTEM_ERROR | Lỗi hệ thống | Bit field | R | uint16 |
| 265 | 0x0109 | REG_RESET_ERROR_CMD | Lệnh reset lỗi | 1 | W | uint16 |

### Chi tiết:

#### REG_DEVICE_ID (0x0100)
- **Mặc định**: 1
- **Giá trị hợp lệ**: 1-247
- **Ghi chú**: Thay đổi địa chỉ Modbus slave. Sau khi ghi, địa chỉ mới có hiệu lực ngay lập tức.

#### REG_CONFIG_BAUDRATE (0x0101)
- **1**: 9600 bps
- **2**: 19200 bps
- **3**: 38400 bps
- **4**: 57600 bps
- **5**: 115200 bps (mặc định)

**⚠️ Cảnh báo**: Sau khi thay đổi baudrate, phải:
1. Đợi response của slave
2. Đổi baudrate của Master
3. Reconnect với baudrate mới

#### REG_CONFIG_PARITY (0x0102)
- **0**: None (mặc định)
- **1**: Even
- **2**: Odd

#### REG_CONFIG_STOP_BITS (0x0103)
- **1**: 1 stop bit (mặc định)
- **2**: 2 stop bits

#### REG_MODULE_TYPE (0x0104)
- **0x0002**: DAL Module (Data Acquisition and Logging)
- Read-only

#### REG_FIRMWARE_VERSION (0x0105)
- **Format**: 0xMMmm (Major.minor)
- **Ví dụ**: 0x0101 = v1.01
- Read-only

#### REG_HARDWARE_VERSION (0x0106)
- **Format**: 0xMMmm (Major.minor)
- **Ví dụ**: 0x0101 = v1.01
- Read-only

#### REG_SYSTEM_STATUS (0x0107)
- **Bit 0**: IMU status (1 = OK, 0 = Not ready)
- **Bit 1**: NFC status (1 = OK, 0 = Not ready)
- **Bit 2-15**: Reserved

**Ví dụ**:
- Giá trị = 0x03 (0b11) → IMU và NFC đều OK
- Giá trị = 0x01 (0b01) → Chỉ IMU OK
- Giá trị = 0x02 (0b10) → Chỉ NFC OK
- Giá trị = 0x00 → Cả 2 đều chưa sẵn sàng

#### REG_SYSTEM_ERROR (0x0108)
- **Bit 0**: IMU error (1 = Error, 0 = No error)
- **Bit 1**: NFC error (1 = Error, 0 = No error)
- **Bit 2-15**: Reserved

#### REG_RESET_ERROR_CMD (0x0109)
- **Ghi 1**: Reset tất cả error flags
- Write-only
- LED_FAULT sẽ tắt sau khi reset

---

## 6. LED Indicators

Module có 3 LED chỉ báo trạng thái:

| LED | Pin | Chức năng |
|-----|-----|-----------|
| LED | PC13 | System heartbeat - blink 500ms |
| LED_MB | PC14 | Modbus activity indicator |
| LED_FAULT | PC15 | System fault indicator |

### LED_MB (Modbus Activity)
- **Bật**: Khi nhận Modbus frame hợp lệ (CRC OK, địa chỉ đúng)
- **Tắt**: Sau khi gửi response thành công
- **Mục đích**: Giúp debug và monitor hoạt động Modbus

### LED_FAULT (System Fault)
- **Bật**: Khi `REG_SYSTEM_ERROR != 0`
- **Tắt**: Khi không có lỗi hoặc sau khi reset error
- **Mục đích**: Cảnh báo lỗi hệ thống

---

## 7. Modbus Function Codes hỗ trợ

| Function Code | Tên | Mô tả |
|---------------|-----|-------|
| 0x03 | Read Holding Registers | Đọc nhiều thanh ghi (max 125) |
| 0x04 | Read Input Registers | Đọc nhiều thanh ghi (max 125) |
| 0x06 | Write Single Register | Ghi 1 thanh ghi |
| 0x10 | Write Multiple Registers | Ghi nhiều thanh ghi (max 123) |

---

## 8. Ví dụ sử dụng

### 8.1. Python với pyModbus

```python
from pymodbus.client import ModbusSerialClient

# Kết nối
client = ModbusSerialClient(
    port='COM3',
    baudrate=115200,
    parity='N',
    stopbits=1,
    timeout=1
)

if client.connect():
    # Đọc System Info
    result = client.read_holding_registers(0x0100, 9, slave=1)
    if not result.isError():
        print(f"Device ID: {result.registers[0]}")
        print(f"Module Type: 0x{result.registers[4]:04X}")
        print(f"Firmware: v{result.registers[5] >> 8}.{result.registers[5] & 0xFF:02d}")
    
    # Đọc IMU Data
    result = client.read_holding_registers(0x0006, 2, slave=1)
    if not result.isError():
        velocity = result.registers[0] / 100.0  # m/s
        heading = result.registers[1] / 10.0    # degrees
        print(f"Velocity: {velocity:.2f} m/s")
        print(f"Heading: {heading:.1f}°")
    
    # Đọc NFC Card UID
    result = client.read_holding_registers(0x0025, 2, slave=1)
    if not result.isError():
        uid_high = result.registers[0]
        uid_low = result.registers[1]
        uid = (uid_high << 16) | uid_low
        print(f"Card UID: 0x{uid:08X}")
    
    # Reset errors
    client.write_register(0x0109, 1, slave=1)
    print("Errors reset!")
    
    client.close()
```

### 8.2. Modbus Poll (Windows)

**Đọc tất cả IMU data**:
- Function: 03
- Address: 0
- Length: 11
- Scan Rate: 1000ms

**Đọc System Status**:
- Function: 03
- Address: 256
- Length: 9

**Reset Errors**:
- Function: 06
- Address: 265
- Value: 1

### 8.3. C/C++ với libmodbus

```c
#include <modbus.h>

modbus_t *ctx;
uint16_t tab_reg[10];

// Kết nối
ctx = modbus_new_rtu("COM3", 115200, 'N', 8, 1);
modbus_set_slave(ctx, 1);
modbus_connect(ctx);

// Đọc velocity và heading
modbus_read_registers(ctx, 0x0006, 2, tab_reg);
float velocity = (int16_t)tab_reg[0] / 100.0f;
float heading = tab_reg[1] / 10.0f;
printf("Velocity: %.2f m/s, Heading: %.1f°\n", velocity, heading);

// Đọc NFC UID
modbus_read_registers(ctx, 0x0025, 2, tab_reg);
uint32_t uid = ((uint32_t)tab_reg[0] << 16) | tab_reg[1];
printf("Card UID: 0x%08X\n", uid);

modbus_close(ctx);
modbus_free(ctx);
```

---

## 9. Error Handling

### Exception Codes

| Code | Tên | Mô tả |
|------|-----|-------|
| 0x01 | Illegal Function | Function code không được hỗ trợ |
| 0x02 | Illegal Data Address | Địa chỉ thanh ghi không hợp lệ |
| 0x03 | Illegal Data Value | Giá trị dữ liệu không hợp lệ |
| 0x04 | Slave Device Failure | Lỗi thiết bị slave |

### Timeout
- **Giá trị đề xuất**: 1000ms (1 giây)
- **Frame timeout**: 5ms (T3.5 character time)

---

## 10. Performance

| Metric | Giá trị |
|--------|---------|
| Baudrate max | 115200 bps |
| Response time | < 10ms @ 115200 bps |
| Throughput | ~100 transactions/second |
| Max registers per read | 125 |
| Max registers per write | 123 |

---

## 11. Troubleshooting

### Không nhận được response
1. Kiểm tra kết nối phần cứng (TX↔RX cross)
2. Verify baudrate, parity, stop bits
3. Kiểm tra Slave Address
4. Kiểm tra CRC

### CRC Error
1. Giảm baudrate xuống 9600 bps test
2. Kiểm tra cable chất lượng
3. Thêm delay giữa các request

### Dữ liệu không đúng
1. Verify địa chỉ thanh ghi
2. Kiểm tra scaling factors
3. Verify byte order (Big-Endian)

---

## 12. Change Log

| Version | Date | Changes |
|---------|------|---------|
| v1.0 | 2025-01-17 | Initial release với Modbus RTU |
| - | - | Hỗ trợ IMU, NFC, Digital Inputs |
| - | - | LED indicators |
| - | - | Runtime configuration |

---

## 13. Liên hệ & Hỗ trợ

- **Author**: tiensy
- **Project**: DAL Module Firmware
- **Platform**: STM32F103C8T6
- **Framework**: HAL + FreeRTOS

---

**Copyright © 2025 TBE. All rights reserved.**

