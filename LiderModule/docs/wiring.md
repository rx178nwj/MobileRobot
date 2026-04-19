# Wiring

## XIAO ESP32-C3 Pins

| XIAO pin | GPIO | Function | Connects to |
| --- | ---: | --- | --- |
| D6 | GPIO20 | UART0 TX | STS3215 servo driver RX |
| D7 | GPIO21 | UART0 RX | STS3215 servo driver TX |
| D2 | GPIO4 | UART1 RX | YDLIDAR T-mini Pro TX |
| D3 | GPIO5 | UART1 TX | YDLIDAR T-mini Pro RX |
| D4 | GPIO6 | I2C SDA | MPU6050 SDA |
| D5 | GPIO7 | I2C SCL | MPU6050 SCL |
| USB-C | USB CDC | Host link | Raspberry Pi USB |

## Power

- STS3215: 7.4 V servo power, 3 A or more recommended.
- YDLIDAR T-mini Pro: 5 V external supply.
- XIAO ESP32-C3: USB-C or regulated 5 V.
- MPU6050: XIAO 3.3 V.
- All grounds must be common.

## YDLIDAR T-mini Pro

| Pin | Signal | Connects to |
| --- | --- | --- |
| 1 | VCC | 5 V |
| 2 | TX | XIAO D2 / GPIO4 |
| 3 | GND | common GND |
| 4 | RX | XIAO D3 / GPIO5 |

The UART signal level is treated as 3.3 V.

## MPU6050

| MPU6050 pin | Connects to |
| --- | --- |
| VCC | XIAO 3.3 V |
| GND | common GND |
| SDA | XIAO D4 / GPIO6 |
| SCL | XIAO D5 / GPIO7 |
| AD0 | GND, address 0x68 |

## Servo Notes

- STS3215 ID is expected to be `1`.
- Center position is expected to be `2048`.
- The firmware limits requested tilt to +/-50 degrees.
- Confirm direction and center mechanically before running a full scan.
